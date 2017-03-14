#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "os.h"

// Task priorities which determine which queue a task is scheduled in.
#define SYSTEM 3
#define PERIODIC 2
#define ROUND_ROBIN 1

/*******
 * External functions
 *******/

/* These two are defined in cswitch.S. They handle context switching. */
extern void Exit_Kernel();
extern void Enter_Kernel();

/******
 * OS internal data structures and types
 ******/

/* Not declared with "static" because they need to be visible to cswitch.S. */
volatile uint8_t *CurrentSp;
volatile uint8_t *KernelSp;

// TODO: A bunch of this stuff might need the "volatile" keyword.
typedef struct {
	PID pid; // A number for this task; also its (index - 1) in "tasks" array.
	uint8_t stack[WORKSPACE]; // Stack space. Includes the task's argument, arg.
	uint8_t priority; // Priority level (Sys, Per, RR).
	volatile uint8_t *sp; // Stack pointer.
	TICK period; // These 3 TICK properties are only used by periodic tasks.
	TICK offset;
	TICK wcet; // wcet = worst-case execution time.
} task_t;
// Allocate space for each task...
static task_t tasks[MAXTHREAD];
// If MAXTHREAD is changed to over 16, we will need a bigger mask.
static uint16_t task_mask = 0; // Bitmask for used (1) tasks and free (0) tasks.
// ...and create a wait queue for each priority level.
static task_t *current_task = NULL;
static task_t *system_tasks[MAXTHREAD];
static task_t *rr_tasks[MAXTHREAD];
static uint8_t sys_q_size = 0,
			   sys_q_head = 0,
			   rr_q_size = 0,
			   rr_q_head = 0;
/* The PID of a periodic task to be run ASAP, if any is ready.
 * If the scheduler goes to set this field and it is already set,
 * then that should be considered a timing violation. */
static volatile PID periodic_task_ready = NULL;

typedef uint8_t CHAN_STATE;
#define FREE 1
#define SENDER_BLOCKED 2
#define RECEIVER_BLOCKED 3
#define MESSAGE_WAITING 4
typedef struct {
	CHAN number;
	int data;
	CHAN_STATE state;
	PID sender;
	PID receivers[MAXTHREAD];
} chan_t;
static chan_t channels[MAXCHAN];
// If MAXTHREAD is changed to over 16, we will need a bigger mask.
static uint16_t channel_mask = 0;

/*******
 * Internal functions
 *******/

static void task_terminate(void) {
	// This function is like Task_Next, except we do not put this thread back
	// on the queue. Instead, we mark it as free, then just enter the kernel.
	cli();
	task_mask &= ~(1 << (uint8_t)(current_task->pid-1));
	Enter_Kernel();
}

static void task_create(unsigned int idx, void (*f)(void), int arg) {
	tasks[idx].pid = (PID)idx+1;

	/* Set up the stack.
	 * Code adapted from the example project by Scott and Justin.
	 *
	 * From the AVR-GCC ABI's "Frame Layout" section:
	 * "After the function prologue, the frame pointer will point one byte
	 * below the stack frame, i.e. Y+1 points to the bottom of the stack
	 * frame."
	 * So the stack pointer should point just past the data. 
	 * The ABI specifies "the stack grows downward," from high addresses to low
	 * addresses, with incoming arguments (in our case, arg) at the very
	 * bottom, the return address just above that, saved registers above that,
	 * and then finally stack space.
	 * We reserve 42 bytes here: 
	 *  * 2 for "int arg"
	 *  * 3 for the return address (those tricky "17-bit" addresses)
	 *  * 3 for the address of the task's first instruction (f)
	 *  * 34 for saved registers (32 + SREG + EIND)
	 * This order mirrors that of the ABI, with one exception: the address of
	 * f is on the stack too. This is because of how cswitch works: it expects
	 * that the next thing on the stack after the saved registers is the
	 * "return address" into f. This is only necessary for the task's first
	 * run; after that, the return addresses will just work out.
	 * Below that, we put a pointer to task_terminate; when f (actually) 
	 * returns, it will return into task_terminate and clean itself up.
	 * NB: In this code, the array indexes may be confusing and seem backwards.
	 * To be clear: the "top" of the stack is the highest address, the "bottom"
	 * is the lowest, and the stack grows into even-lower addresses. */
	uint8_t* stack_bottom = tasks[idx].stack + (WORKSPACE-1) - (43);

	/* Populate the initial stack from bottom to top.
	 * stack_bottom[0] is the first free byte in the stack.
	 * Next come all the saved registers.
	 * stack_bottom[1] is EIND, always set to 0 to start. */
	stack_bottom[1] = (uint8_t) 0;
	/* stack_bottom[2] is SREG. We clear it and set the I flag.*/
	stack_bottom[2] = (uint8_t) _BV(SREG_I);
	/* stack_bottom[3] is r31. */
	/*stack_bottom[33] is r1, the zero register. */
	stack_bottom[33] = (uint8_t) 0;
	/* stack_bottom[34] is r0. 
	 * Above that, we place a pointer to f so that the kernel "returns" into f
	 * after pointing SP at this task's stack pointer. */
	stack_bottom[35] = (uint8_t) 0;
	stack_bottom[36] = (uint8_t)((uint16_t)f >> 8);
	stack_bottom[37] = (uint8_t)(uint16_t)f;
	/* stack_bottom[35] is the first (highest-order) bit of our 3-byte return
	 * address, whose value is written to EIND when returning. */
	stack_bottom[38] = (uint8_t) 0;
	stack_bottom[39] = (uint8_t)((uint16_t)task_terminate >> 8);
	stack_bottom[40] = (uint8_t)(uint16_t)task_terminate;
	/* stack_bottom[38] and [39] hold the argument to f. */
	stack_bottom[41] = (uint8_t)(arg >> 8);
	stack_bottom[42] = (uint8_t)arg;
	
	tasks[idx].sp = stack_bottom;
}

/* Assumption with both of these enqueue functions: interrupts are disabled.
 * If interrupts are not already disabled, then these tasks are dangerous. */
static void enqueue_system_task(task_t *t) {
	if (sys_q_size < MAXTHREAD) {
		system_tasks[(sys_q_head + sys_q_size++) % MAXTHREAD] = t;
	} // TODO: else error!
}

static void enqueue_rr_task(task_t *t) {
	if (rr_q_size < MAXTHREAD) {
		rr_tasks[(rr_q_head + rr_q_size++) % MAXTHREAD] = t;
	} // TODO: else error!
}

/*******
 * Interface implementation
 * (i.e. the good stuff)
 *******/

void OS_Abort(unsigned int error) {
	// TODO: Jump to main, or (if necessary) the beginning of the
	// "crt" stuff added by the compiler before the main function.
}

PID Task_Create_System(void (*f)(void), int arg) {
	unsigned int i;
	for(i = 0; i < MAXTHREAD; i++) {
		if ((task_mask & (1 << i)) == 0) {
			// Mark this task as used.
			task_mask |= (1 << i);

			tasks[i].priority = SYSTEM;
			task_create(i, f, arg);
			break;
		}
	}
	if (i != MAXTHREAD) return tasks[i].pid;
	return NULL;
}

PID Task_Create_RR(void (*f)(void), int arg) {
	unsigned int i;
	for(i = 0; i < MAXTHREAD; i++) {
		if ((task_mask & (1 << i)) == 0) {
			//Mark task as used.
			task_mask |= (1 << i);

			tasks[i].priority = ROUND_ROBIN;
			task_create(i, f, arg);
			break;
		}
	}
	if (i != MAXTHREAD) return tasks[i].pid;
	return NULL;
}

PID Task_Create_Period(void (*f)(void), int arg, TICK period, TICK wcet, TICK offset) {
	return NULL;
}

/* Task_Next is called by any task which is not the kernel. When called, it
 * will cause the calling task to be put back in its proper queue, and then
 * the kernel is invoked to choose a new task. */
void Task_Next(void) {
	cli();
	// Put the current task back in its proper queue.
	if(current_task->priority == SYSTEM) {
		enqueue_system_task(current_task);
	} else if (current_task->priority == ROUND_ROBIN) {
		enqueue_rr_task(current_task);
	} else {
		// TODO: What do if a periodic task calls Task_Next? Probs an error.
	}
	Enter_Kernel();
}

int Task_GetArg(void) {
	// The task's argument is stored at the very top of their stack.
	// The argument is 16-bit (two bytes long).
	return current_task->stack[WORKSPACE - 1 - 2];
}

CHAN Chan_Init() {
	return NULL;
}

void Send(CHAN ch, int v) {

}

int Recv(CHAN ch) {
	return 0;
}

void Write(CHAN ch, int v) {

}

unsigned int Now() {
	return 0;
}

/* The stack space allocated for this main function is the "kernel stack."
 * All other "stacks" in memory are inside the "task_t" structs declared above.
 * CurrentSp will (almost) always point to one of these stacks, and KernelSp
 * will point to this main function's stack. */
int main(void) {
	// Set to CTC mode, and set the prescaler to 64.
	TCCR3A = 0;
	TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);

	// When the prescaler is set to 64, 250 clock ticks is one ms.
	OCR3A = 250*MSECPERTICK;

	// Start the counter from 0. (entirely optional)
	TCNT3 = 0;
	// Clear the interrupt flag in case it was still set. (also optional)
	TIFR3 = _BV(OCF3A);

	// Enable interrupt A for timer 3.
	TIMSK3 = _BV(OCIE3A);
	// Every time the "TIMER3 COMPA" interrupt fires, that's one OS "tick".

	// Schedule the application's main task.
	PID a_pid = Task_Create_System(a_main, NULL);
	enqueue_system_task(&tasks[a_pid-1]);

	// Here we go...
	while(1) {
		// Pick a new task.
		task_t *next_task = NULL;
		if (sys_q_size > 0) {
			next_task = system_tasks[sys_q_head];
			sys_q_size -= 1;
			sys_q_head = (sys_q_head + 1) % MAXTHREAD;
		} else if (periodic_task_ready) {
			next_task = &tasks[periodic_task_ready-1];
			periodic_task_ready = NULL;
		} else if (rr_q_size > 0) {
			next_task = rr_tasks[rr_q_head];
			rr_q_size -= 1;
			rr_q_head = (rr_q_head + 1) % MAXTHREAD;
		}
		if (next_task == NULL) continue; // Wait for a task to be scheduled.
		// TODO: Maybe put the AVR into sleep mode if there's nothing to do?

		CurrentSp = next_task->sp;
		current_task = next_task;

		// Exit_Kernel() enables interrupts as it returns.
		Exit_Kernel();
		// When a thread re-enters the kernel, execution resumes from here.
		// Disable interrupts while we're in the kernel.
		cli();
	}

	// Should never reach here.
	return ~0;
}

ISR(TIMER3_COMPA_vect)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// Scheduler code goes here. Remember:
		// System tasks alternate with system tasks,
		// RR tasks alternate with RR tasks.
		// If two periodic tasks conflict, that's a timing violation!
	}
}

