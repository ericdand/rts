#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "os.h"

#define LED_BUILTIN _BV(PB7);

// Task priorities which determine which queue a task is scheduled in.
#define SYSTEM 3
#define PERIODIC 2
#define ROUND_ROBIN 1

/******
 * OS internal data structures and types
 ******/

/* Not declared with "static" because they need to be visible to cswitch.S. */
volatile uint8_t *CurrentSp;
volatile uint8_t *KernelSp;

typedef struct {
	PID pid; // A number for this task; also its (index - 1) in "tasks" array.
	uint8_t stack[WORKSPACE]; // Stack space for this task.
	uint8_t priority; // Priority level (Sys, Per, RR).
	volatile uint8_t *sp; // Stack pointer.
	int arg; // The argument passed to Task_Create.
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
// The PID of a periodic task to be run ASAP, if any is ready.
static PID periodic_task_ready = NULL;

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

static void task_terminate(void) {
	// TODO: Clean up the calling task, freeing its resources.
	// TODO: Do we then return? Or do we call Enter_Kernel?
}

/*******
 * External functions
 *******/

/* These two are defined in cswitch.S. They handle context switching. */
extern void Exit_Kernel();
extern void Enter_Kernel();

/*******
 * Interface implementation
 * (i.e. the good stuff)
 *******/

void OS_Abort(unsigned int error) {

}

PID Task_Create_System(void (*f)(void), int arg) {
	unsigned int i;
	for(i = 0; i < MAXTHREAD; i++) {
		if ((task_mask & (1 << i)) == 0) {
			task_mask |= (1 << i);
			tasks[i].pid = i + 1;
			tasks[i].priority = SYSTEM;
			tasks[i].sp = tasks[i].stack;

			// Set up the stack.
			// Code lifted from the example project by Scott and Justin, modified slightly.
			uint8_t* stack_top = tasks[i].stack + (WORKSPACE-1) - (38);

			/* stack_top[0] is the byte above the stack.
			 * stack_top[1] is r0. */
			stack_top[2] = (uint8_t) 0; /* r1 is the "zero" register. */
			/* stack_top[32] is r31. */
			stack_top[33] = (uint8_t) _BV(SREG_I); /* set SREG_I bit in stored SREG. */
			stack_top[34] = (uint8_t) 0; /* EIND is 0 so we don't suddenly end up in 
											extended memory. */

			/* We are placing the address (16-bit) of the functions
			 * onto the stack in reverse byte order (least significant first, followed
			 * by most significant).  This is because the "return" assembly instructions
			 * (ret and reti) pop addresses off in BIG ENDIAN (most sig. first, least sig.
			 * second), even though the AT90 is LITTLE ENDIAN machine.
			 */
			stack_top[35] = (uint8_t)((uint16_t)(kernel_request_create_args.f) >> 8);
			stack_top[36] = (uint8_t)(uint16_t)(kernel_request_create_args.f);
			stack_top[37] = (uint8_t)((uint16_t)task_terminate >> 8);
			stack_top[38] = (uint8_t)(uint16_t)task_terminate;

			/*
			 * Make stack pointer point to cell above stack (the top).
			 * Make room for 32 registers, SREG, EIND, and two return addresses.
			 */
			tasks[i].sp = stack_top;

			tasks[i].arg = arg;
			break;
		}
	}
	if (i != MAXTHREAD) return tasks[i].pid;
	return NULL;
}

PID Task_Create_RR(void (*f)(void), int arg) {
	return NULL;
}

PID Task_Create_Period(void (*f)(void), int arg, TICK period, TICK wcet, TICK offset) {
	return NULL;
}

void Task_Next(void) {
	// TODO: Put the current task back in its proper queue.
	Enter_Kernel();
}

int Task_GetArg(void) {
	return 0;
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

	// Set PB7 (the LED's port and pin) to output, and then zero the port.
	DDRB = LED_BUILTIN;
	PORTB = 0x00;

	// Schedule the application's main task.
	Task_Create_System(a_main, NULL);

	// Here we go...
	while(1) {
		// Pick a new task.
		task_t *next_task;
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

