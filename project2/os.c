#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "os.h"

// Task priorities which determine which queue a task is scheduled in.
#define SYSTEM 3
#define PERIODIC 2
#define ROUND_ROBIN 1

/********
 * Debugging stuff
 ********/

#ifdef NDEBUG
#define debug(M)
#else 
#define debug(M) usart_debug(M, __FILE__, __LINE__, __func__)
#define BAUD 19200
#include <util/setbaud.h>
#include <stdio.h>

/* usart_* functions borrowed from Francesco Balducci, at https://balau82.wordpr
 * ess.com/2014/12/23/using-a-rain-sensor-with-arduino-uno-in-c/ */
static void usart_init(void)
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif
    UCSR0B = _BV(TXEN0); /* Only TX */
}

static void usart_tx(char c) {
    while(!(UCSR0A & _BV(UDRE0)));
    UDR0 = c;
}

static void usart_puts(const char *s)
{
	while(*s != '\0')
	{
		usart_tx(*s++);
	}
}

/* buf may be used by tasks preparing strings to use with debug() */
static char buf[128];
static char debug_msg_buf[256];
void usart_debug(const char* M, const char* filename, const int linenumber, const char* funcname) {
	// To make this work, we need to "suspend" the ticker timer so it's
	// invisible to the RTOS. Otherwise, timing testing may not work.
	// NB: There may still be a tiny added cost from just calling this function.
	// But nowhere near a whole tick.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		uint16_t timer_val = TCNT3;
		snprintf(debug_msg_buf, 256, "DEBUG %s:%d:%s: %s\n", filename, linenumber, funcname, M);
		usart_puts(debug_msg_buf);
		// Clear the interrupt flag, since it's probably been set by now.
		TIFR3 = _BV(OCF3A);
		// Restore the old timer value.
		TCNT3 = timer_val;
	}
}

#endif

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
uint8_t* volatile CurrentSp;
uint8_t* volatile KernelSp;

typedef struct {
	PID pid; // A number for this task; also its (index - 1) in "tasks" array.
	uint8_t stack[WORKSPACE]; // Stack space. Includes the task's argument, arg.
	uint8_t priority; // Priority level (Sys, Per, RR).
	uint8_t* volatile sp; // Stack pointer.
	TICK period; // These 3 TICK properties are only used by periodic tasks.
	TICK offset;
	TICK wcet; // wcet = worst-case execution time.
	volatile TICK ticks_consumed;
} task_t;

// FIXME: More of this stuff might need the "volatile" keyword.
// Allocate space for each task...
static task_t tasks[MAXTHREAD];
// ...and create a wait queue for each priority level.
static volatile task_t* volatile current_task = NULL;
// Bitmask for used (1) tasks and free (0) tasks.
// If MAXTHREAD is changed to over 16, we will need a bigger mask.
static volatile uint16_t task_mask = 0;
static task_t *system_tasks[MAXTHREAD];
static task_t *rr_tasks[MAXTHREAD];
static volatile uint8_t sys_q_size = 0, sys_q_head = 0,
						rr_q_size = 0, rr_q_head = 0;
// We start the current tick here so that it rolls over to 0 to begin with.
static volatile TICK tick = 0xFFFF;
/* The PID of a periodic task to be run ASAP, if any is ready.
 * If the scheduler goes to set this field and it is already set,
 * then that should be considered a timing violation. */
static task_t* volatile periodic_task_ready = NULL;

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
static volatile uint16_t channel_mask = 0;

/*******
 * Internal functions
 *
 * All internal functions assume that interrupts are already cleared on entry,
 * (save those that begin with cli()).
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
	tasks[idx].ticks_consumed = 0;

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
	 * is the lowest, and the stack grows down into even-lower addresses. */
	uint8_t* stack_bottom = tasks[idx].stack + (WORKSPACE-1) - (43);

	/* stack_bottom[41] and [42] hold the argument to f. */
	stack_bottom[42] = (uint8_t)arg;
	stack_bottom[41] = (uint8_t)(arg >> 8);
	/* A 3-byte return address follows; we set it to task_terminate to shut down
	 * the user function if/when it returns. */
	stack_bottom[40] = (uint8_t)(uint16_t)task_terminate;
	stack_bottom[39] = (uint8_t)((uint16_t)task_terminate >> 8);
	/* stack_bottom[38] is the first (highest-order) bit of our 3-byte return
	 * address, whose value is written to EIND when returning. */
	stack_bottom[38] = (uint8_t) 0;
	/* Above that, we place a pointer to f so that the kernel "returns" into f
	 * after pointing SP at this task's stack pointer. */
	stack_bottom[37] = (uint8_t)(uint16_t)f;
	stack_bottom[36] = (uint8_t)((uint16_t)f >> 8);
	stack_bottom[35] = (uint8_t) 0;
	/* Last come all the saved registers. */
	/* stack_bottom[34] is r0. */
	/*stack_bottom[33] is r1, the zero register. */
	stack_bottom[33] = (uint8_t) 0;
	/* stack_bottom[32:3] are registers r2-31. */
	/* stack_bottom[2] is SREG. We clear it and set the I flag.*/
	stack_bottom[2] = (uint8_t) _BV(SREG_I);
	/* stack_bottom[1] is EIND, always set to 0 to start. */
	stack_bottom[1] = (uint8_t) 0;
	/* stack_bottom[0] is the first free byte in the stack. */
	
	tasks[idx].sp = (uint8_t* volatile) stack_bottom;
}

/* Assumption with both of these enqueue functions: interrupts are disabled.
 * If interrupts are not already disabled, then these tasks are dangerous. */
static void enqueue_system_task(task_t *t) {
	// debug("enq sys task");
	if (sys_q_size < MAXTHREAD) {
		system_tasks[(sys_q_head + sys_q_size++) % MAXTHREAD] = t;
	} else {
		debug("ERROR: no more space in sys queue");
	}
}

static void enqueue_rr_task(task_t *t) {
	// debug("enq rr task");
	if (rr_q_size < MAXTHREAD) {
		rr_tasks[(rr_q_head + rr_q_size++) % MAXTHREAD] = t;
	} else {
		debug("ERROR: no more space in rr queue");
	}
}

static BOOL periodic_tasks_are_scheduled() {
	for(int i = 0; i < MAXTHREAD; i++) {
		if ((task_mask & (1 << i)) != 0 &&
				tasks[i].priority == PERIODIC) {
			return TRUE;
		}
	}
	return FALSE;
}

/* From the AVR Libc Reference Manual FAQ:
 * "The canonical way to perform a software reset of non-XMega AVR's is to use
 * the watchdog timer. Enable the watchdog timer to the shortest timeout
 * setting, then go into an infinite, do-nothing loop. The watchdog will then
 * reset the processor. ... The reason why using the watchdog timer is
 * preferable over jumping to the reset vector, is that when the watchdog
 * resets the AVR, the registers will be reset to their known, default
 * settings. Whereas jumping to the reset vector will leave the registers in
 * their previous state, which is generally not a good idea." 
 *
 * Furthermore: "CAUTION! On newer AVRs, once the watchdog is enabled, then it
 * stays enabled, even after a reset! For these newer AVRs a function needs to
 * be added to the .init3 section (i.e. during the startup code, before main())
 * to disable the watchdog early enough so it does not continually reset the
 * AVR." 
 * 
 * So below we define a function to be run in the .init3 section which disables
 * the watchdog timer again ASAP as the Arduino starts up. */ 
void disable_watchdog (void) __attribute__ ((naked)) \
		 __attribute__ ((section (".init3")));
void disable_watchdog (void)
{
	MCUSR = 0;
    wdt_disable();
}

/*******
 * Interface implementation
 * (i.e. the good stuff)
 *******/

void OS_Abort(unsigned int error) {
	// Do a soft reset: enable the watchdog timer, set to its shortest
	// duration, then spin forever until it resets us.
	wdt_enable(WDTO_15MS);
    for(;;) {}
}

PID Task_Create_System(void (*f)(void), int arg) {
	unsigned int i;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for(i = 0; i < MAXTHREAD; i++) {
			if ((task_mask & (1 << i)) == 0) {
				// Mark this task as used.
				task_mask |= (1 << i);

				tasks[i].priority = SYSTEM;
				task_create(i, f, arg);
				break;
			}
		}
	}
	// debug("created sys task");
	if (i != MAXTHREAD) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			enqueue_system_task(&tasks[i]);
		}
		return tasks[i].pid;
	}
	return 0;
}

PID Task_Create_RR(void (*f)(void), int arg) {
	unsigned int i;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for(i = 0; i < MAXTHREAD; i++) {
			if ((task_mask & (1 << i)) == 0) {
				// Mark task as used.
				task_mask |= (1 << i);

				tasks[i].priority = ROUND_ROBIN;
				task_create(i, f, arg);
				break;
			}
		}
	}
	// debug("created rr task");
	if (i != MAXTHREAD) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			enqueue_rr_task(&tasks[i]);
		}
		return tasks[i].pid;
	}
	return 0;
}

PID Task_Create_Period(void (*f)(void), int arg, TICK period, TICK wcet, TICK offset) {
	unsigned int i;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for(i = 0; i < MAXTHREAD; i++) {
			if ((task_mask & (1 << i)) == 0) {
				// Mark task as used.
				task_mask |= (1 << i);

				tasks[i].priority = PERIODIC;
				tasks[i].period = period;
				tasks[i].wcet = wcet;
				tasks[i].offset = offset;
				task_create(i, f, arg);
				break;
			}
		}
	}
	// debug("created periodic task");
	if (i != MAXTHREAD) {
		// Periodic tasks are "automatically" scheduled; see the ISR.
		return tasks[i].pid;
	}
	return 0;
}

/* Task_Next is called by any task which is not the kernel. When called, it
 * will cause the calling task to be put back in its proper queue, and then
 * the kernel is invoked to choose a new task. */
void Task_Next(void) {
	cli();
	// debug("Task_Next");
	// Put the current task back in its proper queue.
	if(current_task->priority == SYSTEM) {
		enqueue_system_task((task_t*)current_task);
	} else if (current_task->priority == ROUND_ROBIN) {
		enqueue_rr_task((task_t*)current_task);
	} else {
		// Periodic tasks should not call Task_Next.
		debug("ERROR: periodic task called Task_Next");
	}
	Enter_Kernel();
}

int Task_GetArg(void) {
	// The task's argument is stored at the top of its stack. (WORKSPACE-1)
	// The argument is 16-bit (two bytes long). Its first byte is therefore
	// at WORKSPACE-2.
	return current_task->stack[WORKSPACE - 2];
}

CHAN Chan_Init() {
	return 0;
}

void Send(CHAN ch, int v) {

}

int Recv(CHAN ch) {
	return 0;
}

void Write(CHAN ch, int v) {

}

unsigned int Now() {
	return (unsigned int)tick;
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
	Task_Create_System(a_main, 0);

#ifndef NDEBUG
	usart_init();
	// debug("usart initialized");
#endif

	// Here we go...
	while(1) {
		// debug("entered kernel");
		// Pick a new task.
		task_t *next_task = NULL;
		if (sys_q_size > 0) {
			next_task = system_tasks[sys_q_head];
			sys_q_size -= 1;
			sys_q_head = (sys_q_head + 1) % MAXTHREAD;
			// debug("switching to sys task");
		} else if (periodic_task_ready) {
			next_task = periodic_task_ready;
			periodic_task_ready = NULL;
			// debug("switching to periodic task");
		} else if (rr_q_size > 0) {
			next_task = rr_tasks[rr_q_head];
			rr_q_size -= 1;
			rr_q_head = (rr_q_head + 1) % MAXTHREAD;
			// debug("switching to rr task");
		}
		if (next_task == NULL) {
			// If there are no more tasks to be run, then maybe a periodic
			// task will be scheduled. Put the processor to sleep.
			if (periodic_tasks_are_scheduled()) {
				debug("awaiting periodic task; enabling sleep");
				sei();
				sleep_mode();
				cli();
			}
			// Otherwise PANIC!
			debug("ERROR: no task left to run");
			sleep_mode();
		}

		CurrentSp = next_task->sp;
		current_task = next_task;

#ifndef NDEBUG
		/* snprintf(buf, 128, "next_task PID: %d, CurrentSp: %p, addr at stack: %p", 
				next_task->pid, CurrentSp, *(void **)(CurrentSp+2));
		debug(buf); */
#endif

		// debug("exiting kernel");
		// Exit_Kernel() enables interrupts as it returns.
		Exit_Kernel();
		// When a thread re-enters the kernel, execution resumes from here.
		// Disable interrupts while we're in the kernel.
		cli();
		// debug("re-entered kernel");
		if (current_task != NULL) {
			current_task->ticks_consumed = 0;
			current_task->sp = CurrentSp;
		}
	}

	// Should never reach here.
	return ~0;
}

ISR(TIMER3_COMPA_vect)
{
	unsigned int i;
	task_t* t; // t holds current_task.

	// Note that here we use RESTORESTATE, but if we enter the kernel then
	// interrupts will remain disabled (which is the desired behaviour).
	// NOTE: Don't return from this block early! I'm not sure that the macro
	// will handle it properly.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		tick += 1;

		// current_task is volatile; save work while interrupts
		// are disabled by stashing it in a local variable.
		t = (task_t*) current_task;
		if (t != NULL)
			t->ticks_consumed += 1;

		// Check for periodic tasks waiting.
		task_t *pt;
		for(i = 0; i < MAXTHREAD; i++) {
			pt = &tasks[i];
			if ((task_mask & (1 << i)) != 0 &&				// task exists
					pt->priority == PERIODIC &&				// task is periodic
					(pt->period + pt->offset) % tick == 0) {// task runs this tick
				if (periodic_task_ready != NULL) {
					// There was already a periodic task waiting!
					debug("time viol'n: multiple periodic tasks scheduled.");
				} else if (t != NULL && 
						t->priority == PERIODIC) {
					// The current task is periodic and isn't finished yet!
					debug("time viol'n: periodic task preempting another");
				} else {
					periodic_task_ready = pt;
				}
			}
		}

		// Check for pre-emption by waiting system tasks.
		if (t != NULL && t->priority != SYSTEM && sys_q_size > 0) {
			debug("non-sys task preempted by sys task");
			// Reschedule the pre-empted task.
			if (t->priority == PERIODIC) {
				periodic_task_ready = t;
			} else { // priority == RR
				rr_tasks[(rr_q_head + rr_q_size++) % MAXTHREAD] = t;
			}
			Enter_Kernel();
		}

		// Has the current task has been running since the last tick?
		// If so, we reschedule it and switch to another worthy task.
		if (t != NULL && t->ticks_consumed > 1) {
			if (t->priority == SYSTEM) {
				// Reschedule this task.
				system_tasks[(sys_q_head + sys_q_size++) % MAXTHREAD] = t;
				debug("sys task has run for a whole tick. switching");
				Enter_Kernel();
			} else if (t->priority == PERIODIC) {
				if (t->ticks_consumed > t->wcet) {
					debug("timing violation: task exceeded wcet");
				}
				// Periodic task still within its worst-case execution time.
				// Let it run on.
			} else { // priority == ROUND_ROBIN
				// Reschedule this RR task.
				rr_tasks[(rr_q_head + rr_q_size++) % MAXTHREAD] = t;
				debug("rr task has run for a whole tick. switching");
				Enter_Kernel();
			}
		}
	}
	/* Datasheet section 7.8.1:
	 * "A return from an interrupt handling routine takes five clock cycles.
	 * During these five clock cycles, the Program Counter (three bytes) is
	 * popped back from the Stack, the Stack Pointer is incremented by three,
	 * and the I-bit in SREG is set." */
}

