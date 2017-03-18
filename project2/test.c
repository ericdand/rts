#include <stdio.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

/* 
 * util/setbaud.h is a set of macros that compute the proper values of the
 * UBRRn registers to get the desired baudrate. Just define BAUD and then
 * include the header. The header defines 4 macros: 
 *     UBRR_VALUE, UBRRH_VALUE, UBRRL_VALUE, and USE_2X.
 * USE_2X is set to 1 if the U2X bit of the UCSRA register should be enabled.
 */
#define BAUD 19200
#include <util/setbaud.h>

#include "os.h"

#define LED_BUILTIN _BV(PB7)
// PIN_12 is Arduino pin 12; attach a LA to it to help with debugging.
#define PIN_12 _BV(PB6)

/**********
 * ON WRITING TESTS
 *
 * All tests write integers to the array "buf", then verify their output and
 * report success. buf is not cleared between tests, but the index buf_n is
 * zeroed before each test is run. Tests are responsible for ensuring that any
 * tasks they start have terminated before they return.
 **********/

// This buffer is scratch space for all tests.
// buf_n is set to 0 between tests, but buf is not zeroed.
static volatile int buf[128];
static volatile int buf_n;

/* Stack space is only 256B; s exists here as a general scratch register
 * for writing formatted strings in preparation for output. */
static char s[128];

static void usart_tx(char c) {
    while(!(UCSR0A & _BV(UDRE0)));
    UDR0 = c;
}

static void usart_puts(const char *s)
{
	// To make this work, we need to "suspend" the ticker timer so it's
	// invisible to the RTOS. Otherwise, timing testing may not work.
	// NB: There may still be a tiny added cost from just calling this function.
	// But nowhere near a whole tick.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		uint16_t timer_val = TCNT3;
		while(*s != '\0')
		{
			usart_tx(*s++);
		}
		// Clear the interrupt flag, since it's probably been set by now.
		TIFR3 = _BV(OCF3A);
		// Restore the old timer value.
		TCNT3 = timer_val;
	}
}

// wait for n ticks, using _delay_ms.
static void wait(TICK n) {
	TICK start = Now();
	while(Now() - start < n) {
		_delay_ms(1);
	}
}

/* task1 and task2 are classic "ping-pong" tasks; they are meant to alternate
 * writing 1 and 2 to buf. The can be used to test all 3 kinds of task. 
 * They should both run in well under one tick. They will loop 5 times, exiting
 * at the end of the 5th loop. */
static void task1(void) {
	int i;
	for(i = 0; i < 4; i++) {
		buf[buf_n++] = 1;
		Task_Next();
	}
	buf[buf_n++] = 1;
}

static void task2(void) {
	int i;
	for(i = 0; i < 4; i++) {
		buf[buf_n++] = 2;
		Task_Next();
	}
	buf[buf_n++] = 2;
}

static BOOL test_switching_system_tasks(void) {
	BOOL success = TRUE;
	// This task is a round-robin task.
	// Each of these tasks will take over immediately when scheduled.
	Task_Create_System(task1, 0);
	Task_Create_System(task2, 0);
	// When execution reaches here, the system tasks have terminated.
	for(int i = 0; i < 10; i++) {
		// Make sure the buffer holds the right values.
		if(buf[i] != i/5+1) success = FALSE;
	}
	return success;
}

static BOOL test_switching_periodic_tasks(void) {
	// No two periodic tasks can be scheduled for the same time. So we set
	// these to alternate ticks. They should finish with nearly 3ms to spare.
	Task_Create_Period(task1, 0, 2, 0, 1);
	Task_Create_Period(task2, 0, 2, 0, 2);
	// Wait for them to finish.
	wait(12);
	// Verify output.
	for(int i = 0; i < 10; i += 2) {
		if (buf[i] != 1) return FALSE;
		if (buf[i+1] != 2) return FALSE;
	}
	return TRUE;
}

static BOOL test_switching_rr_tasks(void) {
	BOOL success = TRUE;
	Task_Create_RR(task1, 0);
	Task_Create_RR(task2, 0);
	// This task is an RR task too, so it can check output as the test goes.
	for(int i = 0; i < 10; i += 2) {
		usart_puts("DEBUG: RR LOOP\n");
		Task_Next();
		if (buf[i] != 1) success = FALSE;
		if (buf[i+1] != 2) success = FALSE;
	}
	return success;
}

static void delay_40ms(void) {
	_delay_ms(40);
}

static void fib(void) {
	if (buf_n == 0) buf[0] = 1;
	else if (buf_n == 1) buf[1] = 1;
	else buf[buf_n] = buf[buf_n-1] + buf[buf_n-2];
	buf_n += 1;
}

static void start_important_work(void) {
	for(int i = 0; i < 10; i++) {
		Task_Create_System(fib, 0);
		Task_Next();
	}
	Task_Create_System(fib, 0);
}

static BOOL test_periodic_task_starting_system_tasks(void) {
	// Start a long-running low priority task.
	Task_Create_RR(delay_40ms, 0);
	// Start a periodic task which creates a new system task.
	Task_Create_Period(start_important_work, 0, 1, 0, 0);
	Task_Next();
	// After 12 ticks it should all be done.
	wait(12);
	BOOL success = TRUE;
	for(int i = 2; i < 10; i++) {
		if (buf[i] != buf[i-1] + buf[i-2]) success = FALSE;
	}
	return success;
}

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

/* This is the general "test runner" function. It takes 2 arguments:
 * * A function pointer to the test to be run.
 * * A name for the test, to be reported over serial.
 * The name must be a null-terminated string. */
static void run_test_and_report(BOOL (*test)(void), char *name) {
	buf_n = 0;
	sprintf(s, "Running test: %s\n", name);
	usart_puts(s);
	if (test()) {
		// Report success.
		usart_puts("\nSUCCESS\n\n");
	} else {
		// Report failure.
		usart_puts("\nFAILURE\n\n");
	}
}

void test_runner(void) {
	run_test_and_report(test_switching_system_tasks,
			"switching system tasks");
	run_test_and_report(test_switching_periodic_tasks, 
			"switching periodic tasks");
	run_test_and_report(test_switching_rr_tasks,
			"switching round-robin tasks");
	run_test_and_report(test_periodic_task_starting_system_tasks,
			"periodic task starting system tasks");
}

void a_main(void) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		usart_init();
		_delay_ms(500); // Wait for the computer to attach a serial monitor.
		usart_puts("\nTesting...\n\n");
	}

	// Create the task runner at a lower priority level so it doesn't get in
	// the way of running tests.
	Task_Create_RR(test_runner, 0);

	// When this task returns, the OS will switch to the only remaining one:
	// test_runner.
	return;

	// LED-blinking code.
	// DDRB = LED_BUILTIN;
	// PORTB = (uint8_t) 0;
	// while(1) {
	// 	PORTB ^= LED_BUILTIN;
	// 	_delay_ms(100);
	// 	Task_Next();
	// }
}
