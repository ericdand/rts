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

/* task1 and task2 are classic "ping-pong" tasks; they are meant to alternate
 * writing 0 and 1 to buf. The can be used to test all 3 kinds of task. 
 * They *should* both run in under one tick. */
static void task1(void) {
	int i;
	for(i = 0; i < 5; i++) {
		buf[buf_n++] = 1;
		Task_Next();
	}
}

static void task2(void) {
	int i;
	for(i = 0; i < 5; i++) {
		buf[buf_n++] = 2;
		Task_Next();
	}
}

static BOOL test_switching_system_tasks(void) {
	BOOL success = TRUE;
	Task_Create_System(task1, 0);
	Task_Create_System(task2, 0);
	for(int i = 0; i < 10; i += 2) {
		// This task is also a system task.
		// Use Task_Next to allow task1 and task2 to run.
		Task_Next();
		// Make sure the buffer holds the right values.
		if(buf[i] != 1) success = FALSE;
		if(buf[i+1] != 2) success = FALSE;
	}
	// Call Task_Next one more time to let the tasks terminate.
	Task_Next();
	return success;
}

static BOOL test_switching_periodic_tasks(void) {

	return TRUE;
}

static BOOL test_switching_rr_tasks(void) {
	return TRUE;
}

static BOOL test_switching_mixed_priority_tasks(void) {
	return TRUE;
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
		sprintf(s, "SUCCESS\n");
		usart_puts(s);
	} else {
		// Report failure.
		sprintf(s, "FAILURE\n");
		usart_puts(s);
	}
}

void a_main(void) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		usart_init();
		_delay_ms(1000); // Wait for the computer to attach a serial monitor.
		usart_puts("Hello, world!\n");
	}

	run_test_and_report(test_switching_system_tasks,
			"switching system tasks");
	run_test_and_report(test_switching_periodic_tasks, 
			"switching periodic tasks");
	run_test_and_report(test_switching_rr_tasks,
			"switching round-robin tasks");
	run_test_and_report(test_switching_mixed_priority_tasks,
			"switching mixed-priority tasks");

	// LED-blinking code.
	// DDRB = LED_BUILTIN;
	// PORTB = (uint8_t) 0;
	// while(1) {
	// 	PORTB ^= LED_BUILTIN;
	// 	_delay_ms(100);
	// 	Task_Next();
	// }
	return;
}
