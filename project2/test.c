#include <avr/io.h>
#include <util/delay.h>

/* 
 * util/setbaud.h is a set of macros that compute the proper values of the
 * UBRRn registers to get the desired baudrate. Just define BAUD and then
 * include the header. The header defines 4 macros: 
 *     UBRR_VALUE, UBRRH_VALUE, UBRRL_VALUE, and USE_2X.
 * USE_2X is set to 1 if the U2X bit of the UCSRA register should be enabled.
 */
#define BAUD 9600
#include <util/setbaud.h>

#include "os.h"

#define LED_BUILTIN _BV(PB7)

// This buffer is scratch space for all tests.
// buf_n is set to 0 between tests, but buf is not zeroed.
static volatile int buf[128];
static volatile int buf_n;

/* task1 and task2 are classic "ping-pong" tasks; they are meant to alternate
 * writing 1 and 2 to buf. The can be used to test all 3 kinds of task. */
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
	return TRUE;
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

/* This is the general "test runner" function. It takes 2 arguments:
 * * A function pointer to the test to be run.
 * * A name for the test, to be reported over serial.
 * The name must be a null-terminated string. */
static void run_test_and_report(BOOL (*test)(void), char *name) {
	buf_n = 0;
	if (test()) {
		// Report success.
	} else {
		// Report failure.
	}
}

void a_main(void) {
	usart_init();
	usart_puts("Hello, world!");

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
