#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "os.h"

#define PRIORITY_HIGHEST 3
#define PRIORITY_MEDIUM 2
#define PRIORITY_LOWEST 1

#define LED_BUILTIN (1 << 7)

typedef struct {
	PID pid;
	unsigned int priority;
} task_t;

typedef struct {
	CHAN number;
	int data;
} chan_t;

void OS_Abort(unsigned int error) {

}

PID Task_Create_System(void (*f)(void), int arg) {
	return NULL;
}

PID Task_Create_RR(void (*f)(void), int arg) {
	return NULL;
}

PID Task_Create_Period(void (*f)(void), int arg, TICK period, TICK wcet, TICK offset) {
	return NULL;
}

void Task_Next(void) {

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

int main(void) {
	// Set to CTC mode, and set the prescaler to 64.
	TCCR3A = 0;
	TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);

	// When the prescaler is set to 64, 250 ticks is one ms.
	OCR3A = 250*MSECPERTICK;

	// Start the counter from 0. (entirely optional)
	TCNT3 = 0;
	// Clear the interrupt flag in case it was still set. (also optional)
	TIFR3 = _BV(OCF3A);

	// Enable interrupt A for timer 3.
	TIMSK3 = _BV(OCIE3A);
	// Every time the "TIMER3 COMPA" interrupt fires, that's one OS "tick".

	// Set PB7 (the LED's port and pin) to output, and then zero the port.
	DDRB = 0xFF | LED_BUILTIN;
	PORTB = 0x00;

	// Blink the LED forever.
	while(1) {
		_delay_ms(1000);
		PORTB ^= LED_BUILTIN;
	}

	// Should never reach here.
	return 0;
}

ISR(TIMER3_COMPA_vect)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// Anything done once per tick should be done here.
	}
}

