#include <avr/io.h>
#include <util/delay.h>
#include "os.h"

#define LED_BUILTIN _BV(PB7)

static void blink_LED(void) {
	// Blink the LED forever.
	while(1) {
		PORTB ^= LED_BUILTIN;
		Task_Next();
	}
}

void a_main(void) {
	// Task_Create_Period(blink_LED, NULL, 100, 1, 0);
	DDRB = LED_BUILTIN;
	PORTB = (uint8_t) 0;

	while(1) {
		PORTB ^= LED_BUILTIN;
		_delay_ms(100);
		Task_Next();
	}
	return;
}
