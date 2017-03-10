#include <avr/io.h>
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

	while(1) {
		PORTB ^= LED_BUILTIN;
		Task_Next();
	}
	return;
}
