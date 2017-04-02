#include <stdint.h>

#include <avr/io.h>

#include "os.h"
#include "bluetooth_proto.h"

// ADC pins thumbsticks ought to be plugged into.
#define TURRET_THUMB_X 0
#define TURRET_THUMB_Y 1
#define ROOMBA_THUMB_X 2
#define ROOMBA_THUMB_Y 3

int sample_adc(uint8_t pin) {
	// This code is adapted from the Arduino libraries.
	uint8_t low, high;

	// The MUX5 bit of ADCSRB selects between analog pins 0-7 (MUX5 low)
	// and analog pins 8-15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);

	// Use VCC as an analog voltage reference, and choose the pin.
	ADMUX = _BV(REFS0) | (pin & 0x07);

	// Datasheet says: "A normal conversion takes 13 ADC clock cycles. The
	// first conversion after the ADC is switched on (ADEN in ADCSRA is set)
	// takes 25 ADC clock cycles in order to initialize the analog circuitry."

	// Use a prescaler of 8; this will make those 13 clock cycles take
	// eight times longer, but will give a cleaner ADC reading.
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);

	ADCSRA |= _BV(ADSC);        // Start conversion.
	while (ADCSRA & _BV(ADSC)); // Wait for conversion to finish.
	low = ADCL; high = ADCH;    // Remember: read low byte first!

	// Uncomment the following to turn the ADC back off again to save power.
	// This will consume an additional 12 clock cycles per analog read, as the
	// hardware must be re-initialized each time.
	//ADCSRA = 0;

	return (high << 8) | low;
}

void joystick_interpreter(void) {
	int pan, tilt, rot, vel;
	for(;;) {
		// Sample thumbsticks.
		pan = sample_adc(TURRET_THUMB_X);
		tilt = sample_adc(TURRET_THUMB_Y);
		rot = sample_adc(ROOMBA_THUMB_X);
		vel = sample_adc(ROOMBA_THUMB_Y);
		// Interpret thumbsticks to movement.

		// Send commands over bluetooth via UART helper.

		Task_Next();
	}
}

/*
 * a_main is the entry point of the application; 
 * it is the first system task run by the OS.
 */
void a_main(void) {
	Task_Create_RR(joystick_interpreter, 0);
}

