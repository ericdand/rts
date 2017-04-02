#include <stdint.h>

#include <avr/io.h>

#include "os.h"
#include "bluetooth_proto.h"

// The size of the joystick deadzone; current set to 1/8th of total range.
#define JS_DEADZONE (0xFF / 8)

// ADC pins thumbsticks ought to be plugged into.
#define TURRET_THUMB_X 0
#define TURRET_THUMB_Y 1
#define ROOMBA_THUMB_X 2
#define ROOMBA_THUMB_Y 3

// Used to track whether we got an ACK on our last stop command.
// Also used to tell whether we're allowed to send another stop command;
// if it's FALSE, then a stop command may be sent. If TRUE, then suppress
// any further stop commands. Set to FALSE when a movement command is sent.
BOOL stop_acknowledged;

#define TX_Q_SIZE 32
uint8_t bt_tx_q[TX_Q_SIZE];
uint8_t bt_tx_head, bt_tx_n;

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

static void enqueue_simple_command(uint8_t cmd) {
	bt_tx_q[(bt_tx_head + bt_tx_n++) % TX_Q_SIZE] = cmd;
	// ACK expectation and handling is left to functions which call
	// enqueue_simple_command, and the UART Rx interrupt service routine.
}

static void enqueue_data_command(uint8_t cmd, uint8_t data) {
	bt_tx_q[(bt_tx_head + bt_tx_n++) % TX_Q_SIZE] = cmd;
	bt_tx_q[(bt_tx_head + bt_tx_n++) % TX_Q_SIZE] = data;
}

void joystick_interpreter(void) {
	int pan, tilt, rot, vel;
	BOOL moving;
	for(;;) {
		// Sample thumbsticks.
		pan = sample_adc(TURRET_THUMB_X);
		tilt = sample_adc(TURRET_THUMB_Y);
		rot = sample_adc(ROOMBA_THUMB_X);
		vel = sample_adc(ROOMBA_THUMB_Y);
		// Interpret thumbsticks to movement.
		pan = pan >> 2; // Shift right by two to go from 0-1023 to 0-255.
		tilt = tilt >> 2;

		// Send commands over bluetooth.
		// Turret commands are stateless; just send them.
		if (pan > 127 + JS_DEADZONE || pan < 127 - JS_DEADZONE)
			enqueue_data_command(T_PAN, (uint8_t)pan);
		if (tilt > 127 + JS_DEADZONE || tilt < 127 - JS_DEADZONE)
			enqueue_data_command(T_TILT, (uint8_t)tilt);
		
		// Roomba commands use a "stop" command, so we have to track state.
		moving = FALSE;
		if (vel > 127 + JS_DEADZONE || vel < 127 - JS_DEADZONE) {
			moving = TRUE;
			enqueue_data_command(R_VEL, (uint8_t)vel);
		}
		if (rot > 127 + JS_DEADZONE || rot < 127 - JS_DEADZONE) {
			moving = TRUE;
			enqueue_data_command(R_ROT, (uint8_t)rot);
		}
		
		if (moving) {
			// Roomba is moving; now allowed to send another stop command.
			stop_acknowledged = FALSE;
		} else if (!stop_acknowledged) {
			// Joystick not displaced, and we've not yet sent a "stop" 
			// command which was ACK'd. Tell the Roomba to stop.
			enqueue_simple_command(R_STOP);
		}
		// Pass until next time.
		Task_Next();
		// Code resumes here.
	}
}

/*
 * a_main is the entry point of the application; 
 * it is the first system task run by the OS.
 */
void a_main(void) {
	Task_Create_RR(joystick_interpreter, 0);
}

