#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "os.h"
#include "bluetooth_usart.h"

// Connect the Roomba BRC to pin 53, and the laser to pin 52.
#define ROOMBA_BRC _BV(PB0)
#define LASER_PIN  _BV(PB1)

volatile uint8_t r_vel, r_rot, t_pan, t_tilt;

#define RX_Q_SIZE 32
uint8_t bt_rx_q[RX_Q_SIZE];
volatile uint8_t bt_rx_head, bt_rx_n;

// NB: This will blindly do as asked. Make sure there's something there first!
static uint8_t get_rx_q_head(void) {
	uint8_t data = bt_rx_q[bt_rx_head];
	bt_rx_head = (bt_rx_head + 1) % RX_Q_SIZE;
	bt_rx_n--;
	return data;
}

// Plug the pan servo into pin 11, tilt into pin 12.
static void servo_init(void) {
	// Set OC1A (PB5) and OC1B (PB6) to output mode.
	DDRB = _BV(PB5) | _BV(PB6);
	// Set to Fast PWM Mode (WGM13:0 = 14), 
	// set the OC registers to clear when triggered (COM1x1:0 = 2),
	// and set the prescaler to 64 (CS12:0 = 3).
	TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
	
	// ICR1 controls when the timer resets.
	ICR1 = 5000; // (16M / 50 Hz) / 64 = 5000.
	OCR1A = 375; // The OCR registers control when OC1A and OC1B are cleared.
	OCR1B = 375; // 375 should turn out to be 1500 microseconds.
}

// Connect the Roomba to USART2.
static void roomba_write(uint8_t b) {
	while(!(UCSR2A & _BV(UDRE2))); // Wait until ready.
	UDR2 = b;
}

// Some select Roomba commands.
#define ROOMBA_START 128
#define ROOMBA_SAFE  131
#define ROOMBA_DRIVE 137

#define BAUD 19200
#include <util/setbaud.h>
static void roomba_init(void) {
	// Set up USART2 for Roomba comms.
	UBRR2H = UBRRH_VALUE;
	UBRR2L = UBRRL_VALUE;
#if USE_2X
	UCSR2A |= _BV(U2X2);
#else
	UCSR2A &= ~_BV(U2X2);
#endif
	UCSR2B = _BV(TXEN2); // Tx only.

	DDRB |= ROOMBA_BRC;  // Enable it for output.
	PORTB |= ROOMBA_BRC; // Set it.
	//Set Roomba baud rate by toggling the BRC pin 3 times.
	_delay_ms(2500);
	PORTB &= ~ROOMBA_BRC;
	_delay_ms(300);
	PORTB |= ROOMBA_BRC;
	_delay_ms(300);
	PORTB &= ~ROOMBA_BRC;
	_delay_ms(300);
	PORTB |= ROOMBA_BRC;
	_delay_ms(300);
	PORTB &= ~ROOMBA_BRC;
	_delay_ms(300);
	PORTB |= ROOMBA_BRC;

	//Power on
	roomba_write(ROOMBA_START);
	_delay_ms(200);
	//Enter safe mode
	roomba_write(ROOMBA_SAFE);
}

/********
 * Tasks
 ********/
static void command_interpreter(void) {
	uint8_t cmd = 0,
			data = 0;
	for(;;) {
		if (bt_rx_n == 0) {
			// Nothing to do!
			Task_Next();
			continue;
		}

		cmd = get_rx_q_head();
		if (!(cmd & (1 << 7))) {
			// This is a data command.
			if (bt_rx_n == 0) {
				// data hasn't arrived yet. Wait for it.
				Task_Next();
				// Resume from here. Next thing in the queue by now *must* be
				// the data byte we were waiting for.
				if (bt_rx_n == 0) {
					// ERROR! No data byte ever arrived.
					// Fail gracefully: just start over again.
					// The data must have got lost in the ether.
					continue;
				}
			}
			data = get_rx_q_head();

			// Interpret data command.
			switch(cmd) {
				case R_VEL:
					r_vel = data;
					break;
				case R_ROT:
					r_rot = data;
					break;
				case T_PAN:
					t_pan = data;
					break;
				case T_TILT:
					t_tilt = data;
					break;
				default:
					// ERROR!
					break;
			}		
		} else {
			// This is a simple command.
			// Send an ACK right away, in this thread.
			usart_tx(cmd);
			// Act on it.
			if (cmd == L_ON) {
				PORTB |= LASER_PIN;
			} else if (cmd == L_OFF) {
				PORTB &= ~(LASER_PIN);
			} else if (cmd == R_STOP) {
				r_vel = 0;
				r_rot = 0;
			} else {
				// ERROR!
			}
		}
		// Don't call Task_Next here: there could be more input to read!
	}
}

static void turret_controller(void) {
	for(;;) {
		// Thanks to how the PWM generator clock skew works, we are seeking to
		// map pan and tilt inputs from [0, 255] to [250, 500].
		// The easiest way to do this is to just add 250, as long as we're ok
		// with exceeding 500 slightly. 
		// This is fine, since it shouldn't harm the servos.
		OCR1A = t_pan + 250;
		OCR1B = t_tilt + 250;
		Task_Next();
	}
}

static void roomba_controller(void) {
	int16_t radius, velocity;
	uint8_t	oldr = 0, oldv = 0;
	for(;;) { 
		while (r_rot == oldr && r_vel == oldv) 
			// Only send a command when something changes.
			Task_Next();

		// Map rotation from [0, 255] to [-2000, 2000], reversed.
		// (i.e. 255 -> 1, 0 -> -1, 130 -> 2000, 126 -> -2000, 128 -> 0).
		if (r_rot > 129) {
			radius = 2000 - (r_rot << 4);
			if (radius < 0) radius = 1;
		} else if (r_rot < 127){
			radius = -2000 + (r_rot << 4);
			if (radius > 0) radius = -1;
		} else { // r_rot ~= 128
			radius = 0x7FFF; // Set radius to 32767 to drive straight.
		}

		// Map velocity from [0, 255] to [-500, 500].
		if (r_vel > 129) {
			velocity = (r_vel - 128) * 4;
			if (velocity > 500) velocity = 500;
		} else if (r_vel < 127) {
			velocity = -r_vel * 4;
			if (velocity < -500) velocity = -500;
		} else { // r_vel ~= 128
			velocity = 0;
		}

		roomba_write(ROOMBA_DRIVE);
		roomba_write((uint8_t)(velocity >> 8));
		roomba_write((uint8_t)velocity);
		roomba_write((uint8_t)(radius >> 8));
		roomba_write((uint8_t)radius);
		oldr = r_rot;
		oldv = r_vel;
		Task_Next();
	}
}

void a_main(void) {
	usart_init();
	servo_init();
	roomba_init();
	DDRB |= LASER_PIN;

	Task_Create_Period(command_interpreter, 0, 10, 0, 0);
	Task_Create_Period(turret_controller, 0, 10, 0, 4);
	Task_Create_Period(roomba_controller, 0, 10, 1, 7);
}

ISR(USART1_RX_vect) {
	bt_rx_q[(bt_rx_head + bt_rx_n++) % RX_Q_SIZE] = UDR1;

	if (bt_rx_n >= RX_Q_SIZE) {
		// Error: Rx queue overflow!
	}
}

