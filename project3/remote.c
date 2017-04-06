#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "os.h"
#include "bluetooth_usart.h"

#include "debug.h"
#ifndef NDEBUG
#include <stdio.h>
#endif
// Use the DEBUG(M) macro to write debug messages over the USB (baud 19200).
// If NDEBUG is defined, no DEBUG messages will be compiled in.
void dbg_print_byte(uint8_t data) {
#ifndef NDEBUG
	char s[16];
	sprintf(s, "%d", data);
#endif
	DEBUG(s);
}

// Connect the Roomba BRC to pin 53, and the laser to pin 52.
#define ROOMBA_BRC (1 << PB0)
#define LASER_PIN  (1 << PB1)

volatile uint8_t r_vel, r_rot;
volatile int8_t t_pan, t_tilt;

#define RX_Q_SIZE 32
uint8_t bt_rx_q[RX_Q_SIZE];
volatile uint8_t bt_rx_head, bt_rx_n;

// NB: This will blindly do as asked. Make sure there's something there first!
static uint8_t get_rx_q_head(void) {
	uint8_t data;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		data = bt_rx_q[bt_rx_head];
		bt_rx_head = (bt_rx_head + 1) % RX_Q_SIZE;
		bt_rx_n--;
	}
	return data;
}

// Plug the pan servo into pin 11 (PB5), tilt into pin 12 (PB6).
static void servo_init(void) {
	// Set OC1A (PB5) and OC1B (PB6) to output mode.
	DDRB |= _BV(PB5) | _BV(PB6);
	// Set to Fast PWM Mode (WGM13:0 = 14), 
	// set the OC registers to clear when triggered (COM1x1:0 = 2),
	// and set the prescaler to 64 (CS12:0 = 3).
	TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);

	// ICR1 controls when the timer resets.
	ICR1 = 5000; // (16M / 50 Hz) / 64 = 5000.
	OCR1A = 375; // The OCR registers control when OC1A and OC1B are set low.
	OCR1B = 375; // 375 should turn out to be 1500 microseconds.
}

// Connect the Roomba to USART2.
static void roomba_write(uint8_t b) {
	while(!(UCSR2A & _BV(UDRE2))); // Wait until ready.
	UDR2 = b;
}

// Some Roomba commands.
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

	DDRB |= _BV(DDB0);   // Enable pin 53 for output.
	_delay_ms(100);
	PORTB |= ROOMBA_BRC; // Pull the Roomba BRC pin high.
	//Set Roomba baud rate by toggling the BRC pin 3 times.
	_delay_ms(300);
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
		// dbg_print_byte(cmd);
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
					if (data > 128) t_pan = 5;
					else if (data < 128) t_pan = -5;
					break;
				case T_TILT:
					if (data > 128) t_tilt = 5;
					else if (data < 128) t_tilt = -5;
					break;
				default:
					// ERROR!
					break;
			}		
		} else {
			// This is a simple command.
			// Send an ACK right away, then act on it.
			if (cmd == L_ON) {
				usart_tx(cmd);
				PORTB |= LASER_PIN;
			} else if (cmd == L_OFF) {
				usart_tx(cmd);
				PORTB &= ~(LASER_PIN);
			} else if (cmd == R_STOP) {
				usart_tx(cmd);
				r_vel = 128;
				r_rot = 128;
			} else {
				// Probably a misinterpreted data byte, or just line noise.
			}
		}
		// Don't call Task_Next here: there could be more input to read!
	}
}

static void turret_controller(void) {
	for(;;) {
		if (t_pan > 0 && OCR1A < 500) {
			OCR1A += 5;
		} else if (t_pan < 0 && OCR1A > 250) {
			OCR1A -= 5;
		}
		t_pan = 0;
		// Tilt is backwards: when t_tilt is low, OCR1B must be increased.
		if (t_tilt < 0 && OCR1B < 500) {
			OCR1B += 5;
		} else if (t_tilt > 0 && OCR1B > 250) {
			OCR1B -= 5;
		}
		t_tilt = 0;
		Task_Next();
	}
}

// Borrowed from the Arduino library.
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void roomba_controller(void) {
	int16_t radius, velocity;
	uint8_t	oldr = 0, oldv = 0;
	for(;;) { 
		// TODO: If wall_detected, back up for N ticks.
		while (r_rot == oldr && r_vel == oldv) 
			// Only send a command when something changes.
			Task_Next();

		// TODO: Y U NO TURN
		// Map rotation from [0, 255] to [-2000, 2000], reversed.
		// (i.e. 255 -> 1, 0 -> -1, 130 -> 2000, 126 -> -2000, 128 -> 0).
		if (r_rot > 128) {
			radius = map(r_rot, 128, 255, 2000, 1);
		} else if (r_rot < 128){
			radius = map(r_rot, 0, 128, -1, -2000);
		} else { // r_rot == 128
			radius = 0x7FFF; // Set radius to 32767 to drive straight.
		}

		// Map velocity from [0, 255] to [-500, 500].
		if (r_vel > 128) {
			velocity = map(r_vel, 128, 255, 0, 500);
		} else if (r_vel < 128) {
			velocity = map(r_vel, 0, 128, -500, 0);
		} else { // r_vel == ~128
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

BOOL wall_detected(void) {
	roomba_write(142); // "Sensors" command
	roomba_write(8); // Wall sensor
	int value = roomba_read(); // TODO: Write roomba_read.
	if (value) return TRUE;

	roomba_write(142); // "Sensors" command
	roomba_write(13); // Virtual wall sensor
	value = roomba_read();
	return value ? TRUE : FALSE;
}

void a_main(void) {
	DDRB = 0; // Zero it initially; we'll use |= to set pins later.
#ifndef NDEBUG
	debug_init();
	DEBUG("STARTING");
#endif
	usart_init();
	servo_init();
	roomba_init();
	DDRB |= LASER_PIN;

	Task_Create_Period(command_interpreter, 0, 10, 1, 1);
	Task_Create_Period(turret_controller, 0, 10, 1, 4);
	Task_Create_Period(roomba_controller, 0, 10, 1, 7);
	DEBUG("STARTED");
}

ISR(USART1_RX_vect) {
	uint8_t data = UDR1;
	dbg_print_byte(data);

	bt_rx_q[(bt_rx_head + bt_rx_n) % RX_Q_SIZE] = data;
	bt_rx_n += 1;

	if (bt_rx_n >= RX_Q_SIZE) {
		// Error: Rx queue overflow!
	}
}

