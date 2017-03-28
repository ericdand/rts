#include "os.h"
#include "bluetooth_proto.h"

int sample_adc(uint8_t pin) {
	return 0;
}

void joystick_interpreter(void) {
	int pan, tilt, rot, vel;
	for(;;) {
		// Sample thumbsticks.
		pan = sample_adc(TURRET_THUMB_X);
		tilt = sample_adc(TURRET_THUMB_Y);
		rot = sample_adc(ROOMBA_THUMB_X);
		vel = sample_adc(ROOMBA_THUM_Y);
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

