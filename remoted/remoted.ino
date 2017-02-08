#include "scheduler.h"
#include "bluetooth.h"

// PIN DEFINITIONS
//////////////////

#define LASER 31
#define PSERVO 32 // pan servo
#define TSERVO 33 // tilt servo

// Roomba drive pin and serial connection
#define R_DRIVE 34
#define R_SERIAL 2


// GLOBAL DATA STRUCTURES
/////////////////////////

/* Notes:
 *  The Mega 2560 has a 64-byte software serial buffer.
 *  You can check if it overflowed with Serial.overflow().
 */

#define BUF_SIZE 16
uint8_t rx_buf[BUF_SIZE];
uint8_t bytes_available = 0;
uint8_t tx_buf[BUF_SIZE];
uint8_t bytes_to_send = 0;

// Roomba object
Roomba roomba(R_SERIAL, R_DRIVE);
bool r_initialized = false;


// HELPER FUNCTIONS
///////////////////

void read_bluetooth() {
 digitalWrite(2, HIGH);
  while(Serial1.available() && bytes_available < BUF_SIZE) {
    rx_buf[bytes_available++] = Serial1.read();
  }
 digitalWrite(2, LOW);
}

void write_bluetooth() {
  digitalWrite(3, HIGH);
  while(bytes_to_send-- > 0) {
    Serial1.write(tx_buf[bytes_to_send]);
  }
 digitalWrite(3, LOW);
}


// TTA-SCHEDULED TASKS
//////////////////////

void laser_task(void)
{
  return;
}

void turret_task(void)
{
  return;
}

void roomba_task(void)
{
  if(!initialized) {
    roomba.init();
    initialized = true;
  }

  // uint8_t data over bt
  // vvvvrrrr
  // split into vel and radius [0, 15]
  // scale to roomba values
  // roomba vel [-2000, 2000]
  // roomba rad [-500, 500]
  uint16_t r_vel = r_data >> 4;
  uint16_t r_rad = r_data & 0xF;
  r_vel = r_vel * 133; // approx 2000/15
  r_rad = r_rad * 33; // approc 500/15
  switch(r_cmd) // TODO r_cmd, r_data
  {
    case 'R_FORWARD': 
      roomba.drive(r_vel, 32768);
      break;
    case 'R_BACKWARD':
      roomba.drive(-r_vel, 32768);
      break;
    case 'R_RIGHT':
      roomba.drive(50, -r_rad);
      break;
    case 'R_LEFT':
      roomba.drive(50, r_rad);
      break;
    case 'R_FORWARD_RIGHT':
      roomba.drive(r_vel, -r_rad);
      break;
    case 'R_FORWARD_LEFT':
      roomba.drive(r_vel, r_rad);
      break;
    case 'R_BACKWARD_RIGHT':
      roomba.drive(-r_vel, -r_rad);
      break;
    case 'R_BACKWARD_LEFT':
      roomba.drive(-r_vel, r_rad);
      break;
    default:
      break;
  }
  return;
}


// ARDUINO FUNCTIONS
////////////////////

void setup() {
  // Serial zero is used (sparingly!) for debug output to the PC.
	Serial.begin(9600);
  // The Bluetooth module should be connected to Serial1.
  Serial1.begin(9600);
  while(!Serial || !Serial1); // Wait for serial ports to be ready.

  // These pins are used for monitoring using the logic analyzer.
	pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  // Initialize roomba. This opens Serial<R_SERIAL>.
  roomba.init();
  r_initialized = true;

	Scheduler_Init();

	// General form for starting a task:
	// Scheduler_StartTask(offset, period, function_pointer);
  Scheduler_StartTask(30, 100, read_bluetooth);
  Scheduler_StartTask(60, 100, write_bluetooth);
}

void loop() {
	uint32_t time_to_next_task = Scheduler_Dispatch();
	if (time_to_next_task) {
		digitalWrite(4, HIGH);
		delay(time_to_next_task);
		digitalWrite(4, LOW);
	}
}
