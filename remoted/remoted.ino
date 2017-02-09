/* remoted.c
 * 
 * 8 Feb 2017
 * team 1
 *
 * Remoted station.
 */

#include "scheduler.h"
#include "bluetooth.h"
#include "Roomba_Driver.h"
#include <Servo.h>

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

#define BUF_SIZE 64
uint8_t tx_buf[BUF_SIZE];
uint8_t bytes_to_send = 0;
uint8_t expecting_data_byte = 0;

// Roomba object
Roomba roomba(R_SERIAL, R_DRIVE);
bool r_initialized = false;
uint8_t r_cmd = 0;
uint8_t r_data;

// Servo objects
Servo tservo; // tilt
Servo pservo; // pan
uint8_t t_cmd = 0;
uint8_t t_data;
uint8_t laser_cmd = 0;


// TTA-SCHEDULED TASKS
//////////////////////

void bluetooth_task(void)
{
  digitalWrite(2, HIGH);

  // Send stuff.
  if (bytes_to_send > 0) {
    Serial1.write(tx_buf, bytes_to_send);
    bytes_to_send = 0;
  }

  // Receive stuff.
  while(Serial1.available()) {
    uint8_t b, device, cmd;

    b = Serial1.read();
    Serial.print(b);

    // If we happened to be reading between a command and its data, then we
    // might still be waiting for the data. expecting_data_byte contains the
    // command which was expecting the byte.
    if (expecting_data_byte) {
      device = expecting_data_byte >> 6;
      cmd = expecting_data_byte & 0x3F;
      if (device == ROOMBA) {
        r_cmd = cmd;
        r_data = b;
      } else if (device == TURRET) {
        t_cmd = cmd;
        t_data = b;
      }
      expecting_data_byte = 0;
      continue;
    }

    device = b >> 6;
    cmd = b & 0x3F;
    if (device == ROOMBA) {
      r_cmd = cmd;
      // The first 8 commands are movement commands, the last
      // of which is the backward-right command.
      if (cmd <= R_BACKWARD_RIGHT) {
        // All movement commands have an additional data byte.
        int data = Serial1.read();
        if (data == -1) {
          expecting_data_byte = b;
        } else {
          r_data = (uint8_t)data;
        }
      } // else: data-less commands here.
    } else if (device == TURRET) {
      // T_PAN_RIGHT_TILT_DOWN is the last turret command with data.
      if (cmd <= T_PAN_RIGHT_TILT_DOWN) {
        t_cmd = cmd;
        int data = Serial1.read();
        if (data == -1) {
          expecting_data_byte = b;
        } else {
          t_data = (uint8_t)data;
        }
      } else {
        if (cmd == T_LASER_ON) {
          laser_cmd = 1;
        } else if (cmd = T_LASER_OFF) {
          laser_cmd = 0;
        }
      }
    } else {
      // TODO: complain loudly. sing a song!
    }
  }

  digitalWrite(2, LOW);
}

void laser_task(void)
{
  if(laser_cmd == 1)
  {
    digitalWrite(LASER, HIGH);
  }
  else if(laser_cmd == 0)
  {
    digitalWrite(LASER, LOW);
  }
}

void turret_task(void)
{
  if (!t_cmd) return;

  int pan = (t_data >> 4) * 34; // 34 ~= 500/15
  int tilt = (t_data & 0x0F) * 34;
  switch(t_cmd) {
  case T_PAN_LEFT:
    pservo.writeMicroseconds(1500 - pan);
    break;
  case T_PAN_RIGHT:
    pservo.writeMicroseconds(1500 + pan);
    break;
  case T_TILT_UP:
    tservo.writeMicroseconds(1500 + tilt);
    break;
  case T_TILT_DOWN:
    tservo.writeMicroseconds(1500 - tilt);
    break;
  case T_PAN_LEFT_TILT_UP:
    pservo.writeMicroseconds(1500 - pan);
    tservo.writeMicroseconds(1500 + tilt);
    break;
  case T_PAN_LEFT_TILT_DOWN:
    pservo.writeMicroseconds(1500 - pan);
    tservo.writeMicroseconds(1500 - tilt);
    break;
  case T_PAN_RIGHT_TILT_UP:
    pservo.writeMicroseconds(1500 + pan);
    tservo.writeMicroseconds(1500 + tilt);
    break;
  case T_PAN_RIGHT_TILT_DOWN:
    pservo.writeMicroseconds(1500 + pan);
    tservo.writeMicroseconds(1500 - tilt);
    break;
  }
  t_cmd = 0;
}

void roomba_task(void)
{
  // TODO can this be in here? might cause timing violation
  // or unnecessarily long period (due to rarity of init)
  if(!r_initialized) {
    roomba.init();
    r_initialized = true;
  }

  if(!r_cmd) return;

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
  switch(r_cmd)
  {
    case R_FORWARD: 
      roomba.drive(r_vel, 32768);
      break;
    case R_BACKWARD:
      roomba.drive(-r_vel, 32768);
      break;
    case R_RIGHT:
      roomba.drive(50, -r_rad);
      break;
    case R_LEFT:
      roomba.drive(50, r_rad);
      break;
    case R_FORWARD_RIGHT:
      roomba.drive(r_vel, -r_rad);
      break;
    case R_FORWARD_LEFT:
      roomba.drive(r_vel, r_rad);
      break;
    case R_BACKWARD_RIGHT:
      roomba.drive(-r_vel, -r_rad);
      break;
    case R_BACKWARD_LEFT:
      roomba.drive(-r_vel, r_rad);
      break;
    default:
      break;
  }
  r_cmd = 0;
}


// ARDUINO FUNCTIONS
////////////////////

void setup() {
  // Serial zero is used (sparingly!) for debug output to the PC.
  Serial.begin(9600);
  // The Bluetooth module should be connected to Serial1.
  Serial1.begin(9600);
  while(!Serial || !Serial1); // Wait for serial ports to be ready.

  // Set up pins.
  pinMode(LASER, OUTPUT);
  // These pins are used for monitoring using the logic analyzer.
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  // Configure servos.
  // Between 1000 and 2000 nanoseconds pulse.
  pservo.attach(PSERVO, 1000, 2000);
  tservo.attach(TSERVO, 1000, 2000);

  // Initialize roomba. This opens Serial<R_SERIAL>.
  roomba.init();
  r_initialized = true;

  Scheduler_Init();

  // General form for starting a task:
  // Scheduler_StartTask(offset, period, function_pointer);
  Scheduler_StartTask(30, 100, bluetooth_task);
  Scheduler_StartTask(0, 0, laser_task);
  Scheduler_StartTask(0, 0, turret_task);
  Scheduler_StartTask(0, 0, roomba_task);
}

void loop() {
	uint32_t time_to_next_task = Scheduler_Dispatch();
	if (time_to_next_task) {
		digitalWrite(4, HIGH);
		delay(time_to_next_task);
		digitalWrite(4, LOW);
	}
}
