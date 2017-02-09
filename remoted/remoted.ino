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
int8_t r_rad_data = 0;
int8_t r_vel_data = 0;
bool r_stop = false;

// Servo objects
Servo tservo; // tilt
Servo pservo; // pan
int pan_pos = 1500;
int tilt_pos = 1500;
int8_t t_pan_data = 0;
int8_t t_tilt_data = 0;

// HELPER FUNCTIONS
///////////////////

boolean cmd_takes_data(uint8_t b) {
  return ((b & 0x3F) < (1 << 5));
}

void handle_data_cmd(uint8_t device, uint8_t cmd, uint8_t data) {
  if (device == ROOMBA) {
        if (cmd == R_VEL) {
          if (data & 0x80) r_vel_data = -(data - 0x7F);
          else r_vel_data = data;
        } else if (cmd == R_ROT) {
          if (data & 0x80) r_rad_data = -(data - 0x7F);
          else r_rad_data = data;
        } else if (cmd == R_STOP) {
          r_stop = true;
        } else {
          // TODO: Report unexpected command.
        }
  } else if (device == TURRET) {
        if (cmd == T_PAN) {
          if (data & 0x80) t_pan_data = -(data - 0x7F);
          else t_pan_data = data;
          Serial.write("T_PAN ");
          Serial.println(t_pan_data); // DEBUG
        } else if (cmd == T_TILT) {
          if (data & 0x80) t_tilt_data = -(data - 0x7F);
          else t_tilt_data = data;
          Serial.write("T_TILT ");
          Serial.println(t_tilt_data); // DEBUG
        } else if (cmd == T_LASER_ON) {
          digitalWrite(LASER, HIGH);
        } else if (cmd == T_LASER_OFF) {
          digitalWrite(LASER, LOW);
        } else {
          // TODO: Report unexpected command.
        }
  }
}

void handle_cmd(uint8_t device, uint8_t cmd) {
  handle_data_cmd(device, cmd, 0);
}

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

    // If we happened to be reading between a command and its data, then we
    // might still be waiting for the data. expecting_data_byte contains the
    // command which was expecting the byte.
    if (expecting_data_byte) {
      device = expecting_data_byte >> 6;
      cmd = expecting_data_byte & 0x3F;
      handle_data_cmd(device, cmd, b);
      expecting_data_byte = 0;
      continue;
    }

    if (cmd_takes_data(b)) {
      int data = Serial1.read();
      if (data == -1) {
        expecting_data_byte = b;
        continue;
      }
      device = b >> 6;
      cmd = b & 0x3F;
      handle_data_cmd(device, cmd, (uint8_t)data);
    } else {
      handle_cmd(device, cmd);
    }
  }

  digitalWrite(2, LOW);
}

void turret_task(void)
{
  int8_t d_pan, d_tilt;
  d_pan = map((int)t_pan_data*t_pan_data, 0, 0x3F01, 0, 32);
  if (t_pan_data < 0) {
    d_pan = -d_pan;
  }
  if (d_pan) {
    Serial.write("Pan by "); // DEBUG
    Serial.println(d_pan);
  }
  
  d_tilt = map((int)t_tilt_data*t_tilt_data, 0, 0x3F01, 0, 32);
  if (t_tilt_data < 0) {
    d_tilt = -d_tilt;
  }
  if (d_tilt) {
    Serial.write("Tilt by "); // DEBUG
    Serial.println(d_tilt);
  }
  
  if (d_pan) {
    pan_pos += d_pan;
    if (pan_pos > 2000) pan_pos = 2000;
    else if (pan_pos < 1000) pan_pos = 1000;
    pservo.writeMicroseconds(pan_pos);
  }
  if (d_tilt) {
    tilt_pos += d_tilt;
    if (tilt_pos > 2000) tilt_pos = 2000;
    else if (tilt_pos < 1000) tilt_pos = 1000;
    tservo.writeMicroseconds(tilt_pos);
  }
  t_pan_data = 0;
  t_tilt_data = 0;
}

void roomba_task(void)
{
  // TODO can this be in here? might cause timing violation
  // or unnecessarily long period (due to rarity of init)
  if(!r_initialized) {
    roomba.init();
    r_initialized = true;
  }

  // signed 8-bit data over bt, [-128, 127]
  // scale to roomba values
  // roomba rad [-2000, 2000]
  // roomba vel [-500, 500]
  int16_t r_vel = r_vel_data << 4;
  if (r_vel < -2000) r_vel = -2000;
  else if (r_vel > 2000) r_vel = 2000;
  int16_t r_rad = r_rad_data << 2;
  if (r_rad < -500) r_rad = -500;
  else if (r_rad > 500) r_rad = 500;

  if (r_rad && r_vel) {
    roomba.drive(r_vel, r_rad);
  } else if (r_vel && !r_rad) {
    roomba.drive(r_vel, 32768);
  } else if (!r_vel && r_rad) {
    roomba.drive(50, r_rad);
  }
  if (r_stop) {
    roomba.drive(0, 0);
  }
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
  Scheduler_StartTask(130, 100, bluetooth_task);
  Scheduler_StartTask(150, 100, turret_task);
  Scheduler_StartTask(0, 100, roomba_task);
}

void loop() {
	uint32_t time_to_next_task = Scheduler_Dispatch();
	if (time_to_next_task) {
		digitalWrite(4, HIGH);
		delay(time_to_next_task);
		digitalWrite(4, LOW);
	}
}
