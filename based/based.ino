/* based.c
 * 
 * 3 Feb 2017
 * team 1
 *
 * Based station. Document that shintoist.
 */

#include "scheduler.h"
#include "bluetooth.h"
#include <LiquidCrystal.h>

// PIN DEFINITIONS
//////////////////

#define PHOTOCELL 8

// Pins for the turret joystick button and axes.
#define TJS_BUTTON 30
#define TJS_X 9
#define TJS_Y 10

// Pins for the roomba joystick button and axes.
#define RJS_BUTTON 31
#define RJS_X 11
#define RJS_Y 12

// Size of "deadzone" in the middle of the joystick. If the difference from the
// zero position does not exceed the deadzone, then no command will be sent.
#define JS_DEADZONE 64


// GLOBAL DATA STRUCTURES
/////////////////////////

// photocell
int light_level;
// turret joystick
int tjs_xpos, tjs_ypos, tjs_button;
// roomba joystick
int rjs_xpos, rjs_ypos, rjs_button;

// bluetooth send queue
uint8_t bt_tx_n = 0;
uint8_t bt_tx_q[BT_Q_SIZE];

// magic LCD incantation
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


// HELPER FUNCTIONS
///////////////////

/* There are two versions of this function: one that takes a "data" argument
 * and one that doesn't. One will append two bytes to the Tx queue, while the
 * other will append only one. Make sure you use the right one! */
void bt_queue_message(uint8_t device, uint8_t command)
{
	if (bt_tx_n >= BT_Q_SIZE - 1) {
		// TODO: panic and report the overflow.
		return;
	}
	// The device is identified by the first two bits,
	// then the next six contain the command to that device.
	bt_tx_q[bt_tx_n++] = (device << 6) | command;
}

void bt_queue_message(uint8_t device, uint8_t command, uint8_t data)
{
	if (bt_tx_n >= BT_Q_SIZE - 2) {
		// TODO: panic and report the overflow.
		return;
	}
	// The device is identified by the first two bits,
	// then the next six contain the command to that device.
	bt_tx_q[bt_tx_n++] = (device << 6) | command;
	bt_tx_q[bt_tx_n++] = data;
}


// TTA-SCHEDULED TASKS
//////////////////////

void laser_task(void)
{
  // read button of turret joystick
  tjs_button = digitalRead(TJS_BUTTON);
  // send on/off cmd to remote
  return;
}

void photocell_task(void)
{
  // read photocell
  light_level = analogRead(PHOTOCELL);
  // store for LCD
  return;
}

void lcd_task(void)
{
  // update LCD with photocell, joystick(, servo, laser) status
  lcd.clear();
  lcd.setCursor(0, 0);

  // With one bank of lights on in the lab, light_level is 
  // around 300. With both, it gets up to 400. In the shade, 
  // it usually drops below 100. We use 200 as a nice divider.
  if (light_level > 200) {
    lcd.print("light");
  } else {
    lcd.print("dark");
  }
  
  // Display turret joystick position.
  lcd.setCursor(6, 0);
  lcd.print("tx");
  lcd.print(tjs_xpos);
  lcd.setCursor(11, 0);
  lcd.print("y");
  lcd.print(tjs_ypos);

  // Display roomba joystick position.
  lcd.setCursor(6, 0);
  lcd.print("rx");
  lcd.print(rjs_xpos);
  lcd.setCursor(11, 0);
  lcd.print("y");
  lcd.print(rjs_ypos);
  return;
}

void servo_task(void)
{
  // read turret joystick
  // store for LCD
  tjs_xpos = analogRead(TJS_X);
  tjs_ypos = analogRead(TJS_Y);
  // send cmd to remote
  return;
}

void roomba_task(void)
{
  // read roomba joystick
  // store for LCD
  rjs_xpos = analogRead(RJS_X);
  rjs_ypos = analogRead(RJS_Y);
  rjs_button = digitalRead(RJS_BUTTON);

  // send cmd to remote
  // TODO: I suspect forward and backward are, well, backwards here.
  // TODO how is the data byte packed? is it even packed consistently?
  if (rjs_xpos > (512 + JS_DEADZONE)) {
    uint8_t right = (rjs_xpos - 512) >> 5;
    if (rjs_ypos > (512 + JS_DEADZONE)) {
      uint8_t forward = (rjs_ypos - 512) >> 5;
      bt_queue_message(ROOMBA, R_FORWARD_RIGHT, (right << 4) | forward);
    } else if (rjs_ypos < (512 - JS_DEADZONE)) {
      uint8_t backward = (512 - rjs_ypos) >> 5;
      bt_queue_message(ROOMBA, R_BACKWARD_RIGHT, (right << 4) | backward);
    } else {
      bt_queue_message(ROOMBA, R_RIGHT, right);
    }
  } else if (rjs_xpos < (512 - JS_DEADZONE)) {
    uint8_t left = (512 - rjs_xpos) >> 5;
    if (rjs_ypos > (512 + JS_DEADZONE)) {
      uint8_t forward = (rjs_ypos - 512) >> 5;
      bt_queue_message(ROOMBA, R_FORWARD_LEFT, (left << 4) | forward);
    } else if (rjs_ypos < (512 - JS_DEADZONE)) {
      uint8_t backward = (512 - rjs_ypos) >> 5;
      bt_queue_message(ROOMBA, R_BACKWARD_LEFT, (left << 4) | backward);
    } else {
      bt_queue_message(ROOMBA, R_LEFT, left);
	  }
  } else if (rjs_ypos > (512 + JS_DEADZONE)) {
    bt_queue_message(ROOMBA, R_FORWARD, (rjs_ypos - 512) >> 5);
  } else if (rjs_ypos < (512 - JS_DEADZONE)) {
    bt_queue_message(ROOMBA, R_BACKWARD, (512 - rjs_ypos) >> 5);
  }
  return;
}

void bt_send_task(void)
{
  // send packets to remote
  if (bt_tx_n > 0) {
    Serial1.write(bt_tx_q, bt_tx_n);
    bt_tx_n = 0;
  }
  return;
}

void bt_receive_task(void)
{
  // get packets from remote
  // pass info on to relevant tasks
  return;
}

void idle(uint32_t idle_period)
{
  // any low-priority tasks? making music?
  delay(idle_period); // don't actually do this
}


// ARDUINO FUNCTIONS
////////////////////

void setup()
{
  lcd.begin(16, 2);
  pinMode(30, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  Serial1.begin(9600); // Bluetooth
  while (!Serial1); // wait for Serial1 to be ready
  
  Scheduler_Init();

  Scheduler_StartTask(20, 100, laser_task);
  Scheduler_StartTask(30, 100, photocell_task);
  Scheduler_StartTask(40, 100, lcd_task);
  Scheduler_StartTask(50, 100, servo_task);
  Scheduler_StartTask(60, 100, roomba_task);
  Scheduler_StartTask(5, 50, bt_send_task);
  Scheduler_StartTask(15, 50, bt_receive_task);
}

void loop()
{
  uint32_t idle_period = Scheduler_Dispatch();
  if(idle_period)
  {
    idle(idle_period);
  }
}

