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

// Pins for logic analyzer.
#define LA_PHOTOCELL 41
#define LA_LASER 43
#define LA_LCD 45
#define LA_SERVO 47
#define LA_ROOMBA 49
#define LA_BT_S 51

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
  digitalWrite(LA_LASER, HIGH);
  
  // read button of turret joystick
  tjs_button = digitalRead(TJS_BUTTON);
  // send on/off cmd to remote
  // TODO: Don't just send it every time.
  if (tjs_button == 1)
    bt_queue_message(TURRET, T_LASER_OFF);
  else
    bt_queue_message(TURRET, T_LASER_ON);

  digitalWrite(LA_LASER, LOW);
}

void photocell_task(void)
{
  digitalWrite(LA_PHOTOCELL, HIGH);
  // read photocell
  light_level = analogRead(PHOTOCELL);
  // store for LCD
  digitalWrite(LA_PHOTOCELL, LOW);
}

void lcd_task(void)
{
  digitalWrite(LA_LCD, HIGH);
  
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
  lcd.setCursor(5, 1);
  lcd.print("Tx");
  lcd.print(tjs_xpos);
  lcd.setCursor(11, 1);
  lcd.print("y");
  lcd.print(tjs_ypos);

  // Display roomba joystick position.
  lcd.setCursor(5, 0);
  lcd.print("Rx");
  lcd.print(rjs_xpos);
  lcd.setCursor(11, 0);
  lcd.print("y");
  lcd.print(rjs_ypos);
  
  digitalWrite(LA_LCD, LOW);
}

// This task will queue up to two 2-byte commands: 
// one giving an x displacement for the turret, and one giving 
// a y displacement. If the joystick is centred, then no command
// will be sent and the turret will naturally come to a rest.
void servo_task(void)
{
  digitalWrite(LA_SERVO, HIGH);
  
  // read turret joystick
  // store for LCD
  tjs_xpos = analogRead(TJS_X);
  tjs_ypos = analogRead(TJS_Y);
  // send cmd to remote
  if (tjs_xpos > (512 + JS_DEADZONE) ||
      tjs_xpos < (512 - JS_DEADZONE)) {
    int pan = map(tjs_xpos, 0, 1024, -128, 128);
    if (pan < 0) {
      pan = -pan + 0x7F;
    }
    bt_queue_message(TURRET, T_PAN, pan);
	}
  if (tjs_ypos > (512 + JS_DEADZONE) ||
      tjs_ypos < (512 - JS_DEADZONE)) {
    int tilt = map(tjs_ypos, 0 , 1024, -128, 128);
    if (tilt < 0) {
      tilt = -tilt + 0x7F;
    }
    bt_queue_message(TURRET, T_TILT, tilt);
  }

  digitalWrite(LA_SERVO, LOW);
}

// This task will send either a velocity and turning angle for 
// the Roomba, both [-128, 127], or it will send the stop command.
void roomba_task(void)
{
  digitalWrite(LA_ROOMBA, HIGH);
  
  // read roomba joystick
  // store for LCD
  rjs_xpos = analogRead(RJS_X);
  rjs_ypos = analogRead(RJS_Y);
  rjs_button = digitalRead(RJS_BUTTON);
  int8_t x = (rjs_xpos - 512) >> 2,
         y = (rjs_ypos - 512) >> 2;

  // send cmd to remote
  if (rjs_xpos > (512 + JS_DEADZONE) || 
      rjs_xpos < (512 - JS_DEADZONE) || 
      rjs_ypos > (512 + JS_DEADZONE) ||
      rjs_ypos < (512 - JS_DEADZONE)) {
    bt_queue_message(ROOMBA, R_VEL, y);
    bt_queue_message(ROOMBA, R_ROT, x);
  }

  if (x == 0 && y == 0) {
    bt_queue_message(ROOMBA, R_STOP);
  }

  digitalWrite(LA_ROOMBA, LOW);
}

void bt_send_task(void)
{
  digitalWrite(LA_BT_S, HIGH);
  
  // send packets to remote
  if (bt_tx_n > 0) {
    Serial1.write(bt_tx_q, bt_tx_n);
    bt_tx_n = 0;
  }

  digitalWrite(LA_BT_S, LOW);
}

void idle(uint32_t idle_period)
{
  return;
}


// ARDUINO FUNCTIONS
////////////////////

void setup()
{
  lcd.begin(16, 2);
  pinMode(TJS_BUTTON, INPUT_PULLUP);
  pinMode(RJS_BUTTON, INPUT_PULLUP);

  pinMode(LA_PHOTOCELL, OUTPUT);
  pinMode(LA_LASER, OUTPUT);
  pinMode(LA_LCD, OUTPUT);
  pinMode(LA_SERVO, OUTPUT);
  pinMode(LA_ROOMBA, OUTPUT);
  pinMode(LA_BT_S, OUTPUT);
  
  Serial.begin(9600);
  Serial1.begin(9600); // Bluetooth
  while (!Serial || !Serial1); // wait for Serial to be ready
  
  Scheduler_Init();

  Scheduler_StartTask(20, 100, laser_task);
  Scheduler_StartTask(30, 100, photocell_task);
  Scheduler_StartTask(40, 100, lcd_task);
  Scheduler_StartTask(50, 100, servo_task);
  Scheduler_StartTask(60, 100, roomba_task);
  Scheduler_StartTask(70, 100, bt_send_task);
}

void loop()
{
  uint32_t idle_period = Scheduler_Dispatch();
  if(idle_period)
  {
    idle(idle_period);
  }
}

