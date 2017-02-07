/* based.c
 * 
 * 3 Feb 2017
 * team 1
 *
 * Based station. Document that shintoist.
 */

#include "scheduler.h"
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

// GLOBAL DATA STRUCTURES
/////////////////////////

// photocell
int light_level;
// turret joystick
int tjs_xpos, tjs_ypos, tjs_button;
// roomba joystick
int rjs_xpos, rjs_ypos, rjs_button;

// magic LCD incantation
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

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
  rjs_button = digitalRead(RJS_BUTTON)
  // send cmd to remote
  return;
}

void bt_send_task(void)
{
  // send packets to remote
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

void setup()
{
  lcd.begin(16, 2);
  pinMode(30, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  
  Scheduler_Init();

  Scheduler_StartTask(0, 0, laser_task);
  Scheduler_StartTask(0, 0, photocell_task);
  Scheduler_StartTask(0, 0, lcd_task);
  Scheduler_StartTask(0, 0, servo_task);
  Scheduler_StartTask(0, 0, roomba_task);
  Scheduler_StartTask(0, 0, bt_send_task);
  Scheduler_StartTask(0, 0, bt_receive_task);
}

void loop()
{
  uint32_t idle_period = Scheduler_Dispatch();
  if(idle_period)
  {
    idle(idle_period);
  }
}

