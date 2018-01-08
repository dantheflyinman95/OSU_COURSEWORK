/* robot_ctrl.ino 
 * D. Eisenbach
 * 6.7.17
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <PID_v1.h>
#include <Adafruit_SoftServo.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include "robot_ctrl.h"

void setup() {
  pinMode(ARM_HT_MP1, OUTPUT);
  pinMode(ARM_HT_MP2, OUTPUT);
  pinMode(ARM_LEN_EN, OUTPUT);
  pinMode(ARM_LEN_DIR, OUTPUT);
  pinMode(ARM_LEN_PUL, OUTPUT);
  pinMode(CLAMP_PIN, OUTPUT);
  pinMode(HINGE_PIN, OUTPUT);
  
  pinMode(ENCOD_PINA, INPUT_PULLUP);  //quadrature encoder input A
  pinMode(ENCOD_PINB, INPUT_PULLUP);  //quadrature encoder input B
  pinMode(ARM_EXT_PIN, INPUT_PULLUP);
  pinMode(ARM_RETRACT_PIN, INPUT_PULLUP);
  pinMode(ARM_LOWERED_PIN, INPUT_PULLUP);
  pinMode(BB_SENSOR_PIN, INPUT_PULLUP);
  pinMode(TOWER_INIT_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT);

  digitalWrite(LED_BUILTIN, LOW);

  clamp.attach(CLAMP_PIN);
  hinge.attach(HINGE_PIN);

  //attachInterrupt(1, bb_sensor_ISR, FALLING);
  attachInterrupt(0, encoder, FALLING);  //update encoder position
  
  TCCR1B = TCCR1B & 0b11111000 | 1;  //set 31KHz PWM to prevent motor noise

  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  
  arm_ht_PID.SetMode(AUTOMATIC);
  arm_ht_PID.SetSampleTime(1);
  arm_ht_PID.SetOutputLimits(-255, 255);
  
  //Serial.begin (115200);  //for debugging

  init_playable_blocks();
  encoderPos = 0;
  cntrl_ps = WAIT;
}

void loop() {
  switch (cntrl_ps) {
    case WAIT:
      clamp.write(clamp_open);
      hinge.write(hinge_lower);
      if (digitalRead(START_PIN) == LOW) done = 0;
      if ((digitalRead(START_PIN) == HIGH) && !done && !end_game && !lower_arm && arm_retracted) cntrl_ns = START;
      else {
        if (!digitalRead(ARM_LOWERED_PIN) == LOW) lower_arm = 1;
        if (!digitalRead(ARM_RETRACT_PIN) == LOW) step(-1);
        else {
          arm_retracted = 1;
          arm_extended = 0;
        }
        cntrl_ns = WAIT;
      }
      break;
    case START:
      hinge.write(hinge_raise);
      if (!arm_extended) {
        step(steps_clear_hinge);
        arm_extended = 1;
        delay(500);
      }
      if (arm_raised) {
        raise_arm = 0;
        cntrl_ns = EXT_ARM; 
      }
      else {
        raise_arm = 1;
        cntrl_ns = START;
      }
      break;
    case EXT_ARM:
      if (digitalRead(ARM_EXT_PIN) == LOW) {
        tower_cleared = 0;
        cntrl_ns = CHECK_BLOCK;
      }
      else {
        step(1);
        cntrl_ns = EXT_ARM;
      }
      break;
    case CHECK_BLOCK:
      if (isSideBlock) {
        grab_block = 1;
        clamp.write(clamp_close);  //grab side block
        delay(100);
      }
      else grab_block = 0;
      grab_block = 1;
      clamp.write(clamp_close);  //grab side block
      delay(100);
      isSideBlock = 0;
      playable_blocks[topmost_block] = 0;
      cntrl_ns = CLEAR_TOWER;
      break;
    case CLEAR_TOWER:
      if (!tower_cleared) {
        step(-steps_clear_tower);
        tower_cleared = 1; 
        delay(500);
      }
      if (grab_block || end_game) cntrl_ns = LOWER_ARM;
      else {
        raise_arm = 1;
        if (arm_raised) {
          raise_arm = 0;
          cntrl_ns = EXT_ARM;
        }
        else cntrl_ns = CLEAR_TOWER; 
      }
      break;
    case LOWER_ARM:
      if (!digitalRead(ARM_LOWERED_PIN) == LOW) {
        lower_arm = 1;
        cntrl_ns = LOWER_ARM;
      }
      else cntrl_ns = RETURN_HOME;
      break;
    case RETURN_HOME:
      if (digitalRead(ARM_RETRACT_PIN) == LOW) {
        hinge.write(hinge_lower);
        delay(200);
        clamp.write(clamp_open);
        arm_retracted = 1;
        arm_extended = 0;
        done = 1;
        cntrl_ns = WAIT;
      }
      else {
        step(-1);
        cntrl_ns = RETURN_HOME;
      }
      break;
  }
  cntrl_ps = cntrl_ns;
}


