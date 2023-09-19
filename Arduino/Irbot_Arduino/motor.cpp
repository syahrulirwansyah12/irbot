/** 
  Code to assign motor pin in arduino with BTS7960 motor driver
  Author  : Achmad Syahrul Irwansyah
  Project : Undergraduate Final Project
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#include <Arduino.h>
#include "motor.h"

// Constructor to assign motor pin
Motor::Motor(int rpwm, int lpwm, int en){
  rPWM = rpwm;
  lPWM = lpwm;
  EN = en;
};

// Method to set the motor pin mode
void Motor::start(){
  pinMode(rPWM, OUTPUT);
  pinMode(lPWM, OUTPUT);
  pinMode(EN, OUTPUT);
}

// Method to set enable pin
// Set "ena" to be HIGH to activate the motor
void Motor::setEnable(int _val){
    // Set the enable pin to be HIGH when "ena" is HIGH
    val = _val;
    if (val >= 0) {
        analogWrite(EN,val);
    } else {
        analogWrite(EN,-val);
    }
};

// Method to assign PWM value and rotate the motor
void Motor::rotate(){
    // Rotate CW when the value of "val" is more than 0
    // and rotate CCW when the value is less then 0
    if (val >= 0) {
        digitalWrite(rPWM, HIGH);
        digitalWrite(lPWM, LOW);
    } else {
        digitalWrite(rPWM, LOW);
        digitalWrite(lPWM, HIGH);
    }
};

// Method to show pin assignment
void Motor::debug(){
  Serial.print(rPWM);Serial.print("\t");
  Serial.print(lPWM);Serial.print("\t");
  Serial.print(EN);Serial.print("\t");
}
