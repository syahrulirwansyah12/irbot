/** 
  Code to assign motor pin in arduino with BTS7960 motor driver
  Author  : Achmad Syahrul Irwansyah
  Project : Undergraduate Final Project
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
    public:
    Motor(int rpwm, int lpwm, int en);

    void start();
    void setEnable(int _val);
    void rotate();
    void debug();

    private:
    int rPWM;
    int lPWM;
    int EN;
    int val;
};

#endif
