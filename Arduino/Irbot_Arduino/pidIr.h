/** 
  Code to compute PID control - Header File
  Author  : Achmad Syahrul Irwansyah
  Project : Undergraduate Final Project
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]R. C. Dorf and R. H. Bishop, Modern Control Systems. Essex: Pearson, Cop, 2017.
**/

#ifndef PIDIR_H
#define PIDIR_H

#include "math.h"

class pidIr{
    public:
    //p=proporsional, i=integral, d=differential, Ts=sampling time
    pidIr(float p, float i, float d);
    
    float compute(float setpoint, float feedback, float max, float Ts);
    void reset();

    private:
    float error;
    float last_error;
    float output;
    float sum_error;
    float Kp;
    float Ki;
    float Kd;
};

#endif
