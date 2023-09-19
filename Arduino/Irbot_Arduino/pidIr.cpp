/** 
  Code to compute PID control
  Author  : Achmad Syahrul Irwansyah
  Project : Undergraduate Final Project
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]R. C. Dorf and R. H. Bishop, Modern Control Systems. Essex: Pearson, Cop, 2017.
**/

#include "pidIr.h"

// Constructor to assign PID parameters
pidIr::pidIr(float p, float i, float d){
    Kp = p;
    Ki = i;
    Kd = d;
    last_error = 0;
    sum_error = 0;
};

// Method to compute PID
float pidIr::compute(float setpoint, float feedback, float max_output, float Ts){
    error = setpoint - feedback;
    sum_error += error;

    output = Kp*error + Ki*Ts*sum_error + Kd/Ts*(error-last_error);
    if (fabs(output) > max_output){
        output = fabs(output)/output*max_output;
    }
    last_error = error;
    
    return output;
};

void pidIr::reset(){
  output = 0;
  last_error = 0;
  sum_error = 0;
}
