/** 
  Code for Digital High Pass Filter
  Author  : Achmad Syahrul Irwansyah
  Project : Undergraduate Final Project
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]W. J. Palm, System dynamics. New York, Ny: Mcgraw-Hill, 2014.
**/
#include "HPF.h"
#define PI 3.141592653

// Constructor to assign HPF parameters
HPF::HPF(float _fc){
    fc = _fc;
    filtered = 0;
    last_filtered = 0;
    last_raw = 0;
    
};

float HPF::filter(float raw, float _Ts){

    alfa = 1.0/(1.0+(2.0*PI*fc*_Ts));
    filtered = alfa*(last_filtered + raw - last_raw);
    
    last_raw = raw;
    last_filtered = filtered;

    return filtered;
};
