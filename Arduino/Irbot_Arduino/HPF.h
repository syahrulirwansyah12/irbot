/** 
  High Pass Filter code - Header File
  Author  : Achmad Syahrul Irwansyah
  Project : Undergraduate Final Project
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]W. J. Palm, System dynamics. New York, Ny: Mcgraw-Hill, 2014.
**/

#ifndef HPF_H
#define HPF_H

class HPF{
    public:
    //
    HPF(float _fc);
    
    float filter(float raw, float _Ts);

    private:
    float fc;
    float filtered;
    float last_filtered;
    float last_raw;
    float alfa;
    float err;
};

#endif
