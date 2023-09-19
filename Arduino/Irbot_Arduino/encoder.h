/** 
  Header file of encoder code
  Author  : Achmad Syahrul Irwansyah
  Project : Undergraduate Final Project
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
    public:
      Encoder(int pin_a, int pin_b); // Constructor to assign encoder pin
      void start(void(*userFuncA)(void),void(*userFuncB)(void)); // Method to start the encoder pulse reading
      int getPinA(){return enc_a;}; // Method that return to the value of encoder A pin
      int getPinB(){return enc_b;}; // Method that return to the value of encoder B pin
      int32_t getPos(){return enc_pos;}; // Method that return to the value of recent value of encoder position 
      void doEncoderB(); // Function that will be done when there is pulse change in encoder B pin
      void doEncoderA(); // Function that will be done when there is pulse change in encoder A pin
      friend void callBackFunction(Encoder);
      
    private:
      int enc_a;
      int enc_b;
      int32_t enc_pos;    
};



#endif
