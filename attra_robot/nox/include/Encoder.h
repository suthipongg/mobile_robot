#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

class Encoder
{
  friend int wiringPiISR(Encoder &obj, int mode, void (*function)(void));
  public:
    Encoder(uint8_t pin_a, uint8_t pin_b);
    ~Encoder();
    
  protected:
    uint8_t _pin_a;
    uint8_t _pin_b;
};

#endif
