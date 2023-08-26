#ifndef CYTRON_MOTOR_DRIVER_H
#define CYTRON_MOTOR_DRIVER_H

#include <stdint.h>

class CytronMD
{
  public:
    CytronMD(uint8_t pwm, uint8_t dir);
    void setSpeed(int16_t speed);
    ~CytronMD();
    
  protected:
  	uint8_t _pwm;
    uint8_t _dir;
};

#endif