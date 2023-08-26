#include "CytronMotorDriver.h"
#include <wiringPi.h>
#include <iostream>

#define MAX_PULSE 1024

CytronMD::CytronMD(uint8_t pwm, uint8_t dir)
{
      _pwm = pwm;
      _dir = dir;
      
      if (wiringPiSetup() == -1)
      {
        std::cerr << "Failed to initialize WiringPi." << std::endl;
        exit(0);
      }

      pinMode(_pwm, PWM_OUTPUT);
      pinMode(_dir, OUTPUT);

      pwmWrite(_pwm, 0);
      digitalWrite(_dir, LOW);
}

void CytronMD::setSpeed(int16_t speed)
{
      if (speed > MAX_PULSE)
      {
            speed = MAX_PULSE;
      }
      else if (speed < -MAX_PULSE)
      {
            speed = -MAX_PULSE;
      }

      if (speed >= 0)
      {
            pwmWrite(_pwm, speed);
            digitalWrite(_dir, LOW);
      }
      else
      {
            pwmWrite(_pwm, -speed);
            digitalWrite(_dir, HIGH);
      }
}

CytronMD::~CytronMD() {
      std::cout << "disconnected Cytron driver board" << std::endl;
}