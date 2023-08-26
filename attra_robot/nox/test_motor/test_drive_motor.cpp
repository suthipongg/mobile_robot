#include <iostream>
#include <wiringPi.h>
#include "CytronMotorDriver.h"

CytronMD motor_l(23, 22);
CytronMD motor_r(26, 21);

int pwm_start = 0;
int pwm_step = 10;
int PWM_MAX = 1024;

int main()
{
    while (true)
    {
        std::cout << pwm_start << std::endl;
        motor_l.setSpeed(pwm_start);
        motor_r.setSpeed(pwm_start);
        pwm_start += pwm_step;
        if (pwm_start > PWM_MAX) {
            motor_l.setSpeed(0);
            motor_r.setSpeed(0);
            break;
        }
        delay(100);
    }

    return 0;
}
