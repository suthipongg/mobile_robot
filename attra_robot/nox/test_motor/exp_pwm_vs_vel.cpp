#include <iostream>
#include <chrono>
#include <string>
#include <cstdlib>
#include <signal.h>
#include <wiringPi.h>
#include "Encoder.h"
#include "CytronMotorDriver.h"

#define MILLI 1000
#define RAD_PER_ROUND 2*3.14159

#define STARTTIME 100   //millisecond
#define WAITTIME 200    //millisecond
#define LOOPTIME 300    //millisecond

Encoder encoder_L(0, 3);
Encoder encoder_R(2, 4);

CytronMD motor_l(23, 22);
CytronMD motor_r(26, 21);

const double radius = 0.152/2;          //Wheel radius, in m

const double encoder_cpr = 500;         //Encoder ticks or counts per rotation
const double gear_ratio = 100;          //gear rstio : 5500/50 = 110 but measure is 100
const double real_encoder_cpr = encoder_cpr*gear_ratio;
const double encoder_to_dist = (RAD_PER_ROUND*radius*MILLI)/(real_encoder_cpr);

int pwm_start = 0;
int pwm_step = 20;
int pwm_stop = 1024;

int dir = 1;
int PWM_Motor = pwm_start;

volatile int encoder_L_Count = 0;
volatile int encoder_R_Count = 0;

double speed_act_left = 0;
double speed_act_right = 0;

std::chrono::high_resolution_clock::time_point current_time, last_time;
double delta_time = 0;

enum time_stage {START, WAIT, COMPUTE, END, EXIT, DUMMY};
time_stage stage {START};

void read_Encoder_L() {
    encoder_L_Count++;
}
void read_Encoder_R() {
    encoder_R_Count++;
}

void signal_callback_handler(int signum) {
    std::cout << "Caught signal " << signum << std::endl;
    motor_l.setSpeed(0);
    motor_r.setSpeed(0);
    exit(signum);
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signal_callback_handler);

    if (argc > 1) {
        std::cout << "first arg : " << std::stoi(argv[1]) << std::endl
                  << "second arg : " << std::stoi(argv[2]) << std::endl
                  << "third arg : " << std::stoi(argv[3]) << std::endl;
        pwm_start = std::stoi(argv[1]);
        pwm_stop = std::stoi(argv[2]);
        pwm_step = std::stoi(argv[3]);
        PWM_Motor = pwm_start;
    } 
    else {
        std::cout << "use default" << std::endl;
    }
    
    std::cout << "pwm_start, pwm_stop, pwm_step" << std::endl
	      << pwm_start << ", " << pwm_stop << ", " << pwm_step << std::endl;

    if (wiringPiSetup() == -1)
    {
        std::cerr << "Failed to initialize WiringPi." << std::endl;
        return 1;
    }

    wiringPiISR(encoder_L, INT_EDGE_RISING, &read_Encoder_L);
    wiringPiISR(encoder_R, INT_EDGE_RISING, &read_Encoder_R);

    while (true)
    {
        if(stage == START) {
            stage = WAIT;
            std::cout << "Start experiment" << std::endl;
            std::cout << "PWM_Motor, speed_act_left, speed_act_right" << std::endl;
            motor_l.setSpeed(PWM_Motor);
            motor_r.setSpeed(PWM_Motor);
            delay(STARTTIME);
            last_time = std::chrono::high_resolution_clock::now();
        }
        
        else if(delta_time >= WAITTIME && stage == WAIT) {
            stage = COMPUTE;
            last_time = std::chrono::high_resolution_clock::now();

            encoder_L_Count = 0;
            encoder_R_Count = 0;
        }
        
        else if(delta_time >= LOOPTIME && stage == COMPUTE) {
            stage = WAIT;

            dir = abs(PWM_Motor)/PWM_Motor;
            speed_act_left= dir*(encoder_L_Count*encoder_to_dist)/delta_time;
            speed_act_right= dir*(encoder_R_Count*encoder_to_dist)/delta_time;

            last_time = std::chrono::high_resolution_clock::now();

            std::cout << PWM_Motor << ", " << speed_act_left << ", " << speed_act_right << std::endl;

            encoder_L_Count = 0;
            encoder_R_Count = 0;

            PWM_Motor += pwm_step;

            if (abs(PWM_Motor) > abs(pwm_stop)) {
                stage = END;
            }
            
            motor_l.setSpeed(PWM_Motor);
            motor_r.setSpeed(PWM_Motor);
        }

        else if (stage == END) {
            motor_l.setSpeed(0);
            motor_r.setSpeed(0);
            break;
        }

    current_time = std::chrono::high_resolution_clock::now();
    delta_time = std::chrono::duration<double, std::milli>(current_time-last_time).count();     //Time in ms

    }

    std::cout << "Exit" << std::endl;

    return 0;
}
