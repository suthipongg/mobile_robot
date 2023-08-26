
#include "CytronMotorDriver.h"
#include <Wire.h>

#define MILLI 1000
#define RAD_PER_ROUND 2*PI

#define STARTTIME 100
#define WAITTIME 100
#define LOOPTIME 100

CytronMD motor1L(PWM_DIR, 6, 7);   // PWM 1A = Pin 3, PWM 1B = Pin 9.
CytronMD motor1R(PWM_DIR, 8, 9); // PWM 2A = Pin 10, PWM 2B = Pin 11.

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor        
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;              //B channel for encoder of right motor

unsigned long lastMilli = 0;
unsigned long delta_time = 0;

volatile long pos_left = 0;       //Left motor encoder position
volatile long pos_right = 0;      //Right motor encoder position


double ang_speed_act_left = 0;
double ang_speed_act_right = 0;

enum time_stage {START, WAIT, COMPUTE, END, EXIT, DUMMY};
time_stage stage {START};

long last_pos_left = 0;
long last_pos_right = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start experiment");

  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT_PULLUP);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT_PULLUP);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (pos_left != last_pos_left) {
    Serial.print("encoder L : ");
    Serial.println(pos_left);
    last_pos_left = pos_left;
  }

  if (pos_right != last_pos_right) {
    Serial.print("encoder L : ");
    Serial.println(pos_right);
    last_pos_right = pos_right;
  }
}

void encoderLeftMotor() {
  pos_left++;
}

void encoderRightMotor() {
  pos_right++;
}
