
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

const double radius = 0.152/2;

const double encoder_cpr = 500;
const double gear_ratio = 5500/50;

const int MAX_PWM = -255;
const int step_PWM = -5;
const int MIN_PWM = -210;
// ====================================
const double real_encoder_cpr = encoder_cpr*gear_ratio;
const double encoder_to_vel = (1/real_encoder_cpr)*RAD_PER_ROUND*radius*MILLI;

unsigned long lastMilli = 0;
unsigned long delta_time = 0;

volatile long pos_left = 0;       //Left motor encoder position
volatile long pos_right = 0;      //Right motor encoder position

int dir = 1;
int PWM_Motor = MIN_PWM;

double ang_speed_act_left = 0;
double ang_speed_act_right = 0;

enum time_stage {START, WAIT, COMPUTE, END, EXIT, DUMMY};
time_stage stage {START};

void setup() {
  Serial.begin(9600);
  Serial.println("Start experiment");

  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
}

void loop() {
  delta_time = millis()-lastMilli;
  
  if(stage == START) {
    stage = WAIT;
    Serial.print("PWM_Motor");
    Serial.print(", ang_speed_act_left");
    Serial.print(", ang_speed_act_right");
    motor1L.setSpeed(PWM_Motor);
    motor1R.setSpeed(PWM_Motor);
    delay(STARTTIME);
    lastMilli = millis();
    Serial.println("PWM, posleft, posright");
  }
  
  else if(delta_time >= WAITTIME && stage == WAIT) {
    stage = COMPUTE;
    lastMilli = millis();

    pos_left = 0;
    pos_right = 0;
  }
  
  else if(delta_time >= LOOPTIME && stage == COMPUTE) {

    stage = WAIT;

    dir = abs(PWM_Motor)/PWM_Motor;
    ang_speed_act_left= dir*(pos_left*encoder_to_vel)/delta_time;
    ang_speed_act_right= dir*(pos_right*encoder_to_vel)/delta_time;

    lastMilli = millis();

    Serial.print(PWM_Motor);
    Serial.print(", ");
    Serial.print(ang_speed_act_left, 6);
    Serial.print(", ");
    Serial.println(ang_speed_act_right, 6);
    
    pos_left = 0;
    pos_right = 0;

    PWM_Motor += step_PWM;

    if (abs(PWM_Motor) > abs(MAX_PWM)) {
      stage = END;
    }
    
    motor1L.setSpeed(PWM_Motor);
    motor1R.setSpeed(PWM_Motor);
    
  }

  else if (stage == END) {
    Serial.println("Success!");
    stage = EXIT;
    motor1L.setSpeed(0);
    motor1R.setSpeed(0);
  }
}

void encoderLeftMotor() {
  pos_left++;
}

void encoderRightMotor() {
  pos_right++;
}
