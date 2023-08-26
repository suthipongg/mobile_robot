#include "CytronMotorDriver.h"
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
 
//initializing all the variables
#define RAD_PER_ROUND 2*PI
#define LOOPTIME 100                       //Looptime in millisecond
#define MILLI 0.001
const byte noCommLoopMax = 10;            //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;             //main loop without communication counter

const int PIN_ENCOD_A_MOTOR_LEFT = 2;     //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 4;     //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;    //A channel for encoder of right motor        
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;    //B channel for encoder of right motor

const int PIN_SIDE_LIGHT_LED = 46;        //Side light blinking led pin

CytronMD motor1L(PWM_DIR, 6, 7);          // PWM 1A = Pin 3, PWM 1B = Pin 9.
CytronMD motor1R(PWM_DIR, 8, 9);          // PWM 2A = Pin 10, PWM 2B = Pin 11.

//--- Robot-specific constants ---
const double radius = 0.152/2;            //Wheel radius, in m
const double wheelbase = 0.35;            //Wheelbase, in m
const double encoder_cpr = 500;           //Encoder ticks or counts per rotation
const double gear_ratio = 5500/50;
const double max_speed = 0.28;             //Max speed in m/s
const double pwm_per_speed_left = 631.59;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_pwm_cmd_left = 1.6497;      //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).
const double pwm_per_speed_right = 608.62;
const double min_pwm_cmd_right = 3.4386;
const int encodertreshould = 5; 
const int MAX_PWM_LEFT = 180;
const int MAX_PWM_RIGHT = 180;

// PID Parameters
const double PID_left_param[] = { 0.45, 0.01, 0 };  //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0.45, 0.01, 0 }; //Respectively Kp, Ki and Kd for right motor PID
//--------------------------------

const double real_encoder_cpr = encoder_cpr*gear_ratio;
const double encoder_to_dist = (RAD_PER_ROUND*radius)/(MILLI*real_encoder_cpr);

double speed_req = 0;                     //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;             //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                //Desired speed for left wheel in m/s
double speed_act_left = 0;                //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                //Command speed for left wheel in m/s

double speed_req_right = 0;               //Desired speed for right wheel in m/s
double speed_act_right = 0;               //Actual speed for right wheel in m/s
double speed_cmd_right = 0;               //Command speed for right wheel in m/s
                       
int PWM_leftMotor = 0;                    //PWM command for left motor
int PWM_rightMotor = 0;                   //PWM command for right motor

unsigned long lastMilli = 0;
unsigned long delta_time = 0;

volatile float pos_left = 0;              //Left motor encoder position
volatile float pos_right = 0;             //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

const int lightIncNumber = 30;                                                                                                                                       //Number of lightIncrements for side light blinking
int lightInc = 0;                                                                                                                                                    //Init increment for side light blinking
int lightValue [lightIncNumber]= { 10, 40, 80, 160, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 160, 80, 40, 10, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
int lightValueNoComm [25]= { 255, 0, 255, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
int lightT = 0; //init light period

ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
 
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
 
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

//__________________________________________________________________________

void setup() {
  
  pinMode(PIN_SIDE_LIGHT_LED, OUTPUT);      //set pin for side light leds as output
  analogWrite(PIN_SIDE_LIGHT_LED, 255);     //light up side lights

  nh.getHardware()->setBaud(115200);
 
  nh.initNode();                            //init ROS node
  //nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic

  //setting motor speeds to zero
  motor1L.setSpeed(0);
  motor1R.setSpeed(0);
 
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
   
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  delta_time = millis()-lastMilli;
  if(delta_time >= LOOPTIME)  
  {                                                                           // enter timed loop
    lastMilli = millis();
   
//    if (!nh.connected()){
//      analogWrite(PIN_SIDE_LIGHT_LED, lightValueNoComm[lightInc]);
//      lightInc=lightInc+1;
//      if (lightInc >= 25){
//        lightInc=0;
//      }
//    }
//    else{
//      analogWrite(PIN_SIDE_LIGHT_LED, lightValue [lightInc]);
//      lightT = 3000 - ((2625/max_speed)*((abs(speed_req_left)+abs(speed_req_right))/2));
//      lightInc=lightInc+(30/(lightT/delta_time));
//      if (lightInc >= lightIncNumber){
//        lightInc=0;
//      }
//    }
   
    if (abs(pos_left) < encodertreshould || PWM_leftMotor == 0){    //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left = abs(PWM_leftMotor)/PWM_leftMotor * pos_left*encoder_to_dist/delta_time;           // calculate speed of left wheel
    }
   
    if (abs(pos_right) < encodertreshould || PWM_rightMotor == 0){   //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
      speed_act_right = abs(PWM_rightMotor)/PWM_rightMotor * pos_right*encoder_to_dist/delta_time;          // calculate speed of right wheel
    }
   
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(speed_req_left*pwm_per_speed_left + sgn(speed_req_left)*min_pwm_cmd_left + speed_cmd_left*pwm_per_speed_left, -MAX_PWM_LEFT, MAX_PWM_LEFT);

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(speed_req_right*pwm_per_speed_right + sgn(speed_req_right)*min_pwm_cmd_right + speed_cmd_right*pwm_per_speed_right, -MAX_PWM_RIGHT, MAX_PWM_RIGHT);

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      motor1L.setSpeed(0);
      motor1R.setSpeed(0);
    }
    else {                                                //Drive motor
      motor1L.setSpeed(PWM_leftMotor);
      motor1R.setSpeed(PWM_rightMotor);
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    if (noCommLoops < noCommLoopMax){
      noCommLoops++;
    }
   
    publishSpeed(delta_time);   //Publish odometry on ROS topic
  }
  delay(1);
 }

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Left motor encoder counter
void encoderLeftMotor() {
  pos_left++;
}

//Right motor encoder counter
void encoderRightMotor() {
  pos_right++;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
