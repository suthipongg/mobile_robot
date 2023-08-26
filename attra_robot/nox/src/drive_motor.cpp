#include "CytronMotorDriver.h"
#include "Encoder.h"
#include "PID_v1.h"
#include <wiringPi.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

//initializing all the variables
#define RAD_PER_ROUND 2*3.14159
#define MILLI 1000

CytronMD motor_L(23, 22);               //PWM_PIN, DIR_PIN
CytronMD motor_R(26, 21);               //PWM_PIN, DIR_PIN

Encoder encoder_L(0, 3);                //A, B channel for encoder of left motor
Encoder encoder_R(2, 4);                //A, B channel for encoder of right motor

//--- Robot-specific constants ---
const double radius = 0.152/2;          //Wheel radius, in m
const double wheelbase = 0.35;          //Wheelbase, in m
const double encoder_cpr = 500;         //Encoder ticks or counts per rotation
const double gear_ratio = 100;          //gear rstio : 5500/50 = 110 but measure is 100

double max_speed = 0.4;          //Max speed in m/s
const int MAX_PWM_LEFT = 1024;
const int MAX_PWM_RIGHT = 1024;
const int encodertreshould = 5;

//Ratio to convert speed (in m/s) to PWM value. 
//It was obtained by plotting the wheel speed in relation to the PWM motor command.
const int pwm_per_speed_left = 2027;    //the value is the slope of the linear function.
const int min_pwm_cmd_left = 114;       //the value is the constant of the linear function.
const int pwm_per_speed_right = 1930;
const int min_pwm_cmd_right = 118;

// PID Parameters
double PID_left_param[] = {0.45, 0.01, 0};  //Respectively Kp, Ki and Kd for left motor PID
double PID_right_param[] = {0.45, 0.01, 0}; //Respectively Kp, Ki and Kd for right motor PID
int SampleTime_left = 95;
int SampleTime_right = 95;
//--------------------------------

const double real_encoder_cpr = encoder_cpr*gear_ratio;
const double encoder_to_dist = (RAD_PER_ROUND*radius*MILLI)/(real_encoder_cpr);

volatile float pos_left = 0;        //Left motor encoder position
volatile float pos_right = 0;       //Right motor encoder position

double speed_req = 0;               //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;       //Desired angular speed for the robot, in rad/s

double speed_act_left = 0;          //Actual speed for left wheel in m/s
double speed_cmd_left = 0;          //Command speed for left wheel in m/s
double speed_req_left = 0;          //Desired speed for left wheel in m/s

double speed_act_right = 0;         //Actual speed for right wheel in m/s
double speed_cmd_right = 0;         //Command speed for right wheel in m/s
double speed_req_right = 0;         //Desired speed for right wheel in m/s

int PWM_leftMotor = 0;              //PWM command for left motor
int PWM_rightMotor = 0;             //PWM command for right motor

const int noCommLoopMax = 10;       //number of main loops will execute without communication before stop
unsigned int noCommLoops = 0;       //main loop without communication counter


void encoderLeftMotor();
void encoderRightMotor();
double compute_speed(int& PWM, volatile float& tick, double& time);
int speed2pwm(double& vel_req, double& vel_cmd, 
            const int pwm_per_speed, const int min_pwm_cmd, const int MAX_PWM);
void comput_time();
void check_small_disturbances(volatile float& pos, int& pwm, double& speed_act);

template<typename T> T constrain(const T val, const T min, const T max);
template<typename T> int sgn(T val);

void handle_cmd(const geometry_msgs::Twist& cmd_vel);
void publish_vel(ros::Publisher& speed_pub);

// ros config
double rate = 10.0;
ros::Time ros_current_time;
std::chrono::high_resolution_clock::time_point current_time, last_time;
double delta_time = 0;
geometry_msgs::Vector3Stamped speed_msg;

int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_motor");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    ros::Subscriber cmd_vel = n.subscribe("cmd_vel", 50, handle_cmd);
    ros::Publisher speed_pub = n.advertise<geometry_msgs::Vector3Stamped>("speed", 10);
    nh_private_.getParam("publish_rate", rate);
    nh_private_.getParam("SampleTime_left", SampleTime_left);
    nh_private_.getParam("SampleTime_right", SampleTime_right);
    nh_private_.getParam("max_speed", max_speed);
    nh_private_.getParam("kp_l", PID_left_param[0]);
    nh_private_.getParam("ki_l", PID_left_param[1]);
    nh_private_.getParam("kd_l", PID_left_param[2]);
    nh_private_.getParam("kp_r", PID_right_param[0]);
    nh_private_.getParam("ki_r", PID_right_param[1]);
    nh_private_.getParam("kd_r", PID_right_param[2]);
    
    ros::Rate loop_rate(rate);

    PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, 
                    PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          
    PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, 
                    PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   
    PID_leftMotor.SetSampleTime(SampleTime_left);
    PID_rightMotor.SetSampleTime(SampleTime_right);
    PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
    PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
    PID_leftMotor.SetMode(AUTOMATIC);
    PID_rightMotor.SetMode(AUTOMATIC);

    if (wiringPiSetup() == -1)
    {
        std::cerr << "=========== Failed to initialize WiringPi. ===========" << std::endl;
        return 1;
    }

    wiringPiISR(encoder_L, INT_EDGE_RISING, &encoderLeftMotor);
    wiringPiISR(encoder_R, INT_EDGE_RISING, &encoderRightMotor);

    last_time = std::chrono::high_resolution_clock::now();

    while(n.ok()) {
        ros::spinOnce();
        //Stopping if too much time without command
        if (noCommLoops >= noCommLoopMax) { 
            speed_req_left = speed_req_right = 0;
        }
        else {
            noCommLoops++;
        }

        comput_time();
        
        //Avoid taking in account small disturbances
        check_small_disturbances(pos_left, PWM_leftMotor, speed_act_left);
        check_small_disturbances(pos_right, PWM_rightMotor, speed_act_right);

        speed_cmd_left = constrain<double>(speed_cmd_left, -max_speed, max_speed);
        PID_leftMotor.Compute();
        PWM_leftMotor = speed2pwm(speed_req_left, speed_cmd_left, 
                                pwm_per_speed_left, min_pwm_cmd_left, MAX_PWM_LEFT);

        speed_cmd_right = constrain<double>(speed_cmd_right, -max_speed, max_speed);
        PID_rightMotor.Compute();
        PWM_rightMotor = speed2pwm(speed_req_right, speed_cmd_right, 
                                pwm_per_speed_right, min_pwm_cmd_right, MAX_PWM_RIGHT);
        
        motor_L.setSpeed(PWM_leftMotor);
        motor_R.setSpeed(PWM_rightMotor);
	
	publish_vel(speed_pub);
	loop_rate.sleep();
    }
}


void handle_cmd(const geometry_msgs::Twist& cmd_vel) {
    noCommLoops = 0;                        //Reset the counter for number of main loops without communication

    speed_req = cmd_vel.linear.x;           //Extract the commanded linear speed from the message
    angular_speed_req = cmd_vel.angular.z;  //Extract the commanded angular speed from the message
    //Calculate the required speed for the motor to comply with commanded linear and angular speeds
    speed_req_left = constrain<double>(speed_req - angular_speed_req*(wheelbase/2), -max_speed, max_speed);
    speed_req_right = constrain<double>(speed_req + angular_speed_req*(wheelbase/2), -max_speed, max_speed);
}

void publish_vel(ros::Publisher& speed_pub) {
    speed_msg.header.stamp = ros::Time::now();      //timestamp for odometry data
    speed_msg.vector.x = speed_act_left;            //left wheel speed (in m/s)
    speed_msg.vector.y = speed_act_right;           //right wheel speed (in m/s)
    speed_msg.vector.z = delta_time/MILLI;
    speed_pub.publish(speed_msg);
}

void encoderLeftMotor() {pos_left++;}
void encoderRightMotor() {pos_right++;}

template<typename T> 
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template<typename T>
T constrain(const T val, const T min, const T max) {
    if(val < min) {
        return min;
    }
    else if(max < val) {
        return max;
    }
    else
        return val;
}

double compute_speed(int& PWM, volatile float& tick, double& time) {
    return sgn(PWM) * (tick*encoder_to_dist/time);
}

int speed2pwm(double& vel_req, double& vel_cmd, 
            const int pwm_per_speed, const int min_pwm_cmd, const int MAX_PWM) {
    return constrain<int>(vel_req*pwm_per_speed + sgn(vel_req)*min_pwm_cmd + vel_cmd*pwm_per_speed, 
                        -MAX_PWM, MAX_PWM);
}

void comput_time() {
    current_time = std::chrono::high_resolution_clock::now();
    delta_time = std::chrono::duration<double, std::milli>(current_time-last_time).count(); //Time in ms
    last_time = current_time;
}

void check_small_disturbances(volatile float& pos, int& pwm, double& speed_act) {
    if (abs(pos) < encodertreshould || pwm == 0){    
        speed_act = 0;
    }
    else {
        // calculate speed of wheel
        speed_act = compute_speed(pwm, pos, delta_time);
    }
    pos = 0;
}
