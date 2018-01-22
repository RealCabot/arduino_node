#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "IMUReader.h"
#include "Motor.h"
#include <arduino_msg/Motor.h>
#include <geometry_msgs/Vector3.h>
#include "Touch.h"
ros::NodeHandle nh;
Timer t;

const int PID_DELAY = 1000 / PID_FREQ;
const int SENSOR_DELAY = 1000 / ENCODER_FREQ;
const int MOTOR_DELAY = 200; //used in motor timeout

#define MOTOR_TIMEOUT 5000  //milliseconds

#define HEARTBEAT_CYCLE 500

#define ENCODER_RIGHT_PIN_A 18
#define ENCODER_RIGHT_PIN_B 19
#define ENCODER_LEFT_PIN_A  2
#define ENCODER_LEFT_PIN_B  3
#define MOTOR_RIGHT_PIN_A  6
#define MOTOR_RIGHT_PIN_B  5
#define MOTOR_LEFT_PIN_A   10
#define MOTOR_LEFT_PIN_B   11
#define LED_PIN 13

float speed_req_R = 0;
float speed_req_L = 0;
long motorUpdateTime = 0;
//PID coefficients
const int RKp=35, RKi=16, RKd=15, LKp=35, LKi=16, LKd=15;

Motor motor_L(
  MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B,
  ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B,
  LKp, LKi, LKd
);
Motor motor_R(
  MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B,
  ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B,
  RKp, RKi, RKd
);

IMUReader myIMUReader;
Touch touchReader;  //MPR121 touch sensor
void heartbeat();
void updateSensors();
void motorControl();
void encoders_publish();
void setMotorSpeed(const arduino_msg::Motor& speed_msg);
void setPIDParam(const geometry_msgs::Vector3& pid_param_msg);
void motorTimeout();

arduino_msg::Motor speed_msg;
ros::Publisher encoder_publisher("encoder", &speed_msg);
ros::Subscriber<arduino_msg::Motor> motor_speed_sub("motorSpeed", &setMotorSpeed);
ros::Subscriber<geometry_msgs::Vector3> pid_param_sub("tunePID", &setPIDParam);


void setup()
{
  Serial.begin(57600);

  nh.initNode();

  nh.advertise(encoder_publisher);
  nh.advertise(myIMUReader.get_publisher());
    nh.advertise(touchReader.get_publisher());
  nh.subscribe(motor_speed_sub);
  nh.subscribe(pid_param_sub);
  //motor settings
  pinMode(MOTOR_RIGHT_PIN_A, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_B, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_A, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_B, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  while(!nh.connected()) {nh.spinOnce();}

  float imu_offset = 180; //Default value
  if (! nh.getParam("~imu_offset", &imu_offset)){ 
    nh.logwarn("IMU offset not set. Using default value 180.");
  }
  myIMUReader.realInit(imu_offset);
  if (touchReader.init() == -1){
      nh.logwarn("Error initializing touch sensor (MPR121) - is it wired correctly?");
  }
  t.every(SENSOR_DELAY, updateSensors);
  t.every(HEARTBEAT_CYCLE, heartbeat);
  t.every(PID_DELAY, motorControl);
  t.every(MOTOR_DELAY, motorTimeout);

}

void loop()
{
  nh.spinOnce();  
  t.update();
}

void motorControl(){
  motor_L.go(speed_req_L);
  motor_R.go(speed_req_R);
}

void updateSensors()
{
  motor_L.encoder.update();
  motor_R.encoder.update();
  myIMUReader.update();
    //touchReader.update();
  encoders_publish();
  myIMUReader.publish(nh);
  //touchReader.publish(nh);
  nh.spinOnce();
}

 // blink LED periodically to indicate everything's gonna be alright
void heartbeat(){
  static int status = HIGH;
  digitalWrite(LED_PIN, status);
  status = (status== HIGH)? LOW:HIGH;
}

 // publishes the left and right encoder speeds
void encoders_publish(){ 
  speed_msg.left_speed = motor_L.encoder.speed;
  speed_msg.right_speed = motor_R.encoder.speed;
  speed_msg.header.stamp = nh.now();
  encoder_publisher.publish( &speed_msg );
}

 // this callback sets the speed of the left and right motors
 // subscribes to rostopic "motorSpeed"
void setMotorSpeed(const arduino_msg::Motor& speed_msg){
	//constrain motor speeds
  speed_req_L = speed_msg.left_speed;
  speed_req_R = speed_msg.right_speed;
  motorUpdateTime = millis();  //record last time motors received speeds

}

 // stop motors if it has been a while since receiving a motor command
void motorTimeout(){
  if ((millis() - motorUpdateTime) > MOTOR_TIMEOUT){
    speed_req_R = 0;
    speed_req_L = 0;
  }
}

 // this callback sets PID coefficient. ONLY USED IN TUNING PARAMETERS
void setPIDParam(const geometry_msgs::Vector3& pid_param_msg){
  // X = proportional term, Y = integral term, Z = derivative term
  motor_L.pid.setParam(pid_param_msg.x, pid_param_msg.y, pid_param_msg.z);
  motor_R.pid.setParam(pid_param_msg.x, pid_param_msg.y, pid_param_msg.z);
}