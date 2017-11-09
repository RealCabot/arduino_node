#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "IMUReader.h"
#include "Motor.h"
#include <arduino_msg/Motor.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;
Timer t;

const int PID_DELAY = 1000 / PID_FREQ;
const int SENSOR_DELAY = 1000 / ENCODER_FREQ;

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

float speed_req_R = 0.15;
float speed_req_L = 0.15;

//PID
const int RKp=35, RKi=5, RKd=10, LKp=35, LKi=5, LKd=10;

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

void heartbeat();
void updateSensors();
void motorControl();
void encoders_publish();
void setMotorSpeed(const arduino_msg::Motor& speed_msg);
void setPIDParam(const geometry_msgs::Vector3& pid_param_msg);

arduino_msg::Motor speed_msg;
ros::Publisher encoder_publisher("encoder", &speed_msg);
ros::Subscriber<arduino_msg::Motor> motor_speed_sub("motorSpeed", &setMotorSpeed);
ros::Subscriber<geometry_msgs::Vector3> pid_param_sub("tunePID", &setPIDParam);

void setup()
{
  Serial.begin(57600);
  
  // myIMUReader.realInit();

  nh.initNode();

  nh.advertise(encoder_publisher);
  // nh.advertise(myIMUReader.get_publisher());
  nh.subscribe(motor_speed_sub);
  nh.subscribe(pid_param_sub);
  //motor settings
  pinMode(MOTOR_RIGHT_PIN_A, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_B, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_A, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_B, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  t.every(SENSOR_DELAY, updateSensors);
  t.every(HEARTBEAT_CYCLE, heartbeat);
  t.every(PID_DELAY, motorControl);
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
  // myIMUReader.update();
  encoders_publish();
  // myIMUReader.publish(nh);
  nh.spinOnce();
}

void heartbeat(){
  static int status = HIGH;
  digitalWrite(LED_PIN, status);
  status = (status== HIGH)? LOW:HIGH;
}

void encoders_publish(){ 
  speed_msg.left_speed = motor_L.encoder.speed;
  speed_msg.right_speed = motor_R.encoder.speed;
  speed_msg.header.stamp = nh.now();
  encoder_publisher.publish( &speed_msg );
}

void setMotorSpeed(const arduino_msg::Motor& speed_msg){
  speed_req_L = speed_msg.left_speed;
  speed_req_R = speed_msg.right_speed;
}

void setPIDParam(const geometry_msgs::Vector3& pid_param_msg){
  motor_L.pid.setParam(pid_param_msg.x, pid_param_msg.y, pid_param_msg.z);
  motor_R.pid.setParam(pid_param_msg.x, pid_param_msg.y, pid_param_msg.z);
}