#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "IMUReader.h"
#include "Motor.h"
#include <arduino_msg/Motor.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "Touch.h"
ros::NodeHandle nh;
Timer t;

const int PID_DELAY = 1000 / PID_FREQ;
const int SENSOR_DELAY = 1000 / ENCODER_FREQ;
const int MOTOR_DELAY = 200; //used in motor timeout
const int TOUCH_PIN = 5; //pin that touch pad is connected to on MPR121

#define MOTOR_TIMEOUT 5000  //milliseconds

#define HEARTBEAT_CYCLE 500

#define ENCODER_RIGHT_PIN_A 18
#define ENCODER_RIGHT_PIN_B 19
#define ENCODER_LEFT_PIN_A  2
#define ENCODER_LEFT_PIN_B  3
//#define MOTOR_RIGHT_PIN_A  6 
#define MOTOR_RIGHT  6 // changed for Sabertooth motordriver
// #define MOTOR_LEFT_PIN_A   10
#define MOTOR_LEFT   5  //originally 11, changed for Sabertooth motordriver
#define LED_PIN 13
#define WHEELS_SEPERATION 0.125 //in meters

int isTeleoped; // 1 if is remote control, 0 otherwise 
bool touchPresent = false;  //true if MPR121 is detected
bool prevTouched = false;
bool canGo = false;
float speed_req_R = 0;
float speed_req_L = 0;
long motorUpdateTime = 0;
//PID coefficients
const int RKp=12, RKi=6, RKd=7, LKp=12, LKi=6, LKd=7;

Motor motor_L(
  MOTOR_LEFT,
  ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B,
  LKp, LKi, LKd
);
Motor motor_R(
  MOTOR_RIGHT,
  ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B,
  RKp, RKi, RKd
);

IMUReader myIMUReader;
Touch touchReader;  //MPR121 touch sensor
void heartbeat();
void updateSensors();
void motorControl();
void encoders_publish();
void setMotorSpeed(const geometry_msgs::Twist& twist_msg);
void setPIDParam(const geometry_msgs::Vector3& pid_param_msg);
void checkMotors();

arduino_msg::Motor speed_msg;
ros::Publisher encoder_publisher("encoder", &speed_msg);
//ros::Subscriber<arduino_msg::Motor> motor_speed_sub("motorSpeed", &setMotorSpeed);
ros::Subscriber<geometry_msgs::Twist> motor_twist_sub("cmd_vel", &setMotorSpeed);
ros::Subscriber<geometry_msgs::Vector3> pid_param_sub("tunePID", &setPIDParam);


void setup()
{
  Serial.begin(57600);

  nh.initNode();

  nh.advertise(encoder_publisher);
  nh.advertise(myIMUReader.get_publisher());
  nh.subscribe(motor_twist_sub);
  nh.subscribe(pid_param_sub);
  //motor settings
  // pinMode(MOTOR_RIGHT_PIN_A, OUTPUT);
  // pinMode(MOTOR_RIGHT, OUTPUT);
  // pinMode(MOTOR_LEFT_PIN_A, OUTPUT);
  // pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  motor_L.init();
  motor_R.init();

  while(!nh.connected()) {nh.spinOnce();}

  if (! nh.getParam("~isTeleoped", &isTeleoped)){ 
      //default values
      isTeleoped = 0; 
  }

  myIMUReader.realInit();

  touchPresent = touchReader.init();
  if (!touchPresent){
      nh.logerror("Error initializing touch sensor (MPR121) - is it wired correctly?");
  }
  else{ //success, so initialize publisher
    //nh.advertise(touchReader.get_publisher());
  }
  t.every(SENSOR_DELAY, updateSensors);
  t.every(HEARTBEAT_CYCLE, heartbeat);
  t.every(PID_DELAY, motorControl);
  t.every(MOTOR_DELAY, checkMotors);

}

void loop()
{
  nh.spinOnce();  
  t.update();
}

void motorControl(){
  motor_L.go(speed_req_L, nh);
  motor_R.go(speed_req_R, nh);
}

void updateSensors()
{
  motor_L.encoder.update();
  motor_R.encoder.update();
  myIMUReader.update();
  nh.spinOnce();
  encoders_publish();
  myIMUReader.publish(nh);
//  if (touchPresent){
//    touchReader.publish(nh);
//  }
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
void setMotorSpeed(const geometry_msgs::Twist& twist_msg){
  //constrain motor speeds, and only change speed if user is touching handle, and motor commands haven't timed out
  if (canGo){
    speed_req_L = twist_msg.linear.x - (twist_msg.angular.z * WHEELS_SEPERATION/2);
    speed_req_R = twist_msg.linear.x + (twist_msg.angular.z * WHEELS_SEPERATION/2);
  }
  // char logStr[40];
  // sprintf (logStr, "Set Motor Speed: %d, %d", (int)(speed_req_L*100), (int)(speed_req_R*100));
  // nh.loginfo(logStr);

  motorUpdateTime = millis();  //record last time motors received speeds
}

//stop CaBot if user lets go of handle or if Arduino hasn't received command in a while
void checkMotors(){
  bool isTouched = (touchPresent) ? touchReader.getTouched(TOUCH_PIN) : false;   //is user touching handle?
  bool timeOut = (millis() - motorUpdateTime) > MOTOR_TIMEOUT; //have motors received command recently?

  canGo = (!isTeleoped && isTouched && !timeOut) || isTeleoped;
  // nh.loginfo(canGo?"Cabot can go":"Cabot CANNOT go");

  if (!canGo){  //we should stop CaBot
    speed_req_R = 0;
    speed_req_L = 0;
  }

  if (touchPresent && prevTouched!=isTouched){
    if (isTouched){
      nh.loginfo("TOUCHED");
    }
    else{
      nh.loginfo("RELEASED");
    }
  }

  prevTouched = isTouched;
}

 // this callback sets PID coefficient. ONLY USED IN TUNING PARAMETERS
void setPIDParam(const geometry_msgs::Vector3& pid_param_msg){
  // X = proportional term, Y = integral term, Z = derivative term
  motor_L.pid.setParam(pid_param_msg.x, pid_param_msg.y, pid_param_msg.z);
  motor_R.pid.setParam(pid_param_msg.x, pid_param_msg.y, pid_param_msg.z);
}