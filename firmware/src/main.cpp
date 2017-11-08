#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "EncoderReader.h"
#include "IMUReader.h"
#include "PID.h"

ros::NodeHandle nh;
Timer t;

const int PID_DELAY = 1000 / PID_FREQ
const int SENSOR_DELAY = 1000 / ENCODER_FREQ;
#define HEARTBEAT_CYCLE 500

#define ENCODER_RIGHT_PIN_A 2
#define ENCODER_RIGHT_PIN_B 3
#define ENCODER_LEFT_PIN_A 18
#define ENCODER_LEFT_PIN_B 19
#define RMOTA 10
#define RMOTB 11
#define LMOTA 5
#define LMOTB 6
#define LED_PIN 13

float LOOPTIME= 10; 
unsigned long lastMilli = 0;
float Rspeed_req = .6;
float Rspeed_act = 0;
float Lspeed_req = .2;
float Lspeed_act = 0;
int PWM_valR = 0;
int PWM_valL = 0;

//PID
int RKp=30, RKi=5, RKd=10, LKp=50, LKi=2, LKd=30;

EncoderReader myREncoderReader(ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B);
EncoderReader myLEncoderReader(ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B);
PID rightPID(RKp, RKi, RKd);
PID leftPID(LKp, LKi, LKd);

void heartbeat();
void updateSensors();
void motorControl();

void setup()
{
  Serial.begin(57600);
  
  //myIMUReader.realInit();

  nh.initNode();

  nh.advertise(myREncoderReader.get_publisher());
  nh.advertise(myLEncoderReader.get_publisher());
  //nh.advertise(myIMUReader.get_publisher());

  //motor settings
  pinMode(RMOTA, OUTPUT);
  pinMode(RMOTB, OUTPUT);
  pinMode(LMOTA, OUTPUT);
  pinMode(LMOTB, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  t.every(SENSOR_DELAY, updateSensors);
  t.every(HEARTBEAT_CYCLE, heartbeat);
  t.every(1000/LOOPTIME, motorControl);
}

void loop()
{
  t.update();
}

void motorControl(){
  Rspeed_act = myREncoderReader.speed;
  Lspeed_act = myLEncoderReader.speed;
  PWM_valR = rightPID.update(Rspeed_req, Rspeed_act);
  PWM_valL = leftPID.update(Lspeed_req, Lspeed_act);
  if (Rspeed_req < 0){
    analogWrite(RMOTA, 0);
    analogWrite(RMOTB, PWM_valR);
  }

  if(Rspeed_req > 0){
    analogWrite(RMOTA, PWM_valR);
    analogWrite(RMOTB, 0);
  }

  if (Lspeed_req < 0){
    analogWrite(LMOTA, PWM_valL);
    analogWrite(LMOTB, 0);
  }
  if(Lspeed_req > 0){

    analogWrite(LMOTA, 0);
    analogWrite(LMOTB, PWM_valL);
  }
}

void updateSensors()
{
  myREncoderReader.update();
  myLEncoderReader.update();
  //myIMUReader.update();
  myREncoderReader.publish(nh);
  myLEncoderReader.publish(nh);
  //myIMUReader.publish(nh);
  nh.spinOnce();
}

void heartbeat(){
  static int status = HIGH;
  digitalWrite(LED_PIN, status);
  status = (status== HIGH)? LOW:HIGH;
}
