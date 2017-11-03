#include <ros.h>
#include <Arduino.h>
#include "EncoderReader.h"
#include "IMUReader.h"

ros::NodeHandle nh;
unsigned long timer;

const int delay_time = 1000 / ENCODER_FREQ;

#define ENCODER_PIN_A 8
#define ENCODER_PIN_B 9

//Motor A
const int motorPin1  = 5;  // Pin 14 of L293
const int motorPin2  = 6;  // Pin 10 of L293
//Motor B
const int  motorPin3  = 10; // Pin  7 of L293
const int  motorPin4  = 11;  // Pin  2 of L293

EncoderReader myEncoderReader(ENCODER_PIN_A, ENCODER_PIN_B);
IMUReader myIMUReader;

void readAndPublishVelocityHeading();

void setup()
{
  Serial.begin(57600);
  myIMUReader.realInit();
  nh.initNode();
  nh.advertise(myEncoderReader.get_publisher());
  nh.advertise(myIMUReader.get_publisher());

    //Set pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  digitalWrite(motorPin1, 0);
  digitalWrite(motorPin2, 1);
  digitalWrite(motorPin3, 1);
  digitalWrite(motorPin4, 0);
}

void loop()
{
  if ( (millis()-timer) > delay_time){
    readAndPublishVelocityHeading();
    timer =  millis();
  }
}

void readAndPublishVelocityHeading()
{
  myEncoderReader.update();
  myIMUReader.update();
  myEncoderReader.publish(nh);
  myIMUReader.publish(nh);
  nh.spinOnce();
}
