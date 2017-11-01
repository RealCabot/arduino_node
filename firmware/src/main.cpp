#include <ros.h>
#include <Arduino.h>
#include "EncoderReader.h"
#include "IMUReader.h"

ros::NodeHandle nh;
unsigned long timer;

const int delay_time = 1000 / ENCODER_FREQ;

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

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
