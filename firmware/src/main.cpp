#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "EncoderReader.h"
#include "IMUReader.h"

ros::NodeHandle nh;
Timer t;

const int delay_time = 1000 / ENCODER_FREQ;

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

EncoderReader myEncoderReader(ENCODER_PIN_A, ENCODER_PIN_B);
IMUReader myIMUReader;

void readAndPublishVelocityHeading();

void setup()
{
  Serial.begin(9600);
  // myIMUReader.realInit();
  nh.initNode();
  nh.advertise(myEncoderReader.get_publisher());
  // nh.advertise(myIMUReader.get_publisher());
  t.every(delay_time, readAndPublishVelocityHeading);
}

void loop()
{
  t.update();
}

void readAndPublishVelocityHeading()
{
  myEncoderReader.update();
  // myIMUReader.update();
  myEncoderReader.publish(nh);
  // myIMUReader.publish(nh);
  nh.spinOnce();
}
