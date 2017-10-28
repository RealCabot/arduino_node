#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "EncoderReader.h"

ros::NodeHandle nh;
Timer t;

const int delay_time = 1000 / ENCODER_FREQ;

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

EncoderReader myEncoderReader(ENCODER_PIN_A, ENCODER_PIN_B);
void readAndPublishVelocity();

void setup()
{
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(myEncoderReader.velocity_pub);
  t.every(delay_time, readAndPublishVelocity);
}

void loop()
{
  t.update();
}

void readAndPublishVelocity()
{
  myEncoderReader.update();
  myEncoderReader.publish(nh);
  nh.spinOnce();
}
