#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "EncoderReader.h"
#include "IMUReader.h"

ros::NodeHandle nh;
Timer t;

const int delay_time = 1000 / ENCODER_FREQ;
#define HEARTBEAT_CYCLE 500

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
#define LED_PIN 13

EncoderReader myEncoderReader(ENCODER_PIN_A, ENCODER_PIN_B);
// IMUReader myIMUReader;

void heartbeat();
void readAndPublishVelocityHeading();

void setup()
{
  Serial.begin(57600);
  pinMode(LED_PIN, OUTPUT);
  // myIMUReader.realInit();
  nh.initNode();
  nh.advertise(myEncoderReader.get_publisher());
  // nh.advertise(myIMUReader.get_publisher());
  t.every(delay_time, readAndPublishVelocityHeading);
  t.every(HEARTBEAT_CYCLE, heartbeat);
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

void heartbeat(){
  static int status = HIGH;
  digitalWrite(LED_PIN, status);
  Serial.println(status);
  status = (status== HIGH)? LOW:HIGH;
}