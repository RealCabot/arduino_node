#include <ros.h>
#include <arduino_msg/Encoder.h>

#include <Arduino.h>
#include <Encoder.h>
#include <Timer.h>

ros::NodeHandle nh;
Timer t;

#define ENCODER_FREQ 5 //How many times in a second

const int delay_time = 1000 / ENCODER_FREQ;
// Set up ROS publishers
arduino_msg::Encoder encoder_msg;
ros::Publisher pub("encoder", &encoder_msg);

// Tell Arduino Encoder library which code to use
Encoder myEnc(2, 3);
long oldPosition  = -999;

void updateEncoderReading();

void setup()
{
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  nh.initNode();
  nh.advertise(pub);
  t.every(delay_time, updateEncoderReading);
}

void loop()
{
  t.update();
}

void updateEncoderReading()
{
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
    encoder_msg.left = newPosition;
    encoder_msg.right = 0;
    encoder_msg.header.stamp = nh.now();
    pub.publish( &encoder_msg );
  }
  nh.spinOnce();
}
