#define ENCODER_OPTIMIZE_INTERRUPTS

#include <ros.h>
#include <arduino_msg/Encoder.h>
#include <Arduino.h>
#include <Encoder.h>
#include <Timer.h>

ros::NodeHandle nh;
Timer t;

#define ENCODER_FREQ 5 // How many times in a second
#define RESET_THRESHOLD 2000000000 // Close to Arduino Long limit. It's actually 2,147,483,647
#define TICK_PER_METER 390000 //How many ticks does it take for the robot to go 1m

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

  float speed = float(newPosition - oldPosition) / TICK_PER_METER * ENCODER_FREQ; // m/s
  if (newPosition < RESET_THRESHOLD){
    oldPosition = newPosition;
  } else { // Reset the counter
    myEnc.write(0);
    oldPosition = 0;
  }
  Serial.println(speed);
  encoder_msg.speed = speed;
  encoder_msg.header.stamp = nh.now();
  pub.publish( &encoder_msg );
    
  nh.spinOnce();
}
