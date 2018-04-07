#ifndef ARDUINO_NODE_MOTOR_H
#define ARDUINO_NODE_MOTOR_H

#include "PID.h"
#include "Arduino.h"
#include "EncoderReader.h"
#include <Servo.h>

class Motor {
    short pinA;
    //short pinB; //Sabertooth only needs 1 pin per motor
public:
    PID pid;
    EncoderReader encoder;        
    Servo mot; //use servo.write to drive motors
    Motor(
        short motor_pinA,
        short encoder_pinA, short encoder_pinB,
        int Kp, int Ki, int Kd
    );
    void init();
    void go(float desiredSpeed, ros::NodeHandle &nh);
    void reset();
};

#endif //ARDUINO_NODE_MOTOR_H
