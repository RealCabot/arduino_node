#ifndef ARDUINO_NODE_MOTOR_H
#define ARDUINO_NODE_MOTOR_H

#include "PID.h"
#include "Arduino.h"
#include "EncoderReader.h"
#include <Servo.h>

class Motor {
    short pinA;
    short pinB;
public:
    PID pid;
    EncoderReader encoder;        
    Servo mot;
    Motor(
        short motor_pinA, short motor_pinB,
        short encoder_pinA, short encoder_pinB,
        int Kp, int Ki, int Kd
    );
    void go(float desiredSpeed, ros::NodeHandle &nh);
};

#endif //ARDUINO_NODE_MOTOR_H
