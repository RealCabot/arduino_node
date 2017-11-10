#ifndef ARDUINO_NODE_MOTOR_H
#define ARDUINO_NODE_MOTOR_H

#include "PID.h"
#include "Arduino.h"
#include "EncoderReader.h"

class Motor {
    short pinA;
    short pinB;
public:
    PID pid;
    EncoderReader encoder;        
    Motor(
        short motor_pinA, short motor_pinB,
        short encoder_pinA, short encoder_pinB,
        int Kp, int Ki, int Kd
    );
    void go(float desiredSpeed);
};

#endif //ARDUINO_NODE_MOTOR_H