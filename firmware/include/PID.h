//
// Created by andrew on 11/7/17.
//

#ifndef ARDUINO_NODE_PID_H
#define ARDUINO_NODE_PID_H

#include <Arduino.h>

#define PID_FREQ 100

class PID {
    int Kp, Ki, Kd;
    float integral_ = 0;
    float lastError_ = 0;
public:
    PID(int Kp, int Ki, int Kd);
    int getPWM(float desiredSpeed, float currSpeed);
    void setParam(int Kp, int Ki, int Kd);
    void reset();
};


#endif //ARDUINO_NODE_PID_H
