//
// Created by andrew on 11/7/17.
//

#ifndef ARDUINO_NODE_PID_H
#define ARDUINO_NODE_PID_H

#define PID_FREQ 10

class PID {
    int Kp, Ki, Kd;
public:
    PID(int Kp, int Ki, int Kd);
    int getPWM(float desiredSpeed, float currSpeed);
};


#endif //ARDUINO_NODE_PID_H
