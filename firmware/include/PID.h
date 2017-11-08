//
// Created by andrew on 11/7/17.
//

#ifndef ARDUINO_NODE_PID_H
#define ARDUINO_NODE_PID_H


class PID {
    int Kp, Ki, Kd;

public:
    PID(int, int, int);
    update(float, float);
};


#endif //ARDUINO_NODE_PID_H
