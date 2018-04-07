//
// Created by andrew on 1/20/18.
//

#ifndef ARDUINO_NODE_TOUCH_H
#define ARDUINO_NODE_TOUCH_H
#include <Wire.h>
#include "Adafruit_MPR121.h"
#include "SensorReader.h"
#include "std_msgs/Int16.h"

class Touch : public SensorReader{
    int16_t touchData;
    std_msgs::Int16 currTouched; //each of 12 channels are represented as 1 bit in message
    Adafruit_MPR121 cap = Adafruit_MPR121();

public:
    Touch();
    bool init();
    void publish(ros::NodeHandle &nh);
    bool getTouched(int pinNum);
    void reset();
};

#endif //ARDUINO_NODE_TOUCH_H