#ifndef ARDUINO_NODE_SENSORREADER_H
#define ARDUINO_NODE_SENSORREADER_H

#include <ros.h>
#include <Arduino.h>

class SensorReader {
public:
    virtual void update()=0;
    virtual void publish(ros::NodeHandle &nh)=0;
};


#endif //ARDUINO_NODE_SENSORREADER_H
