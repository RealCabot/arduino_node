#ifndef ARDUINO_NODE_SENSORREADER_H
#define ARDUINO_NODE_SENSORREADER_H

#include <ros.h>
#include <Arduino.h>

#define ENCODER_FREQ 20 // How many times in a second

class SensorReader {
protected:
    ros::Publisher pub;
public:
    SensorReader(const char * topic_name, ros::Msg * msg)
        :pub(topic_name, msg)
    {}
    virtual void publish(ros::NodeHandle &nh)=0;
    virtual void reset()=0;
    ros::Publisher& get_publisher(){return pub;}
};


#endif //ARDUINO_NODE_SENSORREADER_H
