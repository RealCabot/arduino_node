#ifndef ARDUINO_NODE_SENSORREADER_H
#define ARDUINO_NODE_SENSORREADER_H

#include <ros.h>
#include <Arduino.h>

class SensorReader {
protected:
    ros::Publisher pub;
public:
    SensorReader(const char * topic_name, ros::Msg * msg)
        :pub(topic_name, msg)
    {}
    virtual void publish(ros::NodeHandle &nh)=0;
    ros::Publisher& get_publisher(){return pub;}
};


#endif //ARDUINO_NODE_SENSORREADER_H
