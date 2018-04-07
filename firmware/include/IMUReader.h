#ifndef ARDUINO_NODE_IMUREADER_H
#define ARDUINO_NODE_IMUREADER_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "SensorReader.h"

class IMUReader : public SensorReader{
    Adafruit_BNO055 imu_ = Adafruit_BNO055(55);
    geometry_msgs::Vector3Stamped euler_msg_;
    float offset_ = 0;
public:
    IMUReader();
    void realInit();
    void update();
    void publish(ros::NodeHandle &nh);
    void reset();
};


#endif //ARDUINO_NODE_IMUREADER_H
