#ifndef ARDUINO_NODE_IMUREADER_H
#define ARDUINO_NODE_IMUREADER_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "SensorReader.h"

class IMUReader : public SensorReader{
    Adafruit_BNO055 imu = Adafruit_BNO055(55);
    geometry_msgs::Vector3Stamped euler_msg;

public:
    ros::Publisher pub;
    IMUReader();
    void update();
    void publish(ros::NodeHandle &nh);
};


#endif //ARDUINO_NODE_IMUREADER_H
