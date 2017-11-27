#include "IMUReader.h"

static float angle_constrain(float angle){
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
    	angle +=360;
    }
   	return angle;
}

IMUReader::IMUReader()
        : SensorReader("imu", &euler_msg)
{}

void IMUReader::realInit(){
    if(!imu.begin())
    {
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    delay(1000);
    imu.setExtCrystalUse(true);
}

void IMUReader::update(){
    sensors_event_t event;
    this->imu.getEvent(&event);
    //apply coordinate transform: pi - theta
    euler_msg.vector.x = angle_constrain(180 - event.orientation.x); 
    euler_msg.vector.y = event.orientation.y;
    euler_msg.vector.z = event.orientation.z;
}

void IMUReader::publish(ros::NodeHandle &nh){
    this->euler_msg.header.stamp = nh.now();

    this->pub.publish( &euler_msg );
}
