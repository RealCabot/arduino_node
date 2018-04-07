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
        : SensorReader("imu", &euler_msg_)
{}

void IMUReader::realInit(){
    if(!imu_.begin())
    {
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    delay(1000);
    imu_.setExtCrystalUse(true);
}

void IMUReader::update(){
    sensors_event_t event;
    imu_.getEvent(&event);
    //apply coordinate transform: pi - theta
    float previous_angle = euler_msg_.vector.x;
    euler_msg_.vector.x = angle_constrain(360 - (event.orientation.x - offset_)) ; //in degree
    float angular_velocity = angle_constrain(euler_msg_.vector.x - previous_angle) * ENCODER_FREQ * 71 / 4068; // in rad/s
    euler_msg_.vector.y = angular_velocity;
    euler_msg_.vector.z = 0;
}

void IMUReader::publish(ros::NodeHandle &nh){
    this->euler_msg_.header.stamp = nh.now();
    this->pub.publish( &euler_msg_ );
}


void IMUReader::reset(){
    offset_ = euler_msg_.vector.x;
}