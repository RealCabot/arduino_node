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

void IMUReader::realInit(float initial_offset){
    if(!imu.begin())
    {
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    this->initial_offset = initial_offset;
    delay(1000);
    imu.setExtCrystalUse(true);
}

void IMUReader::update(){
    sensors_event_t event;
    this->imu.getEvent(&event);
    //apply coordinate transform: pi - theta
    float previous_angle = euler_msg.vector.x;
    euler_msg.vector.x = angle_constrain(360 - event.orientation.x); //in degree
    float angular_velocity = angle_constrain(euler_msg.vector.x - previous_angle) * ENCODER_FREQ * 71 / 4068; // in rad/s
    euler_msg.vector.y = angular_velocity;
    euler_msg.vector.z = 0;
}

void IMUReader::publish(ros::NodeHandle &nh){
    this->euler_msg.header.stamp = nh.now();

    this->pub.publish( &euler_msg );
}
