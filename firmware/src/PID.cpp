//
// Created by andrew on 11/7/17.
//
#include <stdlib.h>
#include "PID.h"
#include "ros.h"

//#define motorControl 10
#define MAX_I 100
#define MAX_PWM 90 //are u happy Yanda??

PID::PID(int Kp, int Ki, int Kd)
    :Kp(Kp), Ki(Ki), Kd(Kd)
{}

int PID::getPWM(float desiredSpeed, float currSpeed){
    ros::NodeHandle nh;
	if (desiredSpeed < 0)
		desiredSpeed *= -1;
	if (currSpeed < 0)
		currSpeed *= -1;

    //calc error
    float error = desiredSpeed - currSpeed;
    //accumulate error in integral
    integral_ += error;

    integral_ = constrain(integral_, -MAX_I, MAX_I);

    float derivative = error - lastError_;

    //calc control variable for RIGHT motor
    int pwm = (Kp * error) + (Ki * integral_) + (Kd * derivative);

    //limit pwm to range: [0, 255]
    pwm = constrain(pwm, 0, MAX_PWM);

    lastError_ = error; //save last error

    return pwm;
}

void PID::setParam(int Kp, int Ki, int Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::reset(){
    integral_ = 0;
    lastError_ = 0;
}