//
// Created by andrew on 11/7/17.
//
#include <stdlib.h>
#include "PID.h"
#include "ros.h"

#define motorControl 10
#define MAX_I 100
#define MAX_PWM 255 //are u happy Yanda??

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

        char cstr[50];
        char d_str[6];
        char curr_str[6];

        /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
        // dtostrf(desiredSpeed, 4, 2, d_str);
        // dtostrf(currSpeed, 4, 2, curr_str);
        // sprintf(cstr, "desired speed: %s, currSpeed: %s", d_str, curr_str);
        // nh.loginfo(cstr);

    //accumulate error in integral
    if (!(desiredSpeed > 0 && abs(currSpeed)<0.001)){
        integral += error;
    }
    integral = constrain(integral, -MAX_I, MAX_I);

    float derivative = error - lastError;

    //calc control variable for RIGHT motor
    int pwm = (Kp * error) + (Ki * integral) + (Kd * derivative);
    // ros::NodeHandle nh;
    // char cstr[16];
    // nh.loginfo(itoa(pwm, cstr, 10));

    //limit pwm to range: [0, 255]
    pwm = constrain(pwm, 0, MAX_PWM);

    lastError = error; //save last error
    //Serial.print("L: "); Serial.print(pwm); Serial.print("\t"); Serial.println(currSpeed);

    return pwm;
}

void PID::setParam(int Kp, int Ki, int Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}