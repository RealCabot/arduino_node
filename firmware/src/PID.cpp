//
// Created by andrew on 11/7/17.
//

#include "PID.h"

PID::PID(int Kp, int Ki, int Kd){
    this-> Kp = Kp;
    this-> Ki = Ki;
    this-> Kd = Kd;
}

int PID::update(float desiredSpeed, float currSpeed){
    static float integral = 0;
    static float lastError = 0;

    //calc error
    float error = desiredSpeed - currSpeed;
    //accumulate error in integral
    integral += error;
    float derivative = error - lastError;

    //calc control variable for RIGHT motor
    int pwm = (Kp * error) + (Ki * integral) + (Kd * derivative);

    //limit pwm to range: [-255, 255]
    if (pwm < -255) pwm = -255;
    if (pwm > 255)  pwm = 255;

    lastError = error; //save last error
    //Serial.print("L: "); Serial.print(pwm); Serial.print("\t"); Serial.println(currSpeed);

    return pwm;
}
