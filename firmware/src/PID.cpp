//
// Created by andrew on 11/7/17.
//

#include "PID.h"

#define motorControl 10

PID::PID(int Kp, int Ki, int Kd)
    :Kp(Kp), Ki(Ki), Kd(Kd)
{}

int PID::getPWM(float desiredSpeed, float currSpeed){
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
