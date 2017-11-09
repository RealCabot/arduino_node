#include "Motor.h"

Motor::Motor(
    short motor_pinA, short motor_pinB,
    short encoder_pinA, short encoder_pinB,
    int Kp, int Ki, int Kd
)
    : pinA(motor_pinA), pinB(motor_pinB), 
    encoder(encoder_pinA, encoder_pinB),
    pid(Kp, Ki, Kd)
{}

void Motor::go(float desiredSpeed){
    auto actual_speed = this->encoder.speed;
    auto PWM_val = this->pid.getPWM(desiredSpeed, actual_speed);
    if (desiredSpeed < 0){
      analogWrite(pinA, 0);
      analogWrite(pinB, PWM_val);
    } else {
      analogWrite(pinA, PWM_val);
      analogWrite(pinB, 0);
    }
}