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

void Motor::go(float desiredSpeed, ros::NodeHandle &nh){
    auto actual_speed = this->encoder.speed;
    auto PWM_val = this->pid.getPWM(desiredSpeed, actual_speed);
    
    //char logStr[60];
    // sprintf (logStr, "Actual PWM: %d, desiredSpeed: %d, actual_speed: %d", PWM_val, (int)(desiredSpeed*100), (int)(actual_speed*100));
    // nh.loginfo(logStr);

    if (desiredSpeed < 0){
      // analogWrite(pinA, 0);
      // analogWrite(pinB, PWM_val);
    	mot.write(90-PWM_val);
    } else {
      // analogWrite(pinA, PWM_val);
      // analogWrite(pinB, 0);
    	mot.write(90+PWM_val);
    }
}