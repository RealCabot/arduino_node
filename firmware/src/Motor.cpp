#include "Motor.h"

Motor::Motor(
    short motor_pinA,
    short encoder_pinA, short encoder_pinB,
    int Kp, int Ki, int Kd
)
    : pinA(motor_pinA), //pinB(motor_pinB), 
    encoder(encoder_pinA, encoder_pinB),
    pid(Kp, Ki, Kd)
{}

//attach servo object to provided pin
//sorry, there's prob a better way to do this
void Motor::init(){
	mot.attach(pinA, 1000,2000); //1000 and 2000 center it around 90 for stopping
}

void Motor::go(float desiredSpeed, ros::NodeHandle &nh){
    auto actual_speed = this->encoder.speed;
    auto PWM_val = this->pid.getPWM(desiredSpeed, actual_speed);
    
    int writePWM = 0;
    if (desiredSpeed < 0){
    	writePWM = 90-PWM_val; //90 = stop, 180 = full forward, 0 = full backwards

    } else {
    	writePWM = 90+PWM_val;
    }    

    mot.write(writePWM);
}

void Motor::reset(){
    pid.reset();
    encoder.reset();
}