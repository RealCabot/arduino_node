#include <ros.h>
#include <Arduino.h>
#include "EncoderReader.h"
#include "IMUReader.h"
// #include <sstream>

ros::NodeHandle nh;
unsigned long timer;

const int delay_time = 1000 / ENCODER_FREQ;

#define RENCODER_PIN_A 2
#define RENCODER_PIN_B 3
#define LENCODER_PIN_A 18
#define LENCODER_PIN_B 19
#define RMOTA 10
#define RMOTB 11
//#define RPWM  12
#define LMOTA 5
#define LMOTB 6
//#define LPWM  9

float LOOPTIME=.5;
unsigned long lastMilli = 0;
//int Rspeed_req = 180;
float Rspeed_req = 1;
int Lspeed_reg = 180;
float Rspeed_act = 0;
int Lspeed_req = 0;
int Lspeed_act = 0;
int PWM_valR = 0;
int PWM_valL = 0;

//PID
int RKp=800, RKi=0, RKd=0, LKp=50, LKi=2, LKd=30;

using namespace std;

EncoderReader myREncoderReader(RENCODER_PIN_A, RENCODER_PIN_B);
EncoderReader myLEncoderReader(LENCODER_PIN_A, LENCODER_PIN_B);
//IMUReader myIMUReader;

void readAndPublishVelocityHeading();
int updatePID(int command, int targetVal, int curVal, double Kp, double Ki, double Kd);
int PIDtry2(float desiredSpeed, float currSpeed);
void setup()
{
  Serial.begin(57600);
  //myIMUReader.realInit();
  nh.initNode();
  nh.advertise(myREncoderReader.get_publisher());
  nh.advertise(myLEncoderReader.get_publisher());
  //nh.advertise(myIMUReader.get_publisher());

  //motor settings
  pinMode(RMOTA, OUTPUT);
  pinMode(RMOTB, OUTPUT);
  pinMode(LMOTA, OUTPUT);
  pinMode(LMOTB, OUTPUT);
  //pinMode(RPWM, OUTPUT);
  //pinMode(LPWM, OUTPUT);
}

void loop()
{

  if ( (millis()-timer) > delay_time){
    readAndPublishVelocityHeading();
    timer =  millis();
      //Serial.print("fuck ros");
  }

  Rspeed_act = myREncoderReader.speed;
  //Serial.println(Rspeed_act);
  //Rspeed_act = Rspeed_act;
  //Lspeed_act = myLEncoderReader.speed;

  // ostringstream try1;
  // try1<<Rspeed_act;
  // try2=try1.str();
  //const char *a = "f";
  //const char *try1 = const char*(Rspeed_act);

  //String hello = "hello";
  //nh.loginfo("hello"); //nh.loginfo(try2); //nh.loginfo();

   //printf("hello, %d", Rspeed_act);
    //Serial.println(Rspeed_act);


  if (abs(Rspeed_req) > 0){
    if((millis()-lastMilli) >= LOOPTIME){
      lastMilli = millis();
      //PWM_valR = updatePID(PWM_valR, Rspeed_req, Rspeed_act, RKp, RKi, RKd);
      // Serial.println(Rspeed_act);
      PWM_valR = PIDtry2(Rspeed_req, Rspeed_act);

    }
  }

  if (Rspeed_req < 0){
    analogWrite(RMOTA, 0);
    analogWrite(RMOTB, PWM_valR);

  }
  else if(Rspeed_req > 0){
    analogWrite(RMOTA, PWM_valR);
    analogWrite(RMOTB, 0);

  }

}

void readAndPublishVelocityHeading()
{
  myREncoderReader.update();
  //myIMUReader.update();
  myREncoderReader.publish(nh);
  //myIMUReader.publish(nh);
  nh.spinOnce();
}

// void getMotorData() {
//   static long countAnt = 0;
//   Rspeed_act = myREncoderReader.speed*(60*1000/LOOPTIME); //1200 counts per revolution for output shaft
//   countAnt = count;

// }

//code based on: https://tutorial.cytron.io/2012/06/22/pid-for-embedded-design/
//return PWM. If pwm is positive, direction = forward. If PWM is negative, direction = reverse 
int PIDtry2(float desiredSpeed, float currSpeed){
  static float integral = 0;
  static float lastError = 0;

   //calc error
  float error = desiredSpeed - currSpeed;
   //accumulate error in integral 
  integral += error; 
  float derivative = error - lastError;

  //calc control variable for RIGHT motor
  int pwm = (RKp * error) + (RKi * integral) + (RKd * derivative);

   //limit pwm to range: [-255, 255]
  if (pwm < -255) pwm = -255;
  if (pwm > 255)  pwm = 255;

  lastError = error; //save last error 
  Serial.print(pwm); Serial.print("\t"); Serial.println(currSpeed);

  return pwm;
} 

// int updatePID(int command, int targetVal, int curVal, double Kp, double Kd, double Ki){
//   float pidTerm = 0;
//   int error = 0;
//   int toterror = 0;
//   static int last_error = 0;
//   error = abs(targetVal) - abs(curVal);
//   toterror += error;
//   pidTerm = (Kp+error) + (Kd*(error - last_error)) + (Ki * (toterror));
//   last_error = error;

//   return (command + int(pidTerm));

// }
