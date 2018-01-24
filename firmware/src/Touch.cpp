//
// Created by andrew on 1/20/18.
//

#include "Touch.h"

Touch::Touch()
        : SensorReader("touch", &currTouched)
{}

bool Touch::init(){
    // Default address is 0x5A, if tied to 3.3V its 0x5B
    // If tied to SDA its 0x5C and if SCL then 0x5D

    //first check if MPR121 is plugged in
    Wire.beginTransmission(0x5A);
    uint8_t error = Wire.endTransmission();

    if (error == 0){ //success, initialize MPR121
        cap.begin(0x5A);
        return true;

    }
    else{  //unknown error, return failure
        return false;
    }
}

void Touch::publish(ros::NodeHandle &nh){
    currTouched.data = touchData;
    this->pub.publish( &currTouched );
}

bool Touch::getTouched(int pinNum){
    touchData = cap.touched();
    //touchData is a 12 bit number, shift touchData by the pin number
    if ((touchData >> pinNum) & 0x1){
        return true;
    }
    return false;
}
