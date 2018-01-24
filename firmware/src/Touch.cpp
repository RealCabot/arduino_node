//
// Created by andrew on 1/20/18.
//

#include "Touch.h"

Touch::Touch()
        : SensorReader("touch", &currTouched)
{}

int Touch::init(){
//
    // Default address is 0x5A, if tied to 3.3V its 0x5B
    // If tied to SDA its 0x5C and if SCL then 0x5D
    if (!cap.begin(0x5A)) {     //TODO: THIS LINE IS CAUSING PROGRAM TO HANG
        Serial.println("MPR121 not found, check wiring?");
        return -1;
    }
    else{
        Serial.println("MPR121 found!");
        return 0;
    }
}

void Touch::publish(ros::NodeHandle &nh){
    currTouched.data = touchData;
    this->pub.publish( &currTouched );
}

bool Touch::getTouched(int pinNum){
    touchData = cap.touched();
    if ((touchData >> pinNum) & 0x1){
        return true;
    }
    return false;
}
