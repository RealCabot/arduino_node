//
// Created by andrew on 1/20/18.
//

#include "Touch.h"

Touch::Touch()
        : SensorReader("touch", &currTouched)
{}

void Touch::init(){
    Serial.println("Adafruit MPR121 Capacitive Touch sensor test");

    // Default address is 0x5A, if tied to 3.3V its 0x5B
    // If tied to SDA its 0x5C and if SCL then 0x5D
    if (!cap.begin(0x5A)) {
        Serial.println("MPR121 not found, check wiring?");
        while (1);
    }
    Serial.println("MPR121 found!");
}

void Touch::update(){
    // Get the currently touched pads
    currTouched.data = cap.touched();
}

void Touch::publish(ros::NodeHandle &nh){
    this->pub.publish( &currTouched );
}
