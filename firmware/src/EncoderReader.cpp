//
// Created by yanda on 10/28/17.
//

#include "EncoderReader.h"

EncoderReader::EncoderReader(short pinA, short pinB)
        : velocity_pub("encoder", &encoder_msg)
        , myEnc(pinA, pinB)
        , oldPosition(-999)
{}

void EncoderReader::update(){
    long newPosition = myEnc.read();
    this->speed = float(newPosition - oldPosition) / TICK_PER_METER * ENCODER_FREQ; // m/s
    if (newPosition < RESET_THRESHOLD){
        oldPosition = newPosition;
    } else { // Reset the counter
        myEnc.write(0);
        oldPosition = 0;
    }
}

void EncoderReader::publish(ros::NodeHandle &nh){
    this->encoder_msg.speed = speed;
    this->encoder_msg.header.stamp = nh.now();
    this->velocity_pub.publish( &encoder_msg );
}