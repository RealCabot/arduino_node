#include "EncoderReader.h"

EncoderReader::EncoderReader(short pinA, short pinB)
        : SensorReader("encoder", &encoder_msg)
        , myEnc(pinA, pinB)
{}

void EncoderReader::update(){
    long newPosition = myEnc.read();
    this->speed = float(newPosition - oldPosition_) / TICK_PER_METER * ENCODER_FREQ; // m/s
    if (newPosition < RESET_THRESHOLD){
        oldPosition_ = newPosition;
    } else { // Reset the counter
        this->reset();
    }
}

//Depricated
void EncoderReader::publish(ros::NodeHandle &nh){
    this->encoder_msg.speed = speed;
    this->encoder_msg.header.stamp = nh.now();
    this->pub.publish( &encoder_msg );
}

void EncoderReader::reset(){
    myEnc.write(0);
    oldPosition_ = 0;
}
