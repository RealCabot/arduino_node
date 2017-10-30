#ifndef ARDUINO_NODE_ENCODERREADER_H
#define ARDUINO_NODE_ENCODERREADER_H

#include <arduino_msg/Encoder.h>
#include <Encoder.h>
#include "SensorReader.h"

#define ENCODER_FREQ 5 // How many times in a second
#define RESET_THRESHOLD 2000000000 // Close to Arduino Long limit. It's actually 2,147,483,647
#define TICK_PER_METER 390000 //How many ticks does it take for the robot to go 1m

class EncoderReader : public SensorReader{
    arduino_msg::Encoder encoder_msg;
    Encoder myEnc;
    long oldPosition = -999;
    float speed;
public:
    ros::Publisher pub;
    EncoderReader(short pinA, short pinB);
    void update();
    void publish(ros::NodeHandle &nh);
};


#endif //ARDUINO_NODE_ENCODERREADER_H
