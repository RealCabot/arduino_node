#ifndef ARDUINO_NODE_ENCODERREADER_H
#define ARDUINO_NODE_ENCODERREADER_H

#include <arduino_msg/Encoder.h>
#include <Encoder.h>
#include "SensorReader.h"

#define ENCODER_FREQ 20 // How many times in a second
#define RESET_THRESHOLD 2000000000 // Close to Arduino Long limit. It's actually 2,147,483,647

// wheel size = 2.875 inches = .073025 m. 1m/(pi * .073025m ) = 4.3589 rotations for 1 meter
// 3200 ticks per rotation * 4.3589 = 13949
#define TICK_PER_METER 13949 //How many ticks does it take for the robot to go 1m

class EncoderReader : public SensorReader{
    arduino_msg::Encoder encoder_msg;
    Encoder myEnc;
    long oldPosition = 0;

public:
    EncoderReader(short pinA, short pinB);
    void update();
    void publish(ros::NodeHandle &nh);
    float speed;
};


#endif //ARDUINO_NODE_ENCODERREADER_H
