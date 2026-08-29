#ifndef SERVO_COMMAND_HPP_
#define SERVO_COMMAND_HPP_

#include <string>

namespace hexapod_hardware
{

struct ServoCommand 
{
    unsigned int leg;
    double angle;
};

struct ServoChannelMap
{
    unsigned int address;
    unsigned int channel;
};

struct ServoMap
{
    std::string servo_name_;
    
    int address_;
    int channel_;
    double angle_;
};

}

#endif // SERVO_COMMAND_HPP_