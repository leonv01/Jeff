/**
 * @file servo_command.hpp
 * @brief Class to manage commands directed to the actuators.
 */

#ifndef SERVO_COMMAND_HPP_
#define SERVO_COMMAND_HPP_

#include <string>

namespace hexapod_hardware
{

/**
 * @class ServoCommand
 * @brief Wrapper class to 
 */
struct ServoCommand 
{
    unsigned int leg;
    double angle;
};

/**
 * @class ServoChannelMap
 * @brief Wrapper class containing the address and i2c channel of the actuator.
 */
struct ServoChannelMap
{
    unsigned int address;
    unsigned int channel;
};

} // namespace hexapod_hardware

#endif // SERVO_COMMAND_HPP_