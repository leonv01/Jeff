#ifndef SERVO_STRATEGY_HPP_
#define SERVO_STRATEGY_HPP_

#include <vector>
#include <string>
#include "hal_data/servo_command.hpp"

namespace hexapod_hardware
{
class ServoStrategy
{
public:
    virtual ~ServoStrategy() = default;

    virtual bool initialize() = 0;
    virtual void configure_channels(const std::vector<std::string> &joint_names) { (void)joint_names; }

    virtual void set_angle_rad(ServoCommand &servo_command) = 0;
    virtual void set_angle_degree(ServoCommand &servo_command) = 0;

    virtual bool get_angle_rad(ServoCommand &servo_command) = 0;

    virtual void shutdown() = 0;
};
}
#endif // SERVO_STRATEGY_HPP_