#ifndef PCA9685_CONCRETE_HPP_
#define PCA9685_CONCRETE_HPP_

#include "hal_strategy/servo_strategy.hpp"
#include "hal_data/servo_command.hpp"

#include "PiPCA9685/PCA9685.h"

#include <vector>
#include <unordered_map>
#include <cmath>
#include <tuple>
#include <string>

namespace hexapod_hardware
{
class PCA9685Concrete : public ServoStrategy
{
public:
    PCA9685Concrete(std::vector<unsigned int> ids_ = { 0x40, 0x41 });

    bool initialize() override;
    void configure_channels(const std::vector<std::string> &joint_names) override;

    void set_angle_rad(ServoCommand &servo_command) override;
    void set_angle_degree(ServoCommand &servo_command) override;

    bool get_angle_rad(ServoCommand &servo_command) override;

    void shutdown() override;

private:
    std::unordered_map<unsigned int, std::unique_ptr<PiPCA9685::PCA9685>> pcas_;
    std::vector<ServoChannelMap> channel_mapping_;
    
    const double FREQUENCY_ = 50.0;

    const double MIN_RAD_ = 0.0;
    const double MAX_RAD_ = M_PI;

    const int MIN_TICKS_ = 100;
    const int MAX_TICKS_ = 500;

    unsigned int rad_to_pwm(double angle);
};
}

#endif // PCA9685_CONCRETE_HPP_