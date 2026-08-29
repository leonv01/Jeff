#include "hal_strategy/pca9685_concrete.hpp"

#include <string>
#include <algorithm>

namespace hexapod_hardware
{
PCA9685Concrete::PCA9685Concrete(std::vector<unsigned int> ids_)
{
    const std::string device = "/dev/i2c-1";

    for (auto id : ids_)
    {
        pcas_[id] = std::make_unique<PiPCA9685::PCA9685>(device, id);
        pcas_[id]->set_pwm_freq(FREQUENCY_);
    }

    channel_mapping_ = {
        // Left Front (LF)
        { 0x40, 0 }, { 0x40, 1 }, { 0x40, 2 },
        // Left Middle (LM)
        { 0x40, 4 }, { 0x40, 5 }, { 0x40, 6 },
        // Left Rear (LR)
        { 0x40, 8 }, { 0x40, 9 }, { 0x40, 10 },
        // Right Front (RF)
        { 0x41, 0 }, { 0x41, 1 }, { 0x41, 2 },
        // Right Middle (RM)
        { 0x41, 4 }, { 0x41, 5 }, { 0x41, 6 },
        // Right Rear (RR)
        { 0x41, 12 }, { 0x41, 13 }, { 0x41, 14 }
    };
}

bool PCA9685Concrete::initialize() 
{
    return true;
}

void PCA9685Concrete::configure_channels(const std::vector<std::string> &joint_names)
{
    static const std::unordered_map<std::string, ServoChannelMap> joint_lookup = {
        // Left Front (LF)
        { "coxa_joint_2",  { 0x40, 0 } }, { "femur_joint_2", { 0x40, 1 } }, { "tibia_joint_2", { 0x40, 2 } },
        // Left Middle (LM)
        { "coxa_joint_3",  { 0x40, 4 } }, { "femur_joint_3", { 0x40, 5 } }, { "tibia_joint_3", { 0x40, 6 } },
        // Left Rear (LR)
        { "coxa_joint_5",  { 0x40, 8 } }, { "femur_joint_5", { 0x40, 9 } }, { "tibia_joint_5", { 0x40, 10 } },
        // Right Front (RF)
        { "coxa_joint_1",  { 0x41, 0 } }, { "femur_joint_1", { 0x41, 1 } }, { "tibia_joint_1", { 0x41, 2 } },
        // Right Middle (RM)
        { "coxa_joint",    { 0x41, 4 } }, { "femur_joint",   { 0x41, 5 } }, { "tibia_joint",   { 0x41, 6 } },
        // Right Rear (RR) — FIXED CHANNELS TO 12, 13, 14:
        { "coxa_joint_4",  { 0x41, 12 } }, { "femur_joint_4", { 0x41, 13 } }, { "tibia_joint_4", { 0x41, 14 } }
    };

    channel_mapping_.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); i++)
    {
        auto it = joint_lookup.find(joint_names[i]);
        if (it != joint_lookup.end())
        {
            channel_mapping_[i] = it->second;
        }
    }
}

void PCA9685Concrete::set_angle_rad(ServoCommand &servo_command) 
{
    if (servo_command.leg >= channel_mapping_.size()) return;

    const auto &target = channel_mapping_[servo_command.leg];

    unsigned int pwm_tick = rad_to_pwm(servo_command.angle);
    pcas_[target.address]->set_pwm(target.channel, 0, pwm_tick);    
}

void PCA9685Concrete::set_angle_degree(ServoCommand &servo_command) 
{
    (void)servo_command;
}

bool PCA9685Concrete::get_angle_rad(ServoCommand &servo_command) 
{
    (void)servo_command;
    return true;
}

void PCA9685Concrete::shutdown() 
{
    for (size_t i = 0; i < 18; i++)
    {
        ServoCommand command{ static_cast<unsigned int>(i), 0.0 };
        set_angle_rad(command);
    }
}

unsigned int PCA9685Concrete::rad_to_pwm(double angle)
{
    double clamp = std::clamp(angle, MIN_RAD_, MAX_RAD_);

    return static_cast<unsigned int>(MIN_TICKS_ + (clamp / MAX_RAD_) * (MAX_TICKS_ - MIN_TICKS_));
}

}