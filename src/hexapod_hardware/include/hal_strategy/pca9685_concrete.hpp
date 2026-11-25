/**
 * @file pca9685_concrete.hpp
 * @brief Class handling and interacting with PCA9685 driver strategy to communicate with hardware and actuators.
 */

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

/**
 * @class PCA9685Concrete
 * @brief Class to wrap PCa9685 driver interaction as replaceable strategy.
 */
class PCA9685Concrete : public ServoStrategy
{
public:
    /**
     * @brief Constructor to initialize i2c device handles for the PCA9685.
     * @param ids_ List of I2C addresses of PCA9685 devices to be initialiized.
     */
    PCA9685Concrete(std::vector<unsigned int> ids_ = { 0x40, 0x41 });

    /**
     * @brief Initializes the hardware drivers.
     * @return Returns always true.
     */
    bool initialize() override;

    /**
     * @brief Maps all connected actuators of the PCA9685 to joint strings.
     * @param List of joint names
     */
    void configure_channels(const std::vector<std::string> &joint_names) override;

    /**
     * @brief Sets the angle to the desired degree.
     * @param servo_command Reference to the servo information.
     */
    void set_angle_rad(ServoCommand &servo_command) override;

    /**
     * @brief Sets the angle to the desired radiant.
     * @param servo_command Reference to the servo information.
     */
    void set_angle_degree(ServoCommand &servo_command) override;

    /**
     * @brief Returns the current angle of the desired actuator in radiants.
     * @param servo_command Reference to the servo information.
     * @return False if the actuator is not in the mapping.
     */
    bool get_angle_rad(ServoCommand &servo_command) override;

    /// @brief Deactivates the actuators.
    void shutdown() override;

private:
    /// @brief Map of I2C channel and PCA9685 handle.
    std::unordered_map<unsigned int, std::unique_ptr<PiPCA9685::PCA9685>> pcas_;
    /// @brief Mapping of all actuators and their channels and id.
    std::vector<ServoChannelMap> channel_mapping_;
    
    /// @brief Default frequency for actuators
    const double FREQUENCY_ = 50.0;

    /// @brief Minimum rotation in radians.
    const double MIN_RAD_ = 0.0;
    /// @brief Maximum rotation in radians.
    const double MAX_RAD_ = M_PI;

    /// @brief Minimum ticks for the PWM calculation.
    const int MIN_TICKS_ = 100;
    /// @brief Maximum ticks for the PWM calculation.
    const int MAX_TICKS_ = 500;

    /**
     * @brief Helper function to transform radiants into PWM signal.
     * @param angle Angle to rotate in radiants.
     * @return PWM sequence length.
     */
    unsigned int rad_to_pwm(double angle);
};

} // namespace hexapod_hardware

#endif // PCA9685_CONCRETE_HPP_