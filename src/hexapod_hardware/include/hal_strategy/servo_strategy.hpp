/**
 * @brief servo_strategy.hpp
 * @brief Base class to define strategies to access and use hardware.
 */

#ifndef SERVO_STRATEGY_HPP_
#define SERVO_STRATEGY_HPP_

#include <vector>
#include <string>
#include "hal_data/servo_command.hpp"

namespace hexapod_hardware
{
/**
 * @class ServoStrategy
 * @brief Base class for strategies for accessing hardware.
 */
class ServoStrategy
{
public:
    /// @brief Virtual destructor.
    virtual ~ServoStrategy() = default;

    /**
     * @brief Initializes the strategy and returns that status.
     * @return Status of the intialization.
     */
    virtual bool initialize() = 0;

    /**
     * @brief Maps actuators to joint strings.
     * @param List of joint names
     */
    virtual void configure_channels(const std::vector<std::string> &joint_names) { (void)joint_names; }

    /**
     * @brief Sets the angle to the desired degree.
     * @param servo_command Reference to the servo information.
     */
    virtual void set_angle_rad(ServoCommand &servo_command) = 0;

    /**
     * @brief Sets the angle to the desired radiant.
     * @param servo_command Reference to the servo information.
     */
    virtual void set_angle_degree(ServoCommand &servo_command) = 0;

    /**
     * @brief Returns the current angle of the desired actuator in radiants.
     * @param servo_command Reference to the servo information.
     * @return False if the actuator is not in the mapping.
     */
    virtual bool get_angle_rad(ServoCommand &servo_command) = 0;

    /// @brief Deactivates the strategy.
    virtual void shutdown() = 0;
};

} // namespace hexapod_hardware

#endif // SERVO_STRATEGY_HPP_