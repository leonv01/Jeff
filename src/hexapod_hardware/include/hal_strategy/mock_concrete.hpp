/**
 * @file mock_concrete.hpp
 * @brief Class containing mock strategy to imitate hardware and actuators.
 */

#ifndef MOCK_CONCRETE_HPP_
#define MOCK_CONCRETE_HPP_

#include "hal_strategy/servo_strategy.hpp"
#include "hal_data/servo_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unordered_map>

namespace hexapod_hardware
{
/**
 * @class MockConcrete
 * @brief Class to mock hardware and actuators.
 */
class MockConcrete : public ServoStrategy
{
public:
    /// @brief Default constructor and logs the status.
    MockConcrete();

    /**
     * @brief Initializes the mock, logs, and returns that status.
     * @return Returns always true.
     */
    bool initialize() override;

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
    /// @brief Logger handle to log the actuators.
    rclcpp::Logger logger_;
    /// @brief Map of actuators and their rotation.
    std::unordered_map<unsigned int, double> mock_angles_;
};

} // namespace hexapod_hardware

#endif // MOCK_CONCRETE_HPP_