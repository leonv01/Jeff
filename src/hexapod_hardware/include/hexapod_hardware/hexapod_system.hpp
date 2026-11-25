/**
 * @file hexapod_system.hpp
 * @brief Class to extend and adapt the SystemInterface of ROS2 for standardized hardware access.
 */
#ifndef HEXAPOD_SYSTEM_HPP_
#define HEXAPOD_SYSTEM_HPP_

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "hal_strategy/servo_strategy.hpp"

namespace hexapod_hardware
{

/**
 * @class HexapodSystem
 * @brief Class to adapt the standardized ROS2 hardware access to the robot specific controls.
 */
class HexapodSystem : public hardware_interface::SystemInterface
{
public:
    /**
     * @brief Function to initialize robot hardware.
     * @param info Hardware information.
     * @return CallbackReturn
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info
    ) override;

    /**
     * @brief Function to export state interfaces.
     * @return List of StateInterface.
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    /**
     * @brief Function to export command interfaces.
     * @return List of CommandInterface.
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /**
     * @brief Activation function.
     * @param previous_state Previous state.
     * @return CallbackReturn
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    /**
     * @brief Deactivation function.
     * @param previous_state Previous state.
     * @return CallbackReturn
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    /**
     * @brief Read function.
     * @param previous_state Previous state.
     * @return CallbackReturn
     */
    hardware_interface::return_type read(
        const rclcpp::Time &time, 
        const rclcpp::Duration &period
    ) override;

    /**
     * @brief Write function.
     * @param previous_state Previous state.
     * @return CallbackReturn
     */
    hardware_interface::return_type write(
        const rclcpp::Time &time, 
        const rclcpp::Duration &period
    ) override;

private:
    /// @brief List of hardware commands.
    std::vector<double> hw_commands_;
    /// @brief List of hardware states.
    std::vector<double> hw_states_;

    /// @brief Current hardware strategy.
    std::unique_ptr<ServoStrategy> servo_strategy_;

    /// @brief Subscription node for hexapod_hw_joint_cmd_sub topic.
    rclcpp::Node::SharedPtr sub_node_;
    /// @brief Subscription for joint states topic.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    /// @brief Map of joint names and their corresponding index.
    std::unordered_map<std::string, size_t> joint_name_to_index_;
};

} // namespace hexapod_hardware

#endif // HEXAPOD_SYSTEM_HPP_