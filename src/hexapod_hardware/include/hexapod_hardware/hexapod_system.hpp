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

class HexapodSystem : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info
    ) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, 
        const rclcpp::Duration &period
    ) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, 
        const rclcpp::Duration &period
    ) override;

private:

    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

    std::unique_ptr<ServoStrategy> servo_strategy_;

    rclcpp::Node::SharedPtr sub_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    std::unordered_map<std::string, size_t> joint_name_to_index_;
};

}

#endif // HEXAPOD_SYSTEM_HPP_