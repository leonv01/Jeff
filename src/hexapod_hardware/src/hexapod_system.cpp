#include "hexapod_hardware/hexapod_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "hal_strategy/pca9685_concrete.hpp"
#include "hal_strategy/mock_concrete.hpp"
#include "hal_data/servo_command.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hexapod_hardware
{
hardware_interface::CallbackReturn HexapodSystem::on_init(
    const hardware_interface::HardwareInfo &info
) 
{
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS
    )
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_states_.resize(info_.joints.size(), 0.0);

    std::vector<std::string> joint_names;
    joint_names.reserve(info_.joints.size());
    joint_name_to_index_.clear();
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        joint_names.push_back(info_.joints[i].name);
        joint_name_to_index_[info_.joints[i].name] = i;
    }

    servo_strategy_ = std::make_unique<MockConcrete>();
    servo_strategy_->configure_channels(joint_names);

    sub_node_ = rclcpp::Node::make_shared("hexapod_hw_joint_cmd_sub");
    joint_cmd_sub_ = sub_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_commands", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); i++)
            {
                auto it = joint_name_to_index_.find(msg->name[i]);
                if (it != joint_name_to_index_.end() && i < msg->position.size())
                {
                    hw_commands_[it->second] = msg->position[i];
                }
            }
        }
    );

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HexapodSystem::export_state_interfaces() 
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]
        );
    }

    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> HexapodSystem::export_command_interfaces() 
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]
        );
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn HexapodSystem::on_activate(
    const rclcpp_lifecycle::State &previous_state
) 
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexapodSystem::on_deactivate(
    const rclcpp_lifecycle::State &previous_state
) 
{
    servo_strategy_->shutdown();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HexapodSystem::read(
    const rclcpp::Time &time, 
    const rclcpp::Duration &period
) 
{
    for (size_t i = 0; i < hw_states_.size(); i++)
    {
        hw_states_[i] = hw_commands_[i];
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HexapodSystem::write(
    const rclcpp::Time &time, 
    const rclcpp::Duration &period
) 
{
    if (sub_node_)
    {
        rclcpp::spin_some(sub_node_);
    }

    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
        ServoCommand command{ static_cast<unsigned int>(i), hw_commands_[i] }; 
        servo_strategy_->set_angle_rad(command);
    }

    return hardware_interface::return_type::OK;
}
}

PLUGINLIB_EXPORT_CLASS(
    hexapod_hardware::HexapodSystem, 
    hardware_interface::SystemInterface
)