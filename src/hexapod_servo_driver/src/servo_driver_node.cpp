/**
 * @file servo_driver_node.cpp
 * @brief Standalone ROS 2 node for controlling PCA9685 hexapod servos directly from joint commands.
 */

#include "hexapod_servo_driver/servo_driver_node.hpp"

namespace hexapod_servo_driver
{

ServoDriverNode::ServoDriverNode(const rclcpp::NodeOptions &options)
    : Node("hexapod_servo_driver_node", options)
{
    this->declare_parameter<std::string>("hardware_type", "pca9685");
    this->declare_parameter<std::string>("joint_commands_topic", "/joint_commands");

    std::string hardware_type = this->get_parameter("hardware_type").as_string();
    std::string topic_name = this->get_parameter("joint_commands_topic").as_string();

    if (hardware_type == "pca9685")
    {
        servo_strategy_ = std::make_unique<hexapod_hardware::PCA9685Concrete>();
        RCLCPP_INFO(this->get_logger(), "Initialized PCA9685 Physical Hardware Strategy");
    }
    else
    {
        servo_strategy_ = std::make_unique<hexapod_hardware::MockConcrete>();
        RCLCPP_INFO(this->get_logger(), "Initialized Mock Hardware Strategy");
    }

    servo_strategy_->initialize();

    static const std::vector<std::string> joint_order = {
        "coxa_joint_1", "femur_joint_1", "tibia_joint_1", // RF
        "coxa_joint_2", "femur_joint_2", "tibia_joint_2", // LF
        "coxa_joint_3", "femur_joint_3", "tibia_joint_3", // LM
        "coxa_joint_4", "femur_joint_4", "tibia_joint_4", // RR
        "coxa_joint_5", "femur_joint_5", "tibia_joint_5", // LR
        "coxa_joint",   "femur_joint",   "tibia_joint"    // RM
    };

    joint_name_to_index_.clear();
    for (size_t i = 0; i < joint_order.size(); i++)
    {
        joint_name_to_index_[joint_order[i]] = i;
    }

    servo_strategy_->configure_channels(joint_order);

    joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        topic_name, 10,
        std::bind(&ServoDriverNode::joint_command_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Hexapod Servo Driver Node listening on topic: %s", topic_name.c_str());
}

ServoDriverNode::~ServoDriverNode()
{
    if (servo_strategy_)
    {
        servo_strategy_->shutdown();
    }
}

void ServoDriverNode::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!servo_strategy_) return;

    for (size_t i = 0; i < msg->name.size(); i++)
    {
        auto it = joint_name_to_index_.find(msg->name[i]);
        if (it != joint_name_to_index_.end() && i < msg->position.size())
        {
            hexapod_hardware::ServoCommand command{
                static_cast<unsigned int>(it->second),
                msg->position[i]
            };
            servo_strategy_->set_angle_rad(command);
        }
    }
}

} // namespace hexapod_servo_driver

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hexapod_servo_driver::ServoDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
