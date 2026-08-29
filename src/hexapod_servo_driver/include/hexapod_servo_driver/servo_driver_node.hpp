#ifndef SERVO_DRIVER_NODE_HPP_
#define SERVO_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "hal_strategy/servo_strategy.hpp"
#include "hal_strategy/pca9685_concrete.hpp"
#include "hal_strategy/mock_concrete.hpp"
#include "hal_data/servo_command.hpp"

namespace hexapod_servo_driver
{

class ServoDriverNode : public rclcpp::Node
{
public:
    explicit ServoDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~ServoDriverNode();

private:
    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::unique_ptr<hexapod_hardware::ServoStrategy> servo_strategy_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;

    std::unordered_map<std::string, size_t> joint_name_to_index_;
};

} // namespace hexapod_servo_driver

#endif // SERVO_DRIVER_NODE_HPP_
