/**
 * @file servo_driver_node.hpp
 * @brief Class to handle servo control.
 */

#ifndef HEXAPOD_HARDWARE_SERVO_DRIVER_NODE_HPP_
#define HEXAPOD_HARDWARE_SERVO_DRIVER_NODE_HPP_

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

namespace hexapod_hardware
{

class ServoDriverNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the servo driver node.
     * @param options Node specific options.
     */
    explicit ServoDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    /// @brief Virtual destructor.
    virtual ~ServoDriverNode();

private:
    /**
     * @brief Callback function to act on received message to control actuators.
     * @param msg Joint state information.
     */
    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /// @brief Current servo strategy.
    std::unique_ptr<ServoStrategy> servo_strategy_;
    /// @brief Subscription for joint activation and use.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    /// @brief Mapping of joint strings to index.
    std::unordered_map<std::string, size_t> joint_name_to_index_;
};

} // namespace hexapod_hardware

#endif // HEXAPOD_HARDWARE_SERVO_DRIVER_NODE_HPP_

