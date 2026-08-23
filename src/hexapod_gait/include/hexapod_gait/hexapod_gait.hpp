/**
 * @file hexapod_gait.hpp
 * @brief ROS 2 Node for controlling hexapod robot gait generation and leg control.
 * @details Handles ROS 2 topics for velocity commands, gait pattern selection, and body pose adjustment,
 *          periodically propagating gait strides and publishing joint states.
 */

#ifndef HEXAPOD_GAIT_HPP_
#define HEXAPOD_GAIT_HPP_

#include "hexapod_gait/hexapod_ik.hpp"

#include "gait_strategy/gait_strategy.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

namespace hexapod_gait
{
/**
 * @brief Control loop timer callback period in milliseconds.
 */
#define CONTROL_TIMER_INTERVAL 20

/**
 * @class HexapodGait
 * @brief Primary ROS 2 Node executing hexapod gait algorithms and joint state publishing.
 */
class HexapodGait : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the HexapodGait ROS 2 node, initializing subscribers, publisher, and control timer.
     */
    HexapodGait();

private:
    /// @brief Global gait step counter tracking phase progression across total cycle steps.
    int step_counter_;

    /// @brief Active gait strategy implementation (e.g. tripod, wave, ripple).
    std::unique_ptr<GaitStrategy> gait_strategy_;

    /// @brief Current target velocity received from /cmd_vel topic.
    geometry_msgs::msg::Twist current_velocity_;

    /// @brief Subscription for switching gait mode dynamically.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_mode_sub_;

    /// @brief Subscription for target linear and angular velocities.
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    /// @brief Subscription for body pose offsets.
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub;

    /// @brief Publisher for output joint states of all 6 legs (18 joints total).
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    /**
     * @brief Callback function for processing incoming velocity commands.
     * @param msg Received Twist velocity message.
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback function for switching the active gait strategy.
     * @param msg Received String message containing gait strategy name ("tripod_gait", "wave_gait", "ripple_gait").
     */
    void gait_mode_callback(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief Callback function for processing body pose target inputs.
     * @param msg Received Pose message representing body position/orientation target.
     */
    void body_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    /// @brief Wall timer driving periodic control loop execution.
    rclcpp::TimerBase::SharedPtr control_timer_;

    /**
     * @brief Periodic control loop callback executing gait propagation, inverse kinematics, and joint state publication.
     */
    void control_timer_callback();
};

} // namespace hexapod_gait

#endif // HEXAPOD_GAIT_HPP_

