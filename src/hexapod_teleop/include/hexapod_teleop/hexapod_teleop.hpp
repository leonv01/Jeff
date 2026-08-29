#ifndef HEXAPOD_TELEOP_HPP_
#define HEXAPOD_TELEOP_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

namespace hexapod_teleop
{

enum GAIT_TYPE
{
    WAVE_GAIT,
    RIPLE_GAIT,
    TRIPOD_GAIT
};

class HexapodTeleop : public rclcpp::Node
{
public:
    HexapodTeleop();

private:
    void JoySubscriber(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    GAIT_TYPE gait_type;
    bool is_button_pressed(const sensor_msgs::msg::Joy::SharedPtr msg, const std::string &param_name);
};

}

#endif // HEXAPOD_TELEOP_HPP_
