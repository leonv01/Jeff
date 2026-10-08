#include "hexapod_teleop/hexapod_teleop.hpp"

namespace hexapod_teleop
{

HexapodTeleop::HexapodTeleop() : rclcpp::Node("hexapod_teleop_node")
{
  this->declare_parameter<int>("buttons.tripod_gait", 0);
  this->declare_parameter<int>("buttons.ripple_gait", 1);
  this->declare_parameter<int>("buttons.wave_gait", 3); 
  this->declare_parameter<int>("buttons.toggle_pose_mode", 2);
  this->declare_parameter<int>("buttons.stand_sit_toggle", 9);
  this->declare_parameter<int>("buttons.movement_toggle", 8);

  this->declare_parameter<int>("axes.linear_x", 1);
  this->declare_parameter<int>("axes.linear_y", 0);
  this->declare_parameter<int>("axes.angular_z", 3);

  this->declare_parameter<double>("scale.linear_x", 0.5);
  this->declare_parameter<double>("scale.linear_y", 0.5);
  this->declare_parameter<double>("scale.angular_z", 1.0);

  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, 
    std::bind(&HexapodTeleop::JoySubscriber, this, std::placeholders::_1));

  gait_publisher_ = this->create_publisher<std_msgs::msg::String>("/gait_mode", 10);
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  body_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/body_pose", 10);

  RCLCPP_INFO(this->get_logger(), "Hexapod Teleop Node initialized listening to /joy and publishing to /cmd_vel and /gait_mode");
}

void HexapodTeleop::JoySubscriber(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  int axis_lx = this->get_parameter("axes.linear_x").as_int();
  int axis_ly = this->get_parameter("axes.linear_y").as_int();
  int axis_az = this->get_parameter("axes.angular_z").as_int();

  double scale_lx = this->get_parameter("scale.linear_x").as_double();
  double scale_ly = this->get_parameter("scale.linear_y").as_double();
  double scale_az = this->get_parameter("scale.angular_z").as_double();

  bool dpad_up = (msg->axes.size() > 7 && msg->axes[7] > 0.5);
  bool dpad_down = (msg->axes.size() > 7 && msg->axes[7] < -0.5);
  bool dpad_left = (msg->axes.size() > 6 && msg->axes[6] > 0.5);
  bool dpad_right = (msg->axes.size() > 6 && msg->axes[6] < -0.5);

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = dpad_right ? 1.0 : (dpad_left ? -1.0 : 0.0);
  pose_msg.position.z = dpad_up ? 1.0 : (dpad_down ? -1.0 : 0.0);

  body_pose_publisher_->publish(pose_msg);

  geometry_msgs::msg::Twist twist;

  if (axis_lx >= 0 && static_cast<size_t>(axis_lx) < msg->axes.size())
  {
    twist.linear.x = msg->axes[axis_lx] * scale_lx;
  }
  if (axis_ly >= 0 && static_cast<size_t>(axis_ly) < msg->axes.size())
  {
    twist.linear.y = msg->axes[axis_ly] * scale_ly;
  }
  if (axis_az >= 0 && static_cast<size_t>(axis_az) < msg->axes.size())
  {
    twist.angular.z = msg->axes[axis_az] * scale_az;
  }

  cmd_vel_publisher_->publish(twist);

  std::string gait_mode = "";

  if (is_button_pressed(msg, "buttons.tripod_gait")) gait_mode = "tripod_gait";
  else if (is_button_pressed(msg, "buttons.wave_gait")) gait_mode = "wave_gait";
  else if (is_button_pressed(msg, "buttons.ripple_gait")) gait_mode = "ripple_gait";
  else if (is_button_pressed(msg, "buttons.toggle_pose_mode")) gait_mode = "toggle_pose";
  else if (is_button_pressed(msg, "buttons.stand_sit_toggle")) gait_mode = "stand_sit_toggle";
  else if (is_button_pressed(msg, "buttons.movement_toggle")) gait_mode = "movement_toggle";

  if (gait_mode.empty() == false)
  {
    std_msgs::msg::String gait_msg;
    gait_msg.data = gait_mode;
    this->gait_publisher_->publish(gait_msg);

    RCLCPP_INFO(this->get_logger(), "Selected gait: %s", gait_mode.c_str());
  }
}

bool HexapodTeleop::is_button_pressed(const sensor_msgs::msg::Joy::SharedPtr msg, const std::string &param_name)
{
  int button_idx = this->get_parameter(param_name).as_int();
  return (
    button_idx >= 0 &&
    static_cast<size_t>(button_idx) < msg->buttons.size() &&
    msg->buttons[button_idx] == 1
  );
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_teleop::HexapodTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
