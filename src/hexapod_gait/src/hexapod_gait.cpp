/**
 * @file hexapod_gait.cpp
 * @brief Implementation of the HexapodGait ROS 2 node and control loop.
 */

#include <cstdio>

#include "hexapod_gait/hexapod_gait.hpp"

#include "gait_factory/gait_factory.hpp"

#include <string>

namespace hexapod_gait
{


HexapodGait::HexapodGait() : rclcpp::Node("hexapod_gait_node")
{
  step_counter_ = 0;

  this->declare_parameter<int>("total_steps", 30);
  this->declare_parameter<double>("max_stride_length", 0.080);
  this->declare_parameter<double>("swing_height", 0.040);
  this->declare_parameter<double>("speed_factor", 0.5);
  this->declare_parameter<double>("speed_increment", 0.1);
  this->declare_parameter<double>("step_distance", 0.110);
  this->declare_parameter<double>("step_depth", -0.08);

  phase_sm_ = HexapodPhaseSM();

  int total_steps = this->get_parameter("total_steps").as_int();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&HexapodGait::cmd_vel_callback, this, std::placeholders::_1)
  );

  gait_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/gait_mode", 10,
    std::bind(&HexapodGait::gait_mode_callback, this, std::placeholders::_1)
  );

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_commands", 10
  );

  body_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
    "/body_pose", 10,
    std::bind(&HexapodGait::body_pose_callback, this, std::placeholders::_1)
  );

  const std::string default_strategy = "tripod_gait";
  gait_strategy_ = std::move(GaitFactory::create_gait(default_strategy, total_steps));

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(CONTROL_TIMER_INTERVAL),
    std::bind(&HexapodGait::control_timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Hexapod Gait Node initialized with total_steps=%d", total_steps);
}

void HexapodGait::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_velocity_ = *msg;
}

void HexapodGait::gait_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string new_gait = msg->data;

  if (new_gait == "toggle_stand_sit")
  {
    phase_sm_.toggle_stand_sit();

    return;
  } 

  int total_steps = this->get_parameter("total_steps").as_int();

  auto strategy = GaitFactory::create_gait(new_gait, total_steps);

  if (strategy != nullptr)
  {
    gait_strategy_ = std::move(strategy);
    RCLCPP_INFO(this->get_logger(), "Switched gait strategy to: %s (total_steps=%d)", new_gait.c_str(), total_steps);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unkown gait strategy requested: %s", new_gait.c_str());
  }
}

void HexapodGait::control_timer_callback()
{
  if (gait_strategy_ == nullptr)
  {
    return;
  }

  double dt = CONTROL_TIMER_INTERVAL / 1000.0;

  double vx = current_velocity_.linear.x;
  double vy = current_velocity_.linear.y;
  double speed = std::sqrt(vx * vx + vy * vy);
  bool has_velocity = (speed > 0.001 || std::abs(current_velocity_.angular.z) > 0.001);

  /* -------------------------- Update state machine -------------------------- */
  phase_sm_.update(dt, has_velocity);

  /* ------------------------ Calculate desired height ------------------------ */
  double standing_depth = this->get_parameter("step_depth").as_double();
  const double sitting_depth = 0.000;
  double height_factor = phase_sm_.get_height_factor();

  double current_depth = sitting_depth + height_factor * (standing_depth - sitting_depth);

  /* ---------- Handling walking cycle only if the robot should walk ---------- */
  double max_stride_length = this->get_parameter("max_stride_length").as_double();
  double speed_factor = this->get_parameter("speed_factor").as_double();
  double speed_increment = this->get_parameter("speed_increment").as_double();
  double swing_height = this->get_parameter("swing_height").as_double();

  bool is_walking = phase_sm_.can_walk();
  double stride_length = (is_walking) ? std::clamp(speed * speed_factor, speed_increment, max_stride_length) : 0.0;
  swing_height = (is_walking) ? swing_height : 0.0;
  
  if (is_walking)
  {
    step_counter_ = (step_counter_ + 1) % this->get_parameter("total_steps").as_int();
    gait_strategy_->update_current_steps(step_counter_);
  }

  /* ---------------------------- Generate targets ---------------------------- */
  double step_distance = this->get_parameter("step_distance").as_double();

  double alpha = (speed > 0.001) ? std::atan2(vy, vx) : 0.0;

  Eigen::Vector3d relative_target(step_distance, 0.0, current_depth);

  std::unordered_map<LEG, LegData> leg_data = gait_strategy_->propagate_gait(step_counter_, stride_length, swing_height, relative_target, alpha);

  /* ----------------------------- Prepare message ---------------------------- */
  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = this->now();

  static const std::unordered_map<LEG, LEG_NAMING_MAP> joint_names = 
  {
    { LF_LEG, { "coxa_joint_5", "femur_joint_5",  "tibia_joint_5" } },
    { LM_LEG, { "coxa_joint_3", "femur_joint_3",  "tibia_joint_3" } },
    { LR_LEG, { "coxa_joint_2", "femur_joint_2",  "tibia_joint_2" } },
    { RR_LEG, { "coxa_joint_4", "femur_joint_4",  "tibia_joint_4" } },
    { RM_LEG, { "coxa_joint",   "femur_joint",    "tibia_joint" } },
    { RF_LEG, { "coxa_joint_1", "femur_joint_1",  "tibia_joint_1" } }
  };

  joint_msg.name.reserve(18);
  joint_msg.position.resize(18);

  static const std::array<LEG, 6> leg_order = {
    LF_LEG, LM_LEG, LR_LEG, RF_LEG, RM_LEG, RR_LEG
  };

  for(auto &leg : leg_data)
  {
    this->adjust_leg_angles(leg.second);
  }

  int idx = 0;

  for (auto leg : leg_order)
  {
    const auto &names = joint_names.at(leg);
    joint_msg.name.push_back(names.tibia_joint_);
    joint_msg.name.push_back(names.femur_joint_);
    joint_msg.name.push_back(names.coxa_joint_);

    const LegData &data = leg_data[leg];
    joint_msg.position[idx++] = data.tibia_joint_;
    joint_msg.position[idx++] = data.femur_joint_;
    joint_msg.position[idx++] = data.coxa_joint_;
  }

  /* ----------------------------- Publish message ---------------------------- */
  joint_state_pub_->publish(joint_msg);
}

void HexapodGait::body_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  (void)msg;  
}

void HexapodGait::adjust_leg_angles(LegData &leg_data)
{
  if (leg_data.leg_side_ == LEG_SIDE::LEFT)
  {
    leg_data.coxa_joint_ = std::abs(M_PI - (M_PI / 2.0 + leg_data.coxa_joint_));
    leg_data.femur_joint_ = std::abs(leg_data.femur_joint_ + M_PI / 2.0);
    leg_data.tibia_joint_ = std::abs(leg_data.tibia_joint_);
  }
  else
  {
    leg_data.coxa_joint_ = std::abs(M_PI - (M_PI / 2.0 + leg_data.coxa_joint_));
    leg_data.femur_joint_ = std::abs(M_PI - leg_data.femur_joint_ - M_PI / 2.0);
    leg_data.tibia_joint_ = M_PI - std::abs(leg_data.tibia_joint_);
  }

  leg_data.coxa_joint_ = std::clamp(leg_data.coxa_joint_, 0.0, M_PI);
  leg_data.femur_joint_ = std::clamp(leg_data.femur_joint_, 0.0, M_PI);
  leg_data.tibia_joint_ = std::clamp(leg_data.tibia_joint_, 0.0, M_PI - 1/4 * M_PI);
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_gait::HexapodGait>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
