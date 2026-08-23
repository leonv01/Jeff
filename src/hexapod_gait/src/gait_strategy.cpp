/**
 * @file gait_strategy.cpp
 * @brief Implementation of base GaitStrategy and LegData member functions for hexapod gait propagation.
 */

#include "gait_strategy/gait_strategy.hpp"

#include "hexapod_gait/trajectory_generator.hpp"
#include "hexapod_gait/hexapod_ik.hpp"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace hexapod_gait
{


#define CALCULATE_LEG_MOUNT(x) ((2.0 * M_PI / LEG_COUNT) * (x))

LegData::LegData(double coxa_joint, double tibia_joint, double femur_joint, LEG_SIDE leg_side, double leg_phase, double leg_stance_offset, double leg_mount_angle)
{
    coxa_joint_ = coxa_joint;
    tibia_joint_ = tibia_joint;
    femur_joint_ = femur_joint;
    leg_phase_ = leg_phase;
    leg_stance_offset_ = leg_stance_offset;
    leg_mount_angle_ = leg_mount_angle;
    leg_side_ = leg_side;

    base_point_ = Eigen::Vector3d(
        std::cos(leg_mount_angle) * LEG_DISTANCE,
        std::sin(leg_mount_angle) * LEG_DISTANCE,
        0.0
    );
}

LegData::LegData(const Eigen::Vector3d& angles, LEG_SIDE leg_side, double leg_phase, double leg_stance_offset, double leg_mount_angle) : LegData(angles.x(), angles.y(), angles.z(), leg_side, leg_phase, leg_stance_offset, leg_mount_angle){}

GaitStrategy::GaitStrategy(double duty_cycle, int total_cycle_steps)
    : GaitStrategy(duty_cycle, {}, total_cycle_steps)
{
}

GaitStrategy::GaitStrategy(double duty_cycle, std::unordered_map<LEG, double> leg_offsets, int total_cycle_steps)
    : duty_cycle_(duty_cycle),
      total_cycle_steps_(total_cycle_steps),
      leg_map_({
          { RF_LEG, { 0.0, 0.0, 0.0, LEFT, 0.0, 0.0, CALCULATE_LEG_MOUNT(1) } },
          { RM_LEG, { 0.0, 0.0, 0.0, LEFT, 0.0, 0.0, CALCULATE_LEG_MOUNT(0) } },
          { RR_LEG, { 0.0, 0.0, 0.0, LEFT, 0.0, 0.0, CALCULATE_LEG_MOUNT(2) } },
          { LF_LEG, { 0.0, 0.0, 0.0, RIGHT, 0.0, 0.0, CALCULATE_LEG_MOUNT(4) } },
          { LM_LEG, { 0.0, 0.0, 0.0, RIGHT, 0.0, 0.0, CALCULATE_LEG_MOUNT(3) } },
          { LR_LEG, { 0.0, 0.0, 0.0, RIGHT, 0.0, 0.0, CALCULATE_LEG_MOUNT(5) } }
      }),
      leg_offsets_(leg_offsets.empty() ? std::unordered_map<LEG, double>{
          { LF_LEG, 0.0 }, { LM_LEG, 0.0 }, { LR_LEG, 0.0 },
          { RF_LEG, 0.0 }, { RM_LEG, 0.0 }, { RR_LEG, 0.0 }
      } : std::move(leg_offsets))
{
}

std::unordered_map<LEG, LegData> GaitStrategy::propagate_gait(int step, double stride_length, double swing_height, double alpha)
{
    std::for_each(leg_map_.begin(), leg_map_.end(), [this, step, stride_length, swing_height, alpha](auto& leg_entry) {
        leg_entry.second = this->propagate_leg(leg_entry.first, step, stride_length, swing_height, alpha);
    });

    return leg_map_;
}

LegData GaitStrategy::propagate_leg(LEG leg, int step, double stride_length, double swing_height, double alpha)
{
    LegData leg_data = get_leg(leg);

    double swing_fraction = 1.0 - duty_cycle_;
    double global_progress = static_cast<double>(step % total_cycle_steps_) / static_cast<double>(total_cycle_steps_);
    double leg_phase = std::fmod(global_progress - leg_offsets_[leg] + 1.0, 1.0);
    double mount_angle = leg_data.leg_mount_angle_;

    const Eigen::Vector3d local_neutral(0.17, 0.0, -0.12);
    Eigen::Matrix3d T_leg = Eigen::AngleAxisd(mount_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d body_neutral = T_leg * local_neutral;

    Eigen::Vector3d body_foot_pos;

    if (leg_phase < swing_fraction)
    {
        double tau = leg_phase / swing_fraction; 
        double phi = tau * M_PI;

        double s = (tau - 0.5) * stride_length;
        if (leg_data.leg_side_ == RIGHT)
        {
            s = -s;
        }
        double z_lift = swing_height * std::sin(phi);

        Eigen::Vector3d stride_dir(std::cos(alpha), std::sin(alpha), 0.0);
        body_foot_pos = body_neutral + (stride_dir * s) + Eigen::Vector3d(0, 0, z_lift);
    }
    else
    {
        double tau = (leg_phase - swing_fraction) / duty_cycle_; // 0.0 to 1.0

        double s = (0.5 - tau) * stride_length;
        if (leg_data.leg_side_ == RIGHT)
        {
            s = -s;
        }

        Eigen::Vector3d stride_dir(std::cos(alpha), std::sin(alpha), 0.0);
        body_foot_pos = body_neutral + (stride_dir * s);
    }

    Eigen::Vector3d leg_target_local = T_leg.transpose() * body_foot_pos;

    const Eigen::Vector3d angles = HexapodIk::solve_ik(leg_target_local);

    if (leg_data.leg_side_ == LEFT)
    {
        leg_data.coxa_joint_  = angles[0];  
        leg_data.femur_joint_ = angles[1]; 
        leg_data.tibia_joint_ = angles[2]; 
    }
    else // RIGHT side legs (URDF joint frames are 180 deg rotated relative to Left side)
    {
        leg_data.coxa_joint_  = -angles[0];  
        leg_data.femur_joint_ = -angles[1]; 
        leg_data.tibia_joint_ = -angles[2]; 
    }

    return leg_data;
}

LegData GaitStrategy::get_leg(LEG leg_id)
{
    auto idx = leg_map_.find(leg_id);
    if (idx != leg_map_.end())
    {
        return idx->second;
    }
    return LegData{};
}

} // namespace hexapod_gait
