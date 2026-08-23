/**
 * @file hexapod_ik.cpp
 * @brief Implementation of 3-DOF analytical inverse kinematics for hexapod legs.
 */

#include "hexapod_gait/hexapod_ik.hpp"

#include <algorithm>
#include <cmath>

namespace hexapod_gait {


Eigen::Vector3d HexapodIk::solve_ik(const Eigen::Vector3d &target)
{
    // Get target coordinates
    double x = target.x();
    double y = target.y();
    double z = target.z();

    // Safeguard for coxa angle
    double theta1 = std::atan2(y, x);
    static constexpr double MAX_COXA_ANGLE = M_PI / 2.0;
    theta1 = std::clamp(theta1, -MAX_COXA_ANGLE, MAX_COXA_ANGLE);

    // Add minimum distance threshold (COXA length + 10cm)
    double xy_dist = std::sqrt(x * x + y * y);
    static constexpr double MIN_REACH = COXA_LENGTH + 0.10;
    xy_dist = xy_dist < MIN_REACH ? MIN_REACH : xy_dist;
    double L = xy_dist - COXA_LENGTH;

    // Clamp leg extension reach
    static constexpr double MAX_EXTENSION = 0.95 * (FEMUR_LENGTH + TIBIA_LENGTH);
    static constexpr double MIN_EXTENSION = 0.080; // 8cm
    double c = std::sqrt(L * L + z * z);
    c = std::clamp(c, MIN_EXTENSION, MAX_EXTENSION);

    double alpha = std::atan2(z, L);

    double cos_t3 = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - c * c) /
                    (2.0 * FEMUR_LENGTH * TIBIA_LENGTH);
    cos_t3 = std::clamp(cos_t3, -1.0, 1.0);
    double theta3 = std::acos(cos_t3) - M_PI;

    double cos_beta = (c * c + FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) /
                      (2.0 * c * FEMUR_LENGTH);
    cos_beta = std::clamp(cos_beta, -1.0, 1.0);
    double beta = std::acos(cos_beta);
    double theta2 = alpha + beta;

    static constexpr double JOINT_LIMIT = M_PI / 2.0;
    theta1 = std::clamp(theta1, -JOINT_LIMIT, JOINT_LIMIT);
    theta2 = std::clamp(theta2, -JOINT_LIMIT, JOINT_LIMIT);
    theta3 = std::clamp(theta3, -JOINT_LIMIT, JOINT_LIMIT);

    return Eigen::Vector3d(theta1, theta2, theta3);
}

} // namespace hexapod_gait