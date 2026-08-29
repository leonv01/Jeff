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
    double x = target.x();
    double y = target.y();
    double z = target.z();

    double theta1 = std::atan2(y, x);
    
    double xy_dist = std::sqrt(x * x + y * y);
    double L = xy_dist - COXA_LENGTH;

    double c = std::sqrt(L * L + z * z);

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

    return Eigen::Vector3d(theta1, theta2, theta3);
}

} // namespace hexapod_gait