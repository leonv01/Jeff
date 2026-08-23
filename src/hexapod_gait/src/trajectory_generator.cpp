/**
 * @file trajectory_generator.cpp
 * @brief Implementation of 3D trajectory generation functions for swing and stance foot paths.
 */

#include "hexapod_gait/trajectory_generator.hpp"

#include <cmath>
#include <algorithm>

namespace hexapod_gait
{


Eigen::Vector3d TrajectoryGenerator::calculate_swing_trajectory(
    double stride_length, double swing_height, double gait_phase)
{
    double phase = std::clamp(gait_phase, 0.0, 1.0);
    double phi = phase * M_PI;

    // MATLAB trajectory.m: x = L/2 * (1 - cos(phi)), z = H * sin(phi)
    double x_local = (stride_length / 2.0) * (1.0 - std::cos(phi));
    double y_local = 0.0;
    double z_local = -swing_height * std::sin(phi);

    return Eigen::Vector3d(x_local, y_local, z_local);
}

Eigen::Vector3d TrajectoryGenerator::calculate_stance_trajectory(
    double stride_length, double stance_phase)
{
    double tau_stance = std::clamp(stance_phase, 0.0, 1.0);

    // Stance phase ground return: L -> 0
    double x_local = stride_length * (1.0 - tau_stance);
    double y_local = 0.0;
    double z_local = 0.0;

    return Eigen::Vector3d(x_local, y_local, z_local);
}

} // namespace hexapod_gait