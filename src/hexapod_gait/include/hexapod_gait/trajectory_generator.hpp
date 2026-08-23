/**
 * @file trajectory_generator.hpp
 * @brief Foot trajectory generator for hexapod swing and stance leg phases.
 */

#ifndef TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>

namespace hexapod_gait
{

/**
 * @class TrajectoryGenerator
 * @brief Calculates 3D Cartesian foot position trajectories during leg swing and stance phases.
 */
class TrajectoryGenerator
{
public:
    /**
     * @brief Generates a 3D foot position trajectory during the swing phase (foot in air).
     * @param stride_length Target total stride length in meters.
     * @param swing_height Maximum ground clearance lift height in meters.
     * @param gait_phase Normalized phase progress of swing phase (0.0 to 1.0).
     * @return Eigen::Vector3d 3D displacement vector (x, y, z) in local leg trajectory space.
     */
    static Eigen::Vector3d calculate_swing_trajectory(double stride_length, double swing_height, double gait_phase);

    /**
     * @brief Generates a 3D foot position trajectory during the stance phase (foot grounded).
     * @param stride_length Target total stride length in meters.
     * @param stance_phase Normalized phase progress of stance phase (0.0 to 1.0).
     * @return Eigen::Vector3d 3D displacement vector (x, y, z) in local leg trajectory space.
     */
    static Eigen::Vector3d calculate_stance_trajectory(double stride_length, double stance_phase);
};

} // namespace hexapod_gait

#endif // TRAJECTORY_GENERATOR_HPP_