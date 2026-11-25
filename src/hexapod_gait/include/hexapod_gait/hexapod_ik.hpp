/**
 * @file hexapod_ik.hpp
 * @brief 3-DOF Inverse Kinematics solver for hexapod leg mechanisms.
 */

#ifndef HEXAPOD_IK_HPP_
#define HEXAPOD_IK_HPP_

#include <Eigen/Dense>

namespace hexapod_gait {

/**
 * @class HexapodIk
 * @brief Provides analytical inverse kinematics calculations for a 3-DOF hexapod leg.
 */
class HexapodIk
{
public:
    /**
     * @brief Solves analytical inverse kinematics for a target 3D Cartesian foot position in leg local frame.
     * @param target Target 3D coordinates (x, y, z) in meters relative to the leg base frame.
     * @return Eigen::Vector3d Vector containing calculated joint angles (coxa, femur, tibia) in radians.
     */
    static Eigen::Vector3d solve_ik(const Eigen::Vector3d &target);

    /**
     * @brief Helper function to create a 3x3 rotation matrix.
     * @param alpha Rotation about the x-axis in radians.
     * @param beta Rotation about the y-axis in radians.
     * @param gamma Rotation about the z-axis in radians.
     * @return Eigen::Matrix3d 3x3 rotation matrix.
     */
    static Eigen::Matrix3d rotation_matrix(double alpha, double beta, double gamma);

private:
    /// @brief Coxa link length in meters (distance from coxa axis to femur axis: 40.0 mm).
    static constexpr double COXA_LENGTH = 0.040;

    /// @brief Femur link length in meters (69.50804 mm).
    static constexpr double FEMUR_LENGTH = 0.06950804;

    /// @brief Tibia link length in meters (112.09523 mm).
    static constexpr double TIBIA_LENGTH = 0.11209523;
};

} // namespace hexapod_gait

#endif // HEXAPOD_IK_HPP_