/**
 * @file hexapod_ik.hpp
 * @brief 3-DOF Analytical Inverse Kinematics solver for hexapod leg mechanisms.
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

private:
    /// @brief Coxa link length in meters (distance from coxa axis to femur axis: 52.0 mm).
    static constexpr double COXA_LENGTH = 0.052;

    /// @brief Femur link length in meters (75.208 mm).
    static constexpr double FEMUR_LENGTH = 0.075208;

    /// @brief Tibia link length in meters (98.265 mm).
    static constexpr double TIBIA_LENGTH = 0.098265;
};

} // namespace hexapod_gait

#endif // HEXAPOD_IK_HPP_