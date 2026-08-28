/**
 * @file gait_strategy.hpp
 * @brief Base class and data structures defining hexapod gait strategies and leg state propagation.
 */

#ifndef GAIT_STRATEGY_HPP_
#define GAIT_STRATEGY_HPP_

#include <unordered_map>
#include <algorithm>
#include <Eigen/Dense>
#include <string>

namespace hexapod_gait
{

/// @brief Total number of legs on the hexapod robot.
#define LEG_COUNT 6

/// @brief Distance from body center to leg mount base origin in millimeters (117.67 mm).
#define LEG_DISTANCE 117.66964

/**
 * @enum LEG
 * @brief Identifiers for the six hexapod legs.
 */
enum LEG
{
    LF_LEG, ///< Left Front leg
    LM_LEG, ///< Left Middle leg
    LR_LEG, ///< Left Rear leg

    RF_LEG, ///< Right Front leg
    RM_LEG, ///< Right Middle leg
    RR_LEG  ///< Right Rear leg
};

/**
 * @enum LEG_SIDE
 * @brief Lateral body side designation for leg mounting.
 */
enum LEG_SIDE
{
    LEFT,  ///< Left body side leg
    RIGHT  ///< Right body side leg
};

/**
 * @struct LEG_NAMING_MAP
 * @brief Maps joint names corresponding to a leg's coxa, femur, and tibia actuators in URDF.
 */
struct LEG_NAMING_MAP
{
    std::string coxa_joint_;  ///< Name of coxa joint
    std::string femur_joint_; ///< Name of femur joint
    std::string tibia_joint_; ///< Name of tibia joint
};

/**
 * @struct LegData
 * @brief Holds physical parameters, phase states, and joint positions for an individual leg.
 */
struct LegData
{
    double coxa_joint_ = 0.0;   ///< Coxa joint angle in radians
    double tibia_joint_ = 0.0;  ///< Tibia joint angle in radians
    double femur_joint_ = 0.0;  ///< Femur joint angle in radians

    double leg_phase_ = 0.0;          ///< Normalized leg gait phase (0.0 to 1.0)
    double leg_stance_offset_ = 0.0;  ///< Leg phase offset relative to gait cycle start
    double leg_mount_angle_ = 0.0;    ///< Leg mounting angle relative to body forward axis (radians)

    Eigen::Vector3d base_point_;      ///< Leg base mount origin relative to robot body center

    LEG_SIDE leg_side_;               ///< Lateral mounting side (LEFT or RIGHT)

    /// @brief Default constructor initializing default angles and states.
    LegData() = default;

    /**
     * @brief Parameterized constructor using individual joint angles.
     * @param coxa_joint Coxa joint angle in radians.
     * @param femur_joint Femur joint angle in radians.
     * @param tibia_joint Tibia joint angle in radians.
     * @param leg_side Mounting side designation (LEFT or RIGHT).
     * @param leg_phase Initial normalized leg phase.
     * @param leg_stance_offset Stance phase offset.
     * @param leg_mount_angle Body mounting angle in radians.
     */
    LegData(double coxa_joint, double femur_joint, double tibia_joint, LEG_SIDE leg_side, double leg_phase = 0.0, double leg_stance_offset = 0.0, double leg_mount_angle = 0.0);

    /**
     * @brief Parameterized constructor using a 3D vector of joint angles.
     * @param angles Vector of (coxa, femur, tibia) angles in radians.
     * @param leg_side Mounting side designation (LEFT or RIGHT).
     * @param leg_phase Initial normalized leg phase.
     * @param leg_stance_offset Stance phase offset.
     * @param leg_mount_angle Body mounting angle in radians.
     */
    LegData(const Eigen::Vector3d& angles, LEG_SIDE leg_side, double leg_phase = 0.0, double leg_stance_offset = 0.0, double leg_mount_angle = 0.0);
};

/**
 * @class GaitStrategy
 * @brief Base class for implementing leg coordination pattern strategies (e.g. tripod, wave, ripple).
 */
class GaitStrategy
{
public:
    /**
     * @brief Constructs a GaitStrategy with a given duty cycle and total cycle steps.
     * @param duty_cycle Ratio of stance phase duration to total gait period (0.0 to 1.0).
     * @param total_cycle_steps Total discrete control steps per full gait cycle.
     */
    GaitStrategy(double duty_cycle = 0.0, int total_cycle_steps = 60.0);

    /**
     * @brief Constructs a GaitStrategy with specified leg offsets.
     * @param duty_cycle Ratio of stance phase duration to total gait period.
     * @param leg_offsets Map of phase offsets for each individual leg.
     * @param total_cycle_steps Total discrete control steps per full gait cycle.
     */
    GaitStrategy(double duty_cycle, std::unordered_map<LEG, double> leg_offsets, int total_cycle_steps = 60.0);

    /// @brief Virtual destructor for GaitStrategy.
    virtual ~GaitStrategy() = default;

    /**
     * @brief Gets the current stance duty cycle.
     * @return double Stance duty cycle ratio.
     */
    double get_duty_cycle() const { return duty_cycle_; }

    /**
     * @brief Sets the stance duty cycle ratio.
     * @param duty_cycle New stance duty cycle ratio (0.0 to 1.0).
     */
    void set_duty_cycle(double duty_cycle) { duty_cycle_ = duty_cycle; }

    /**
     * @brief Propagates the gait step for all legs and returns updated LegData for each leg.
     * @param step Current global step count in the cycle.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param relative_target Target coordinate offset.
     * @param alpha Heading/direction angle in radians.
     * @return std::unordered_map<LEG, LegData> Map of updated leg states indexed by leg enum.
     */
    virtual std::unordered_map<LEG, LegData> propagate_gait(int step, double stride_length = 0.0, double swing_height = 0.0, const Eigen::Vector3d &relative_target = Eigen::Vector3d(0.08, 0.0, -0.08), double alpha = 0.0);

    /**
     * @brief Computes trajectory and joint angles for a single leg at a given step.
     * @param leg Target leg ID.
     * @param step Current global step count in the cycle.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param relative_target Target coordinate offset.
     * @param alpha Heading/direction angle in radians.
     * @return LegData Updated state for the specified leg.
     */
    virtual LegData propagate_leg(LEG leg, int step, double stride_length = 0.0, double swing_height = 0.0, const Eigen::Vector3d &relative_target = Eigen::Vector3d(0.08, 0.0, -0.08), double alpha = 0.0);

    /**
     * @brief Retrieves stored LegData for a given leg ID.
     * @param leg_id Target leg ID.
     * @return LegData Copy of leg data state.
     */
    virtual LegData get_leg(LEG leg_id);

    /**
     * @brief Updates internal cycle step index.
     * @param steps New step count index.
     */
    void inline update_current_steps(int steps) { current_cycle_steps_ = steps; }

protected:
    /// @brief Ratio of stance phase duration to total gait period (0.0 to 1.0).
    double duty_cycle_;

    /// @brief Total discrete steps constituting one full gait cycle period.
    int total_cycle_steps_;

    /// @brief Current step index within active gait cycle.
    int current_cycle_steps_;

    /// @brief Map maintaining physical parameters and current states of all 6 legs.
    std::unordered_map<LEG, LegData> leg_map_;

    /// @brief Map of relative phase offsets for each leg (0.0 to 1.0).
    std::unordered_map<LEG, double> leg_offsets_;
};

} // namespace hexapod_gait

#endif // GAIT_STRATEGY_HPP_