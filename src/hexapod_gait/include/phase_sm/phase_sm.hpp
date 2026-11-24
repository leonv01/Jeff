/**
 * @file phase_sm.hpp
 * @brief Class to manage the robot phases through a state machine.
 */

#ifndef POSE_SM_HPP_
#define POSE_SM_HPP_

namespace hexapod_gait
{

/// @brief Enum for robot phases.
enum class RobotPhase
{
    SITTING,
    SITTING_DOWN,
    STANDING_UP,
    STANDING,
    WALKING
};

/**
 * @class HexapodPhaseSM
 * @brief Singleton class for managing robot phases.
 */
class HexapodPhaseSM
{
private:
    /// @brief Current phase of the robot.
    RobotPhase current_phase_;
    /// @brief Duration of the transition between phases.
    double transition_duration_;
    /// @brief Current progress in the transition.
    double progress_;

public:
    /**
     * @brief Constructor for the HexapodPhaseSM class. Creates an instance if none is active.
     * @return Singleton object of the HexapodPhaseSM class.
     */
    static HexapodPhaseSM &get_instance();

    /**
     * @brief Constructor for the HexapodPhaseSM class. Creates an instance if none is active.
     * @param transition_duration Default transition duration.
     */
    HexapodPhaseSM(double transition_duration = 2.5) : 
        current_phase_(RobotPhase::SITTING),
        transition_duration_(transition_duration),
        progress_(0.0) { }

    /**
     * @brief Updates the transition progress based on the time elapsed.
     * @param dt Time elapsed since the last update.
     * @param has_velocity Flag indicating if the robot has motion.
     */
    void update(double dt, bool has_velocity);
    
    /// @brief Toggles between stand and sit state.
    void toggle_stand_sit(void);
    
    /**
     * @brief Returns the current height factor.
     * @return Current height factor.
     */
    double get_height_factor(void) const;

    /**
     * @brief Returns the current phase of the robot.
     * @return Current phase of the robot.
     */
    RobotPhase get_phase() const { return current_phase_; }
    
    /**
     * @brief Returns the boolean information if the robot is able to walk.
     * @return Boolean information if the robot is able to walk.
     */
    bool can_walk() const { return current_phase_ == RobotPhase::WALKING; }
    
    /**
     * @brief Returns the boolean information if the robot is able to stand.
     * @return Boolean information if the robot is able to stand.
     */
    bool is_sitting() const { return current_phase_ == RobotPhase::SITTING; }
};

} // namespace hexapod_gait

#endif // POSE_SM_HPP_