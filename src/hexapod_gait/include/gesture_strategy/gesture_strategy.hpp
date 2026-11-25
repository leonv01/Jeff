/**
 * @file gesture_strategy.hpp
 * @brief Base class and data structures defining gesture strategies.
 */

#ifndef GESTURE_STRATEGY_HPP_
#define GESTURE_STRATEGY_HPP_

#include <unordered_map>

#include "gait_strategy/gait_strategy.hpp"

namespace hexapod_gait
{

/// @brief Enum for gesture state.
enum GESTURE_STATE
{
    GESTURE_ACTIVE,
    GESTURE_INACTIVE
};

/**
 * @class GestureStrategy
 * @brief Base class for implementing gesture motions.
 */
class GestureStrategy
{
public:
    /**
     * @brief Constructs a GestureStrategy with a given duration.
     * @param duration The duration of the gesture in seconds.
     */
    GestureStrategy(double duration = 2.0) : 
        progress_(0.0), 
        duration_(duration), 
        state_(GESTURE_INACTIVE) {}
    
    /// @brief Virtual destructor for GestureStrategy.
    virtual ~GestureStrategy() = default;

    /**
     * @brief Starts the gesture with a default duration of 2 seconds.
     * @param duration The duration of the gesture in seconds.
     */
    virtual void start_gesture(double duration = 2.0)
    {
        duration_ = duration;
        progress_ = 0.0;
        state_ = GESTURE_ACTIVE;
    }

    /**
     * @brief Starts the gesture for a specific leg.
     * @param target_leg The leg for which the gesture is being started.
     * @param duration The duration of the gesture in seconds.
     */
    virtual void start_gesture(LEG target_leg, double duration = 2.0)
    {
        (void)target_leg;
        start_gesture(duration);
    }

    /**
     * @brief Resets gesture.
     */
    virtual void reset()
    {
        progress_ = 0.0;
        state_ = GESTURE_INACTIVE;
    }

    /**
     * @brief Gets the current state of the gesture.
     * @return Current state of the gesture.
     */
    GESTURE_STATE get_gesture_state() const { return state_; }

    /**
     * @brief Computes and iterates through the gesture animation.
     * @param dt Delta time.
     * @param step_distance Step distance for the animation.
     * @param current_depth Current depth value to start and return.
     * @return Updated map of all updated legs.
     */
    virtual std::unordered_map<LEG, LegData> propagate_gesture(
        double dt,
        double step_distance,
        double current_depth
    ) = 0;

    /**
     * @brief Calculates the progress of the gesture animation.
     * @return Returns true if the gesture is ongoing, false if the gesture is inactive.
     */
    bool update_progress(double dt)
    {
        if (state_ == GESTURE_INACTIVE)
        {
            return false;
        }

        progress_ += dt / duration_;
        if (progress_ >= 1.0)
        {
            progress_ = 1.0;
            state_ = GESTURE_INACTIVE;
        }

        return true;
    }

protected:
    /// @brief Progress of the gesture.
    double progress_;
    /// @brief Duration of the gesture.
    double duration_;

    /// @brief Current state of the gesture.
    GESTURE_STATE state_;

    /**
     * @brief Returns the leg side corresponding to the given leg.
     * @param leg The leg for which to get the corresponding side.
     * @return The leg side corresponding to the given leg.
     */
    static LEG_SIDE get_leg_side(LEG leg) 
    {
        return (leg == LF_LEG || leg == LM_LEG || leg == LR_LEG) ? LEFT : RIGHT;
    }
};

} // namespace hexapod_gait

#endif // GESTURE_STRATEGY_HPP_