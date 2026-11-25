/**
 * @file wink_gesture_concrete.hpp
 * @brief Concrete implementation of wink gesture.
 */

#ifndef WINK_GESTURE_CONCRETE_HPP_
#define WINK_GESTURE_CONCRETE_HPP_

#include "gesture_strategy/gesture_strategy.hpp"

namespace hexapod_gait
{

/**
 * @brief WinkGestureConcrete
 * @brief Wink gesture strategy executing a wink gesture.
 */
class WinkGestureConcrete : public GestureStrategy
{
public:
    /**
     * @brief Constructs a WinkGestureConcrete strategy.
     * @param target_leg The leg to perform the wink gesture on.
     * @param duration The duration of the gesture.
     */
    WinkGestureConcrete(
        LEG target_leg = RF_LEG, 
        double duration = 2.0) : GestureStrategy(duration), target_leg_(target_leg) {}

    /**
     * @brief Starts the gesture with a default duration of 2 seconds.
     * @param target_leg The leg to perform the wink gesture on.
     * @param duration The duration of the gesture in seconds.
     */
    void start_gesture(LEG target_leg, double duration = 2.0) override
    {
        target_leg_ = target_leg;
        GestureStrategy::start_gesture(duration);
    }

    /**
     * @brief Computes and iterates through the wink gesture animation.
     * @param dt Delta time.
     * @param step_distance Step distance for the animation.
     * @param current_depth Current depth value to start and return.
     * @return Updated map of all updated legs.
     */
    std::unordered_map<LEG, LegData> propagate_gesture(
        double dt,
        double step_distance,
        double current_depth
    ) override;

private:
    /// @brief The leg to perform the wink gesture on.
    LEG target_leg_;
};

} // namespace hexapod_gait

#endif // WINK_GESTURE_CONCRETE_HPP_