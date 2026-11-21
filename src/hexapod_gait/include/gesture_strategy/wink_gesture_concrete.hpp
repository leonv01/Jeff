#ifndef WINK_GESTURE_CONCRETE_HPP_
#define WINK_GESTURE_CONCRETE_HPP_

#include "gesture_strategy/gesture_strategy.hpp"

namespace hexapod_gait
{

class WinkGestureConcrete : public GestureStrategy
{
public:
    WinkGestureConcrete(
        LEG target_leg = RF_LEG, 
        double duration = 2.0) : GestureStrategy(duration), target_leg_(target_leg) {}

    void start_gesture(LEG target_leg, double duration = 2.0) override
    {
        target_leg_ = target_leg;
        GestureStrategy::start_gesture(duration);
    }

    std::unordered_map<LEG, LegData> propagate_gesture(
        double dt,
        double step_distance,
        double current_depth
    ) override;

private:
    LEG target_leg_;
};
}

#endif // WINK_GESTURE_CONCRETE_HPP_