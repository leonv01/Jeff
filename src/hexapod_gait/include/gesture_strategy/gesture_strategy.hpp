#ifndef GESTURE_STRATEGY_HPP_
#define GESTURE_STRATEGY_HPP_

#include <unordered_map>

#include "gait_strategy/gait_strategy.hpp"

namespace hexapod_gait
{

enum GESTURE_STATE
{
    GESTURE_ACTIVE,
    GESTURE_INACTIVE
};

class GestureStrategy
{
public:
    GestureStrategy(double duration = 2.0) : 
        progress_(0.0), 
        duration_(duration), 
        state_(GESTURE_INACTIVE) {}
        
    virtual ~GestureStrategy() = default;

    virtual void start_gesture(double duration = 2.0)
    {
        duration_ = duration;
        progress_ = 0.0;
        state_ = GESTURE_ACTIVE;
    }

    virtual void start_gesture(LEG target_leg, double duration = 2.0)
    {
        (void)target_leg;
        start_gesture(duration);
    }

    virtual void reset()
    {
        progress_ = 0.0;
        state_ = GESTURE_INACTIVE;
    }

    GESTURE_STATE get_gesture_state() const { return state_; }

    virtual std::unordered_map<LEG, LegData> propagate_gesture(
        double dt,
        double step_distance,
        double current_depth
    ) = 0;

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
    double progress_;
    double duration_;

    GESTURE_STATE state_;

    static LEG_SIDE get_leg_side(LEG leg) 
    {
        return (leg == LF_LEG || leg == LM_LEG || leg == LR_LEG) ? LEFT : RIGHT;
    }
};

}

#endif // GESTURE_STRATEGY_HPP_