#include "phase_sm/phase_sm.hpp"

#include <cmath>
#include <algorithm>

namespace hexapod_gait
{
void HexapodPhaseSM::update(double dt, bool has_velocity)
{
    switch (current_phase_)
    {
        case RobotPhase::STANDING_UP:
            progress_ += dt / transition_duration_;

            if (progress_ >= 1.0)
            {
                progress_ = 1.0;
                current_phase_ = RobotPhase::STANDING;
            }
            break;
        case RobotPhase::SITTING_DOWN:
            progress_ -= dt / transition_duration_;

            if (progress_ <= 0.0)
            {
                progress_ = 0.0;
                current_phase_ = RobotPhase::SITTING;
            }
            break;
        case RobotPhase::STANDING:
            if (has_velocity)
            {
                current_phase_ = RobotPhase::WALKING;
            }
            break;
        case RobotPhase::WALKING:
            if (has_velocity == false)
            {
                current_phase_ = RobotPhase::STANDING;
            }
            break;
        case RobotPhase::SITTING:
            break;
        default:
            break;
    }
}

void HexapodPhaseSM::toggle_stand_sit(void)
{
    if (current_phase_ == RobotPhase::STANDING || current_phase_ == RobotPhase::WALKING)
    {
        current_phase_ = RobotPhase::SITTING_DOWN;
        progress_ = 1.0;
    }
    else if (current_phase_ == RobotPhase::SITTING)
    {
        current_phase_ = RobotPhase::STANDING_UP;
        progress_ = 0.0;
    }
}

double HexapodPhaseSM::get_height_factor(void) const
{
    double p = std::clamp(progress_, 0.0, 1.0);
    return (1.0 - std::cos(p * M_PI)) / 2.0;
}

HexapodPhaseSM &HexapodPhaseSM::getInstance()
{
    static HexapodPhaseSM obj;
    return obj;
}

}