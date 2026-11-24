#include "movement_strategy/non_holonomic_movement_concrete.hpp"

#include <cmath>
#include <algorithm>

namespace hexapod_gait
{

double NonHolonomicMovementConcrete::CalculateMovementAngle(double dt, double x, double y)
{
    double speed = std::hypot(x, y);

    if (speed < 0.001)
    {
        return angle_;
    }

    double target_angle = std::atan2(y, x);

    double delta_angle = std::atan2(
        std::sin(target_angle - angle_),
        std::cos(target_angle - angle_)
    );

    const double max_angular_speed = 2.0;
    double max_turn_step = max_angular_speed * dt;

    angle_ += std::clamp(delta_angle, -max_turn_step, max_turn_step);

    angle_ = std::atan2(std::sin(angle_), std::cos(angle_));
    
    return angle_;
}


};