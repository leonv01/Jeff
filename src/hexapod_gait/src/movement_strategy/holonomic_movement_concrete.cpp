#include "movement_strategy/holonomic_movement_concrete.hpp"

#include <cmath>

namespace hexapod_gait
{

double HolonomicMovementConcrete::CalculateMovementAngle(double dt, double x, double y)
{
    (void)dt;
    return std::atan2(y, x);
}

};