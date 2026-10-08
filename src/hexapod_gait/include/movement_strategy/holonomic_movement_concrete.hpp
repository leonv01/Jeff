#ifndef HOLONOMIC_MOVEMENT_CONCRETE_HPP_
#define HOLONOMIC_MOVEMENT_CONCRETE_HPP_

#include "movement_strategy/movement_strategy.hpp"

namespace hexapod_gait
{

class HolonomicMovementConcrete : public MovementStrategy
{
private:

public:
    double CalculateMovementAngle(double dt, double x, double y) override;
};

};

#endif // HOLONOMIC_OMNI_MOVEMENT_CONCRETE_HPP_