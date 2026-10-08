#ifndef NON_HOLONOMIC_MOVEMENT_CONCRETE_HPP_
#define NON_HOLONOMIC_MOVEMENT_CONCRETE_HPP_

#include "movement_strategy/movement_strategy.hpp"

namespace hexapod_gait
{

class NonHolonomicMovementConcrete : public MovementStrategy
{
private:
    double angle_ = 0.0;
public:
    double CalculateMovementAngle(double dt, double x, double y) override;
};

};

#endif // NON_HOLONOMIC_OMNI_MOVEMENT_CONCRETE_HPP_