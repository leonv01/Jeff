/**
 * @file holonomic_movement_concrete.hpp
 * @brief Concrete class for holonomic movement strategy.
 */
#ifndef HOLONOMIC_MOVEMENT_CONCRETE_HPP_
#define HOLONOMIC_MOVEMENT_CONCRETE_HPP_

#include "movement_strategy/movement_strategy.hpp"

namespace hexapod_gait
{

/**
 * @class HolonomicMovementConcrete
 * @brief Class for handling and executing holonomic movement.
 */
class HolonomicMovementConcrete : public MovementStrategy
{
private:

public:
    /**
     * @brief Calculates the movement angle for the movement strategy base on given inputs.
     * @param dt Delta time.
     * @param x X coordinate of the input.
     * @param y Y coordinate of the input.
     * @return Movement angle.
     */
    double CalculateMovementAngle(double dt, double x, double y) override;
};

} // namespace hexapod_gait

#endif // HOLONOMIC_OMNI_MOVEMENT_CONCRETE_HPP_