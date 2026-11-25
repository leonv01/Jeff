/**
 * @file non_holonomic_movement_concrete.hpp
 * @brief Concrete class for non-holonomic movement strategy.
 */
#ifndef NON_HOLONOMIC_MOVEMENT_CONCRETE_HPP_
#define NON_HOLONOMIC_MOVEMENT_CONCRETE_HPP_

#include "movement_strategy/movement_strategy.hpp"

namespace hexapod_gait
{

/**
 * @class NonHolonomicMovementConcrete
 * @brief Class for handling and executing non-holonomic movement.
 */
class NonHolonomicMovementConcrete : public MovementStrategy
{
private:
    /// @brief Angle of the movement
    double angle_ = 0.0;
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

#endif // NON_HOLONOMIC_OMNI_MOVEMENT_CONCRETE_HPP_