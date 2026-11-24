/**
 * @file movement_strategy.hpp
 * @brief Base class and data structures defining movement strategies.
 */

#ifndef MOVEMENT_STRATEGY_HPP_
#define MOVEMENT_STRATEGY_HPP_

namespace hexapod_gait
{

/**
 * @class MovementStrategy
 * @brief Base class for implementing movement strategies.
 */
class MovementStrategy
{
private:

public:
    /// @brief Default constructor.
    MovementStrategy() = default;
    /// @brief Default destructor.
    virtual ~MovementStrategy() = default;

    /**
     * @brief Calculates the movement angle for the movement strategy base on given inputs.
     * @param dt Delta time.
     * @param x X coordinate of the input.
     * @param y Y coordinate of the input.
     * @return Movement angle.
     */
    virtual double CalculateMovementAngle(double dt, double x, double y) = 0;
};

} // namespace hexapod_gait

#endif // MOVEMENT_STRATEGY_HPP_