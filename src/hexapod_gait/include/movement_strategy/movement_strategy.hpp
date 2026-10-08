#ifndef MOVEMENT_STRATEGY_HPP_
#define MOVEMENT_STRATEGY_HPP_

namespace hexapod_gait
{

class MovementStrategy
{
private:

public:
    MovementStrategy() = default;
    virtual ~MovementStrategy() = default;

    virtual double CalculateMovementAngle(double dt, double x, double y) = 0;
};

};

#endif // MOVEMENT_STRATEGY_HPP_