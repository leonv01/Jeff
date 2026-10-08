#include "movement_factory/movement_factory.hpp"

#include "movement_strategy/movement_strategy.hpp"
#include "movement_strategy/non_holonomic_movement_concrete.hpp"
#include "movement_strategy/holonomic_movement_concrete.hpp"

namespace hexapod_gait
{

std::unique_ptr<MovementStrategy> MovementFactory::create_movement(const std::string &type)
{
    if (type == "holonomic")
    {
        return std::make_unique<HolonomicMovementConcrete>();
    }
    else if (type == "non-holonmic")
    {
        return std::make_unique<NonHolonomicMovementConcrete>();
    }

    return nullptr;
}

};