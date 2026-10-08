#ifndef MOVEMENT_FACTORY_HPP_
#define MOVEMENT_FACTORY_HPP_

#include <string>
#include <memory>

#include "movement_strategy/movement_strategy.hpp"

namespace hexapod_gait
{

class MovementFactory
{
private:

public:
    static std::unique_ptr<MovementStrategy> create_movement(const std::string &type);
};

};

#endif // MOVEMENT_FACTORY_HPP_