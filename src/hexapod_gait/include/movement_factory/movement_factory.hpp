/**
 * @file movement_factory.hpp
 * @brief Factory class for instantiating concrete movement strategy objects.
 */

#ifndef MOVEMENT_FACTORY_HPP_
#define MOVEMENT_FACTORY_HPP_

#include <string>
#include <memory>

#include "movement_strategy/movement_strategy.hpp"

namespace hexapod_gait
{

/**
 * @class MovementFactory
 * @brief Factory for dynamically instantiating concrete MovementStrategy instances based on string identifier.
 */
class MovementFactory
{
private:

public:
    /**
     * @brief Instantiates a concrete MovementStrategy instance matching the specified type string.
     * @param type The type string of the desired MovementStrategy.
     * @return A unique_ptr to the instantiated MovementStrategy.
     */
    static std::unique_ptr<MovementStrategy> create_movement(const std::string &type);
};

} // namespace hexapod_gait

#endif // MOVEMENT_FACTORY_HPP_