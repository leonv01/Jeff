/**
 * @file gesture_factory.hpp
 * @brief Factory class for instantiating concrete gesture strategy objects.
 */
#ifndef GESTURE_FACTORY_HPP_
#define GESTURE_FACTORY_HPP_

#include "gesture_strategy/gesture_strategy.hpp"
#include <string>
#include <memory>

namespace hexapod_gait
{

/**
 * @class GestureFactory
 * @brief Factory for dynamically instantiating concrete GestureStrategy instances based on string identifier.
 */
class GestureFactory
{
public:
    /**
     * @brief Instantiates a concrete GestureStrategy matching the specified type string.
     * @param type Gesture strategy identifier ("wink").
     * @return std::unique_ptr<GestureStrategy> Unique pointer to constructed GestureStrategy instance, or nullptr if unrecognized type.
     */
    static std::unique_ptr<GestureStrategy> create_gesture(const std::string &type);
};

} // namespace hexapod_gait

#endif // GESTURE_FACTORY_HPP_