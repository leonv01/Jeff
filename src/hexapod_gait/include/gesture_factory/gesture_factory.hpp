#ifndef GESTURE_FACTORY_HPP_
#define GESTURE_FACTORY_HPP_

#include "gesture_strategy/gesture_strategy.hpp"
#include <string>
#include <memory>

namespace hexapod_gait
{

class GestureFactory
{
public:
    static std::unique_ptr<GestureStrategy> create_gesture(const std::string &type);
};

} // namespace hexapod_gait

#endif // GESTURE_FACTORY_HPP_