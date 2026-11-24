#include "gesture_factory/gesture_factory.hpp"

#include "gesture_strategy/wink_gesture_concrete.hpp"

namespace hexapod_gait
{


std::unique_ptr<GestureStrategy> GestureFactory::create_gesture(const std::string &type)
{
    if (type == "wink_gesture" || type == "wink")
    {
        return std::make_unique<WinkGestureConcrete>();
    }

    return nullptr;
}

}