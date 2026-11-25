#include "movement_sm/movement_sm.hpp"

#include <cmath>
#include <algorithm>

namespace hexapod_gait
{
    void MovementSM::toggle_movement(void)
    {
        if (current_type_ == MovementType::HOLONOMIC)
        {
            current_type_ = MovementType::NON_HOLONOMIC;
            movement_strategy_ = MovementFactory::create_movement("non-holonomic");
        }
        else if (current_type_ == MovementType::NON_HOLONOMIC)
        {
            current_type_ = MovementType::HOLONOMIC;
            movement_strategy_ = MovementFactory::create_movement("holonomic");
        }
    }

    MovementSM &MovementSM::get_instance()
    {
        static MovementSM obj;
        return obj;
    }
}