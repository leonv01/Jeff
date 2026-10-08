#ifndef MOVEMENT_SM_HPP_
#define MOVEMENT_SM_HPP_

#include "movement_strategy/movement_strategy.hpp"
#include "movement_factory/movement_factory.hpp"
#include <memory>

namespace hexapod_gait
{

enum class MovementType
{
    HOLONOMIC,
    NON_HOLONOMIC
};

class MovementSM
{
private:
    MovementType current_type_;
    std::unique_ptr<MovementStrategy> movement_strategy_;

public:
    static MovementSM &get_instance();

    MovementSM() : 
        current_type_(MovementType::HOLONOMIC)
        { 
            movement_strategy_ = MovementFactory::create_movement("holonomic");
        };

    void toggle_movement(void);

    MovementType get_type() const { return current_type_; }

    std::unique_ptr<MovementStrategy> &get_movement_strategy() { return movement_strategy_; }
};

};

#endif // MOVEMENT_SM_HPP_