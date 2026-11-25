/**
 * @file movement_sm.hpp
 * @brief Class to manage movement state machine.
 */

#ifndef MOVEMENT_SM_HPP_
#define MOVEMENT_SM_HPP_

#include "movement_strategy/movement_strategy.hpp"
#include "movement_factory/movement_factory.hpp"
#include <memory>

namespace hexapod_gait
{

/// @brief Enum for movement types.
enum class MovementType
{
    HOLONOMIC,
    NON_HOLONOMIC
};

/**
 * @class MovementSM
 * @brief Singleton class for managing the state machine for movement strategies.
 */
class MovementSM
{
private:
    /// @brief Current movement type.
    MovementType current_type_;
    /// @brief Unique pointer to the movement strategy.
    std::unique_ptr<MovementStrategy> movement_strategy_;

public:
    /**
     * @brief Constructor for the MovementSM class. Creates an instance if none is active.
     * @return Singleton object of the MovementSM class.
     */
    static MovementSM &get_instance();

    /**
     * @brief Destructor for the MovementSM class.
     */
    ~MovementSM() = default;

    /**
     * @brief Constructor for the MovementSM class. Creates an instance if none is active.
     */
    MovementSM() : 
        current_type_(MovementType::HOLONOMIC)
        { 
            movement_strategy_ = MovementFactory::create_movement("holonomic");
        };

    /// @brief Toggle between different movement types when called.
    void toggle_movement(void);

    /**
     * @brief Get the current movement type.
     * @return The current movement type.
     */
    MovementType get_type() const { return current_type_; }

    /**
     * @brief Get the current movement strategy.
     * @return A reference to the current movement strategy.
     */
    std::unique_ptr<MovementStrategy> &get_movement_strategy() { return movement_strategy_; }
};

} // namespace hexapod_gait

#endif // MOVEMENT_SM_HPP_