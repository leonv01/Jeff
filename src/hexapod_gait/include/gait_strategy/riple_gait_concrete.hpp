/**
 * @file riple_gait_concrete.hpp
 * @brief Concrete implementation of ripple gait pattern (4 stance legs, 2 swing legs).
 */

#ifndef RIPLE_GAIT_CONCRETE_HPP_
#define RIPLE_GAIT_CONCRETE_HPP_

#include "gait_strategy/gait_strategy.hpp"

namespace hexapod_gait
{

/**
 * @class RipleGaitConcrete
 * @brief Ripple gait strategy maintaining 4 legs in stance while 2 legs swing concurrently (Duty Cycle: 4/6 = 0.667).
 */
class RipleGaitConcrete : public GaitStrategy
{
public:
    /**
     * @brief Constructs a RipleGaitConcrete strategy.
     * @param duty_cycle Stance phase duty cycle (default 4.0 / 6.0 ~= 0.667).
     * @param total_cycle_steps Total steps per full gait period (default 60.0).
     */
    RipleGaitConcrete(double duty_cycle = 4.0 / 6.0, double total_cycle_steps = 60.0);

    /**
     * @brief Propagates ripple gait step for all legs.
     * @param step Current step count.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param alpha Walking heading angle in radians.
     * @return std::unordered_map<LEG, LegData> Map of updated leg joint states.
     */
    std::unordered_map<LEG, LegData> propagate_gait(int step, double stride_length = 0.0, double swing_height = 0.0, double alpha = 0.0) override;

    /**
     * @brief Propagates ripple gait step for a single leg.
     * @param leg Target leg ID.
     * @param step Current step count.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param alpha Walking heading angle in radians.
     * @return LegData Updated state for the specified leg.
     */
    LegData propagate_leg(LEG leg, int step, double stride_length = 0.0, double swing_height = 0.0, double alpha = 0.0) override;
};

} // namespace hexapod_gait

#endif // RIPLE_GAIT_CONCRETE_HPP_