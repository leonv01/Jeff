/**
 * @file tripod_gait_concrete.hpp
 * @brief Concrete implementation of tripod gait pattern (3 stance legs, 3 swing legs alternating).
 */

#ifndef TRIPOD_GAIT_CONCRETE_HPP_
#define TRIPOD_GAIT_CONCRETE_HPP_

#include "gait_strategy/gait_strategy.hpp"

namespace hexapod_gait
{

/**
 * @class TripodGaitConcrete
 * @brief Tripod gait strategy dividing the 6 legs into two alternating sets of 3 legs (Duty Cycle: 0.5).
 */
class TripodGaitConcrete : public GaitStrategy
{
public:
    /**
     * @brief Constructs a TripodGaitConcrete strategy.
     * @param duty_cycle Stance phase duty cycle (default 3.0 / 6.0 = 0.5).
     * @param total_cycle_steps Total steps per full gait period (default 60.0).
     */
    TripodGaitConcrete(double duty_cycle = 3.0 / 6.0, double total_cycle_steps = 60.0);

    /**
     * @brief Propagates tripod gait step for all legs.
     * @param step Current step count.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param relative_target Target position relative to the hip.
     * @param alpha Walking heading angle in radians.
     * @return std::unordered_map<LEG, LegData> Map of updated leg joint states.
     */
    std::unordered_map<LEG, LegData> propagate_gait(int step, double stride_length = 0.0, double swing_height = 0.0, const Eigen::Vector3d &relative_target = Eigen::Vector3d(0.08, 0.0, -0.08), double alpha = 0.0) override;

    /**
     * @brief Propagates tripod gait step for a single leg.
     * @param leg Target leg ID.
     * @param step Current step count.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param relative_target Target position relative to the hip.
     * @param alpha Walking heading angle in radians.
     * @return LegData Updated state for the specified leg.
     */
    LegData propagate_leg(LEG leg, int step, double stride_length = 0.0, double swing_height = 0.0, const Eigen::Vector3d &relative_target = Eigen::Vector3d(0.08, 0.0, -0.08), double alpha = 0.0) override;
};

} // namespace hexapod_gait

#endif // TRIPOD_GAIT_CONCRETE_HPP_