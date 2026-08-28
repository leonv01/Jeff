/**
 * @file wave_gait_concrete.hpp
 * @brief Concrete implementation of wave gait pattern (5 stance legs, 1 swing leg sequentially).
 */

#ifndef WAVE_GAIT_CONCRETE_HPP_
#define WAVE_GAIT_CONCRETE_HPP_

#include "gait_strategy/gait_strategy.hpp"

namespace hexapod_gait
{

/**
 * @class WaveGaitConcrete
 * @brief Wave gait strategy moving 1 leg at a time while 5 legs maintain ground contact for maximum stability (Duty Cycle: 5/6).
 */
class WaveGaitConcrete : public GaitStrategy
{
public:
    /**
     * @brief Constructs a WaveGaitConcrete strategy.
     * @param duty_cycle Stance phase duty cycle (default 5.0 / 6.0 ~= 0.833).
     * @param total_cycle_steps Total steps per full gait period (default 60.0).
     */
    WaveGaitConcrete(double duty_cycle = 5.0 / 6.0, double total_cycle_steps = 60.0);

    /**
     * @brief Propagates wave gait step for all legs.
     * @param step Current step count.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param relative_target Target coordinate relative to home position.
     * @param alpha Walking heading angle in radians.
     * @return std::unordered_map<LEG, LegData> Map of updated leg joint states.
     */
    std::unordered_map<LEG, LegData> propagate_gait(int step, double stride_length = 0.0, double swing_height = 0.0, const Eigen::Vector3d &relative_target = Eigen::Vector3d(0.08, 0.0, -0.08), double alpha = 0.0) override;

    /**
     * @brief Propagates wave gait step for a single leg.
     * @param leg Target leg ID.
     * @param step Current step count.
     * @param stride_length Target stride length in meters.
     * @param swing_height Maximum swing height in meters.
     * @param relative_target Target coordinate relative to home position.
     * @param alpha Walking heading angle in radians.
     * @return LegData Updated state for the specified leg.
     */
    LegData propagate_leg(LEG leg, int step, double stride_length = 0.0, double swing_height = 0.0, const Eigen::Vector3d &relative_target = Eigen::Vector3d(0.08, 0.0, -0.08), double alpha = 0.0) override;
};

} // namespace hexapod_gait

#endif // WAVE_GAIT_CONCRETE_HPP_