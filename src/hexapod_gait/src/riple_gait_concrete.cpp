/**
 * @file riple_gait_concrete.cpp
 * @brief Implementation of concrete RipleGaitConcrete strategy class.
 */

#include "gait_strategy/riple_gait_concrete.hpp"

namespace hexapod_gait
{


RipleGaitConcrete::RipleGaitConcrete(double duty_cycle, double total_cycle_steps) 
    : GaitStrategy(duty_cycle, {
        { LF_LEG, 3.0 / static_cast<double>(LEG_COUNT) },
        { LM_LEG, 1.0 / static_cast<double>(LEG_COUNT) },
        { LR_LEG, 5.0 / static_cast<double>(LEG_COUNT) },
        { RF_LEG, 0.0 / static_cast<double>(LEG_COUNT) },
        { RM_LEG, 4.0 / static_cast<double>(LEG_COUNT) },
        { RR_LEG, 2.0 / static_cast<double>(LEG_COUNT) }
    }, total_cycle_steps)
{
}

std::unordered_map<LEG, LegData> RipleGaitConcrete::propagate_gait(int step, double stride_length, double swing_height, double alpha)
{
    return GaitStrategy::propagate_gait(step, stride_length, swing_height, alpha);
}

LegData RipleGaitConcrete::propagate_leg(LEG leg, int step, double stride_length, double swing_height, double alpha)
{
    return GaitStrategy::propagate_leg(leg, step, stride_length, swing_height, alpha);
}

}