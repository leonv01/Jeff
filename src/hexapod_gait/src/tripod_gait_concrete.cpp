/**
 * @file tripod_gait_concrete.cpp
 * @brief Implementation of concrete TripodGaitConcrete strategy class.
 */

#include "gait_strategy/tripod_gait_concrete.hpp"

#include <cmath>

namespace hexapod_gait
{

TripodGaitConcrete::TripodGaitConcrete(double duty_cycle, double total_cycle_steps) 
    : GaitStrategy(duty_cycle, {
        { LF_LEG, 0.5},
        { LM_LEG, 0.0},
        { LR_LEG, 0.5},
        { RF_LEG, 0.0},
        { RM_LEG, 0.5},
        { RR_LEG, 0.0}
    }, total_cycle_steps)
{
}

std::unordered_map<LEG, LegData> TripodGaitConcrete::propagate_gait(int step, double stride_length, double swing_height, double alpha)
{
    return GaitStrategy::propagate_gait(step, stride_length, swing_height, alpha);
}

LegData TripodGaitConcrete::propagate_leg(LEG leg, int step, double stride_length, double swing_height, double alpha)
{
    return GaitStrategy::propagate_leg(leg, step, stride_length, swing_height, alpha);
}

}