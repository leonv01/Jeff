/**
 * @file gait_factory.cpp
 * @brief Implementation of GaitFactory create_gait method.
 */

#include "gait_factory/gait_factory.hpp"

#include "gait_strategy/riple_gait_concrete.hpp"
#include "gait_strategy/wave_gait_concrete.hpp"
#include "gait_strategy/tripod_gait_concrete.hpp"

namespace hexapod_gait
{


std::unique_ptr<GaitStrategy> GaitFactory::create_gait(const std::string &type, int total_cycle_steps, double duty_cycle)
{
    if (type == "wave_gait")
    {
        double dc = (duty_cycle > 0.0) ? duty_cycle : (5.0 / 6.0);
        return std::make_unique<WaveGaitConcrete>(dc, total_cycle_steps);
    }
    else if (type == "ripple_gait" || type == "riple_gait")
    {
        double dc = (duty_cycle > 0.0) ? duty_cycle : (4.0 / 6.0);
        return std::make_unique<RipleGaitConcrete>(dc, total_cycle_steps);
    }
    else if (type == "tripod_gait")
    {
        double dc = (duty_cycle > 0.0) ? duty_cycle : (3.0 / 6.0);
        return std::make_unique<TripodGaitConcrete>(dc, total_cycle_steps);
    }

    return nullptr;
}

}