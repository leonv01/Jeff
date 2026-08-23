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


std::unique_ptr<GaitStrategy> GaitFactory::create_gait(const std::string &type, double duty_cycle)
{
    if (type == "wave_gait")
    {
        if (duty_cycle > 0.0)
        {
            return std::make_unique<WaveGaitConcrete>(duty_cycle);
        }
        return std::make_unique<WaveGaitConcrete>();
    }
    else if (type == "ripple_gait" || type == "riple_gait")
    {
        if (duty_cycle > 0.0)
        {
            return std::make_unique<RipleGaitConcrete>(duty_cycle);
        }
        return std::make_unique<RipleGaitConcrete>();
    }
    else if (type == "tripod_gait")
    {
        if (duty_cycle > 0.0)
        {
            return std::make_unique<TripodGaitConcrete>(duty_cycle);
        }
        return std::make_unique<TripodGaitConcrete>();
    }

    return nullptr;
}

}