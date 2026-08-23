/**
 * @file gait_factory.hpp
 * @brief Factory class for instantiating concrete gait strategy objects.
 */

#ifndef GAIT_FACTORY_HPP_
#define GAIT_FACTORY_HPP_

#include "gait_strategy/gait_strategy.hpp"
#include <string>
#include <memory>

namespace hexapod_gait
{

/**
 * @class GaitFactory
 * @brief Factory for dynamically instantiating concrete GaitStrategy instances based on string identifier.
 */
class GaitFactory
{
public:
    /**
     * @brief Instantiates a concrete GaitStrategy matching the specified type string.
     * @param type Gait strategy identifier ("tripod_gait", "wave_gait", "ripple_gait" or "riple_gait").
     * @param duty_cycle Optional custom duty cycle ratio. If <= 0.0, default stance duty cycle of the strategy is used.
     * @return std::unique_ptr<GaitStrategy> Unique pointer to constructed GaitStrategy instance, or nullptr if unrecognized type.
     */
    static std::unique_ptr<GaitStrategy> create_gait(const std::string &type, double duty_cycle = -1.0);
};

} // namespace hexapod_gait

#endif // GAIT_FACTORY_HPP_