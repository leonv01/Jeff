#include "gesture_strategy/wink_gesture_concrete.hpp"
#include "hexapod_gait/hexapod_ik.hpp"
#include <cmath>

namespace hexapod_gait
{
std::unordered_map<LEG, LegData> WinkGestureConcrete::propagate_gesture(
    double dt,
    double step_distance,
    double current_depth
)
{
    std::unordered_map<LEG, LegData> result;

    if (update_progress(dt) == false)
    {
        return result;
    }

    double p = progress_;
    double target_x = step_distance;
    double target_y = 0.0;
    double target_z = current_depth;

    const double lift_height = 0.08;
    const double wave_amplitude = 0.035;
    const int wave_cycles = 2;

    // Lift leg up
    if (p < 0.25)
    {
        double t = p / 0.25;
        target_z = current_depth + 0.5 * (1.0 - std::cos(t * M_PI)) * lift_height;
    }
    // Rotate leg
    else if (p < 0.75)
    {
        target_z = current_depth + lift_height;
        double t = (p - 0.25) / 0.50;
        target_y = wave_amplitude * std::sin(t * wave_cycles * 2.0 * M_PI);
    }
    // Lift leg down
    else
    {
        double t = (p - 0.75) / 0.25;
        target_z = current_depth + 0.5 * (1.0 + std::cos(t * M_PI)) * lift_height;
    }

    Eigen::Vector3d ik = HexapodIk::solve_ik(
        Eigen::Vector3d(
            target_x, target_y, target_z
        )
    );

    LegData data;
    data.coxa_joint_ = ik.x();
    data.femur_joint_ = ik.y();
    data.tibia_joint_ = ik.z();
    data.leg_side_ = get_leg_side(target_leg_);

    result[target_leg_] = data;
    return result;
}
}