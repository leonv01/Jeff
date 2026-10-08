#ifndef POSE_SM_HPP_
#define POSE_SM_HPP_

namespace hexapod_gait
{

enum class RobotPhase
{
    SITTING,
    SITTING_DOWN,
    STANDING_UP,
    STANDING,
    WALKING
};

class HexapodPhaseSM
{
private:
    RobotPhase current_phase_;
    double transition_duration_;
    double progress_;

public:
    static HexapodPhaseSM &get_instance();

    HexapodPhaseSM(double transition_duration = 2.5) : 
        current_phase_(RobotPhase::SITTING),
        transition_duration_(transition_duration),
        progress_(0.0) { }

    void update(double dt, bool has_velocity);
    void toggle_stand_sit(void);
    double get_height_factor(void) const;

    RobotPhase get_phase() const { return current_phase_; }
    bool can_walk() const { return current_phase_ == RobotPhase::WALKING; }
    bool is_sitting() const { return current_phase_ == RobotPhase::SITTING; }
};

};

#endif // POSE_SM_HPP_