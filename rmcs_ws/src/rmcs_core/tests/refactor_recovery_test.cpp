#include <cassert>
#include <cmath>

#include "controller/chassis/wheel_leg/gas_spring.hpp"
#include "controller/chassis/wheel_leg/model/recovery_profile.hpp"

int main() {
    using namespace rmcs_core::controller::chassis::wheel_leg;
    const auto config = make_recovery_config();
    assert(config.prepare_speed_rad_s == 2.);
    assert(config.insert_speed_rad_s == 2.);
    assert(config.orbit_speed_rad_s == 2.);
    assert(config.push_speed_rad_s == 2.);
    assert(config.capture_speed_rad_s == 2.);
    assert(config.side_speed_rad_s == 2.);
    JointReferenceRecoveryController recovery(config);
    JointReferenceRecoveryFeedback feedback;
    feedback.q = config.rl_nominal;
    recovery.reset(feedback.q);
    const auto effort = recovery.torques(feedback, JointReferenceRecoveryVector6::Zero());
    assert(effort.allFinite());
    for (int i = 0; i < 4; ++i)
        assert(std::abs(effort[i]) <= 40.);
    recovery.reset(feedback.q, false);
    assert(!recovery.command().scripted);
    return 0;
}
