#pragma once

#include "../model/jump_v4.hpp"
#include "wheelbipe.hpp"

namespace rmcs_core::controller::chassis {

inline constexpr PolicyParameters kWheelbipeJump = [] {
    auto p = kWheelbipe;
    p.name = "wheelbipe-jump-v4";
    p.observation_version = "wheelbipe-v202-foundation-jump-v4";
    p.observation_sha = wheelbipe_jump_v4::kObservationSha256;
    p.model_spec_sha256 = wheelbipe_jump_v4::kModelSpecSha256;
    p.leg_kp = wheelbipe_jump_v4::kLegKp;
    p.leg_kd = wheelbipe_jump_v4::kLegKd;
    p.winding_bounds = wheelbipe_jump_v4::kBounds;
    p.jump_control = true;
    return p;
}();

} // namespace rmcs_core::controller::chassis
