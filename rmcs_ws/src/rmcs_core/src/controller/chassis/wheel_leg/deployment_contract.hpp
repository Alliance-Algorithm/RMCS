#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

#include "gas_spring_model.hpp"
#include "leg_target_winding.hpp"
#include "policy.hpp"
#include "unified_observation.hpp"

namespace rmcs_core::controller::chassis {

// Startup requirements of a trained actor, separate from tunable command limits.
struct DeploymentContract {
    enum class GasFeedforward { kOptional, kRequired, kDisabled };

    UnifiedObservation::Version observation;
    LegTargetWinding winding;
    std::string_view model_spec_sha256, gas_model_sha256;
    double pd_hz, leg_scale, leg_bound, wheel_bound;
    GasFeedforward gas_feedforward;
    bool fixed_command_limits;
};

inline constexpr DeploymentContract kStandingDeployment{
    UnifiedObservation::Version::kStandingV3,
    LegTargetWinding::kPairedBranch107,
    kBranch107ModelSpecSha256,
    gas_spring_model::kModelSha256,
    1000.0,
    .25,
    4.0,
    90.0,
    DeploymentContract::GasFeedforward::kOptional,
    false};

inline constexpr DeploymentContract kProneV4Deployment = [] {
    auto contract = kStandingDeployment;
    contract.observation = UnifiedObservation::Version::kProneV4;
    contract.model_spec_sha256 = full_vehicle_v202::kModelSpecSha256;
    contract.gas_model_sha256 = full_vehicle_v202::kGasModelSha256;
    contract.gas_feedforward = DeploymentContract::GasFeedforward::kRequired;
    contract.fixed_command_limits = true;
    return contract;
}();

inline constexpr DeploymentContract kProne200Deployment = [] {
    auto contract = kProneV4Deployment;
    contract.observation = UnifiedObservation::Version::kProneV4OdomV2;
    contract.model_spec_sha256 = kFrictionV202ModelSpecSha256;
    contract.gas_model_sha256 = kFrictionV202GasModelSha256;
    contract.pd_hz = 200.0;
    return contract;
}();

inline constexpr DeploymentContract kWheelbipeDeployment = [] {
    auto contract = kProne200Deployment;
    contract.observation = UnifiedObservation::Version::kWheelbipeV202;
    contract.winding = LegTargetWinding::kPairedFullVehicle110;
    contract.leg_scale = .5;
    // V14 has no raw action clip; physical wheel targets saturate downstream.
    contract.leg_bound = contract.wheel_bound = 1e100;
    contract.gas_feedforward = DeploymentContract::GasFeedforward::kDisabled;
    contract.fixed_command_limits = false;
    return contract;
}();

inline constexpr const DeploymentContract& deployment_contract(std::string_view profile) {
    if (profile == kWheelbipePolicyProfile.name)
        return kWheelbipeDeployment;
    if (profile == kProne200PolicyProfile.name)
        return kProne200Deployment;
    if (profile == kFullVehicleV202PolicyProfile.name)
        return kProneV4Deployment;
    return kStandingDeployment;
}

inline UnifiedObservation::Version parse_observation_version(std::string_view name) {
    using Version = UnifiedObservation::Version;
    if (name == "standing_reference_v3")
        return Version::kStandingV3;
    if (name == "wheel-leg-prone-motion-v4")
        return Version::kProneV4;
    if (name == "wheel-leg-prone-motion-v4-odom-v2")
        return Version::kProneV4OdomV2;
    if (name == "wheelbipe-v202-foundation-v2")
        return Version::kWheelbipeV202;
    throw std::runtime_error("Unsupported wheel-leg observation_version: " + std::string{name});
}

} // namespace rmcs_core::controller::chassis
