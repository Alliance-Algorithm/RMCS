// Offline differential replay. The verifier also compiles this against the
// deployed branch's original sources; neither executable constructs hardware.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "chassis_body_geometry.hpp"
#include "joint_reference_recovery_controller.hpp"
#include "policy.hpp"
#include "recovery_profile.hpp"

#ifndef RMCS_WHEEL_LEG_REFERENCE
# include "policy/policy_model.hpp"
#endif

namespace {
#ifdef RMCS_WHEEL_LEG_REFERENCE
using namespace rmcs::rl;
#else
using namespace rmcs_core::controller::chassis;
#endif
using Vector6 = JointReferenceRecoveryVector6;

void replay_recovery(const char* profile, const char* lookup, const nlohmann::json& parameters) {
    auto config = RecoveryProfile::load(profile, lookup).controller;
    config.dt = 1.0 / parameters.at("recovery_frequency").get<double>();
    config.blend_seconds = parameters.at("recovery_blend_seconds");
    config.dynamic_takeover = parameters.at("recovery_dynamic_takeover");
    config.release_motion_on_takeover = parameters.at("recovery_release_motion_on_takeover");
    config.dynamic_capture_height_min = parameters.at("recovery_capture_height_min");
    config.dynamic_capture_height_max = parameters.at("recovery_capture_height_max");
    config.prepare_speed_rad_s = parameters.at("recovery_fold_plant_speed_rad_s");
    config.orbit_speed_rad_s = parameters.at("recovery_orbit_speed_rad_s");
    config.insert_speed_rad_s = parameters.at("recovery_insert_speed_rad_s");
    if (config.dynamic_takeover)
        for (int i = 0; i < 4; ++i)
            config.nominal[i] = parameters.at("recovery_capture_joint_positions").at(i);
    Vector6 nominal, scales;
    for (int i = 0; i < 6; ++i) {
        nominal[i] = parameters.at("default_joint_pos").at(i);
        scales[i] = i < 4 ? 0.25 : 10.0;
    }
    for (int scenario = 0; scenario < 8; ++scenario) {
        JointReferenceRecoveryController recovery{config};
        JointReferenceRecoveryFeedback feedback;
        feedback.q = nominal;
        // Include common encoder turns and a passive initial release.
        feedback.q.head<2>().array() += (scenario % 3 - 1) * 2 * std::numbers::pi;
        recovery.reset(feedback.q, true, scenario == 7 ? 0.04 : 0.0);
        for (int tick = 0; tick < 800; ++tick) {
            const double progress = std::clamp((tick - 80) / 500.0, 0.0, 1.0);
            const double pitch = scenario == 2                  ? 2.8 * (1.0 - progress)
                               : scenario == 3                  ? -2.8 * (1.0 - progress)
                               : scenario == 4 || scenario == 5 ? -0.2
                                                                : 0.0;
            const double roll = scenario == 4 ? 1.5 * (1.0 - progress)
                              : scenario == 5 ? -1.5 * (1.0 - progress)
                                              : 0.0;
            feedback.gravity = Eigen::Vector3d{
                std::sin(pitch), std::cos(pitch) * std::sin(roll),
                -std::cos(pitch) * std::cos(roll)};
            feedback.gyro = Eigen::Vector3d{0.05 * std::sin(tick * 0.02), 0.08, 0.0};
            feedback.support = !(scenario == 6 && tick > 300 && tick < 480);
            feedback.support_confirmed = feedback.support && tick > 100;
            feedback.body_clear = tick > 200;
            feedback.estimated_height = scenario == 0 && tick < 200 ? 0.201
                                      : scenario == 1               ? 0.27
                                                                    : 0.19 + 0.07 * progress;
            feedback.rl_capture_ready = tick > 70 && -feedback.gravity.z() > 0.87;
            feedback.wheel_probe_torque = Eigen::Vector2d{tick % 30 < 15 ? 0.18 : -0.18, 0.0};
            // A deterministic feedback trace with bounded tracking lag. This is
            // input replay, not a physics model or an acceptance test.
            const Vector6 delta = recovery.command().targets - feedback.q;
            feedback.dq.head<4>() = 0.03 * delta.head<4>() / config.dt;
            feedback.q.head<4>() += 0.03 * delta.head<4>();
            feedback.dq.tail<2>() = recovery.command().targets.tail<2>() * 0.8;
            feedback.q.tail<2>() += feedback.dq.tail<2>() * config.dt;
            const auto& command = recovery.update(feedback, config.dt);
            Vector6 actor;
            for (int i = 0; i < 6; ++i)
                actor[i] = static_cast<float>(std::sin(tick * 0.03 + i) * (i < 4 ? 12 : 2));
            const Vector6 effort = recovery.torques(feedback, actor);
#ifdef RMCS_WHEEL_LEG_REFERENCE
            const Vector6 history = recovery.effective_action_history(actor);
#else
            const Vector6 history = recovery.effective_action_history(actor, nominal, scales);
#endif
            std::cout << "recovery " << scenario << ' ' << tick << ' '
                      << static_cast<int>(command.phase) << ' ' << static_cast<int>(command.route)
                      << ' ' << command.failure_code << ' ' << command.motion_hold << ' '
                      << command.release_finished << ' ' << command.pure_rl << ' ' << command.blend;
            for (const auto& values : {command.targets, command.continuous_q, effort, history})
                for (const auto value : values)
                    std::cout << ' ' << value;
            std::cout << '\n';
        }
    }
}

void replay_model(const char* path) {
#ifdef RMCS_WHEEL_LEG_REFERENCE
    OnnxPolicy model{path, "paired_branch107_v1"};
#else
    rmcs_rl::PolicyConfig config;
    config.path = path;
    config.model_type = "wheel_leg_v7";
    config.expected_leg_winding_version = "paired_branch107_v1";
    config.observation_size = config.feature_size = 35;
    config.sequence_length = 1;
    config.action_size = 6;
    rmcs_rl::PolicyModel model{config};
#endif
    for (int sample = 0; sample < 256; ++sample) {
        std::array<float, 35> input;
        for (std::size_t i = 0; i < input.size(); ++i)
            input[i] = static_cast<float>(3 * std::sin(sample * 0.17 + i * 0.23));
#ifdef RMCS_WHEEL_LEG_REFERENCE
        const auto result = model.run(input);
        if (!result)
            throw std::runtime_error(result.error());
        const auto& output = *result;
#else
        const std::array<double, 35> observation = [&] {
            std::array<double, 35> values;
            std::ranges::copy(input, values.begin());
            return values;
        }();
        std::array<double, 6> output;
        std::string error;
        if (!model.run(observation, output, error))
            throw std::runtime_error(error);
#endif
        std::cout << "model " << sample;
        for (const auto value : output)
            std::cout << ' ' << value;
        std::cout << '\n';
    }
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 5)
        return 2;
    try {
        std::ifstream settings{argv[4]};
        nlohmann::json parameters;
        settings >> parameters;
        std::cout << std::setprecision(17);
        replay_recovery(argv[1], argv[2], parameters);
        replay_model(argv[3]);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
