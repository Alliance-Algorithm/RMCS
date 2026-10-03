#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "rl_controller.hpp"

// This is a separate executable from rl_controller_test. Exercise production
// Component registrations through Executor's port-binding friendship without
// accessing controller state, starting CAN devices or constructing an Executor.
namespace rmcs_executor {
class Executor {
public:
    template <typename T>
    static void bind(Component& component, const std::string& name, T& value) {
        for (auto& port : component.input_list_) {
            if (port.name != name)
                continue;
            if (port.type != typeid(T) || port.kind != InterfaceKind::Normal)
                throw std::runtime_error("Wrong input type: " + name);
            port.bind(port.binding, &value);
            return;
        }
        throw std::runtime_error("Input is not registered: " + name);
    }

    template <typename T>
    static const T& output(Component& component, const std::string& name) {
        for (auto& port : component.output_list_) {
            if (port.name != name)
                continue;
            if (port.type != typeid(T) || port.kind != InterfaceKind::Normal)
                throw std::runtime_error("Wrong output type: " + name);
            return *static_cast<const T*>(port.binding);
        }
        throw std::runtime_error("Output is not registered: " + name);
    }

    static void require_bound_inputs(Component& component) {
        for (auto& port : component.input_list_) {
            void* destination = nullptr;
            std::memcpy(&destination, port.binding, sizeof(destination));
            if (!destination)
                throw std::runtime_error("Unbound controller input: " + port.name);
        }
    }
};
} // namespace rmcs_executor

namespace rmcs::rl {
namespace {

using DeployedPolicyContract = LegacyPolicyContract;

class RecoveryHandoverTest : public ::testing::Test {
protected:
    using Clock = std::chrono::steady_clock;
    using Ports = rmcs_executor::Executor;

    void SetUp() override {
        // Synthetic, symmetric closed-chain geometry and identity calibration
        // keep q constant while testing the actual deployed V5 actor. These
        // parameters are neither hardware calibration nor a dynamics model.
        std::vector<std::string> args{"recovery_handover_test",   "--ros-args",  "--params-file",
                                      RL_CONTROLLER_TEST_PROFILE, "--log-level", "error"};
        std::vector<std::string> parameters{
            std::string{"rl_model_path:="} + RL_CONTROLLER_LEGACY_TEST_MODEL,
            "calibration_ready:=true", "soft_limits_ready:=true", "imu_alignment_ready:=true",
            "policy_profile:=v5_flat_12486",
            "nominal_model_pos:=[0.42,-0.13742282595254576,-0.42,0.13741557625658019,0.0,0.0]",
            "leg_motor_to_model:=[1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,"
            "0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]",
            "leg_model_offsets:=[0.0,0.0,0.0,0.0]", "wheel_model_scale:=[1.0,1.0]",
            "hinge_coefficients:=[-1.0,1.0,-1.0,1.0]", "hinge_min:=[-2.0,-2.0]",
            "hinge_max:=[2.0,2.0]", "recovery_enabled:=true", "recovery_profile_ready:=true",
            "recovery_above_rated_budget_s:=1.0",
            "recovery_dm_feedback_position_max:=[12.5,12.5,12.5,12.5]", "recovery_orbit_speed:=5.0",
            "recovery_side_speed:=5.0", "recovery_rollover_speed:=5.25",
            "recovery_capture_speed:=4.0", "recovery_spring_stroke_m:=0.1",
            "recovery_spring_force_n:=[280.0,0.0,0.0,0.0]",
            // Offset this dwell from the policy period so handover also tests
            // an entry between scheduled 50 Hz evaluations.
            "recovery_probe_quiet_s:=0.062",
            "recovery_shell_points_body_m:=[-0.1,-0.1,-0.1,-0.1,0.1,-0.1,"
            "0.1,-0.1,-0.1,0.1,0.1,-0.1]"};
        for (const auto* pose :
             {"fold", "thrust", "side_extended", "stand", "upright", "support_extended",
              "upright_support_extended", "capture_extended"})
            parameters.push_back(std::string{"recovery_"} + pose + "_p4:=[0.5,0.0,-0.5,0.0]");
        for (const auto* side : {"left", "right"}) {
            const auto prefix = std::string{"recovery_"} + side;
            parameters.push_back(prefix + "_delta_rad:=[-1.0,1.0]");
            parameters.push_back(prefix + "_inner_knee_deg:=[50.0,100.0]");
            parameters.push_back(prefix + "_slider_m:=[0.02,0.04]");
            parameters.push_back(prefix + "_slider_slope_m_per_rad:=[0.01,0.01]");
            parameters.push_back(prefix + "_knee_axis_at_hip_zero:=[0.0,1.0,0.0,0.0,1.0,0.0]");
            parameters.push_back(prefix + "_wheel_axis_at_hip_zero:=[0.0,1.0,0.0,0.0,1.0,0.0]");
            parameters.push_back(prefix + "_passive_knee_sign:=1.0");
            parameters.push_back(prefix + "_hip_origin_m:=[0.0,0.0,0.0]");
            parameters.push_back(prefix + "_hip_axis:=[0.0,1.0,0.0]");
            parameters.push_back(prefix + "_spring_compression_at_zero_m:=0.05");
            const auto y = std::string_view{side} == "left" ? "0.2" : "-0.2";
            parameters.push_back(
                prefix + "_wheel_at_hip_zero_m:=[0.0," + y + ",-0.27,0.0," + y + ",-0.27]");
        }
        for (const auto& parameter : parameters) {
            args.push_back("-p");
            args.push_back(parameter);
        }
        std::vector<const char*> argv;
        for (const auto& arg : args)
            argv.push_back(arg.c_str());
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
        rmcs_executor::Component::initializing_component_name = "rl_controller";
        controller_ = std::make_unique<RlController>();
        reference_policy_ = std::make_unique<OnnxPolicy>(
            RL_CONTROLLER_LEGACY_TEST_MODEL, LegacyPolicyContract::kSha256);

        bind("/predefined/update_count", tick_);
        bind("/predefined/update_rate", rate_);
        bind("/predefined/timestamp", time_);
        bind("/wheel_leg/feedback_fresh", feedback_fresh_);
        bind("/wheel_leg/imu/quaternion", orientation_);
        bind("/wheel_leg/imu/angular_velocity", gyro_);
        bind("/wheel_leg/imu/acceleration", acceleration_);
        bind("/wheel_leg/imu/last_steady_ns", imu_ns_);
        bind("/wheel_leg/imu/sequence", imu_sequence_);
        bind("/wheel_leg/imu/acceleration_steady_ns", acceleration_ns_);
        bind("/wheel_leg/imu/acceleration_sequence", acceleration_sequence_);
        bind("/wheel_leg/dm_control_ready", ready_);
        bind("/chassis/control_velocity", velocity_command_);
        bind("/chassis/control_height", height_);
        bind("/chassis/control_state", requested_state_);
        bind("/chassis/reset_count", reset_count_);
        bind("/chassis/control_mode", mode_);
        bind("/chassis/jump_request", jump_);
        bind("/chassis/jump_apex_delta", jump_apex_);
        for (std::size_t i = 0; i < kMotorNames.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kMotorNames[i];
            bind(prefix + "/angle", angle_[i]);
            bind(prefix + "/velocity", velocity_[i]);
            bind(prefix + "/torque", feedback_torque_[i]);
            bind(prefix + "/max_torque", max_torque_[i]);
            bind(prefix + "/feedback_sequence", sequence_[i]);
            bind(prefix + "/feedback_steady_ns", feedback_ns_[i]);
            if (i < 4)
                bind(prefix + "/fault_code", fault_[i]);
            else {
                bind(prefix + "/last_submitted_torque", wheel_submitted_torque_[i - 4]);
                bind(prefix + "/last_submitted_kind", wheel_submitted_kind_[i - 4]);
                bind(prefix + "/last_submitted_steady_ns", wheel_submitted_ns_[i - 4]);
            }
        }
        Ports::require_bound_inputs(*controller_);
        controller_->before_updating();
        ASSERT_EQ(state(), State::kInit);
    }

    void TearDown() override {
        reference_policy_.reset();
        controller_.reset();
        rclcpp::shutdown();
    }

    template <typename T>
    void bind(const std::string& name, T& value) {
        Ports::bind(*controller_, name, value);
    }

    template <typename T>
    const T& output(const std::string& name) {
        return Ports::output<T>(*controller_, "/wheel_leg/" + name);
    }

    State state() { return static_cast<State>(output<int>("rl/state")); }
    RecoveryPhase phase() { return static_cast<RecoveryPhase>(output<int>("rl/recovery/phase")); }
    double torque(std::size_t axis) {
        return output<double>(std::string{kMotorNames[axis]} + "/control_torque");
    }

    PolicyObservation observation() {
        PolicyObservation value;
        for (std::size_t i = 0; i < value.size(); ++i)
            value[i] =
                output<double>(std::string{"rl/observation/"} + std::string{kObservationNames[i]});
        return value;
    }

    PolicyAction action() {
        PolicyAction value;
        for (std::size_t i = 0; i < value.size(); ++i)
            value[i] = output<double>(std::string{"rl/action/"} + kMotorNames[i]);
        return value;
    }

    static std::uint64_t steady_ns() {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count());
    }

    void step() {
        std::this_thread::sleep_until(next_update_);
        next_update_ += std::chrono::milliseconds{5};
        // Four intervening executor ticks would hold the last 200 Hz effort.
        tick_ += 5;
        time_ = Clock::now();
        const auto stamp = steady_ns();
        for (std::size_t i = 0; i < sequence_.size(); ++i) {
            ++sequence_[i];
            feedback_ns_[i] = stamp;
        }
        imu_ns_ = acceleration_ns_ = stamp;
        ++imu_sequence_;
        ++acceleration_sequence_;
        // A small changing gyro marker makes every scheduled inference visible
        // through its published observation, with low angular acceleration.
        gyro_.x() = static_cast<double>(tick_) * 0.0001;
        for (std::size_t side = 0; side < 2; ++side)
            feedback_torque_[4 + side] = wheel_submitted_torque_[side];
        controller_->update();
    }

    void submit_wheel_outputs() {
        // A synthetic CAN adapter records exactly what production emitted.
        // Its next sampled current reply follows this submission in steady time.
        const auto stamp = steady_ns();
        for (std::size_t side = 0; side < 2; ++side) {
            wheel_submitted_torque_[side] = torque(4 + side);
            wheel_submitted_kind_[side] = 1;
            wheel_submitted_ns_[side] = stamp;
        }
    }

    std::unique_ptr<RlController> controller_;
    std::unique_ptr<OnnxPolicy> reference_policy_;
    std::size_t tick_ = 0, reset_count_ = 0;
    double rate_ = 1000.0;
    Clock::time_point time_ = Clock::now(), next_update_ = Clock::now();
    bool feedback_fresh_ = true, ready_ = false, jump_ = false;
    int requested_state_ = 3;
    double height_ = 0.305, jump_apex_ = 0.06;
    rmcs_msgs::ChassisMode mode_ = rmcs_msgs::ChassisMode::AUTO;
    rmcs_description::BaseLink::DirectionVector velocity_command_{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_{0.0, 0.0, 9.81};
    std::uint64_t imu_ns_ = 0, acceleration_ns_ = 0;
    std::uint64_t imu_sequence_ = 0, acceleration_sequence_ = 0;
    std::array<double, 6> angle_{0.5, 0.0, -0.5, 0.0, 0.0, 0.0};
    std::array<double, 6> velocity_{}, feedback_torque_{};
    std::array<double, 6> max_torque_{40.0, 40.0, 40.0, 40.0, 4.5, 4.5};
    std::array<int, 4> fault_{};
    std::array<std::uint64_t, 6> sequence_{}, feedback_ns_{};
    std::array<double, 2> wheel_submitted_torque_{};
    std::array<std::uint8_t, 2> wheel_submitted_kind_{};
    std::array<std::uint64_t, 2> wheel_submitted_ns_{};
};

TEST_F(RecoveryHandoverTest, SubmittedProbesPermitBlendAndPreservePolicyClockAndHistory) {
    // Set the cadence after ROS/ONNX setup, which is not a controller iteration.
    next_update_ = Clock::now();
    const auto start = next_update_;
    // The first sample initializes the observer while MIT enable is pending.
    // A subsequent fresh sample can establish height and select upright PREPARE.
    step();
    ASSERT_EQ(state(), State::kPrepare);
    ASSERT_EQ(phase(), RecoveryPhase::kIdle);
    for (std::size_t axis = 0; axis < kMotorNames.size(); ++axis)
        ASSERT_DOUBLE_EQ(torque(axis), 0.0);
    submit_wheel_outputs();
    ready_ = true;
    std::array<std::array<bool, 2>, 2> submitted_directions{}, responded_directions{};
    PolicyAction history{}, cached_policy_action{};
    PolicyObservation held_observation{};
    bool saw_prepare = false, saw_blend = false, saw_off_clock_boundary = false;
    bool checked_shadow_target_hold = false;
    std::size_t last_policy_tick = 0, shadow_evaluations = 0, blend_evaluations = 0;
    std::size_t rl_evaluations = 0, rl_steps = 0;
    std::size_t blend_entry_tick = 0, first_blend_policy_tick = 0, rl_entry_tick = 0;

    for (int iteration = 0; iteration < 150 && rl_steps < 9; ++iteration) {
        const auto previous_phase = phase();
        const auto previous_state = state();
        step();
        ASSERT_NE(state(), State::kIdle)
            << "phase=" << static_cast<int>(phase())
            << " failure=" << output<int>("rl/recovery/failure")
            << " sensor_issue=" << output<int>("rl/recovery/sensor_issue");
        ASSERT_TRUE(output<bool>("enable_request"));
        ASSERT_TRUE(output<bool>("rl/recovery/sensors_valid"));
        ASSERT_TRUE(output<bool>("rl/recovery/geometry_valid"));
        ASSERT_TRUE(output<bool>("rl/recovery/motion_hold"));
        const bool shadow = state() == State::kPrepare && phase() != RecoveryPhase::kBlend;
        const bool due = last_policy_tick == 0 || tick_ - last_policy_tick == 20;
        const auto observed = observation();

        if (due) {
            EXPECT_FLOAT_EQ(observed[ObservationLayout::kAngularVelocity], gyro_.x() * 0.5);
            for (std::size_t i = 0; i < history.size(); ++i)
                EXPECT_FLOAT_EQ(observed[ObservationLayout::kPreviousAction + i], history[i]);
            const auto raw = reference_policy_->run(observed);
            ASSERT_TRUE(raw.has_value()) << raw.error();
            for (std::size_t i = 0; i < cached_policy_action.size(); ++i) {
                const float limit = i < 4 ? DeployedPolicyContract::kLegActionLimit
                                          : DeployedPolicyContract::kWheelActionLimit;
                cached_policy_action[i] = std::clamp((*raw)[i], -limit, limit);
            }
            if (shadow) {
                ++shadow_evaluations;
                EXPECT_EQ(action(), PolicyAction{});
            } else {
                history = cached_policy_action;
                EXPECT_EQ(action(), history);
                if (state() == State::kRl)
                    ++rl_evaluations;
                else {
                    if (blend_evaluations == 0)
                        first_blend_policy_tick = tick_;
                    ++blend_evaluations;
                }
            }
            held_observation = observed;
            last_policy_tick = tick_;
        } else {
            EXPECT_EQ(observed, held_observation) << "Unexpected inference at tick " << tick_;
            EXPECT_EQ(action(), history);
        }

        if (phase() == RecoveryPhase::kPrepare) {
            saw_prepare = true;
            for (std::size_t side = 0; side < 2; ++side) {
                const double current = feedback_torque_[4 + side];
                if (std::abs(current) >= 0.14) {
                    EXPECT_GT(feedback_ns_[4 + side], wheel_submitted_ns_[side]);
                    responded_directions[side][current > 0.0] = true;
                }
                const double effort = torque(4 + side);
                if (std::abs(effort) >= 0.14) {
                    EXPECT_DOUBLE_EQ(std::abs(effort), 0.18);
                    submitted_directions[side][effort > 0.0] = true;
                }
            }
            if (output<bool>("rl/recovery/support_confirmed"))
                for (std::size_t side = 0; side < 2; ++side)
                    for (std::size_t direction = 0; direction < 2; ++direction) {
                        EXPECT_TRUE(submitted_directions[side][direction]);
                        EXPECT_TRUE(responded_directions[side][direction]);
                    }
        } else if (phase() == RecoveryPhase::kBlend) {
            saw_blend = true;
            ASSERT_TRUE(output<bool>("rl/recovery/support_confirmed"));
            if (previous_phase != RecoveryPhase::kBlend) {
                blend_entry_tick = tick_;
                EXPECT_EQ(previous_phase, RecoveryPhase::kPrepare);
                if (!due) {
                    EXPECT_EQ(history, PolicyAction{});
                }
                EXPECT_DOUBLE_EQ(output<double>("rl/recovery/blend"), 0.0);
                saw_off_clock_boundary |= !due;
            }
            const double alpha = output<double>("rl/recovery/blend");
            for (std::size_t side = 0; side < 2; ++side) {
                // With zero pitch and wheel velocity the scripted wheel effort
                // is zero; the measured blend exposes the cached actor target.
                const double expected = std::clamp(
                    alpha * DeployedPolicyContract::kWheelKp
                        * DeployedPolicyContract::kWheelActionScale
                        * cached_policy_action[4 + side],
                    -max_torque_[4 + side], max_torque_[4 + side]);
                EXPECT_NEAR(torque(4 + side), expected, 1e-7);
                // Even at alpha=0 the production blend requires a valid cached
                // target. A non-scheduled entry must keep that shadow cache and
                // its observation; the assertions above reject an extra run or
                // fault. Positive-alpha frames additionally check its value.
                checked_shadow_target_hold |= blend_evaluations == 0 && !due;
            }
        }
        if (state() == State::kRl) {
            EXPECT_EQ(phase(), RecoveryPhase::kComplete);
            EXPECT_DOUBLE_EQ(output<double>("rl/recovery/blend"), 1.0);
            for (std::size_t side = 0; side < 2; ++side) {
                const double expected = std::clamp(
                    DeployedPolicyContract::kWheelKp * DeployedPolicyContract::kWheelActionScale
                        * cached_policy_action[4 + side],
                    -max_torque_[4 + side], max_torque_[4 + side]);
                EXPECT_NEAR(torque(4 + side), expected, 1e-7);
            }
            if (previous_state != State::kRl) {
                rl_entry_tick = tick_;
                EXPECT_EQ(previous_phase, RecoveryPhase::kBlend);
                saw_off_clock_boundary |= !due;
            }
            ++rl_steps;
        }
        submit_wheel_outputs();
    }

    RecordProperty("blend_entry_tick", std::to_string(blend_entry_tick));
    RecordProperty("first_blend_policy_tick", std::to_string(first_blend_policy_tick));
    RecordProperty("rl_entry_tick", std::to_string(rl_entry_tick));
    EXPECT_TRUE(saw_prepare);
    EXPECT_TRUE(saw_blend);
    EXPECT_TRUE(saw_off_clock_boundary);
    EXPECT_TRUE(checked_shadow_target_hold)
        << "BLEND entry=" << blend_entry_tick << " first live policy=" << first_blend_policy_tick
        << " RL entry=" << rl_entry_tick;
    EXPECT_GE(shadow_evaluations, 5u);
    EXPECT_GE(blend_evaluations, 5u);
    EXPECT_GE(rl_evaluations, 2u);
    EXPECT_EQ(state(), State::kRl);
    EXPECT_EQ(rl_steps, 9u);
    EXPECT_LT(Clock::now() - start, std::chrono::seconds{1});
    // A new fall while RL owns the outputs must disable and latch. It must
    // not turn a continuously held double-MIDDLE request into a second script.
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{1.0, Eigen::Vector3d::UnitY()}};
    step();
    EXPECT_EQ(state(), State::kIdle);
    EXPECT_FALSE(output<bool>("enable_request"));
    EXPECT_EQ(
        output<int>("rl/recovery/failure"), std::to_underlying(RecoveryFailure::kLostUpright));
    for (std::size_t i = 0; i < kMotorNames.size(); ++i)
        EXPECT_DOUBLE_EQ(torque(i), 0.0);
    orientation_ = Eigen::Quaterniond::Identity();
    step();
    EXPECT_EQ(state(), State::kIdle);
    EXPECT_FALSE(output<bool>("enable_request"));
}

} // namespace
} // namespace rmcs::rl
