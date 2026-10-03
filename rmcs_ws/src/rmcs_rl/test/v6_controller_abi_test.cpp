#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "controller_abi_fixture.hpp"
#include "rl_controller.hpp"

// Use the executor's existing port-binding friendship. Production controller
// state and methods remain private; the test sees its real Component outputs.
namespace rmcs_executor {
class Executor {
public:
    template <typename T>
    static void bind(Component& component, const std::string& name, T& value) {
        for (auto& port : component.input_list_) {
            if (port.name == name) {
                if (port.type != typeid(T) || port.kind != InterfaceKind::Normal)
                    throw std::runtime_error("Wrong input type: " + name);
                port.bind(port.binding, &value);
                return;
            }
        }
        throw std::runtime_error("Input is not registered: " + name);
    }

    template <typename T>
    static const T& output(Component& component, const std::string& name) {
        for (auto& port : component.output_list_) {
            if (port.name == name) {
                if (port.type != typeid(T) || port.kind != InterfaceKind::Normal)
                    throw std::runtime_error("Wrong output type: " + name);
                return *static_cast<const T*>(port.binding);
            }
        }
        throw std::runtime_error("Output is not registered: " + name);
    }
};
} // namespace rmcs_executor

namespace rmcs::rl {
namespace {

class V6ControllerAbiTest : public ::testing::Test {
protected:
    using Clock = std::chrono::steady_clock;
    using Ports = rmcs_executor::Executor;

    void SetUp() override {
        std::vector<std::string> args{"v6_controller_abi_test",   "--ros-args",  "--params-file",
                                      RL_CONTROLLER_TEST_PROFILE, "--log-level", "error"};
        for (const auto& parameter : std::vector<std::string>{
                 std::string{"rl_model_path:="} + RL_CONTROLLER_TEST_MODEL,
                 "policy_profile:=v6_flat_14020", "calibration_ready:=true",
                 "soft_limits_ready:=true", "imu_alignment_ready:=true", "recovery_enabled:=false",
                 "auto_enter_rl:=false",
                 "leg_motor_to_model:=[-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,"
                 "0.0,0.0,-1.0,0.0,0.0,0.0,0.0,-1.0]",
                 "leg_model_offsets:=[1.6,2.93,-1.6,-2.93]", "wheel_model_scale:=[1.0,1.0]",
                 "imu_to_base:=[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]",
                 "hinge_coefficients:=[-1.0,1.0,-1.0,1.0]", "hinge_min:=[-100.0,-100.0]",
                 "hinge_max:=[100.0,100.0]", "prepare_stable_seconds:=0.005"}) {
            args.push_back("-p");
            args.push_back(parameter);
        }
        std::vector<const char*> argv;
        for (const auto& arg : args)
            argv.push_back(arg.c_str());
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
        rmcs_executor::Component::initializing_component_name = "rl_controller";
        controller_ = std::make_unique<RlController>();
        bind("/predefined/update_count", tick_);
        bind("/predefined/update_rate", rate_);
        bind("/predefined/timestamp", time_);
        bind("/wheel_leg/feedback_fresh", fresh_);
        bind("/wheel_leg/dm_control_ready", ready_);
        bind("/wheel_leg/imu/quaternion", orientation_);
        bind("/wheel_leg/imu/angular_velocity", gyro_);
        bind("/wheel_leg/imu/acceleration", acceleration_);
        bind("/wheel_leg/imu/acceleration_steady_ns", acceleration_ns_);
        bind("/wheel_leg/imu/last_steady_ns", imu_ns_);
        bind("/wheel_leg/imu/sequence", imu_sequence_);
        bind("/wheel_leg/imu/acceleration_sequence", acceleration_sequence_);
        bind("/chassis/control_velocity", command_);
        bind("/chassis/control_height", height_);
        bind("/chassis/control_state", requested_state_);
        bind("/chassis/reset_count", reset_count_);
        bind("/chassis/control_mode", mode_);
        bind("/chassis/jump_request", jump_);
        bind("/chassis/jump_apex_delta", jump_apex_);
        for (std::size_t axis = 0; axis < kMotorNames.size(); ++axis) {
            const auto prefix = std::string{"/wheel_leg/"} + kMotorNames[axis];
            bind(prefix + "/angle", q_api_[axis]);
            bind(prefix + "/velocity", dq_api_[axis]);
            bind(prefix + "/torque", feedback_torque_[axis]);
            bind(prefix + "/max_torque", max_torque_[axis]);
            bind(prefix + "/feedback_sequence", sequences_[axis]);
            bind(prefix + "/feedback_steady_ns", feedback_ns_[axis]);
            if (axis < 4)
                bind(prefix + "/fault_code", faults_[axis]);
        }
        set_snapshot(test::kControllerCases.front());
        controller_->before_updating();
    }

    void TearDown() override {
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

    void step(std::size_t milliseconds = 1) {
        // Normal V6 checks real steady-clock sample ages and motor continuity.
        // Advance feedback at the stated interval instead of forging time or
        // bypassing the production freshness guard with cached synthetic data.
        std::this_thread::sleep_for(std::chrono::milliseconds{milliseconds});
        tick_ += milliseconds;
        time_ += std::chrono::milliseconds{milliseconds};
        const auto now = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count());
        for (std::size_t axis = 0; axis < sequences_.size(); ++axis) {
            ++sequences_[axis];
            feedback_ns_[axis] = now;
        }
        imu_ns_ = acceleration_ns_ = now;
        ++imu_sequence_;
        ++acceleration_sequence_;
        controller_->update();
    }

    void start_rl() {
        requested_state_ = 3;
        for (int count = 0; count < 30 && output<int>("rl/state") != 3; ++count)
            step();
        ASSERT_EQ(output<int>("rl/state"), 3);
        ASSERT_TRUE(output<bool>("enable_request"));
        ASSERT_GT(output<double>("rl/performance/inference_us"), 0.0);
    }

    void set_snapshot(const test::ControllerCase& fixture) {
        q_api_ = fixture.q_api;
        dq_api_ = fixture.dq_api;
        const auto& quaternion = fixture.orientation_imu_wxyz;
        orientation_ =
            Eigen::Quaterniond{quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
        gyro_ = Eigen::Vector3d{fixture.gyro_imu[0], fixture.gyro_imu[1], fixture.gyro_imu[2]};
        command_.vector << fixture.command[0], fixture.command[1], fixture.command[2];
    }

    void check_snapshot(const test::ControllerCase& fixture) {
        SCOPED_TRACE(fixture.name);
        ASSERT_EQ(output<int>("rl/state"), 3);
        for (std::size_t slot = 0; slot < fixture.observation.size(); ++slot) {
            const double expected = fixture.observation[slot];
            EXPECT_NEAR(
                output<double>("rl/observation/" + std::string{kObservationNames[slot]}), expected,
                1e-5 + 1e-5 * std::abs(expected))
                << "obs slot " << slot;
        }
        for (std::size_t axis = 0; axis < kMotorNames.size(); ++axis) {
            const double action = fixture.clipped_action[axis];
            EXPECT_NEAR(
                output<double>("rl/action/" + std::string{kMotorNames[axis]}), action,
                1e-5 + 1e-5 * std::abs(action))
                << "action axis " << axis;
            // Native Torch PD uses float32 while RMCS deliberately uses double
            // for encoders/targets. The small absolute allowance covers wrapped
            // multi-turn float32 angles; the gain/sign/units remain observable.
            const double effort = fixture.torque_api[axis];
            EXPECT_NEAR(
                output<double>(std::string{kMotorNames[axis]} + "/control_torque"), effort,
                2e-4 + 1e-5 * std::abs(effort))
                << "API effort axis " << axis;
        }
    }

    std::unique_ptr<RlController> controller_;
    std::size_t tick_ = 0, reset_count_ = 0;
    double rate_ = 1000.0, height_ = .305, jump_apex_ = .06;
    Clock::time_point time_ = Clock::now();
    bool fresh_ = true, ready_ = true, jump_ = false;
    int requested_state_ = 1;
    rmcs_msgs::ChassisMode mode_ = rmcs_msgs::ChassisMode::AUTO;
    rmcs_description::BaseLink::DirectionVector command_{0., 0., 0.};
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_{0., 0., 9.81};
    std::uint64_t imu_ns_ = 0, acceleration_ns_ = 0, imu_sequence_ = 0, acceleration_sequence_ = 0;
    std::array<double, 6> q_api_{}, dq_api_{}, feedback_torque_{};
    std::array<std::uint64_t, 6> sequences_{}, feedback_ns_{};
    std::array<double, 6> max_torque_{40., 40., 40., 40., 4.5, 4.5};
    std::array<int, 4> faults_{};
};

TEST_F(V6ControllerAbiTest, NativeSnapshotsMatchObservationActionsHeldTargetsAndApiPd) {
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    for (const auto& fixture : test::kControllerCases) {
        if (fixture.name == "common_encoder_turns")
            continue;
        set_snapshot(fixture);
        if (fixture.advance_ms)
            step(fixture.advance_ms);
        check_snapshot(fixture);
        ASSERT_FALSE(HasFatalFailure());
    }
}

TEST_F(V6ControllerAbiTest, ContinuousCommonEncoderTurnsMatchNativeBeforeFirstEntry) {
    for (const auto& fixture : test::kControllerCases) {
        if (fixture.name != "common_encoder_turns")
            continue;
        set_snapshot(fixture);
        start_rl();
        ASSERT_FALSE(HasFatalFailure());
        check_snapshot(fixture);
        return;
    }
    FAIL() << "Missing native common-turn fixture";
}

TEST_F(V6ControllerAbiTest, ResetClearsPreviousPolicyActionBeforeNextEntry) {
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    requested_state_ = 0;
    step();
    EXPECT_EQ(output<int>("rl/state"), 0);
    EXPECT_FALSE(output<bool>("enable_request"));
    for (const auto* name : kMotorNames) {
        EXPECT_DOUBLE_EQ(output<double>("rl/action/" + std::string{name}), 0.);
        EXPECT_DOUBLE_EQ(output<double>(std::string{name} + "/control_torque"), 0.);
    }
    set_snapshot(test::kControllerCases.front());
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    // Reset may offset the PD cadence. Advance exactly to the next feedback PD
    // without triggering another 20ms inference, then check the held targets.
    step(5);
    check_snapshot(test::kControllerCases.front());
}

} // namespace
} // namespace rmcs::rl
