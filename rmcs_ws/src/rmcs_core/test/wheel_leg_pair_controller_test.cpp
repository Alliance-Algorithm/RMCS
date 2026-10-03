#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

// This test executable does not link or start the production Executor. Use its
// existing friendship only to bind the real Component registrations, including
// their type checks and bind callbacks; never expose controller private state.
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

// Compile the actual controller update(), parameter validation, planner and
// PD loop into this offline test. No hardware class is included or constructed.
#include "identification/wheel_leg_pair_identification_controller.cpp"

namespace rmcs_core::controller::identification {
namespace {

class PairControllerTest : public ::testing::Test {
protected:
    using Clock = std::chrono::steady_clock;
    using Switch = rmcs_msgs::Switch;
    using Ports = rmcs_executor::Executor;
    static constexpr std::array kAxes{"left_hip_joint",   "left_knee_joint", "right_hip_joint",
                                      "right_knee_joint", "left_wheel",      "right_wheel"};

    virtual std::string side() const { return "left"; }
    virtual const char* profile_path() const { return PAIR_CONTROLLER_TEST_PROFILE; }
    virtual std::string delta_policy_override() const { return {}; }
    virtual std::vector<std::string> extra_parameters() const { return {}; }

    void SetUp() override {
        // These are synthetic valid bench coordinates, not a real robot LUT.
        // Load the real profile and override only the test setup through ROS's
        // public parameter mechanism, as an operator-supplied YAML would do.
        std::vector<std::string> args{
            "wheel_leg_pair_controller_test",
            "--ros-args",
            "--params-file",
            profile_path(),
            "--log-level",
            "error"};
        for (const auto& parameter : std::vector<std::string>{
                 "side:=" + side(), "probe_pattern:=multiband_chirp", "experiment_stage:=probe",
                 "phase_api_angle:=true", "model_sign:=[1.0,1.0,1.0,1.0]",
                 "model_offset:=[0.0,0.0,0.0,0.0]", "spring_delta_min:=[0.44,0.44]",
                 "spring_delta_max:=[1.32,1.32]", "pd_kp:=[60.0,60.0,60.0,60.0]",
                 "pd_kd:=[2.0,2.0,2.0,2.0]", "ready_timeout_s:=12.0"}) {
            args.push_back("-p");
            args.push_back(parameter);
        }
        if (!delta_policy_override().empty()) {
            args.push_back("-p");
            args.push_back("probe_measured_delta_policy:=" + delta_policy_override());
        }
        for (const auto& parameter : extra_parameters()) {
            args.push_back("-p");
            args.push_back(parameter);
        }
        std::vector<const char*> argv;
        for (const auto& arg : args)
            argv.push_back(arg.c_str());
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
        rmcs_executor::Component::initializing_component_name =
            "wheel_leg_pair_identification_controller";
        controller_ = std::make_unique<WheelLegPairIdentificationController>();
        bind("/predefined/update_count", tick_);
        bind("/predefined/update_rate", rate_);
        bind("/predefined/timestamp", time_);
        bind("/remote/switch/left", left_);
        bind("/remote/switch/right", right_);
        bind("/wheel_leg/feedback_fresh", feedback_fresh_);
        bind("/wheel_leg/imu/last_steady_ns", imu_ns_);
        bind("/wheel_leg/dr16_fresh", remote_fresh_);
        bind("/wheel_leg/dm_control_ready", ready_);
        max_torque_.fill(40.0);
        for (std::size_t i = 0; i < kAxes.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kAxes[i];
            bind(prefix + "/angle", angle_[i]);
            bind(prefix + "/velocity", velocity_[i]);
            bind(prefix + "/max_torque", max_torque_[i]);
            bind(prefix + "/feedback_steady_ns", feedback_ns_[i]);
            if (i < 4) {
                bind(prefix + "/fault_code", fault_[i]);
                bind(prefix + "/status_code", status_[i]);
            }
        }
        Ports::require_bound_inputs(*controller_);
        ASSERT_EQ(phase(), 0);
        EXPECT_FALSE(enabled());
        EXPECT_FALSE(clearing());
        for (const auto* axis : kAxes)
            EXPECT_TRUE(std::isnan(output<double>(std::string{axis} + "/control_torque")));
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

    int phase() { return output<int>("identification/phase"); }
    int failure() { return output<int>("identification/failure_reason"); }
    bool enabled() { return output<bool>("enable_request"); }
    bool clearing() { return output<bool>("clear_error_request"); }

    void step(bool fresh_feedback = true, std::size_t tick_increment = 1, int elapsed_ms = 1) {
        tick_ += tick_increment;
        time_ += std::chrono::milliseconds{elapsed_ms};
        if (fresh_feedback) {
            const auto now = Clock::now().time_since_epoch();
            const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            feedback_ns_.fill(static_cast<std::uint64_t>(ns));
            imu_ns_ = static_cast<std::uint64_t>(ns);
            feedback_fresh_ = true;
        }
        controller_->update();
        for (const auto* axis : kAxes) {
            const auto torque = output<double>(std::string{axis} + "/control_torque");
            if (phase() == 2) {
                EXPECT_TRUE(std::isfinite(torque));
                EXPECT_LE(std::abs(torque), 40.0);
            } else {
                // NaN releases the controller port. The hardware layer owns
                // conversion to neutral motor frames; this test has no device.
                EXPECT_TRUE(std::isnan(torque)) << axis << ": phase=" << phase();
            }
        }
    }

    void start_running() {
        step();
        ASSERT_EQ(phase(), 1) << failure();
        ASSERT_TRUE(enabled());
        const auto first = side() == "left" ? 0 : 2;
        status_[first] = status_[first + 1] = 1;
        ready_ = true;
        for (int i = 0; i < 7000 && phase() == 1; ++i)
            step();
        ASSERT_EQ(phase(), 2) << failure();
    }

    void expect_down_reset() {
        EXPECT_EQ(phase(), 0);
        EXPECT_EQ(failure(), 0);
        EXPECT_FALSE(enabled());
        EXPECT_FALSE(clearing());
        EXPECT_EQ(output<int>("identification/segment_id"), -1);
        for (const auto* axis : kAxes)
            EXPECT_TRUE(std::isnan(output<double>(std::string{axis} + "/control_torque"))) << axis;
        EXPECT_TRUE(output<Eigen::Vector4d>("identification/torque_integral_model").isZero());
        EXPECT_TRUE(output<Eigen::Vector4d>("identification/velocity_target_model").isZero());
    }

    std::unique_ptr<WheelLegPairIdentificationController> controller_;
    std::size_t tick_ = 0;
    double rate_ = 1000.0;
    Clock::time_point time_ = Clock::now();
    Switch left_ = Switch::MIDDLE, right_ = Switch::MIDDLE;
    bool feedback_fresh_ = true, remote_fresh_ = true, ready_ = false;
    std::uint64_t imu_ns_ = 0;
    std::array<double, 6> angle_{0.0, 1.0, 0.0, 1.0, 0.0, 0.0};
    std::array<double, 6> velocity_{}, max_torque_{};
    std::array<std::uint64_t, 6> feedback_ns_{};
    std::array<int, 4> fault_{}, status_{};
};

TEST_F(PairControllerTest, FirstFreshMiddleRequestsEnableWithoutPriorDownDwell) {
    step();
    EXPECT_EQ(phase(), 1) << failure();
    EXPECT_TRUE(enabled());
    EXPECT_TRUE(clearing());
}

TEST_F(PairControllerTest, FirstMiddleWaitsForInitialFeedbackBeforeRequestingEnable) {
    feedback_fresh_ = false;
    angle_[0] = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 2500; ++i) {
        step(false);
        ASSERT_EQ(phase(), 1) << "tick=" << i << " reason=" << failure();
        ASSERT_FALSE(enabled());
        ASSERT_FALSE(clearing());
    }
    angle_[0] = 0.0;
    step();
    EXPECT_EQ(phase(), 1) << failure();
    EXPECT_TRUE(enabled());
    EXPECT_TRUE(clearing());
}

TEST_F(PairControllerTest, MissingInitialFeedbackTimesOutAndMiddleDoesNotRetry) {
    feedback_fresh_ = false;
    for (int i = 0; i < 12010; ++i)
        step(false);
    ASSERT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
    EXPECT_FALSE(clearing());
    const int latched_reason = failure();
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), latched_reason);
    EXPECT_FALSE(enabled());
}

TEST_F(PairControllerTest, FeedbackLossAfterEnableRequestLatchesInsteadOfWaitingAgain) {
    step();
    ASSERT_EQ(phase(), 1);
    ASSERT_TRUE(enabled());
    feedback_fresh_ = false;
    feedback_ns_.fill(0);
    step(false);
    ASSERT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
    EXPECT_FALSE(clearing());
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
    left_ = right_ = Switch::DOWN;
    step(false);
    expect_down_reset();
    left_ = right_ = Switch::MIDDLE;
    feedback_fresh_ = false;
    feedback_ns_.fill(0);
    step(false);
    EXPECT_EQ(phase(), 1);
    EXPECT_FALSE(enabled());
    EXPECT_FALSE(clearing());
}

TEST_F(PairControllerTest, AccelerationSpikeIsDiagnosticAndDownStillReleasesAllAxes) {
    start_running();
    ASSERT_EQ(phase(), 2);
    // A 0.2 rad/s velocity step over the next 1 ms control interval exceeds
    // the old 30 rad/s^2 stop threshold, but remains within tracking limits.
    velocity_[0] = 0.2;
    for (int i = 0; i < 8; ++i)
        step();
    EXPECT_EQ(phase(), 2);
    EXPECT_EQ(failure(), 0);
    EXPECT_TRUE(enabled());
    EXPECT_GT(std::abs(output<double>("left_hip_joint/control_torque")), 0.0);
    EXPECT_LE(std::abs(output<double>("left_hip_joint/control_torque")), 20.0);

    left_ = right_ = Switch::DOWN;
    feedback_fresh_ = false;
    feedback_ns_.fill(0);
    step(false);
    expect_down_reset();
}

TEST_F(PairControllerTest, RunningFeedbackLossReleasesAllAxesAndLatches) {
    start_running();
    ASSERT_EQ(phase(), 2);
    angle_[0] -= 0.01;
    angle_[1] -= 0.01;
    for (int i = 0; i < 8; ++i)
        step();
    ASSERT_EQ(phase(), 2) << failure();
    ASSERT_GT(std::abs(output<double>("left_hip_joint/control_torque")), 0.0);
    feedback_fresh_ = false;
    feedback_ns_[0] = 0;
    step(false);
    ASSERT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 40);
    EXPECT_FALSE(enabled());
    EXPECT_FALSE(clearing());
    EXPECT_TRUE(output<Eigen::Vector4d>("identification/torque_integral_model").isZero());
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
    left_ = right_ = Switch::DOWN;
    feedback_ns_.fill(0);
    feedback_fresh_ = false;
    step(false);
    expect_down_reset();
}

TEST_F(PairControllerTest, ParkedFaultAndEnabledParkedDriveBothBlockArming) {
    status_[2] = fault_[2] = 8;
    step();
    ASSERT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
    EXPECT_FALSE(clearing());
    left_ = right_ = Switch::DOWN;
    step();
    expect_down_reset();
    fault_[2] = 0;
    status_[2] = 1;
    left_ = right_ = Switch::MIDDLE;
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
    EXPECT_FALSE(clearing());
}

TEST_F(PairControllerTest, StaleRemoteDownCannotResetLatchedFault) {
    step(true, 1);
    step(true, 5);
    ASSERT_EQ(phase(), -1);
    left_ = right_ = Switch::DOWN;
    remote_fresh_ = false;
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
}

class PairMultibandControllerTest : public PairControllerTest {
protected:
    void start_multiband() { start_running(); }
};

TEST_F(PairMultibandControllerTest, ActualProfilePreflightsWithoutTimeoutAndDownCancels) {
    step();
    ASSERT_EQ(phase(), 1);
    status_[0] = status_[1] = 1;
    ready_ = true;
    for (int i = 0; i < 7000 && phase() == 1; ++i)
        step();
    ASSERT_EQ(phase(), 2) << failure();
    EXPECT_TRUE(enabled());
    left_ = right_ = Switch::DOWN;
    step();
    expect_down_reset();
}

TEST_F(PairMultibandControllerTest, TimingBudgetAllowsJitterAndLongGapStillDisables) {
    start_multiband();
    step(true, 1, 20);
    ASSERT_EQ(phase(), 2) << failure();
    step(true, 1, 21);
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 2);
    EXPECT_FALSE(enabled());
}

TEST_F(PairMultibandControllerTest, ShortTimingGapDoesNotBypassFeedbackLossOrDownPriority) {
    start_multiband();
    feedback_ns_[0] = 0;
    step(false, 1, 13);
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 40);
    EXPECT_FALSE(enabled());
    left_ = right_ = Switch::DOWN;
    step(false, 8, -100);
    expect_down_reset();
}

TEST_F(PairMultibandControllerTest, ClockRollbackStillDisablesWithinJitterBudget) {
    start_multiband();
    step(true, 1, -1);
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 2);
    EXPECT_FALSE(enabled());
}

TEST_F(PairMultibandControllerTest, DiagnosticDeltaCrossingContinuesUntilOperatorDown) {
    start_multiband();
    // Reproduce a crossing 0.0012 rad below the configured motor-axis proxy.
    // Keep independent root coordinates finite and within their bounds.
    angle_[1] = .44 + .005 - .0012;
    for (int i = 0; i < 30; ++i)
        step();
    EXPECT_EQ(phase(), 2) << failure();
    EXPECT_TRUE(enabled());
    left_ = right_ = Switch::DOWN;
    step();
    expect_down_reset();
}

TEST_F(PairMultibandControllerTest, DiagnosticDeltaDoesNotAdmitOutsideInitialPose) {
    angle_[1] = .44;
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 103);
    EXPECT_FALSE(enabled());
}

TEST_F(PairMultibandControllerTest, DiagnosticDeltaDoesNotBypassFreshFeedback) {
    start_multiband();
    angle_[1] = .44;
    step();
    ASSERT_EQ(phase(), 2);
    feedback_ns_[1] = 0;
    step(false);
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 41);
    EXPECT_FALSE(enabled());
}

class PairMultibandStrictDeltaTest : public PairMultibandControllerTest {
    std::string delta_policy_override() const override { return "stop"; }
};

TEST_F(PairMultibandStrictDeltaTest, StopPolicyStillRejectsMeasuredDeltaCrossing) {
    start_multiband();
    angle_[1] = .44;
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 103);
    EXPECT_FALSE(enabled());
}

TEST(PairObservationBounds, IgnoringDeltaCannotMaskRootOrNonfiniteFault) {
    PairLimits limits;
    limits.root_min.fill(-2.);
    limits.root_max.fill(2.);
    limits.joint_margin = .05;
    limits.spring_min.fill(.44);
    limits.spring_max.fill(1.32);
    limits.spring_margin = .005;
    EXPECT_EQ(
        check_probe_pair(limits, 0, {2.1, 2.2}, {0., 0.}, false, false), PairFault::kRootLimit);
    EXPECT_EQ(
        check_probe_pair(
            limits, 0, {0., std::numeric_limits<double>::quiet_NaN()}, {0., 0.}, false, false),
        PairFault::kNonfinite);
    EXPECT_EQ(check_probe_pair(limits, 0, {0., .44}, {0., 0.}, false), PairFault::kSpringLimit);
}

class PairPdControllerTest : public PairControllerTest {};

TEST_F(PairPdControllerTest, PurePdRecomputesEveryTickWhileThePositionReferenceIsHeld) {
    start_running();
    ASSERT_EQ(phase(), 2);
    // Hold captures q=0. PD must react to each fresh feedback sample, even
    // though the reference is held; 1 kHz transmission alone is insufficient.
    angle_[0] = .01;
    velocity_[0] = .1;
    step();
    EXPECT_NEAR(output<double>("left_hip_joint/control_torque"), -.8, 1e-12);
    EXPECT_TRUE(output<Eigen::Vector4d>("identification/velocity_target_model").isZero());
    EXPECT_TRUE(output<Eigen::Vector4d>("identification/torque_integral_model").isZero());
    EXPECT_TRUE(output<Eigen::Vector4d>("identification/torque_gravity_ff_model").isZero());
    velocity_[0] = -.2;
    step();
    EXPECT_NEAR(output<double>("left_hip_joint/control_torque"), -.2, 1e-12);
    // Operator disable is never decimated with the PD calculation.
    left_ = right_ = Switch::DOWN;
    step(false);
    expect_down_reset();
}

class RightPairPdControllerTest : public PairPdControllerTest {
    std::string side() const override { return "right"; }
};

TEST_F(RightPairPdControllerTest, RightSelectionRoutesPdToOnlyTheRightPair) {
    start_running();
    ASSERT_EQ(phase(), 2);
    angle_[2] = .01;
    velocity_[2] = .1;
    step();
    EXPECT_NEAR(output<double>("right_hip_joint/control_torque"), -.8, 1e-12);
    EXPECT_DOUBLE_EQ(output<double>("left_hip_joint/control_torque"), 0.);
    EXPECT_DOUBLE_EQ(output<double>("left_knee_joint/control_torque"), 0.);
    EXPECT_DOUBLE_EQ(output<double>("left_wheel/control_torque"), 0.);
    EXPECT_DOUBLE_EQ(output<double>("right_wheel/control_torque"), 0.);
    left_ = right_ = Switch::DOWN;
    step();
    expect_down_reset();
}

TEST_F(PairPdControllerTest, CompletedSequenceReleasesAndRequiresDownBeforeRestart) {
    start_running();
    ASSERT_EQ(phase(), 2);
    for (int i = 0; i < 680000 && phase() == 2; ++i)
        step();
    ASSERT_EQ(phase(), 3) << failure();
    EXPECT_EQ(failure(), 0);
    EXPECT_FALSE(enabled());
    step();
    EXPECT_EQ(phase(), 3);
    EXPECT_FALSE(enabled());
    left_ = right_ = Switch::DOWN;
    step();
    expect_down_reset();
    status_.fill(0);
    ready_ = false;
    left_ = right_ = Switch::MIDDLE;
    start_running();
    EXPECT_EQ(phase(), 2);
}

TEST_F(PairPdControllerTest, LiveTorqueLimitBelowConfiguredAuthorityReleasesThePair) {
    start_running();
    max_torque_[0] = .5;
    angle_[0] = .02;
    for (int i = 0; i < 5; ++i)
        step();
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(failure(), 5);
    EXPECT_TRUE(std::isnan(output<double>("left_hip_joint/control_torque")));
}

TEST(PairPdMath, PositionAndDampingAreClippedTogetherWithoutIntegralOrFeedforward) {
    const auto result = pd_step(.1, 0., -20., 60., 2., 40.);
    EXPECT_DOUBLE_EQ(result.preclip, 46.);
    EXPECT_DOUBLE_EQ(result.torque, 40.);
    EXPECT_DOUBLE_EQ(result.integral_torque, 0.);
    EXPECT_DOUBLE_EQ(result.velocity_target, 0.);
    EXPECT_DOUBLE_EQ(pd_step(-.1, 0., 20., 60., 2., 40.).torque, -40.);
}

TEST_F(PairPdControllerTest, MovingReferenceIsHeldTwentyTicksWhilePdUsesFreshFeedback) {
    step();
    status_[0] = status_[1] = 1;
    ready_ = true;
    for (int i = 0; i < 7000 && phase() == 1; ++i)
        step();
    ASSERT_EQ(phase(), 2) << failure();
    const auto start_tick = tick_;
    auto previous = output<Eigen::Vector4d>("identification/reference_model");
    int changed = 0;
    for (int i = 0; i < 4500; ++i) {
        step();
        ASSERT_EQ(phase(), 2) << failure();
        const auto current = output<Eigen::Vector4d>("identification/reference_model");
        if (!current.isApprox(previous, 1e-14)) {
            EXPECT_EQ((tick_ - start_tick) % 20, 0u);
            ++changed;
        }
        EXPECT_TRUE(output<Eigen::Vector4d>("identification/torque_integral_model").isZero());
        previous = current;
    }
    EXPECT_GT(changed, 5);
}

struct PairClockFault {
    std::size_t tick_increment;
    int elapsed_ms;
    const char* name;
};

class PairClockTest
    : public PairControllerTest
    , public ::testing::WithParamInterface<PairClockFault> {};

TEST_P(PairClockTest, FreshDownResetsTimingFaultAndNextMiddleStartsWithoutDwell) {
    start_running();
    const auto fault = GetParam();
    step(true, fault.tick_increment, fault.elapsed_ms);
    ASSERT_EQ(phase(), -1);
    ASSERT_EQ(failure(), 2);
    EXPECT_FALSE(enabled());
    step();
    EXPECT_EQ(phase(), -1);

    left_ = right_ = Switch::DOWN;
    remote_fresh_ = false;
    step();
    EXPECT_EQ(phase(), -1);
    EXPECT_EQ(output<std::uint32_t>("identification/repetition_id"), 0u);

    remote_fresh_ = true;
    feedback_fresh_ = false;
    feedback_ns_.fill(0);
    rate_ = std::numeric_limits<double>::quiet_NaN();
    step(false, 8, -100);
    expect_down_reset();
    EXPECT_EQ(output<std::uint32_t>("identification/repetition_id"), 1u);

    // Pair identification intentionally has no DOWN dwell; its original
    // feedback admission and enabled-drive preflight still run after reset.
    rate_ = 1000.0;
    status_.fill(0);
    ready_ = false;
    left_ = right_ = Switch::MIDDLE;
    start_running();
    EXPECT_EQ(failure(), 0);
    EXPECT_EQ(output<std::uint32_t>("identification/repetition_id"), 1u);
}

INSTANTIATE_TEST_SUITE_P(
    ClockDiscontinuities, PairClockTest,
    ::testing::Values(
        PairClockFault{8, 1, "TickJump"}, PairClockFault{1, -100, "ClockRollback"},
        PairClockFault{1, 21, "LongInterval"}),
    [](const ::testing::TestParamInfo<PairClockFault>& info) { return info.param.name; });

class V6PairControllerTest : public PairControllerTest {
    const char* profile_path() const override { return V6_CONTROLLER_TEST_PROFILE; }
    std::vector<std::string> extra_parameters() const override {
        return {
            "probe_pattern:=calibrated_recording",
            "pd_kp:=[160.0,160.0,160.0,160.0]",
            "pd_kd:=[2.5,2.5,2.5,2.5]",
            "ready_timeout_s:=20.0",
            "recording_run:=L02",
            "recording_hip_sign:=1.0",
            "recording_hip_zero:=0.0"};
    }
};

TEST_F(V6PairControllerTest, RecordsFkAndHoldsTargetsWhilePdUsesFreshFeedback) {
    start_running();
    ASSERT_EQ(phase(), 2);
    EXPECT_EQ(output<std::uint32_t>("identification/recording/protocol_version"), 1u);
    EXPECT_TRUE(std::isfinite(output<double>("identification/selected/inner_knee_measured_deg")));
    const auto q = output<Eigen::Vector4d>("identification/reference_model");
    const auto reference_tick = output<std::uint64_t>("identification/recording/reference_tick");
    angle_[0] += .01;
    step();
    EXPECT_EQ(output<std::uint64_t>("identification/recording/reference_tick"), reference_tick);
    EXPECT_EQ(output<Eigen::Vector4d>("identification/reference_model"), q);
    EXPECT_NEAR(output<double>("left_hip_joint/control_torque"), -1.6, 1e-10);
    EXPECT_TRUE(std::isnan(output<double>("right_hip_joint/control_torque")) == false);
    EXPECT_EQ(output<double>("right_hip_joint/control_torque"), 0);
    EXPECT_EQ(output<double>("left_wheel/control_torque"), 0);
    left_ = right_ = Switch::DOWN;
    step(false);
    expect_down_reset();
}

class V6CalibratedZeroControllerTest : public V6PairControllerTest {
    std::vector<std::string> extra_parameters() const override {
        return {
            "probe_pattern:=calibrated_recording",
            "pd_kp:=[160.0,160.0,160.0,160.0]",
            "pd_kd:=[2.5,2.5,2.5,2.5]",
            "ready_timeout_s:=20.0",
            "model_sign:=[-1.0,-1.0,-1.0,-1.0]",
            "model_offset:=[1.6,2.93,-1.6,-2.93]",
            "spring_delta_min:=[-0.474,-1.625]",
            "spring_delta_max:=[1.625,0.474]"};
    }
};

TEST_F(V6CalibratedZeroControllerTest, ExistingEncoderZeroStartsWithZeroPdErrorInV6) {
    angle_.fill(0.0);
    start_running();
    EXPECT_NEAR(output<double>("identification/selected/inner_knee_measured_deg"), 105.021, .005);
    EXPECT_NEAR(output<Eigen::Vector4d>("identification/reference_model")[0], 1.6, 1e-12);
    EXPECT_NEAR(output<Eigen::Vector4d>("identification/reference_model")[1], 2.93, 1e-12);
    EXPECT_NEAR(output<double>("left_hip_joint/control_torque"), 0, 1e-12);
    EXPECT_NEAR(output<double>("left_knee_joint/control_torque"), 0, 1e-10);
}

TEST_F(V6PairControllerTest, ManifestUsesNewRunIdentityAndFailureLatches) {
    const auto changed =
        controller_->set_parameters({rclcpp::Parameter("pd_kp", std::vector<double>(4, 170.))});
    ASSERT_EQ(changed.size(), 1u);
    EXPECT_FALSE(changed[0].successful);
    const auto manifest = controller_->get_parameter("recording_manifest_json").as_string();
    EXPECT_NE(manifest.find("pair_v6_recording_v1"), std::string::npos);
    EXPECT_NE(manifest.find("L02"), std::string::npos);
    start_running();
    feedback_ns_[0] = 0;
    step(false);
    EXPECT_EQ(phase(), -1);
    EXPECT_FALSE(enabled());
    step();
    EXPECT_EQ(phase(), -1);
}

} // namespace
} // namespace rmcs_core::controller::identification
