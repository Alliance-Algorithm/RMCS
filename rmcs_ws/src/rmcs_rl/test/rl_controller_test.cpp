#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
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

// Exercise the production Component registrations without constructing an
// Executor, CAN device or ROS publisher. This uses the existing Executor
// friendship only for port binding, not access to controller private state.
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

class RlControllerTest : public ::testing::Test {
protected:
    using Clock = std::chrono::steady_clock;
    using Ports = rmcs_executor::Executor;

    void SetUp() override {
        // Keep the packaged V6 model, nominal pose, 50 Hz policy and 200 Hz PD.
        // Identity calibration and wide synthetic hinge bounds are exclusively
        // test inputs; they do not describe or update the real robot calibration.
        std::vector<std::string> args{"rl_controller_test",       "--ros-args",  "--params-file",
                                      RL_CONTROLLER_TEST_PROFILE, "--log-level", "error"};
        for (const auto& parameter : std::vector<std::string>{
                 std::string{"rl_model_path:="} + RL_CONTROLLER_TEST_MODEL,
                 "calibration_ready:=true", "soft_limits_ready:=true", "imu_alignment_ready:=true",
                 "leg_motor_to_model:=[1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,"
                 "0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]",
                 "leg_model_offsets:=[0.0,0.0,0.0,0.0]", "hinge_coefficients:=[-1.0,1.0,-1.0,1.0]",
                 "hinge_min:=[-2.0,-2.0]", "hinge_max:=[2.0,2.0]"}) {
            args.push_back("-p");
            args.push_back(parameter);
        }
        for (const auto& parameter : additional_parameters()) {
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
        bind("/wheel_leg/feedback_fresh", feedback_fresh_);
        bind("/wheel_leg/imu/quaternion", orientation_);
        bind("/wheel_leg/imu/angular_velocity", gyro_);
        bind("/wheel_leg/imu/acceleration", acceleration_);
        bind("/wheel_leg/imu/acceleration_steady_ns", acceleration_ns_);
        bind("/wheel_leg/imu/last_steady_ns", imu_ns_);
        bind("/wheel_leg/imu/sequence", imu_sequence_);
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
        expect_neutral();
    }

    void TearDown() override {
        controller_.reset();
        rclcpp::shutdown();
    }

    virtual std::vector<std::string> additional_parameters() const { return {}; }
    virtual void refresh_feedback() {
        const auto now_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count());
        for (std::size_t axis = 0; axis < sequence_.size(); ++axis) {
            ++sequence_[axis];
            feedback_ns_[axis] = now_ns;
        }
        acceleration_ns_ = imu_ns_ = now_ns;
        ++imu_sequence_;
        ++acceleration_sequence_;
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
    bool enabled() { return output<bool>("enable_request"); }
    double torque(std::size_t axis) {
        return output<double>(std::string{kMotorNames[axis]} + "/control_torque");
    }

    PolicyAction actions() {
        PolicyAction result;
        for (std::size_t i = 0; i < result.size(); ++i)
            result[i] = output<double>(std::string{"rl/action/"} + kMotorNames[i]);
        return result;
    }

    double observation(std::string_view name) {
        return output<double>(std::string{"rl/observation/"} + std::string{name});
    }

    void step() {
        ++tick_;
        time_ += std::chrono::milliseconds{1};
        update_current_tick();
    }

    void update_current_tick() {
        refresh_feedback();
        controller_->update();
    }

    void start_rl() {
        requested_state_ = 3;
        ready_ = true;
        for (int i = 0; i < 400 && state() != State::kRl; ++i)
            step();
        ASSERT_EQ(state(), State::kRl);
        ASSERT_TRUE(enabled());
        ASSERT_GT(output<double>("rl/performance/inference_us"), 0.0);
    }

    void expect_neutral() {
        EXPECT_FALSE(enabled());
        for (std::size_t i = 0; i < kMotorNames.size(); ++i)
            EXPECT_DOUBLE_EQ(torque(i), 0.0) << kMotorNames[i];
    }

    void expect_latched_idle() {
        EXPECT_EQ(state(), State::kIdle);
        expect_neutral();
        EXPECT_EQ(actions(), PolicyAction{});
        EXPECT_DOUBLE_EQ(output<double>("rl/performance/inference_us"), 0.0);
    }

    void down_reset() {
        // WheelLegChassisController translates fresh double-DOWN into this
        // control-state/reset-count pair. Its own switch decoding is separate.
        requested_state_ = 1;
        ++reset_count_;
        step();
        expect_latched_idle();
    }

    std::unique_ptr<RlController> controller_;
    std::size_t tick_ = 0, reset_count_ = 0;
    double rate_ = 1000.0;
    Clock::time_point time_ = Clock::now();
    bool feedback_fresh_ = true, ready_ = false, jump_ = false;
    int requested_state_ = 1;
    double height_ = 0.305, jump_apex_ = 0.06;
    rmcs_msgs::ChassisMode mode_ = rmcs_msgs::ChassisMode::AUTO;
    rmcs_description::BaseLink::DirectionVector velocity_command_{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_{0.0, 0.0, 9.81};
    std::uint64_t acceleration_ns_ = 0, imu_ns_ = 0, imu_sequence_ = 0, acceleration_sequence_ = 0;
    std::array<double, 2> wheel_submitted_torque_{};
    std::array<std::uint8_t, 2> wheel_submitted_kind_{};
    std::array<std::uint64_t, 2> wheel_submitted_ns_{};
    std::array<double, 6> angle_ = DeployedPolicyContract::kNominalPosition;
    std::array<double, 6> velocity_{}, feedback_torque_{};
    std::array<double, 6> max_torque_{40.0, 40.0, 40.0, 40.0, 4.5, 4.5};
    std::array<int, 4> fault_{};
    std::array<std::uint64_t, 6> sequence_{}, feedback_ns_{};
};

class RlReferenceCalibrationTest : public RlControllerTest {
protected:
    std::vector<std::string> additional_parameters() const override {
        return {
            "leg_motor_to_model:=[-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,-"
            "1.0]",
            "leg_model_offsets:=[1.6,2.93,-1.6,-2.93]"};
    }

    void set_reference_nominal() {
        constexpr std::array offsets{1.6, 2.93, -1.6, -2.93};
        for (std::size_t i = 0; i < offsets.size(); ++i) {
            // API=-raw; V6 model=-API+offset. Apply each sign and offset once.
            const double raw = DeployedPolicyContract::kNominalPosition[i] - offsets[i];
            angle_[i] = -raw;
        }
    }
};

TEST_F(RlReferenceCalibrationTest, ReferenceMotorOffsetsAreAppliedOnceInPolicyCoordinates) {
    set_reference_nominal();
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(
            observation(kObservationNames[ObservationLayout::kJointPosition + i]), 0.0, 1e-7);
    constexpr std::array raw_displacement{0.03, -0.04, 0.05, -0.02};
    constexpr std::array raw_speed{0.2, -0.3, 0.4, -0.5};
    for (std::size_t i = 0; i < 4; ++i) {
        angle_[i] -= raw_displacement[i];
        velocity_[i] = -raw_speed[i];
    }
    for (int tick = 0; tick < 20; ++tick)
        step();
    ASSERT_EQ(state(), State::kRl);
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(
            observation(kObservationNames[ObservationLayout::kJointPosition + i]),
            raw_displacement[i], 1e-7);
        EXPECT_NEAR(
            observation(kObservationNames[ObservationLayout::kJointVelocity + i]),
            0.1 * raw_speed[i], 1e-7);
    }
}

TEST_F(RlReferenceCalibrationTest, XForwardImuPreservesPitchRollAndGyroAxes) {
    set_reference_nominal();
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    constexpr double angle = 0.2;
    gyro_ = Eigen::Vector3d{0.2, -0.4, 0.6};
    for (double direction : {-1.0, 1.0}) {
        orientation_ =
            Eigen::Quaterniond{Eigen::AngleAxisd{direction * angle, Eigen::Vector3d::UnitY()}};
        for (int tick = 0; tick < 20; ++tick)
            step();
        EXPECT_NEAR(observation("projected_gravity/x"), direction * std::sin(angle), 1e-7);
        EXPECT_NEAR(observation("projected_gravity/y"), 0.0, 1e-7);
        EXPECT_NEAR(observation("projected_gravity/z"), -std::cos(angle), 1e-7);
    }
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()}};
    for (int tick = 0; tick < 20; ++tick)
        step();
    ASSERT_EQ(state(), State::kRl);
    EXPECT_NEAR(observation("projected_gravity/x"), 0.0, 1e-7);
    EXPECT_NEAR(observation("projected_gravity/y"), -std::sin(angle), 1e-7);
    EXPECT_NEAR(observation("angular_velocity/x"), 0.1, 1e-7);
    EXPECT_NEAR(observation("angular_velocity/y"), -0.2, 1e-7);
    EXPECT_NEAR(observation("angular_velocity/z"), 0.3, 1e-7);
}

class RlRotatedImuMountTest : public RlControllerTest {
protected:
    std::vector<std::string> additional_parameters() const override {
        return {"imu_to_base:=[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]"};
    }
};

TEST_F(RlRotatedImuMountTest, RowMajorMountRotationKeepsQuaternionAndGyroInTheSameBaseFrame) {
    const Eigen::Quaterniond imu_to_base{
        Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()}};
    orientation_ = imu_to_base;
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    EXPECT_NEAR(observation("projected_gravity/x"), 0.0, 1e-7);
    EXPECT_NEAR(observation("projected_gravity/y"), 0.0, 1e-7);
    EXPECT_NEAR(observation("projected_gravity/z"), -1.0, 1e-7);
    orientation_ =
        Eigen::Quaterniond{Eigen::AngleAxisd{-0.2, Eigen::Vector3d::UnitY()}} * imu_to_base;
    gyro_ = Eigen::Vector3d{0.4, -0.2, 0.6};
    for (int tick = 0; tick < 20; ++tick)
        step();
    ASSERT_EQ(state(), State::kRl);
    EXPECT_NEAR(observation("projected_gravity/x"), -std::sin(0.2), 1e-7);
    EXPECT_NEAR(observation("projected_gravity/y"), 0.0, 1e-7);
    EXPECT_NEAR(observation("projected_gravity/z"), -std::cos(0.2), 1e-7);
    EXPECT_NEAR(observation("angular_velocity/x"), 0.1, 1e-7);
    EXPECT_NEAR(observation("angular_velocity/y"), 0.2, 1e-7);
    EXPECT_NEAR(observation("angular_velocity/z"), 0.3, 1e-7);
}

class RlPendingMechanismLimitsTest : public RlReferenceCalibrationTest {
protected:
    std::vector<std::string> additional_parameters() const override {
        auto parameters = RlReferenceCalibrationTest::additional_parameters();
        parameters.emplace_back("soft_limits_ready:=false");
        return parameters;
    }
};

class RlV6CaptureTest : public RlReferenceCalibrationTest {
protected:
    void SetUp() override {
        RlReferenceCalibrationTest::SetUp();
        set_reference_nominal();
    }

    std::vector<std::string> additional_parameters() const override {
        auto parameters = RlReferenceCalibrationTest::additional_parameters();
        // Use the actual V6 relative-hinge domain, not the generic fixture's
        // wide test limits. The calibration remains synthetic J=-I.
        parameters.emplace_back("hinge_coefficients:=[-1.0,1.0,1.0,-1.0]");
        parameters.emplace_back("hinge_min:=[-0.140638855,-0.140638855]");
        parameters.emplace_back("hinge_max:=[1.329574849,1.329574849]");
        return parameters;
    }

    void request_capture() {
        requested_state_ = 3;
        ready_ = true;
    }

    void expect_waiting_capture() {
        request_capture();
        for (int i = 0; i < 300; ++i) {
            step();
            ASSERT_EQ(state(), State::kPrepare);
            ASSERT_TRUE(enabled());
            ASSERT_DOUBLE_EQ(output<double>("rl/performance/inference_us"), 0.0);
        }
    }
};

TEST_F(RlV6CaptureTest, LoadedNominalAndSmallFreeBasePitchEnterBeforeStaticBalanceDwell) {
    // Frozen evaluation resets include pitch +/-0.02 rad. A passive spring
    // loads the legs away from a static PD's 0.02 rad reach threshold. None of
    // these samples meet the old quiet-velocity/dwell requirement.
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{0.02, Eigen::Vector3d::UnitY()}};
    constexpr std::array model_error{0.04, -0.06, -0.04, 0.06};
    for (std::size_t i = 0; i < model_error.size(); ++i) {
        angle_[i] -= model_error[i];
        velocity_[i] = -0.8; // API=-model for all four leg output axes.
    }
    gyro_.y() = 0.4;
    velocity_[4] = 2.0;
    velocity_[5] = -2.0;
    request_capture();
    step();
    ASSERT_EQ(state(), State::kRl);
    ASSERT_TRUE(enabled());
    ASSERT_TRUE(output<bool>("rl/recovery/sensors_valid"));
    ASSERT_GT(output<double>("rl/performance/inference_us"), 0.0);
    for (std::size_t i = 0; i < model_error.size(); ++i)
        EXPECT_NEAR(
            observation(kObservationNames[ObservationLayout::kJointPosition + i]), model_error[i],
            1e-7);
    EXPECT_NEAR(observation("projected_gravity/x"), std::sin(0.02), 1e-7);
}

TEST_F(RlV6CaptureTest, DriveReadinessStillPrecedesImmediateCapture) {
    requested_state_ = 3;
    for (int i = 0; i < 5; ++i) {
        step();
        ASSERT_EQ(state(), State::kPrepare);
        for (std::size_t axis = 0; axis < kMotorNames.size(); ++axis)
            EXPECT_DOUBLE_EQ(torque(axis), 0.0);
    }
    ready_ = true;
    step();
    EXPECT_EQ(state(), State::kRl);
    EXPECT_GT(output<double>("rl/performance/inference_us"), 0.0);
}

TEST_F(RlV6CaptureTest, TiltOutsideUprightCaptureCannotEnter) {
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{0.21, Eigen::Vector3d::UnitY()}};
    expect_waiting_capture();
}

TEST_F(RlV6CaptureTest, AngularMotionOutsideCaptureCannotEnter) {
    gyro_.x() = 1.01;
    expect_waiting_capture();
}

TEST_F(RlV6CaptureTest, FastLegOutputCannotEnter) {
    velocity_[0] = -2.01;
    expect_waiting_capture();
}

TEST_F(RlV6CaptureTest, FastWheelOutputCannotEnter) {
    velocity_[4] = 5.01;
    expect_waiting_capture();
}

TEST_F(RlV6CaptureTest, LegPoseOutsideBoundedNominalCaptureCannotEnter) {
    angle_[0] -= 0.151;
    expect_waiting_capture();
}

TEST_F(RlV6CaptureTest, LostUprightDisarmsWhileStillPreparingAndRequiresReset) {
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{0.21, Eigen::Vector3d::UnitY()}};
    request_capture();
    step();
    ASSERT_EQ(state(), State::kPrepare);
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{0.80, Eigen::Vector3d::UnitY()}};
    step();
    expect_latched_idle();
    EXPECT_EQ(
        output<int>("rl/recovery/failure"), std::to_underlying(RecoveryFailure::kLostUpright));
    orientation_ = Eigen::Quaterniond::Identity();
    step();
    expect_latched_idle();
}

TEST_F(RlV6CaptureTest, InvalidFeedbackCannotUseTheImmediateCapturePath) {
    feedback_fresh_ = false;
    request_capture();
    step();
    expect_latched_idle();
}

class RlV6NativeRecoveryTest : public RlV6CaptureTest {
protected:
    std::vector<std::string> additional_parameters() const override {
        auto parameters = RlV6CaptureTest::additional_parameters();
        parameters.emplace_back("recovery_enabled:=true");
        parameters.emplace_back("recovery_profile_ready:=true");
        return parameters;
    }
    void refresh_feedback() override {
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
        RlControllerTest::refresh_feedback();
    }
    void set_prepare_reference() {
        constexpr std::array prepare{
            -0.3207126102216724, 0.09491855918261657, 0.32070064924300457, -0.09490058680048478};
        constexpr std::array offsets{1.6, 2.93, -1.6, -2.93};
        for (std::size_t axis = 0; axis < 4; ++axis)
            angle_[axis] = offsets[axis] - prepare[axis];
        requested_state_ = 3;
        ready_ = true;
    }
};

TEST_F(RlV6NativeRecoveryTest, ShadowActorUsesPreviousEffectiveReferenceAtFiftyHertz) {
    set_prepare_reference();
    // Above the 8-degree handover gate, inside native 15-degree PREPARE entry.
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{0.15, Eigen::Vector3d::UnitY()}};
    step();
    ASSERT_EQ(state(), State::kPrepare);
    ASSERT_EQ(output<int>("rl/recovery/native_phase"), 7);
    const auto first_action = actions();
    ASSERT_NE(first_action, PolicyAction{});
    for (int axis = 0; axis < 6; ++axis)
        EXPECT_DOUBLE_EQ(
            observation(kObservationNames[ObservationLayout::kPreviousAction + axis]), 0.0);
    for (int tick = 1; tick < 20; ++tick) {
        step();
        ASSERT_EQ(actions(), first_action);
    }
    step();
    // Four 5 ms native references move each coupled pair by 4 rad/s toward
    // the pitch-compensated goal: 0.08 rad / actor scale 0.25 = 0.32.
    constexpr std::array expected{
        0.3971495591133103 - 0.32,
        -0.17001706708534804 - 0.32,
        -0.3971974030279817 + 0.32,
        0.1700599578304013 + 0.32,
        0.2,
        -0.2};
    for (int axis = 0; axis < 6; ++axis)
        EXPECT_NEAR(
            observation(kObservationNames[ObservationLayout::kPreviousAction + axis]),
            expected[axis], 1e-6);
    EXPECT_TRUE(enabled());
}

TEST_F(RlV6NativeRecoveryTest, NativeProbeBlendAndStableMotionReleaseRetainPolicyHistory) {
    set_prepare_reference();
    bool blended = false, saw_blend_history = false, confirmed_at_blend = false;
    for (int tick = 0; tick < 1500; ++tick) {
        step();
        ASSERT_TRUE(enabled());
        const int phase = output<int>("rl/recovery/native_phase");
        if (phase == 8) {
            blended = true;
            confirmed_at_blend |= output<bool>("rl/recovery/support_confirmed");
            EXPECT_EQ(state(), State::kPrepare);
            EXPECT_TRUE(output<bool>("rl/recovery/motion_hold"));
            if (output<double>("rl/recovery/blend") > 0.0) {
                double history_norm = 0.0;
                for (int axis = 0; axis < 6; ++axis)
                    history_norm += std::abs(
                        observation(kObservationNames[ObservationLayout::kPreviousAction + axis]));
                saw_blend_history |= history_norm > 0.1;
            }
        }
        if (output<bool>("rl/recovery/native_motion_released"))
            break;
    }
    EXPECT_TRUE(blended);
    EXPECT_TRUE(saw_blend_history);
    EXPECT_EQ(state(), State::kRl);
    EXPECT_TRUE(confirmed_at_blend);
    // Native probing ends at BLEND; its 0.6 s evidence may expire during the
    // separate 1 s RL stability interval. It does not become a contact sensor.
    EXPECT_TRUE(output<bool>("rl/recovery/native_motion_released"));
    EXPECT_FALSE(output<bool>("rl/recovery/motion_hold"));
}

TEST_F(
    RlV6NativeRecoveryTest,
    SlightStopPenetrationInvalidatesSupportWithoutInventingAnActuatorFault) {
    set_prepare_reference();
    angle_[1] = 2.93 - (-0.3207126102216724 - 0.141);
    orientation_ =
        Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitY()}};
    step();
    EXPECT_EQ(state(), State::kPrepare);
    EXPECT_TRUE(enabled());
    EXPECT_FALSE(output<bool>("rl/recovery/geometry_valid"));
    EXPECT_FALSE(output<bool>("rl/recovery/support_confirmed"));
    EXPECT_EQ(output<int>("rl/recovery/native_failure"), 0);
}

TEST_F(RlV6NativeRecoveryTest, EveryNewMiddleSessionSelectsCurrentPostureAndSensorLossDisarms) {
    set_prepare_reference();
    step();
    ASSERT_EQ(output<int>("rl/recovery/native_phase"), 7);
    feedback_fresh_ = false;
    step();
    expect_latched_idle();
    feedback_fresh_ = true;
    down_reset();
    orientation_ =
        Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitY()}};
    requested_state_ = 3;
    step();
    EXPECT_EQ(state(), State::kPrepare);
    EXPECT_EQ(output<int>("rl/recovery/native_phase"), 1);
    EXPECT_TRUE(enabled());
}

class RlV6TakeoverTest : public RlV6CaptureTest {
protected:
    virtual double blend_seconds() const { return 0.0; }

    std::vector<std::string> additional_parameters() const override {
        auto parameters = RlV6CaptureTest::additional_parameters();
        parameters.push_back("v6_takeover_blend_seconds:=" + std::to_string(blend_seconds()));
        return parameters;
    }

    double fraction() { return output<double>("rl/v6_takeover/blend_fraction"); }
    double model_q(std::size_t axis) {
        constexpr std::array offsets{1.6, 2.93, -1.6, -2.93};
        return axis < 4 ? offsets[axis] - angle_[axis] : angle_[axis];
    }
    double model_dq(std::size_t axis) { return axis < 4 ? -velocity_[axis] : velocity_[axis]; }

    void capture_loaded_pose() {
        constexpr std::array error{0.04, -0.06, -0.04, 0.06};
        constexpr std::array prepare_nominal{
            -0.3207126102216724, 0.09491855918261657, 0.32070064924300457, -0.09490058680048478};
        for (std::size_t axis = 0; axis < error.size(); ++axis) {
            angle_[axis] -= error[axis];
            velocity_[axis] = -0.3;
            // PREPARE advances the pose by at most 1 rad/s for this first 1 ms.
            prepare_targets_[axis] =
                model_q(axis) + std::clamp(prepare_nominal[axis] - model_q(axis), -0.001, 0.001);
        }
        velocity_[4] = 1.0;
        velocity_[5] = -1.0;
        orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{0.02, Eigen::Vector3d::UnitY()}};
        request_capture();
        step();
        ASSERT_EQ(state(), State::kRl);
        ASSERT_GT(output<double>("rl/performance/inference_us"), 0.0);
        sample_actor_targets();
    }

    void sample_actor_targets() {
        const auto action = actions();           // Published actions from the real C++ ONNX actor.
        for (std::size_t axis = 0; axis < 4; ++axis) {
            const double goal =
                DeployedPolicyContract::kNominalPosition[axis] + 0.25 * action[axis];
            actor_targets_[axis] =
                model_q(axis) + std::remainder(goal - model_q(axis), 2 * std::numbers::pi);
        }
        actor_targets_[4] = 10.0 * action[4];
        actor_targets_[5] = 10.0 * action[5];
    }

    void expect_live_mix(double alpha) {
        ASSERT_EQ(state(), State::kRl);
        EXPECT_NEAR(fraction(), alpha, 1e-12);
        for (std::size_t axis = 0; axis < 6; ++axis) {
            const double prepare =
                axis < 4 ? 160.0 * (prepare_targets_[axis] - model_q(axis)) - 2.5 * model_dq(axis)
                         : -0.6 * model_dq(axis);
            const double actor =
                axis < 4 ? 160.0 * (actor_targets_[axis] - model_q(axis)) - 2.5 * model_dq(axis)
                         : 0.6 * (actor_targets_[axis] - model_dq(axis));
            const double limit = axis < 4 ? 40.0 : 4.5;
            const double native =
                std::clamp((1.0 - alpha) * prepare + alpha * actor, -limit, limit);
            EXPECT_NEAR(torque(axis), axis < 4 ? -native : native, 2e-6) << kMotorNames[axis];
        }
    }

    void exercise_live_window() {
        capture_loaded_pose();
        ASSERT_FALSE(HasFatalFailure());
        expect_live_mix(0.0);
        const auto first_action = actions();
        ASSERT_NE(first_action, PolicyAction{}); // Inference/history start while alpha is zero.
        std::array<double, 6> first_effort;
        for (std::size_t axis = 0; axis < 6; ++axis)
            first_effort[axis] = torque(axis);
        const int duration_ms = static_cast<int>(std::lround(1000.0 * blend_seconds()));
        for (int elapsed_ms = 1; elapsed_ms <= duration_ms + 5; ++elapsed_ms) {
            // Fresh snapshots change both model coordinates and speeds. This
            // distinguishes live PREPARE PD from holding its capture torque.
            if (elapsed_ms % 5 == 0) {
                for (std::size_t axis = 0; axis < 4; ++axis) {
                    angle_[axis] -= (axis % 2 == 0 ? 0.001 : -0.001);
                    velocity_[axis] = -(elapsed_ms % 10 == 0 ? 0.6 : 0.4);
                }
                velocity_[4] = elapsed_ms % 10 == 0 ? 2.0 : 1.5;
                velocity_[5] = -velocity_[4];
            }
            step();
            ASSERT_EQ(state(), State::kRl);
            if (elapsed_ms % 20 == 0)
                sample_actor_targets();
            if (elapsed_ms == 20) {
                for (std::size_t axis = 0; axis < 6; ++axis)
                    EXPECT_DOUBLE_EQ(
                        observation(kObservationNames[ObservationLayout::kPreviousAction + axis]),
                        first_action[axis]);
            }
            if (elapsed_ms < 5) {
                EXPECT_DOUBLE_EQ(fraction(), 0.0);
                for (std::size_t axis = 0; axis < 6; ++axis)
                    EXPECT_DOUBLE_EQ(torque(axis), first_effort[axis]);
            }
            if (elapsed_ms % 5 == 0) {
                const double t = std::min(1.0, static_cast<double>(elapsed_ms) / duration_ms);
                expect_live_mix(t * t * (3.0 - 2.0 * t));
            }
        }
        EXPECT_DOUBLE_EQ(fraction(), 1.0);
    }

    std::array<double, 6> prepare_targets_{}, actor_targets_{};
};

class RlV6Blend100Test : public RlV6TakeoverTest {
protected:
    double blend_seconds() const override { return 0.1; }
};

class RlV6Blend200Test : public RlV6TakeoverTest {
protected:
    double blend_seconds() const override { return 0.2; }
};

TEST_F(RlV6TakeoverTest, DefaultDirectTakeoverAppliesActorPdAtItsFirstCaptureTick) {
    capture_loaded_pose();
    ASSERT_FALSE(HasFatalFailure());
    expect_live_mix(1.0);
}

TEST_F(RlV6Blend100Test, RealActorRunsImmediatelyAndAllSixLiveTorquesBlendFor100Milliseconds) {
    exercise_live_window();
}

TEST_F(RlV6Blend200Test, RealActorRunsImmediatelyAndAllSixLiveTorquesBlendFor200Milliseconds) {
    exercise_live_window();
}

TEST_F(RlV6Blend100Test, DisableClearsEffortAndANewCaptureRestartsTheBlend) {
    capture_loaded_pose();
    ASSERT_FALSE(HasFatalFailure());
    for (int i = 0; i < 50; ++i)
        step();
    ASSERT_NEAR(fraction(), 0.5, 1e-12);
    requested_state_ = 1;
    step();
    expect_latched_idle();
    EXPECT_DOUBLE_EQ(fraction(), 0.0);
    request_capture();
    step();
    ASSERT_EQ(state(), State::kRl);
    EXPECT_DOUBLE_EQ(fraction(), 0.0);
    EXPECT_GT(output<double>("rl/performance/inference_us"), 0.0);
}

TEST_F(RlV6Blend200Test, SensorFaultDuringTheWindowDisarmsAndClearsTheFraction) {
    capture_loaded_pose();
    ASSERT_FALSE(HasFatalFailure());
    for (int i = 0; i < 100; ++i)
        step();
    ASSERT_NEAR(fraction(), 0.5, 1e-12);
    feedback_fresh_ = false;
    step();
    expect_latched_idle();
    EXPECT_DOUBLE_EQ(fraction(), 0.0);
    EXPECT_FALSE(output<bool>("rl/recovery/sensors_valid"));
}

TEST_F(RlV6Blend100Test, BlendParameterIsStartupOnly) {
    const auto result =
        controller_->set_parameter(rclcpp::Parameter{"v6_takeover_blend_seconds", 0.2});
    EXPECT_FALSE(result.successful);
    EXPECT_DOUBLE_EQ(controller_->get_parameter("v6_takeover_blend_seconds").as_double(), 0.1);
}

TEST(V6TakeoverConfiguration, BlendWindowMustBeFiniteAndBounded) {
    for (const auto* parameter :
         {"v6_takeover_blend_seconds:=-0.001", "v6_takeover_blend_seconds:=0.301",
          "v6_takeover_blend_seconds:=.nan"}) {
        SCOPED_TRACE(parameter);
        const std::vector<const char*> argv{
            "v6_takeover_configuration_test",
            "--ros-args",
            "--params-file",
            RL_CONTROLLER_TEST_PROFILE,
            "-p",
            parameter,
            "--log-level",
            "error"};
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
        rmcs_executor::Component::initializing_component_name = "rl_controller";
        EXPECT_THROW(RlController{}, std::runtime_error);
        rclcpp::shutdown();
    }
}

class RlV6ExpiredCaptureSampleTest : public RlV6CaptureTest {
protected:
    void refresh_feedback() override {
        RlControllerTest::refresh_feedback();
        imu_ns_ -= 21'000'000;
    }
};

TEST_F(RlV6ExpiredCaptureSampleTest, CachedAttitudeCannotEnterEvenAtNominalUpright) {
    request_capture();
    step();
    expect_latched_idle();
    EXPECT_FALSE(output<bool>("rl/recovery/sensors_valid"));
}

TEST(V6CaptureConfiguration, StartupOverridesCannotOpenUnboundedCapture) {
    for (const auto* parameter :
         {"v6_capture_max_leg_error_rad:=0.26", "v6_capture_max_angular_velocity:=2.01",
          "v6_capture_max_leg_velocity:=5.01", "v6_capture_max_wheel_velocity:=10.01",
          "v6_capture_max_leg_velocity:=0.0", "prepare_max_tilt_rad:=0.351"}) {
        SCOPED_TRACE(parameter);
        const std::vector<const char*> argv{
            "v6_capture_configuration_test",
            "--ros-args",
            "--params-file",
            RL_CONTROLLER_TEST_PROFILE,
            "-p",
            parameter,
            "--log-level",
            "error"};
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
        rmcs_executor::Component::initializing_component_name = "rl_controller";
        EXPECT_THROW(RlController{}, std::runtime_error);
        rclcpp::shutdown();
    }
}

class RlV5PrepareDwellTest : public RlControllerTest {
protected:
    void SetUp() override {
        angle_ = LegacyPolicyContract::kNominalPosition;
        RlControllerTest::SetUp();
    }
    std::vector<std::string> additional_parameters() const override {
        return {
            "policy_profile:=v5_flat_12486",
            std::string{"rl_model_path:="} + RL_CONTROLLER_LEGACY_TEST_MODEL,
            "nominal_model_pos:=[0.42,-0.13742282595254576,-0.42,0.13741557625658019,0.0,0.0]",
            "recovery_enabled:=false",
            "prepare_kp:=80.0",
            "prepare_kd:=2.0"};
    }
};

TEST_F(RlV5PrepareDwellTest, LegacyStillRequiresItsOriginalStaticBalanceDwell) {
    requested_state_ = 3;
    ready_ = true;
    for (int i = 0; i < 250; ++i) {
        step();
        ASSERT_EQ(state(), State::kPrepare);
        ASSERT_DOUBLE_EQ(output<double>("rl/performance/inference_us"), 0.0);
    }
    step();
    EXPECT_EQ(state(), State::kRl);
}

TEST_F(RlPendingMechanismLimitsTest, ConfirmedImuAndMotorCalibrationStillRequireMechanismLimits) {
    set_reference_nominal();
    EXPECT_TRUE(controller_->get_parameter("calibration_ready").as_bool());
    EXPECT_TRUE(controller_->get_parameter("imu_alignment_ready").as_bool());
    requested_state_ = 3;
    ready_ = true;
    for (int tick = 0; tick < 20; ++tick) {
        step();
        EXPECT_EQ(state(), State::kIdle);
        expect_neutral();
    }
}

TEST_F(RlControllerTest, RuntimeProfileChangesCannotMisrepresentCachedCalibration) {
    const auto result = controller_->set_parameter(rclcpp::Parameter{"calibration_ready", false});
    EXPECT_FALSE(result.successful);
    EXPECT_TRUE(controller_->get_parameter("calibration_ready").as_bool());
}

TEST_F(RlControllerTest, PrepareWaitsForDrivesWithoutAdvancingReferenceOrSendingTorque) {
    angle_[0] -= 0.1;
    requested_state_ = 2;
    for (int i = 0; i < 100; ++i) {
        step();
        ASSERT_EQ(state(), State::kPrepare);
        ASSERT_TRUE(enabled());
        for (std::size_t axis = 0; axis < kMotorNames.size(); ++axis)
            ASSERT_DOUBLE_EQ(torque(axis), 0.0);
        ASSERT_EQ(actions(), PolicyAction{});
    }
    ready_ = true;
    step();
    ASSERT_EQ(state(), State::kPrepare);
    // First ready tick moves the captured target by 1 rad/s * 1 ms only.
    EXPECT_NEAR(torque(0), 160.0 * 0.001, 1e-12);
    EXPECT_DOUBLE_EQ(output<double>("rl/performance/inference_us"), 0.0);
    velocity_[0] = 0.1;
    for (int i = 0; i < 4; ++i) {
        step();
        EXPECT_NEAR(torque(0), 0.16, 1e-12);
    }
    step();
    EXPECT_NEAR(torque(0), 160.0 * 0.006 - 2.5 * 0.1, 1e-12);
}

TEST_F(RlControllerTest, PolicyUpdatesEveryTwentyTicksAndPublishesClippedActionHistory) {
    start_rl();
    ASSERT_EQ(state(), State::kRl);
    const auto first = actions();
    EXPECT_DOUBLE_EQ(observation("height"), static_cast<float>(0.305 * 5.0));
    EXPECT_DOUBLE_EQ(observation("joint_position/left_wheel"), 0.0);
    EXPECT_DOUBLE_EQ(observation("projected_gravity/z"), -1.0);
    velocity_command_.vector.x() = 0.5;
    for (int i = 1; i < 20; ++i) {
        gyro_.x() = 0.01 * i;
        step();
        EXPECT_EQ(actions(), first);
        EXPECT_DOUBLE_EQ(observation("angular_velocity/x"), 0.0);
        EXPECT_DOUBLE_EQ(observation("command/forward"), 0.0);
    }
    gyro_.x() = 0.2;
    step();
    EXPECT_DOUBLE_EQ(observation("angular_velocity/x"), static_cast<float>(0.1));
    EXPECT_NEAR(observation("command/forward"), 0.03, 1e-8);
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_DOUBLE_EQ(
            observation(kObservationNames[ObservationLayout::kPreviousAction + i]), first[i]);
        EXPECT_LE(std::abs(actions()[i]), i < 4 ? 3.0f : 9.0f);
    }
}

TEST_F(RlControllerTest, PdRefreshesFromFeedbackEveryFiveTicksWhilePolicyTargetIsHeld) {
    start_rl();
    ASSERT_EQ(state(), State::kRl);
    const auto first = actions();
    const double initial = torque(4);
    const double wheel_target = 10.0 * first[4];
    velocity_[4] = wheel_target + 1.0;
    for (int i = 0; i < 4; ++i) {
        step();
        EXPECT_DOUBLE_EQ(torque(4), initial);
        EXPECT_EQ(actions(), first);
    }
    step();
    EXPECT_NEAR(torque(4), -0.6, 1e-12);
    velocity_[4] = wheel_target + 2.0;
    for (int i = 0; i < 4; ++i) {
        step();
        EXPECT_NEAR(torque(4), -0.6, 1e-12);
    }
    step();
    EXPECT_NEAR(torque(4), -1.2, 1e-12);
    EXPECT_EQ(actions(), first);
}

TEST_F(RlControllerTest, NativeWheelTorqueLimitPrecedesTheHardwareCurrentLimit) {
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    max_torque_[4] = max_torque_[5] = 20.0;
    velocity_[4] = 10.0 * actions()[4] + 100.0;
    velocity_[5] = 10.0 * actions()[5] - 100.0;
    for (int tick = 0; tick < 5; ++tick)
        step();
    ASSERT_EQ(state(), State::kRl);
    EXPECT_DOUBLE_EQ(torque(4), -4.5);
    EXPECT_DOUBLE_EQ(torque(5), 4.5);
}

TEST_F(RlControllerTest, SpinEntryClearsTranslationAndSlewsYawWithNormalContext) {
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    velocity_command_.vector.x() = 0.5;
    for (int tick = 0; tick < 40; ++tick)
        step();
    ASSERT_GT(observation("command/forward"), 0.0);
    mode_ = rmcs_msgs::ChassisMode::SPIN_FAST;
    velocity_command_.vector = Eigen::Vector3d{0.0, 0.0, -1.0};
    for (int tick = 0; tick < 20; ++tick)
        step();
    ASSERT_EQ(state(), State::kRl);
    EXPECT_DOUBLE_EQ(observation("command/forward"), 0.0);
    EXPECT_DOUBLE_EQ(observation("command/lateral"), 0.0);
    EXPECT_NEAR(observation("command/yaw"), -0.08, 1e-7);
    EXPECT_DOUBLE_EQ(observation("context/normal"), 1.0);
    for (std::size_t i = 1; i < 7; ++i)
        EXPECT_DOUBLE_EQ(observation(kObservationNames[ObservationLayout::kContext + i]), 0.0);
}

TEST(PolicyCapability, V6HeightAndMotionDomainDoesNotInheritV5HighSpeedOrJump) {
    const rmcs_description::BaseLink::DirectionVector zero{0.0, 0.0, 0.0};
    for (double height : {0.23, 0.305, 0.43})
        EXPECT_TRUE(accepts_motion_command(false, height, rmcs_msgs::ChassisMode::AUTO, zero));
    for (double height : {0.2299, 0.4301})
        EXPECT_FALSE(accepts_motion_command(false, height, rmcs_msgs::ChassisMode::AUTO, zero));
    EXPECT_FALSE(accepts_motion_command(true, 0.305, rmcs_msgs::ChassisMode::AUTO, zero));
    EXPECT_FALSE(
        accepts_motion_command(false, 0.305, rmcs_msgs::ChassisMode::AUTO, {0.5001, 0.0, 0.0}));
    for (double yaw : {-1.0, 1.0})
        EXPECT_TRUE(accepts_motion_command(
            false, 0.305, rmcs_msgs::ChassisMode::SPIN_FAST, {0.0, 0.0, yaw}));
    EXPECT_FALSE(
        accepts_motion_command(false, 0.305, rmcs_msgs::ChassisMode::SPIN_FAST, {0.1, 0.0, 1.0}));
    EXPECT_FALSE(
        accepts_motion_command(false, 0.305, rmcs_msgs::ChassisMode::AUTO, {0.0, 0.01, 0.0}));
    EXPECT_FALSE(
        accepts_motion_command(false, 0.305, rmcs_msgs::ChassisMode::AUTO, {0.0, 0.0, -1.01}));
    EXPECT_FALSE(
        accepts_motion_command(false, 0.305, rmcs_msgs::ChassisMode::AUTO, {0.0, 0.0, 1.01}));
}

TEST(PolicyIdentity, LegacyModelCannotBeLoadedWithV6Semantics) {
    EXPECT_THROW(OnnxPolicy{RL_CONTROLLER_LEGACY_TEST_MODEL}, std::runtime_error);
    EXPECT_THROW(
        OnnxPolicy(RL_CONTROLLER_TEST_MODEL, LegacyPolicyContract::kSha256), std::runtime_error);
    EXPECT_NO_THROW(OnnxPolicy{RL_CONTROLLER_TEST_MODEL});
    EXPECT_NO_THROW(OnnxPolicy(RL_CONTROLLER_LEGACY_TEST_MODEL, LegacyPolicyContract::kSha256));
}

TEST_F(RlControllerTest, DownResetClearsAllOutputsBeforeFeedbackAndCommandValidation) {
    start_rl();
    ASSERT_EQ(state(), State::kRl);
    feedback_fresh_ = false;
    velocity_command_.vector.x() = std::numeric_limits<double>::quiet_NaN();
    down_reset();
    feedback_fresh_ = true;
    velocity_command_.vector.setZero();
    for (int i = 0; i < 25; ++i) {
        step();
        expect_latched_idle();
    }
    start_rl();
    EXPECT_EQ(state(), State::kRl);
}

TEST_F(RlControllerTest, FeedbackLossLatchesImmediatelyAndRequiresExplicitReset) {
    start_rl();
    ASSERT_EQ(state(), State::kRl);
    feedback_fresh_ = false;
    step(); // Disable must not wait for the next 200 Hz PD tick.
    expect_latched_idle();
    feedback_fresh_ = true;
    for (int i = 0; i < 25; ++i) {
        step();
        expect_latched_idle();
    }
    down_reset();
    start_rl();
    EXPECT_EQ(state(), State::kRl);
}

TEST_F(RlControllerTest, DriveReadinessLossDuringRlClearsTorqueAndEnableRequest) {
    start_rl();
    ASSERT_EQ(state(), State::kRl);
    ready_ = false;
    step();
    expect_latched_idle();
    ready_ = true;
    step();
    expect_latched_idle();
}

TEST_F(RlControllerTest, MotorFaultDuringPrepareLatchesWithoutWaitingForDrives) {
    requested_state_ = 3;
    step();
    ASSERT_EQ(state(), State::kPrepare);
    ASSERT_TRUE(enabled());
    fault_[2] = 8;
    step();
    expect_latched_idle();
    fault_[2] = 0;
    ready_ = true;
    step();
    expect_latched_idle();
}

TEST_F(RlControllerTest, UnsupportedJumpRequestFailsClosedBeforeNextPolicyTick) {
    start_rl();
    ASSERT_EQ(state(), State::kRl);
    jump_ = true;
    step();
    expect_latched_idle();
    jump_ = false;
    step();
    expect_latched_idle();
}

TEST_F(RlControllerTest, FlatActorFallDisablesBeforeTheNextPdTick) {
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{0.8, Eigen::Vector3d::UnitY()}};
    step();
    expect_latched_idle();
    EXPECT_EQ(
        output<int>("rl/recovery/failure"), std::to_underlying(RecoveryFailure::kLostUpright));
}

class RlFlatSensorGuardTest : public RlControllerTest {
protected:
    void refresh_feedback() override {
        RlControllerTest::refresh_feedback();
        if (expire_wheel_)
            feedback_ns_[5] -= 21'000'000;
        if (missing_imu_)
            imu_sequence_ = 0;
    }
    bool expire_wheel_ = false, missing_imu_ = false;
};

TEST_F(RlFlatSensorGuardTest, V6RequiresFreshWheelFramesEvenWithRecoveryDisabled) {
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    ASSERT_FALSE(controller_->get_parameter("recovery_enabled").as_bool());
    ASSERT_TRUE(output<bool>("rl/recovery/sensors_valid"));
    expire_wheel_ = true;
    step();
    expect_latched_idle();
    expire_wheel_ = false;
    step();
    expect_latched_idle();
}

TEST_F(RlFlatSensorGuardTest, V6CannotReuseAQuaternionWithoutItsMatchingImuSample) {
    start_rl();
    ASSERT_FALSE(HasFatalFailure());
    missing_imu_ = true;
    step();
    expect_latched_idle();
}

TEST(PolicyCapability, V6RejectsMissingFrozenGeometryEvenWhenRecoveryIsEnabled) {
    const std::vector<const char*> argv{
        "v6_recovery_profile_test",
        "--ros-args",
        "--params-file",
        RL_CONTROLLER_TEST_PROFILE,
        "-p",
        "recovery_enabled:=true",
        "-p",
        "v6_recovery_profile_path:=/missing/v5/geometry.json",
        "--log-level",
        "error"};
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    rmcs_executor::Component::initializing_component_name = "rl_controller";
    EXPECT_THROW(RlController{}, std::runtime_error);
    rclcpp::shutdown();
}

TEST_F(RlControllerTest, NonfiniteObservationAtInferenceClearsPreviouslyHeldTorque) {
    start_rl();
    ASSERT_EQ(state(), State::kRl);
    jump_apex_ = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 19; ++i) {
        step();
        ASSERT_EQ(state(), State::kRl);
    }
    step();
    expect_latched_idle();
    jump_apex_ = 0.06;
    step();
    expect_latched_idle();
}

class RlRecoveryControllerTest : public RlControllerTest {
protected:
    void SetUp() override {
        angle_ = LegacyPolicyContract::kNominalPosition;
        RlControllerTest::SetUp();
    }
    enum class FeedbackIssue {
        kNone,
        kMissingMotor,
        kExpiredMotor,
        kMissingImu,
        kExpiredImu,
        kExpiredWheel,
        kExpiredGyro,
        kMissingGyro,
        kImuSkew,
        kFutureWheel
    };

    std::vector<std::string> additional_parameters() const override {
        // This deliberately small, symmetric mechanism exercises the production
        // calibration reader and recovery observer. It is not robot calibration.
        std::vector<std::string> parameters{
            "policy_profile:=v5_flat_12486",
            std::string{"rl_model_path:="} + RL_CONTROLLER_LEGACY_TEST_MODEL,
            "nominal_model_pos:=[0.42,-0.13742282595254576,-0.42,0.13741557625658019,0.0,0.0]",
            "recovery_enabled:=true",
            "recovery_profile_ready:=true",
            "recovery_above_rated_budget_s:=1.0",
            "recovery_dm_feedback_position_max:=[12.5,12.5,12.5,12.5]",
            "recovery_orbit_speed:=5.0",
            "recovery_side_speed:=5.0",
            "recovery_rollover_speed:=5.25",
            "recovery_capture_speed:=4.0",
            "recovery_spring_stroke_m:=0.1",
            "recovery_spring_force_n:=[280.0,0.0,0.0,0.0]",
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
        return parameters;
    }

    void refresh_feedback() override {
        // Sensor freshness uses steady time, independently of the executor's
        // scheduled timestamp. Refresh real port values without sleeping.
        const auto now_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count());
        for (std::size_t axis = 0; axis < sequence_.size(); ++axis) {
            ++sequence_[axis];
            feedback_ns_[axis] = now_ns;
        }
        acceleration_ns_ = now_ns;
        imu_ns_ = now_ns;
        ++imu_sequence_;
        ++acceleration_sequence_;
        switch (issue_) {
        case FeedbackIssue::kMissingMotor: sequence_[0] = 0; break;
        case FeedbackIssue::kExpiredMotor: feedback_ns_[0] -= 21'000'000; break;
        case FeedbackIssue::kMissingImu: acceleration_ns_ = 0; break;
        case FeedbackIssue::kExpiredImu: acceleration_ns_ -= 31'000'000; break;
        case FeedbackIssue::kExpiredWheel: feedback_ns_[5] -= 21'000'000; break;
        case FeedbackIssue::kExpiredGyro: imu_ns_ -= 21'000'000; break;
        case FeedbackIssue::kMissingGyro: imu_sequence_ = 0; break;
        case FeedbackIssue::kImuSkew: acceleration_ns_ -= 11'000'000; break;
        case FeedbackIssue::kFutureWheel: feedback_ns_[4] += 1'000'000; break;
        case FeedbackIssue::kNone: break;
        }
    }

    void expect_zero_effort() {
        for (std::size_t axis = 0; axis < kMotorNames.size(); ++axis)
            EXPECT_DOUBLE_EQ(torque(axis), 0.0) << kMotorNames[axis];
    }

    void start_recovery_from_tick_zero() {
        ASSERT_EQ(tick_, 0u);
        requested_state_ = 3;
        update_current_tick();
        ASSERT_EQ(state(), State::kPrepare);
        ASSERT_TRUE(enabled());
        expect_zero_effort();

        ready_ = true;
        for (int tick = 1; tick < 5; ++tick) {
            step();
            ASSERT_EQ(state(), State::kPrepare);
            ASSERT_TRUE(enabled());
            EXPECT_FALSE(output<bool>("rl/recovery/motion_hold"));
            EXPECT_EQ(output<int>("rl/recovery/phase"), std::to_underlying(RecoveryPhase::kIdle));
            expect_zero_effort();
        }
        step();
        ASSERT_EQ(tick_, 5u);
        ASSERT_EQ(state(), State::kPrepare);
        ASSERT_TRUE(enabled());
        EXPECT_TRUE(output<bool>("rl/recovery/motion_hold"));
        EXPECT_TRUE(output<bool>("rl/recovery/geometry_valid"));
        EXPECT_EQ(output<int>("rl/recovery/phase"), std::to_underlying(RecoveryPhase::kPrepare));
        EXPECT_GT(torque(0), 0.0);
    }

    void expect_fault_and_recovery_after_reset(FeedbackIssue issue) {
        start_recovery_from_tick_zero();
        ASSERT_FALSE(HasFatalFailure());
        issue_ = issue;
        const int detection_ticks = 1;
        for (int tick = 0; tick < detection_ticks; ++tick)
            step();
        expect_latched_idle();
        EXPECT_FALSE(output<bool>("rl/recovery/motion_hold"));

        issue_ = FeedbackIssue::kNone;
        step();
        expect_latched_idle();
        down_reset();

        requested_state_ = 3;
        ready_ = false;
        step();
        ASSERT_EQ(state(), State::kPrepare);
        ASSERT_TRUE(enabled());
        expect_zero_effort();
        ready_ = true;
        for (int tick = 0; tick < 5 && !output<bool>("rl/recovery/motion_hold"); ++tick)
            step();
        ASSERT_EQ(state(), State::kPrepare);
        EXPECT_TRUE(enabled());
        EXPECT_TRUE(output<bool>("rl/recovery/motion_hold"));
        EXPECT_TRUE(output<bool>("rl/recovery/geometry_valid"));
        EXPECT_GT(torque(0), 0.0);
    }

    FeedbackIssue issue_ = FeedbackIssue::kNone;
};

TEST_F(RlRecoveryControllerTest, ReadyDrivesWaitForSensorBaselineBeforeSelectingUprightRoute) {
    requested_state_ = 3;
    ready_ = true;
    update_current_tick();
    ASSERT_EQ(state(), State::kPrepare);
    ASSERT_TRUE(enabled());
    EXPECT_EQ(output<int>("rl/recovery/phase"), std::to_underlying(RecoveryPhase::kIdle));
    expect_zero_effort();
    for (int i = 0; i < 4; ++i) {
        step();
        expect_zero_effort();
    }
    step();
    ASSERT_EQ(state(), State::kPrepare);
    EXPECT_EQ(output<int>("rl/recovery/phase"), std::to_underlying(RecoveryPhase::kPrepare));
    EXPECT_GT(torque(0), 0.0);
}

TEST_F(RlRecoveryControllerTest, TickZeroWaitsForNextPdTickBeforeStartingRecovery) {
    start_recovery_from_tick_zero();
}

TEST_F(RlRecoveryControllerTest, MissingMotorFeedbackDisablesUntilExplicitReset) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kMissingMotor);
}

TEST_F(RlRecoveryControllerTest, ExpiredMotorFeedbackDisablesUntilExplicitReset) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kExpiredMotor);
}

TEST_F(RlRecoveryControllerTest, MissingAccelerationFeedbackDisablesUntilExplicitReset) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kMissingImu);
}

TEST_F(RlRecoveryControllerTest, ExpiredAccelerationFeedbackDisablesUntilExplicitReset) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kExpiredImu);
}

TEST_F(RlRecoveryControllerTest, ExpiredWheelDisablesAtExecutorCadence) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kExpiredWheel);
}

TEST_F(RlRecoveryControllerTest, ExpiredGyroDisablesAtExecutorCadence) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kExpiredGyro);
}

TEST_F(RlRecoveryControllerTest, MissingGyroSequenceDisablesAtExecutorCadence) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kMissingGyro);
}

TEST_F(RlRecoveryControllerTest, ImuSkewDisablesBeforeComputingTorque) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kImuSkew);
}

TEST_F(RlRecoveryControllerTest, FutureWheelTimestampDisablesBeforeComputingTorque) {
    expect_fault_and_recovery_after_reset(FeedbackIssue::kFutureWheel);
}

TEST_F(RlRecoveryControllerTest, EachDownToMiddleSessionSelectsFromCurrentPosture) {
    start_recovery_from_tick_zero();
    ASSERT_FALSE(HasFatalFailure());
    for (int i = 0; i < 10; ++i) {
        step();
        ASSERT_EQ(output<int>("rl/recovery/phase"), std::to_underlying(RecoveryPhase::kPrepare));
    }
    down_reset();
    EXPECT_FALSE(enabled());
    expect_zero_effort();
    EXPECT_EQ(output<int>("rl/recovery/phase"), std::to_underlying(RecoveryPhase::kIdle));
    // The second session sees a side fall; it must not reuse the first
    // session's upright classification or be limited to the first arming.
    orientation_ =
        Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitX()}};
    requested_state_ = 3;
    ready_ = false;
    step();
    ASSERT_EQ(state(), State::kPrepare);
    expect_zero_effort();
    ready_ = true;
    for (int i = 0; i < 5; ++i)
        step();
    ASSERT_EQ(state(), State::kPrepare);
    EXPECT_EQ(output<int>("rl/recovery/phase"), std::to_underlying(RecoveryPhase::kFold));
    down_reset();
    EXPECT_FALSE(enabled());
    expect_zero_effort();
}

TEST_F(RlRecoveryControllerTest, NoSubmittedWheelFramesCannotConfirmSupport) {
    start_recovery_from_tick_zero();
    ASSERT_FALSE(HasFatalFailure());
    for (int i = 0; i < 250; ++i) {
        step();
        ASSERT_EQ(state(), State::kPrepare);
        ASSERT_FALSE(output<bool>("rl/recovery/support_confirmed"));
        ASSERT_EQ(output<double>("rl/recovery/blend"), 0.0);
    }
}

} // namespace
} // namespace rmcs::rl
