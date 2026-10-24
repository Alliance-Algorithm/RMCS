// Uses the production controller, bridge, actor and executor with in-memory
// sensor snapshots only. No hardware component or transport is constructed.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "controller/chassis/wheel_leg/wheel_leg_controller.hpp"
#include "executor.hpp"

namespace {
using namespace std::chrono_literals;
using Controller = rmcs_core::controller::chassis::WheelLegController;
using Component = rmcs_executor::Component;
constexpr std::array kNames{"left_hip_joint",   "left_knee_joint", "right_hip_joint",
                            "right_knee_joint", "left_wheel",      "right_wheel"};

class Snapshots final : public Component {
public:
    explicit Snapshots(std::atomic<int>& phase)
        : phase_(phase) {
        const std::array nominal{-1.2, -1.2 + 1.2945, 1.0, 1.0 - (-4.9422 + 2 * std::numbers::pi)};
        constexpr std::array offsets{1.6, 2.93, -1.6, -2.93};
        for (std::size_t i = 0; i < 6; ++i) {
            const auto path = std::string{"/wheel_leg/"} + kNames[i];
            register_output(path + "/angle", angle_[i], i < 4 ? offsets[i] - nominal[i] : 0.0);
            register_output(path + "/velocity", velocity_[i], 0.0);
            register_output(path + "/torque", torque_[i], 0.0);
            register_output(path + "/max_torque", limit_[i], i < 4 ? 40.0 : 4.5);
            register_output(path + "/feedback_sequence", sequence_[i], std::uint64_t{0});
            register_output(path + "/feedback_steady_ns", stamp_[i], std::uint64_t{0});
            if (i < 4)
                register_output(path + "/fault_code", fault_[i], 0);
        }
        register_output("/wheel_leg/feedback_fresh", fresh_, true);
        register_output("/wheel_leg/dm_control_ready", ready_, true);
        register_output(
            "/wheel_leg/joint_position_pd", motor_pd_,
            std::getenv("RMCS_TEST_MOTOR_POSITION_PD") != nullptr);
        register_output("/wheel_leg/joint_position_kp", motor_kp_, 160.0);
        register_output("/wheel_leg/joint_position_kd", motor_kd_, 2.5);
        register_output("/wheel_leg/imu/quaternion", orientation_, Eigen::Quaterniond::Identity());
        register_output("/wheel_leg/imu/angular_velocity", gyro_, Eigen::Vector3d::Zero().eval());
        register_output("/wheel_leg/imu/acceleration", acceleration_, Eigen::Vector3d{0, 0, 9.81});
        register_output("/wheel_leg/imu/sequence", imu_sequence_, std::uint64_t{0});
        register_output("/wheel_leg/imu/last_steady_ns", imu_stamp_, std::uint64_t{0});
        register_output("/wheel_leg/imu/acceleration_sequence", accel_sequence_, std::uint64_t{0});
        register_output("/wheel_leg/imu/acceleration_steady_ns", accel_stamp_, std::uint64_t{0});
        register_output(
            "/chassis/control_velocity", command_,
            rmcs_description::BaseLink::DirectionVector{0.0, 0.0, 0.0});
        register_output("/chassis/control_height", height_, 0.23);
        register_output("/chassis/control_state", state_, 1);
        register_output("/chassis/reset_count", reset_, std::size_t{1});
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        register_output("/chassis/jump_request", jump_, false);
        register_output("/chassis/jump_apex_delta", apex_, 0.0);
    }
    void update() override {
        const int phase = phase_.load();
        *state_ = phase == 0 || phase == 4 || phase == 9 ? 1 : phase == 5 ? 2 : 3;
        *reset_ = phase < 4 ? 1 : phase < 9 ? 2 : 3;
        *ready_ = phase >= 2 && phase != 14;
        // Deliberately far from nominal, with a whole-turn right encoder offset.
        // This is a fixed sensor fixture, not a recovery trajectory or dynamics model.
        std::array model_q{-1.2, -1.2 + 1.2945, 1.0, 1.0 - (-4.9422 + 2 * std::numbers::pi)};
        if (phase >= 11) {
            hinge_violation_ = std::min(0.25, hinge_violation_ + 0.002);
            model_q[1] += hinge_violation_;
            if (phase >= 12)
                model_q[3] = model_q[2] + .25; // Lower-limit violation on the right.
        }
        constexpr std::array offsets{1.6, 2.93, -1.6, -2.93};
        for (int i = 0; i < 4; ++i) {
            const double turns = phase >= 11 ? 4 * std::numbers::pi : 0.0;
            *angle_[i] = offsets[i] - model_q[i] - turns - (i == 3 ? 2 * std::numbers::pi : 0.0);
            *torque_[i] = phase >= 11 ? 100.0 : 0.0;
            // Large finite velocities must saturate PD, never disarm the policy.
            const double direction = phase == 12 ? -1.0 : 1.0;
            *velocity_[i] = phase >= 11 ? direction * (i % 2 == 0 ? 300.0 : -300.0) : 0.0;
        }
        const double tilt = phase >= 3 ? std::numbers::pi : 0.95;
        *orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{tilt, Eigen::Vector3d::UnitY()}};
        if (phase >= 11)
            orientation_->coeffs() *= 3.0;
        *gyro_ = Eigen::Vector3d{0, phase >= 11 ? 300.0 : 3.0, 0};
        *acceleration_ = Eigen::Vector3d{0, 0, 1};
        command_->vector = phase == 13 ? Eigen::Vector3d{30.0, 5.0, 40.0}
                         : phase >= 11 ? Eigen::Vector3d{3.0, 0.0, 4.0}
                         : phase >= 3  ? Eigen::Vector3d{0.2, 0.0, 0.1}
                                       : Eigen::Vector3d::Zero().eval();
        *height_ = phase == 13 ? 0.8 : 0.23;
        const auto now = std::chrono::steady_clock::now();
        const auto ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        for (std::size_t i = 0; i < 6; ++i) {
            *sequence_[i] += 1;
            *stamp_[i] = ns;
        }
        if (phase != 7) {
            *imu_sequence_ += 1;
            *accel_sequence_ += 1;
            *imu_stamp_ = *accel_stamp_ = ns;
        }
    }

private:
    std::atomic<int>& phase_;
    double hinge_violation_ = 0.0;
    std::array<OutputInterface<double>, 6> angle_, velocity_, torque_, limit_;
    std::array<OutputInterface<std::uint64_t>, 6> sequence_, stamp_;
    std::array<OutputInterface<int>, 4> fault_;
    OutputInterface<bool> fresh_, ready_, motor_pd_, jump_;
    OutputInterface<Eigen::Quaterniond> orientation_;
    OutputInterface<Eigen::Vector3d> gyro_, acceleration_;
    OutputInterface<std::uint64_t> imu_sequence_, imu_stamp_, accel_sequence_, accel_stamp_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    OutputInterface<double> height_, apex_;
    OutputInterface<double> motor_kp_, motor_kd_;
    OutputInterface<int> state_;
    OutputInterface<std::size_t> reset_;
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
};

class Observe final : public Component {
public:
    Observe() {
        for (std::size_t i = 0; i < 6; ++i) {
            register_input(std::string{"/wheel_leg/"} + kNames[i] + "/control_torque", torque_[i]);
            if (i < 4)
                register_input(
                    std::string{"/wheel_leg/rl/model/"} + kNames[i] + "/angle", angle_[i]);
        }
        register_input("/wheel_leg/rl/valid", valid_);
        register_input("/wheel_leg/rl/action_seq", sequence_);
        register_input("/wheel_leg/rl/state", state_);
        register_input("/wheel_leg/enable_request", enable_);
        register_input("/wheel_leg/rl/policy_rate", rate_);
        register_input("/wheel_leg/rl/action_age", age_);
        register_input("/wheel_leg/rl/performance/inference_us", inference_);
        register_input("/wheel_leg/rl/fault_latched", fault_);
        register_input("/wheel_leg/rl/input/command", command_);
        register_input("/wheel_leg/rl/input/projected_gravity", gravity_);
        register_input("/wheel_leg/joint_torque_override", torque_override_);
        for (std::size_t i = 0; i < 6; ++i)
            register_input(std::string{"/wheel_leg/rl/effective_action/"} + kNames[i], history_[i]);
    }
    void update() override {
        ++ticks;
        bool zero = true, bounded = true;
        for (std::size_t i = 0; i < 6; ++i) {
            zero &= *torque_[i] == 0.0;
            bounded &= std::isfinite(*torque_[i]) && std::abs(*torque_[i]) <= (i < 4 ? 40.0 : 4.5);
        }
        if (!bounded)
            invalid_effort = true;
        if (*state_ == 3) {
            for (int side = 0; side < 2; ++side) {
                const int hip = side * 2, knee = hip + 1;
                const double sign = side == 0 ? 1.0 : -1.0;
                const double relative = sign * (*angle_[knee] - *angle_[hip]);
                // This fixture uses J=-I, so model effort is minus motor effort.
                const double outward = sign * (*torque_[hip] - *torque_[knee]);
                const bool upper = relative >= 1.3889088544 - .03;
                const bool lower = relative <= -.140638855 + .03;
                if ((upper && outward > 1e-9) || (lower && outward < -1e-9))
                    invalid_soft_limit = true;
                if ((upper && outward < -1.0) || (lower && outward > 1.0))
                    inward_effort = true;
                if (*torque_override_ && (upper || lower))
                    projected_effort = true;
                if (std::abs(*torque_[hip]) >= 39.999 || std::abs(*torque_[knee]) >= 39.999)
                    saturated_effort = true;
            }
        }
        all_zero = zero;
        valid = *valid_ > 0.5;
        if (*valid_ > 0.5 && *sequence_ != action_sequence.load()) {
            inference_times.push_back(*inference_);
            action_ages.push_back(*age_);
        }
        action_sequence = *sequence_;
        policy_rate = *rate_;
        state = *state_;
        enabled = *enable_;
        fault = *fault_;
        forward = command_->x();
        gravity_z = gravity_->z();
        bool history_zero = true;
        for (auto& value : history_)
            history_zero &= *value == 0.0;
        history_cleared = history_zero;
        if (*state_ == 2 && !zero)
            effort_before_policy = true;
    }
    std::atomic<std::size_t> ticks{0}, action_sequence{0};
    std::atomic<bool> valid{false}, enabled{false}, all_zero{true}, invalid_effort{false};
    std::atomic<int> state{0};
    std::atomic<double> policy_rate{0.0};
    std::atomic<bool> fault{false}, history_cleared{true}, effort_before_policy{false};
    std::atomic<double> forward{0.0}, gravity_z{0.0};
    std::atomic<bool> invalid_soft_limit{false}, inward_effort{false}, projected_effort{false},
        saturated_effort{false};
    // Read only after the executor thread has joined.
    std::vector<double> inference_times, action_ages;

private:
    std::array<InputInterface<double>, 6> torque_;
    std::array<InputInterface<double>, 4> angle_;
    InputInterface<bool> torque_override_;
    InputInterface<double> valid_, rate_, age_, inference_;
    InputInterface<std::size_t> sequence_;
    InputInterface<int> state_;
    InputInterface<bool> enable_;
    InputInterface<bool> fault_;
    InputInterface<Eigen::Vector3d> command_, gravity_;
    std::array<InputInterface<double>, 6> history_;
};

TEST(WheelLegPolicyIntegration, SaturationAndHingeSoftLimitsKeepPolicyActiveWithReceiveTimeouts) {
    rclcpp::executors::SingleThreadedExecutor ros;
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    std::atomic<int> phase{0};
    Component::initializing_component_name = "snapshots";
    auto sensors = std::make_shared<Snapshots>(phase);
    Component::initializing_component_name = "wheel_leg_controller";
    auto controller = std::make_shared<Controller>();
    Component::initializing_component_name = "rl_bridge";
    auto bridge = loader.createSharedInstance("rmcs_rl::RlBridge");
    Component::initializing_component_name = "observe";
    auto observe = std::make_shared<Observe>();
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        executor.add_component(sensors);
        executor.add_component(controller);
        executor.add_component(bridge);
        executor.add_component(observe);
        ASSERT_NO_THROW(executor.start());
        std::this_thread::sleep_for(30ms);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_FALSE(observe->valid);
        // Drive handshake and explicit PREPARE must never move to a scripted pose.
        phase = 1;
        std::this_thread::sleep_for(40ms);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_TRUE(observe->enabled);
        EXPECT_FALSE(observe->valid);
        EXPECT_EQ(observe->state, 2);
        phase = 2;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->valid);
        EXPECT_TRUE(observe->enabled);
        EXPECT_EQ(observe->state, 3); // 54 degrees, moving gyro, nonnominal legs.
        EXPECT_FALSE(observe->all_zero);
        const auto first_sequence = observe->action_sequence.load();
        EXPECT_GE(first_sequence, static_cast<std::size_t>(observe->policy_rate / 20.0));
        phase = 3;
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3); // Upside down must remain policy controlled.
        EXPECT_TRUE(observe->valid);
        EXPECT_NEAR(observe->gravity_z.load(), 1.0, 1e-9);
        EXPECT_NEAR(observe->forward.load(), 0.2, 1e-9);
        phase = 4;
        std::this_thread::sleep_for(30ms);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_TRUE(observe->history_cleared);
        EXPECT_FALSE(observe->valid);
        EXPECT_FALSE(observe->enabled);
        EXPECT_EQ(observe->state, 1);
        phase = 5;
        std::this_thread::sleep_for(40ms);
        EXPECT_EQ(observe->state, 2);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_FALSE(observe->valid);
        phase = 6;
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3);
        EXPECT_TRUE(observe->valid);
        EXPECT_GT(observe->action_sequence.load(), first_sequence);
        phase = 7;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_FALSE(observe->enabled);
        EXPECT_TRUE(observe->fault);
        EXPECT_EQ(observe->state, 1);
        phase = 8;
        std::this_thread::sleep_for(80ms);
        EXPECT_EQ(observe->state, 1); // Fresh feedback alone cannot clear a latched fault.
        EXPECT_TRUE(observe->all_zero);
        EXPECT_TRUE(observe->fault);
        phase = 9;
        std::this_thread::sleep_for(30ms);
        EXPECT_FALSE(observe->fault);
        EXPECT_TRUE(observe->history_cleared);
        phase = 10;
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3);
        EXPECT_TRUE(observe->valid);
        EXPECT_FALSE(observe->invalid_effort);
        phase = 11;
        std::this_thread::sleep_for(200ms);
        EXPECT_EQ(observe->state, 3); // Crossed hinge bounds never disarm RL.
        EXPECT_FALSE(observe->fault);
        EXPECT_TRUE(observe->enabled);
        EXPECT_TRUE(observe->valid);
        EXPECT_NEAR(observe->forward.load(), 3.0, 1e-9);
        EXPECT_TRUE(observe->saturated_effort);
        EXPECT_TRUE(observe->inward_effort);
        phase = 12;
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3);
        EXPECT_TRUE(observe->projected_effort);
        EXPECT_FALSE(observe->invalid_soft_limit);
        phase = 13;
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3);
        EXPECT_TRUE(observe->enabled);
        EXPECT_TRUE(observe->valid);
        EXPECT_FALSE(observe->fault);
        EXPECT_NEAR(observe->forward.load(), 3.0, 1e-9);
        // Let clear-error/enable handshaking take longer than the old one-second cutoff.
        phase = 14;
        std::this_thread::sleep_for(1100ms);
        EXPECT_EQ(observe->state, 2);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_TRUE(observe->enabled);
        EXPECT_FALSE(observe->fault);
        phase = 15;
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3);
        EXPECT_TRUE(observe->valid);
        EXPECT_FALSE(observe->fault);
        EXPECT_FALSE(observe->invalid_effort);
        EXPECT_FALSE(observe->effort_before_policy);
        EXPECT_GT(observe->ticks.load(), 100u);
        rclcpp::shutdown();
    }
    ASSERT_FALSE(observe->inference_times.empty());
    std::ranges::sort(observe->inference_times);
    std::cout << "Policy " << observe->policy_rate << " Hz, " << observe->inference_times.size()
              << " returned actions, inference us p50/p95/max "
              << observe->inference_times[observe->inference_times.size() / 2] << "/"
              << observe->inference_times[observe->inference_times.size() * 95 / 100] << "/"
              << observe->inference_times.back() << ", max accepted age ms "
              << *std::ranges::max_element(observe->action_ages) * 1000.0 << '\n';
}
} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    const char* profile = std::getenv("RMCS_TEST_PROFILE");
    const char* arguments[]{
        "wheel_leg_policy_integration", "--ros-args", "--params-file",
        profile ? profile : RMCS_PROFILE};
    rclcpp::init(4, arguments);
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
