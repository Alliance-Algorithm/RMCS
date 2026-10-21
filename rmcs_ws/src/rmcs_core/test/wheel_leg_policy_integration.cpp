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
        constexpr std::array nominal{-0.42, 0.13742282595395358, 0.42, -0.1374155762580851};
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
        register_output("/wheel_leg/joint_position_pd", motor_pd_, false);
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
        *state_ = phase == 0 || phase == 2 || phase == 5 ? 1 : 3;
        *reset_ = phase < 2 ? 1 : phase < 5 ? 2 : 3;
        const auto now = std::chrono::steady_clock::now();
        const auto ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        for (std::size_t i = 0; i < 6; ++i) {
            *sequence_[i] += 1;
            *stamp_[i] = ns;
        }
        if (phase != 4) {
            *imu_sequence_ += 1;
            *accel_sequence_ += 1;
            *imu_stamp_ = *accel_stamp_ = ns;
        }
    }

private:
    std::atomic<int>& phase_;
    std::array<OutputInterface<double>, 6> angle_, velocity_, torque_, limit_;
    std::array<OutputInterface<std::uint64_t>, 6> sequence_, stamp_;
    std::array<OutputInterface<int>, 4> fault_;
    OutputInterface<bool> fresh_, ready_, motor_pd_, jump_;
    OutputInterface<Eigen::Quaterniond> orientation_;
    OutputInterface<Eigen::Vector3d> gyro_, acceleration_;
    OutputInterface<std::uint64_t> imu_sequence_, imu_stamp_, accel_sequence_, accel_stamp_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    OutputInterface<double> height_, apex_;
    OutputInterface<int> state_;
    OutputInterface<std::size_t> reset_;
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
};

class Observe final : public Component {
public:
    Observe() {
        for (std::size_t i = 0; i < 6; ++i)
            register_input(std::string{"/wheel_leg/"} + kNames[i] + "/control_torque", torque_[i]);
        register_input("/wheel_leg/rl/valid", valid_);
        register_input("/wheel_leg/rl/action_seq", sequence_);
        register_input("/wheel_leg/rl/state", state_);
        register_input("/wheel_leg/enable_request", enable_);
        register_input("/wheel_leg/rl/policy_rate", rate_);
        register_input("/wheel_leg/rl/action_age", age_);
        register_input("/wheel_leg/rl/performance/inference_us", inference_);
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
    }
    std::atomic<std::size_t> ticks{0}, action_sequence{0};
    std::atomic<bool> valid{false}, enabled{false}, all_zero{true}, invalid_effort{false};
    std::atomic<int> state{0};
    std::atomic<double> policy_rate{0.0};
    // Read only after the executor thread has joined.
    std::vector<double> inference_times, action_ages;

private:
    std::array<InputInterface<double>, 6> torque_;
    InputInterface<double> valid_, rate_, age_, inference_;
    InputInterface<std::size_t> sequence_;
    InputInterface<int> state_;
    InputInterface<bool> enable_;
};

TEST(WheelLegPolicyIntegration, AcyclicWorkerPipelineResetRearmAndSensorExpiry) {
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
        phase = 1;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->valid);
        EXPECT_TRUE(observe->enabled);
        EXPECT_GE(observe->state.load(), 2);
        const auto first_sequence = observe->action_sequence.load();
        EXPECT_GE(first_sequence, static_cast<std::size_t>(observe->policy_rate / 20.0));
        phase = 2;
        std::this_thread::sleep_for(30ms);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_FALSE(observe->valid);
        EXPECT_FALSE(observe->enabled);
        EXPECT_EQ(observe->state, 1);
        phase = 3;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->valid);
        EXPECT_GT(observe->action_sequence.load(), first_sequence);
        phase = 4;
        std::this_thread::sleep_for(80ms);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_FALSE(observe->enabled);
        EXPECT_EQ(observe->state, 1);
        phase = 5;
        std::this_thread::sleep_for(30ms);
        phase = 6;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->valid);
        EXPECT_TRUE(observe->enabled);
        EXPECT_FALSE(observe->invalid_effort);
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
