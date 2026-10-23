// Production controller/bridge/executor with memory-only sensors. No hardware.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <thread>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "controller/chassis/wheel_leg/wheel_leg_controller.hpp"
#include "executor.hpp"

namespace {
using namespace std::chrono_literals;
using namespace rmcs_core::controller::chassis;
using Component = rmcs_executor::Component;

class UnifiedSnapshots final : public Component {
public:
    explicit UnifiedSnapshots(std::atomic<int>& phase)
        : phase_(phase) {
        constexpr std::array offsets{1.6, 2.93, -1.6, -2.93};
        for (std::size_t i = 0; i < 6; ++i) {
            const auto path = std::string{"/wheel_leg/"} + kMotorNames[i];
            const double q = i < 4 ? kV6PolicyProfile.nominal[i] + (i < 2 ? .2 : -.3) : 0.0;
            register_output(path + "/angle", q_[i], i < 4 ? offsets[i] - q : 0.0);
            register_output(path + "/velocity", dq_[i], 0.0);
            register_output(path + "/torque", torque_[i], 0.0);
            register_output(path + "/max_torque", limit_[i], i < 4 ? 40.0 : 4.5);
            register_output(path + "/feedback_sequence", sequence_[i], std::uint64_t{0});
            register_output(path + "/feedback_steady_ns", stamp_[i], std::uint64_t{0});
            if (i < 4)
                register_output(path + "/fault_code", fault_[i], 0);
        }
        register_output("/wheel_leg/feedback_fresh", fresh_, true);
        register_output("/wheel_leg/dm_control_ready", ready_, false);
        register_output("/wheel_leg/joint_position_pd", motor_pd_, false);
        register_output("/wheel_leg/imu/quaternion", orientation_, Eigen::Quaterniond::Identity());
        register_output("/wheel_leg/imu/angular_velocity", gyro_, Eigen::Vector3d::Zero().eval());
        register_output("/wheel_leg/imu/acceleration", acceleration_, Eigen::Vector3d{0, 0, 9.81});
        register_output("/wheel_leg/imu/sequence", imu_sequence_, std::uint64_t{0});
        register_output("/wheel_leg/imu/last_steady_ns", imu_stamp_, std::uint64_t{0});
        register_output(
            "/wheel_leg/imu/acceleration_sequence", acceleration_sequence_, std::uint64_t{0});
        register_output(
            "/wheel_leg/imu/acceleration_steady_ns", acceleration_stamp_, std::uint64_t{0});
        register_output(
            "/chassis/control_velocity", command_,
            rmcs_description::BaseLink::DirectionVector{0, 0, 0});
        register_output("/chassis/control_height", height_, .305);
        register_output("/chassis/control_state", state_, 1);
        register_output("/chassis/reset_count", reset_, std::size_t{1});
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        register_output("/chassis/jump_request", jump_, false);
        register_output("/chassis/jump_apex_delta", apex_, 0.0);
    }

    void update() override {
        const int phase = phase_.load();
        *state_ = phase == 0 || phase == 4 || phase == 7 ? 1 : phase == 1 ? 2 : 3;
        *reset_ = phase < 4 ? 1 : phase < 7 ? 2 : 3;
        *ready_ = phase != 0;
        const double roll = phase == 2 ? std::numbers::pi : phase == 3 ? std::numbers::pi / 2 : 0.0;
        *orientation_ = Eigen::Quaterniond{Eigen::AngleAxisd{roll, Eigen::Vector3d::UnitX()}};
        if (phase == 6)
            return;
        const auto now =
            static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::steady_clock::now().time_since_epoch())
                                           .count());
        for (std::size_t i = 0; i < 6; ++i) {
            ++*sequence_[i];
            *stamp_[i] = now;
        }
        ++*imu_sequence_;
        ++*acceleration_sequence_;
        *imu_stamp_ = *acceleration_stamp_ = now;
    }

private:
    std::atomic<int>& phase_;
    std::array<OutputInterface<double>, 6> q_, dq_, torque_, limit_;
    std::array<OutputInterface<std::uint64_t>, 6> sequence_, stamp_;
    std::array<OutputInterface<int>, 4> fault_;
    OutputInterface<bool> fresh_, ready_, motor_pd_, jump_;
    OutputInterface<Eigen::Quaterniond> orientation_;
    OutputInterface<Eigen::Vector3d> gyro_, acceleration_;
    OutputInterface<std::uint64_t> imu_sequence_, imu_stamp_, acceleration_sequence_,
        acceleration_stamp_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    OutputInterface<double> height_, apex_;
    OutputInterface<int> state_;
    OutputInterface<std::size_t> reset_;
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
};

class UnifiedObserve final : public Component {
public:
    UnifiedObserve() {
        for (std::size_t i = 0; i < 6; ++i) {
            register_input(
                std::string{"/wheel_leg/"} + kMotorNames[i] + "/control_torque", torque_[i]);
            register_input(
                std::string{"/wheel_leg/rl/effective_action/"} + kMotorNames[i], applied_[i]);
        }
        for (std::size_t i = 0; i < 35; ++i)
            register_input(
                "/wheel_leg/rl/observation/frame/element_" + std::to_string(i), serialized_[i]);
        register_input("/wheel_leg/rl/input/frame", frame_);
        register_input("/wheel_leg/rl/input/frame_sequence", frame_sequence_);
        register_input("/wheel_leg/rl/valid", valid_);
        register_input("/wheel_leg/rl/state", state_);
        register_input("/wheel_leg/enable_request", enabled_);
        register_input("/wheel_leg/rl/action_seq", action_sequence_);
        register_input("/wheel_leg/rl/fault_latched", fault_);
    }

    void update() override {
        bool zero = true;
        for (std::size_t i = 0; i < 6; ++i) {
            const double value = *torque_[i];
            zero &= value == 0;
            if (!std::isfinite(value) || std::abs(value) > (i < 4 ? 40.0 : 4.5))
                invalid_effort = true;
        }
        all_zero = zero;
        valid = *valid_ > .5;
        state = *state_;
        enabled = *enabled_;
        action_sequence = *action_sequence_;
        fault = *fault_;
        if (*state_ == 2 && !zero)
            effort_before_policy = true;
        if (*frame_sequence_ != 0 && *frame_sequence_ != last_frame_sequence_) {
            ++frames;
            if (!have_first_frame_) {
                std::copy(frame_->begin(), frame_->end(), first_frame.begin());
                have_first_frame_ = true;
            }
            for (std::size_t i = 0; i < 35; ++i)
                serialization_error =
                    std::max(serialization_error, std::abs((*frame_)[i] - *serialized_[i]));
            if (previous_state_ == 3)
                for (std::size_t i = 0; i < 6; ++i)
                    history_error =
                        std::max(history_error, std::abs((*frame_)[22 + i] - previous_applied_[i]));
            last_frame_sequence_ = *frame_sequence_;
        }
        for (std::size_t i = 0; i < 6; ++i)
            previous_applied_[i] = *applied_[i];
        previous_state_ = *state_;
    }

    std::atomic<int> state{0};
    std::atomic<bool> all_zero{true}, valid{false}, enabled{false}, invalid_effort{false},
        fault{false}, effort_before_policy{false};
    std::atomic<std::size_t> action_sequence{0}, frames{0};
    // Read after executor thread shutdown.
    std::array<double, 35> first_frame{};
    double serialization_error = 0, history_error = 0;

private:
    std::array<InputInterface<double>, 6> torque_, applied_;
    std::array<InputInterface<double>, 35> serialized_;
    InputInterface<std::vector<double>> frame_;
    InputInterface<std::size_t> frame_sequence_, action_sequence_;
    InputInterface<double> valid_;
    InputInterface<int> state_;
    InputInterface<bool> enabled_, fault_;
    std::size_t last_frame_sequence_ = 0;
    std::array<double, 6> previous_applied_{};
    int previous_state_ = 0;
    bool have_first_frame_ = false;
};

bool await(const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
        if (ready())
            return true;
        std::this_thread::sleep_for(5ms);
    }
    return ready();
}

TEST(UnifiedPolicyIntegration, RejectsLegacyPreprocessingHeadingHoldAndSlowPd) {
    for (const auto& parameter :
         {rclcpp::Parameter{"unified_full_rl", false},
          rclcpp::Parameter{"heading_hold_enabled", true},
          rclcpp::Parameter{"pd_frequency", 200.0}}) {
        SCOPED_TRACE(parameter.get_name());
        rclcpp::executors::SingleThreadedExecutor ros;
        pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
        std::atomic<int> phase{0};
        Component::initializing_component_name = "snapshots";
        auto sensors = std::make_shared<UnifiedSnapshots>(phase);
        Component::initializing_component_name = "wheel_leg_controller";
        auto options = rmcs_utility::NodeMixin::options().parameter_overrides({parameter});
        auto controller = std::make_shared<WheelLegController>(options);
        Component::initializing_component_name = "rl_bridge";
        auto bridge = loader.createSharedInstance("rmcs_rl::RlBridge");
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        executor.add_component(sensors);
        executor.add_component(controller);
        executor.add_component(bridge);
        EXPECT_THROW(executor.start(), std::runtime_error);
    }
}

TEST(UnifiedPolicyIntegration, NeutralPrepareDirectInvertedAndSideEntryResetAndStaleFeedback) {
    rclcpp::executors::SingleThreadedExecutor ros;
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    std::atomic<int> phase{0};
    Component::initializing_component_name = "snapshots";
    auto sensors = std::make_shared<UnifiedSnapshots>(phase);
    Component::initializing_component_name = "wheel_leg_controller";
    auto controller = std::make_shared<WheelLegController>();
    Component::initializing_component_name = "rl_bridge";
    auto bridge = loader.createSharedInstance("rmcs_rl::RlBridge");
    Component::initializing_component_name = "observe";
    auto observe = std::make_shared<UnifiedObserve>();
    Component::initializing_component_name = "value_broadcaster";
    auto broadcaster = loader.createSharedInstance("rmcs_core::broadcaster::ValueBroadcaster");
    auto broadcaster_node = std::dynamic_pointer_cast<rclcpp::Node>(broadcaster);
    ASSERT_NE(broadcaster_node, nullptr);
    auto forward_list = broadcaster_node->get_parameter("forward_list").as_string_array();
    // Exercise the deployed observation forwarding configuration with memory-only sensors.
    std::erase_if(forward_list, [](const auto& path) {
        return !path.starts_with("/wheel_leg/rl/observation/frame/");
    });
    ASSERT_EQ(forward_list.size(), 4u);
    ASSERT_TRUE(
        broadcaster_node->set_parameter(rclcpp::Parameter{"forward_list", forward_list}).successful);
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        executor.add_component(sensors);
        executor.add_component(controller);
        executor.add_component(bridge);
        executor.add_component(observe);
        executor.add_component(broadcaster);
        ASSERT_NO_THROW(executor.start());
        for (const auto& topic : forward_list)
            EXPECT_GT(broadcaster_node->count_publishers(topic), 0u) << topic;
        std::this_thread::sleep_for(30ms);
        EXPECT_TRUE(observe->all_zero);
        phase = 1;
        EXPECT_TRUE(await([&] { return observe->state == 2 && observe->enabled; }));
        std::this_thread::sleep_for(30ms);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_FALSE(observe->valid);
        phase = 2;
        EXPECT_TRUE(await([&] { return observe->state == 3 && observe->valid; }));
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3);
        phase = 3;
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(observe->state, 3);
        EXPECT_TRUE(observe->valid);
        const auto sequence = observe->action_sequence.load();
        phase = 4;
        EXPECT_TRUE(
            await([&] { return observe->state == 1 && observe->all_zero && !observe->enabled; }));
        phase = 5;
        EXPECT_TRUE(await([&] {
            return observe->state == 3 && observe->valid && observe->action_sequence > sequence;
        }));
        std::this_thread::sleep_for(100ms);
        phase = 6;
        EXPECT_TRUE(
            await([&] { return observe->state == 1 && observe->all_zero && !observe->enabled; }));
        EXPECT_TRUE(observe->fault);
        phase = 5; // Restored sensing cannot clear a latched fault.
        std::this_thread::sleep_for(30ms);
        EXPECT_EQ(observe->state, 1);
        EXPECT_TRUE(observe->all_zero);
        phase = 7;
        std::this_thread::sleep_for(30ms);
        phase = 8;
        EXPECT_TRUE(await([&] { return observe->state == 3 && observe->valid; }));
        EXPECT_FALSE(observe->invalid_effort);
        EXPECT_FALSE(observe->effort_before_policy);
        EXPECT_GT(observe->frames.load(), 50u);
        rclcpp::shutdown();
    }
    auto q = kV6PolicyProfile.nominal;
    for (int i = 0; i < 4; ++i)
        q[i] += i < 2 ? .2 : -.3;
    const auto passive = measured_policy_history(q);
    for (std::size_t i = 0; i < 6; ++i)
        EXPECT_NEAR(observe->first_frame[22 + i], passive[i], 2e-6);
    EXPECT_LT(observe->serialization_error, 1e-9);
    EXPECT_LT(observe->history_error, 2e-6);
    std::cout << "Coherent full-RL frames=" << observe->frames
              << ", serialization max=" << observe->serialization_error
              << ", applied-history max=" << observe->history_error << '\n';
}
} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    const char* profile = std::getenv("RMCS_UNIFIED_TEST_PROFILE");
    const char* arguments[]{
        "wheel_leg_unified_policy_integration", "--ros-args", "--params-file",
        profile ? profile : RMCS_PROFILE};
    rclcpp::init(4, arguments);
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
