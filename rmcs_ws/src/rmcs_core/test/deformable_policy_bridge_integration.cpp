// Regression for the existing 8x32 deformable actor and its delayed action history.
// Inputs are in-memory snapshots; no vehicle component is loaded.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include <eigen3/Eigen/Dense>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_description/tf_description.hpp>

#include "executor.hpp"

namespace {
using namespace std::chrono_literals;
using Component = rmcs_executor::Component;

class Inputs final : public Component {
public:
    explicit Inputs(std::atomic<int>& phase)
        : phase_(phase) {
        register_output("/chassis/deformable/rl_q_cmd", q_, 0.2);
        register_output(
            "/chassis/control_velocity", command_,
            rmcs_description::BaseLink::DirectionVector{0, 0, 0});
        register_output("/chassis/rl/gyro_b", gyro_, Eigen::Vector3d::Zero().eval());
        register_output("/chassis/rl/compensated_gravity_b", gravity_, Eigen::Vector3d{0, 0, -1});
        register_output("/chassis/rl/encoder_twist_b", twist_, Eigen::Vector3d::Zero().eval());
        register_output("/chassis/active_suspension/active", enable_, false);
        register_output("/chassis/deformable/reset_count", reset_, std::size_t{0});
        register_output("/chassis/rl/flat_hold", flat_, false);
        constexpr std::array names{"right_front", "left_front", "left_back", "right_back"};
        for (std::size_t i = 0; i < 4; ++i) {
            const auto base = std::string{"/chassis/"} + names[i];
            register_output(base + "_joint/urdf_angle", angle_[i], 0.2);
            register_output(base + "_joint/urdf_velocity", velocity_[i], 0.0);
            register_output(base + "_joint/feedback_current_raw", current_[i], 0.0);
            register_output(base + "_wheel/rl_velocity", wheel_[i], 0.0);
        }
    }
    void update() override {
        const auto phase = phase_.load();
        *enable_ = phase != 0 && phase != 3;
        *flat_ = phase == 2;
        *reset_ = phase >= 3 ? 1 : 0;
        *q_ = phase == 5 ? std::numeric_limits<double>::quiet_NaN() : 0.2;
    }

private:
    std::atomic<int>& phase_;
    OutputInterface<double> q_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    OutputInterface<Eigen::Vector3d> gyro_, gravity_, twist_;
    OutputInterface<bool> enable_, flat_;
    OutputInterface<std::size_t> reset_;
    std::array<OutputInterface<double>, 4> angle_, velocity_, current_, wheel_;
};

class Observe final : public Component {
public:
    Observe() {
        register_input("/chassis/rl/valid", valid_);
        register_input("/chassis/rl/action_seq", seq_);
        register_input("/chassis/rl/observation_size", obs_size_);
        register_input("/chassis/rl/history_length", history_);
        register_input("/chassis/rl/feature_size", feature_);
        for (std::size_t i = 0; i < 4; ++i) {
            register_input("/chassis/rl/action/joint_leg_" + std::to_string(i + 1), action_[i]);
            register_input(
                "/chassis/rl/observation/last_action:0+1+2+3/" + std::to_string(i), previous_[i]);
        }
    }
    void update() override {
        valid = *valid_ > 0.5;
        sequence = *seq_;
        shape_ok = *obs_size_ == 256 && *history_ == 8 && *feature_ == 32;
        bool zero = true, finite = true, history_zero = true;
        for (std::size_t i = 0; i < 4; ++i) {
            zero &= *action_[i] == 0.0;
            finite &= std::isfinite(*action_[i]);
            history_zero &= *previous_[i] == 0.0;
        }
        all_zero = zero;
        history_is_zero = history_zero;
        actions_finite = finite;
    }
    std::atomic<bool> valid{false}, shape_ok{false}, all_zero{false};
    std::atomic<bool> history_is_zero{false}, actions_finite{false};
    std::atomic<std::size_t> sequence{0};

private:
    InputInterface<double> valid_;
    InputInterface<std::size_t> seq_, obs_size_, history_, feature_;
    std::array<InputInterface<double>, 4> action_, previous_;
};

TEST(DeformablePolicyBridgeIntegration, RealActorDelayedHistoryFlatHoldResetAndInvalidInput) {
    rclcpp::executors::SingleThreadedExecutor ros;
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    std::atomic<int> phase{0};
    Component::initializing_component_name = "snapshots";
    auto inputs = std::make_shared<Inputs>(phase);
    Component::initializing_component_name = "rl_bridge";
    auto bridge = loader.createSharedInstance("rmcs_rl::RlBridge");
    Component::initializing_component_name = "observe";
    auto observe = std::make_shared<Observe>();
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        executor.add_component(inputs);
        executor.add_component(bridge);
        executor.add_component(observe);
        ASSERT_NO_THROW(executor.start());
        std::this_thread::sleep_for(30ms);
        EXPECT_TRUE(observe->shape_ok);
        EXPECT_FALSE(observe->valid);
        phase = 1;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->valid);
        EXPECT_TRUE(observe->actions_finite);
        phase = 2;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->valid);
        EXPECT_TRUE(observe->all_zero);
        EXPECT_TRUE(observe->history_is_zero);
        const auto before_reset = observe->sequence.load();
        phase = 3;
        std::this_thread::sleep_for(30ms);
        EXPECT_FALSE(observe->valid);
        EXPECT_FALSE(observe->actions_finite);
        phase = 4;
        std::this_thread::sleep_for(100ms);
        EXPECT_TRUE(observe->valid);
        EXPECT_GT(observe->sequence.load(), before_reset);
        phase = 5;
        std::this_thread::sleep_for(30ms);
        EXPECT_FALSE(observe->valid);
        EXPECT_FALSE(observe->actions_finite);
        rclcpp::shutdown();
    }
}
} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    const char* args[]{
        "deformable_policy_bridge_integration", "--ros-args", "--params-file", RMCS_PROFILE};
    rclcpp::init(4, args);
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
