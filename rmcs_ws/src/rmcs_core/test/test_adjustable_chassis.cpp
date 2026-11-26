#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "executor.hpp"

namespace {

using Component = rmcs_executor::Component;
using Switch = rmcs_msgs::Switch;
using Angles = std::array<double, 6>;
constexpr Angles initial_angles{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
const std::array<std::string, 4> corners{"left_front", "left_back", "right_back", "right_front"};
const std::array<std::string, 6> lift_names{"master_left_front", "master_left_back",
                                            "master_right_back", "master_right_front",
                                            "slave_left",        "slave_right"};

struct Frame {
    Switch left = Switch::MIDDLE;
    Switch right = Switch::MIDDLE;
    Eigen::Vector2d stick = Eigen::Vector2d::Zero();
    Angles angles = initial_angles;
};

// Feedback and operator input only. Commands are observed by a separate component,
// preserving the same feedback -> controllers -> command dependency graph as hardware.
class MockFeedback final : public Component {
public:
    explicit MockFeedback(std::vector<Frame> frames)
        : frames_(std::move(frames)) {
        register_output("/remote/joystick/left", stick_left_, Eigen::Vector2d::Zero());
        register_output("/remote/joystick/right", stick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/switch/left", switch_left_, Switch::UNKNOWN);
        register_output("/remote/switch/right", switch_right_, Switch::UNKNOWN);
        register_output("/chassis/yaw_imu_velocity", yaw_velocity_, 0.0);
        register_output("/test/frame", frame_index_, std::size_t{0});
        for (std::size_t i = 0; i < corners.size(); ++i) {
            register_output(
                "/chassis/steering/" + corners[i] + "/angle", steering_angle_[i],
                std::numbers::pi / 4.0);
            register_output(
                "/chassis/steering/" + corners[i] + "/velocity", steering_velocity_[i], 0.0);
            register_output("/chassis/wheel/" + corners[i] + "/velocity", wheel_velocity_[i], 0.0);
        }
        for (std::size_t i = 0; i < lift_names.size(); ++i) {
            register_output("/chassis/lift/" + lift_names[i] + "/angle", lift_angle_[i], 0.0);
            register_output("/chassis/lift/" + lift_names[i] + "/velocity", lift_velocity_[i], 0.0);
        }
    }

    void update() override {
        const auto i = std::min(next_frame_++, frames_.size() - 1);
        const auto& frame = frames_[i];
        *frame_index_ = i;
        *switch_left_ = frame.left;
        *switch_right_ = frame.right;
        *stick_left_ = frame.stick;
        for (std::size_t j = 0; j < lift_names.size(); ++j)
            *lift_angle_[j] = frame.angles[j];
    }

private:
    std::vector<Frame> frames_;
    std::size_t next_frame_ = 0;
    OutputInterface<Eigen::Vector2d> stick_left_, stick_right_;
    OutputInterface<Switch> switch_left_, switch_right_;
    OutputInterface<double> yaw_velocity_;
    OutputInterface<std::size_t> frame_index_;
    std::array<OutputInterface<double>, 4> steering_angle_, steering_velocity_, wheel_velocity_;
    std::array<OutputInterface<double>, 6> lift_angle_, lift_velocity_;
};

struct Snapshot {
    Eigen::Vector3d velocity;
    double power_limit;
    std::string state;
    Angles target, lift_torque;
    std::array<double, 4> steering_torque, wheel_torque;
};

class CommandObserver final : public Component {
public:
    explicit CommandObserver(std::size_t frame_count)
        : frame_count_(frame_count) {
        register_input("/test/frame", frame_index_);
        register_input("/chassis/control_velocity", velocity_);
        register_input("/chassis/control_power_limit", power_limit_);
        register_input("/chassis/climb_state", state_);
        for (std::size_t i = 0; i < corners.size(); ++i) {
            register_input(
                "/chassis/steering/" + corners[i] + "/control_torque", steering_torque_[i]);
            register_input("/chassis/wheel/" + corners[i] + "/control_torque", wheel_torque_[i]);
        }
        for (std::size_t i = 0; i < lift_names.size(); ++i) {
            register_input("/chassis/lift/" + lift_names[i] + "/target_angle", target_[i]);
            register_input("/chassis/lift/" + lift_names[i] + "/control_torque", lift_torque_[i]);
        }
    }

    void update() override {
        Snapshot snapshot;
        snapshot.velocity = velocity_->vector;
        snapshot.power_limit = *power_limit_;
        snapshot.state = *state_;
        for (std::size_t i = 0; i < corners.size(); ++i) {
            snapshot.steering_torque[i] = *steering_torque_[i];
            snapshot.wheel_torque[i] = *wheel_torque_[i];
        }
        for (std::size_t i = 0; i < lift_names.size(); ++i) {
            snapshot.target[i] = *target_[i];
            snapshot.lift_torque[i] = *lift_torque_[i];
        }
        {
            const std::lock_guard lock(mutex_);
            snapshots.push_back(std::move(snapshot));
            done_ = *frame_index_ + 1 >= frame_count_;
        }
        if (done_) {
            rclcpp::shutdown();
            condition_.notify_one();
        }
    }

    bool wait() {
        std::unique_lock lock(mutex_);
        return condition_.wait_for(lock, std::chrono::seconds(10), [this] { return done_; });
    }

    std::vector<Snapshot> snapshots;

private:
    std::size_t frame_count_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool done_ = false;
    InputInterface<std::size_t> frame_index_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> velocity_;
    InputInterface<double> power_limit_;
    InputInterface<std::string> state_;
    std::array<InputInterface<double>, 6> target_, lift_torque_;
    std::array<InputInterface<double>, 4> steering_torque_, wheel_torque_;
};

class AdjustableChassisTest : public ::testing::Test {
protected:
    void SetUp() override {
        const char* args[]{
            "test_adjustable_chassis",
            "--ros-args",
            "--params-file",
            HEAVY_CHASSIS_CONFIG,
            "--params-file",
            ADJUSTABLE_CHASSIS_TEST_CONFIG,
            "--log-level",
            "warn"};
        rclcpp::init(static_cast<int>(std::size(args)), args);
    }

    void TearDown() override { rclcpp::shutdown(); }

    std::vector<Snapshot>
        run(const std::vector<Frame>& frames,
            const std::string& upper_name = "adjustable_chassis_controller") {
        pluginlib::ClassLoader<Component> loader("rmcs_executor", "rmcs_executor::Component");
        rclcpp::executors::SingleThreadedExecutor ros_executor;
        auto executor = std::make_shared<rmcs_executor::Executor>("rmcs_executor", ros_executor);
        Component::initializing_component_name = "mock_feedback";
        auto feedback = std::make_shared<MockFeedback>(frames);
        executor->add_component(feedback);
        for (const auto& [class_name, name] : std::array{
                 std::pair{
                     "rmcs_core::controller::chassis::AdjustableChassisController", upper_name},
                 std::pair{
                     "rmcs_core::controller::chassis::SteeringWheelController",
                     std::string{"steering_wheel_controller"}},
                 std::pair{
                     "rmcs_core::controller::chassis::HeavyStabilizingController",
                     std::string{"heavy_stabilizing_controller"}}}) {
            Component::initializing_component_name = name;
            executor->add_component(loader.createSharedInstance(class_name));
        }
        Component::initializing_component_name = "command_observer";
        auto observer = std::make_shared<CommandObserver>(frames.size());
        executor->add_component(observer);
        try {
            executor->start();
            const bool done = observer->wait();
            rclcpp::shutdown();
            executor.reset();     // Join the control thread before examining snapshots.
            if (!done)
                throw std::runtime_error("Timed out waiting for the scripted control frames");
        } catch (...) {
            rclcpp::shutdown();
            executor.reset();
            throw;
        }
        return std::move(observer->snapshots);
    }

    static void expect_stopped(const Snapshot& snapshot) {
        EXPECT_TRUE(snapshot.velocity.array().isNaN().all());
        EXPECT_DOUBLE_EQ(snapshot.power_limit, 0.0);
        for (double angle : snapshot.target)
            EXPECT_TRUE(std::isnan(angle));
        for (double torque : snapshot.lift_torque)
            EXPECT_TRUE(std::isnan(torque));
        for (double torque : snapshot.steering_torque)
            EXPECT_DOUBLE_EQ(torque, 0.0);
        for (double torque : snapshot.wheel_torque)
            EXPECT_DOUBLE_EQ(torque, 0.0);
    }

    static void expect_targets(const Snapshot& snapshot, const Angles& target) {
        for (std::size_t i = 0; i < target.size(); ++i)
            EXPECT_NEAR(snapshot.target[i], target[i], 1e-12) << "lift index " << i;
    }
};

TEST_F(AdjustableChassisTest, DrivesFourDirectionsAndRequiresRearmAfterSignalLoss) {
    const std::vector<Frame> frames{
        {.stick = {1.0, 0.0}},    // Startup cannot move before the both-DOWN handshake.
        {.left = Switch::DOWN, .right = Switch::DOWN},
        {.stick = {1.0, 0.0}},
        {.stick = {-1.0, 0.0}},
        {.stick = {0.0, 1.0}},
        {.stick = {0.0, -1.0}},
        {.stick = {0.03, -0.03}}, // Dead zone.
        {.left = Switch::UNKNOWN},
        {.stick = {1.0, 0.0}},    // Signal recovery alone does not re-enable motion.
        {.left = Switch::DOWN, .right = Switch::DOWN},
        {.stick = {1.0, 0.0}},
        {.stick = {std::numeric_limits<double>::quiet_NaN(), 0.0}},
        {.stick = {1.0, 0.0}},
    };
    const auto snapshots = run(frames);
    ASSERT_EQ(snapshots.size(), frames.size());
    for (std::size_t i : {0, 1, 7, 8, 9, 11, 12})
        expect_stopped(snapshots[i]);
    const std::array<Eigen::Vector2d, 4> directions{
        Eigen::Vector2d{0.5, 0.0}, {-0.5, 0.0}, {0.0, 0.5}, {0.0, -0.5}};
    for (std::size_t i = 0; i < directions.size(); ++i) {
        EXPECT_TRUE(snapshots[i + 2].velocity.head<2>().isApprox(directions[i]));
        expect_targets(snapshots[i + 2], initial_angles);
        EXPECT_DOUBLE_EQ(snapshots[i + 2].power_limit, 70.0);
    }
    EXPECT_TRUE(snapshots[6].velocity.isZero());
    EXPECT_DOUBLE_EQ(snapshots[10].velocity.x(), 0.5);
    EXPECT_TRUE(
        std::any_of(
            snapshots[2].wheel_torque.begin(), snapshots[2].wheel_torque.end(),
            [](double torque) { return std::isfinite(torque) && std::abs(torque) > 1e-9; }));
    for (const auto& snapshot : snapshots) {
        if (snapshot.velocity.allFinite()) {
            EXPECT_DOUBLE_EQ(snapshot.velocity.z(), 0.0); // No spin mode.
        }
    }
}

TEST_F(AdjustableChassisTest, ClimbAdvancesOncePerEdgeAndHoldsUntilOperatorLeaves) {
    std::vector<Frame> frames{
        {.left = Switch::DOWN, .right = Switch::DOWN},
        {},
        {.left = Switch::UP, .stick = {1.0, 1.0}},        // LIFT stops chassis translation.
        {.left = Switch::UP, .right = Switch::UP, .stick = {1.0, 1.0}},
        {.left = Switch::UP, .right = Switch::UP, .stick = {1.0, 1.0}},     // Held switch.
        {.left = Switch::UP},
        {.left = Switch::UP, .right = Switch::UP},                          // Front retracts.
        {.left = Switch::UP},
        {.left = Switch::UP, .right = Switch::UP},                          // Middle retracts.
        {.left = Switch::UP},
        {.left = Switch::UP, .right = Switch::UP},                          // Rear retracts.
        {.left = Switch::UP},
        {.left = Switch::UP, .right = Switch::UP, .stick = {1.0, 0.0}},     // HOLD.
    };
    for (int i = 0; i < 8; ++i)
        frames.push_back({.left = Switch::UP, .right = Switch::UP, .stick = {1.0, 0.0}});
    const Angles measured_after_climb{0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    const auto exit_index = frames.size();
    frames.push_back({.angles = measured_after_climb});
    frames.push_back({.left = Switch::UP, .angles = measured_after_climb}); // Explicit restart.
    frames.push_back({.angles = measured_after_climb}); // Abort captures measured feedback.
    const auto snapshots = run(frames);
    ASSERT_EQ(snapshots.size(), frames.size());

    EXPECT_EQ(snapshots[2].state, "LIFT");
    expect_targets(snapshots[2], {2.0, 2.5, 2.5, 2.0, 1.5, 1.5});
    EXPECT_TRUE(snapshots[2].velocity.isZero());
    EXPECT_GT(snapshots[2].lift_torque[0], 0.0);        // Lower PID consumes upper targets.
    for (std::size_t i : {3, 4, 5})
        EXPECT_EQ(snapshots[i].state, "APPROACHING");
    EXPECT_DOUBLE_EQ(snapshots[3].velocity.x(), 0.15);
    EXPECT_DOUBLE_EQ(snapshots[3].velocity.y(), 0.0);
    EXPECT_EQ(snapshots[6].state, "PRE_CLIMBING");
    expect_targets(snapshots[6], {1.5, 2.5, 2.5, 1.5, 1.5, 1.5});
    EXPECT_EQ(snapshots[8].state, "SLAVE_COLLAPSE");
    expect_targets(snapshots[8], {1.5, 2.5, 2.5, 1.5, 0.5, 0.5});
    EXPECT_EQ(snapshots[10].state, "MASTER_COLLAPSE");
    expect_targets(snapshots[10], {1.5, 2.25, 2.25, 1.5, 0.5, 0.5});
    EXPECT_EQ(snapshots[12].state, "HOLD");
    EXPECT_TRUE(snapshots[12].velocity.isZero());
    EXPECT_EQ(snapshots[exit_index - 1].state, "IDLE");
    expect_targets(snapshots[exit_index - 1], {1.5, 2.25, 2.25, 1.5, 0.5, 0.5});
    expect_targets(snapshots[exit_index], measured_after_climb);
    EXPECT_EQ(snapshots[exit_index + 1].state, "LIFT");
    expect_targets(snapshots[exit_index + 2], measured_after_climb);
    for (const auto& snapshot : snapshots) {
        if (snapshot.velocity.allFinite()) {
            EXPECT_DOUBLE_EQ(snapshot.velocity.z(), 0.0);
        }
    }
}

TEST_F(AdjustableChassisTest, UncalibratedGeometryAllowsDrivingButDoesNotMoveTheLifts) {
    const std::vector<Frame> frames{
        {.left = Switch::DOWN, .right = Switch::DOWN},
        {.stick = {1.0, 0.0}},
        {.left = Switch::UP, .stick = {1.0, 1.0}},
        {.left = Switch::UP, .right = Switch::UP},
        {.left = Switch::UP},
        {.left = Switch::UP, .right = Switch::UP},
    };
    const auto snapshots = run(frames, "uncalibrated_chassis_controller");
    ASSERT_EQ(snapshots.size(), frames.size());
    EXPECT_DOUBLE_EQ(snapshots[1].velocity.x(), 0.5);
    for (std::size_t i = 2; i < snapshots.size(); ++i) {
        EXPECT_EQ(snapshots[i].state, "UNCALIBRATED");
        EXPECT_TRUE(snapshots[i].velocity.isZero());
        expect_targets(snapshots[i], initial_angles);
        for (double torque : snapshots[i].lift_torque)
            EXPECT_DOUBLE_EQ(torque, 0.0);
    }
}

} // namespace
