#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <string>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "component_fixture.hpp"
#include "controller/chassis/wheel_leg_control_state.hpp"

namespace {

using Component = rmcs_executor::Component;

class Signals : public Component {
public:
    double& output(const std::string& path, double initial) {
        register_output(path, outputs_[path], initial);
        return *outputs_[path];
    }
    void input(const std::string& path) { register_input(path, inputs_[path]); }
    double read(const std::string& path) const { return *inputs_.at(path); }
    void update() override {}

private:
    std::map<std::string, OutputInterface<double>> outputs_;
    std::map<std::string, InputInterface<double>> inputs_;
};

class RlConsumersTest : public testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    std::shared_ptr<Component> create(const char* name) {
        Component::initializing_component_name = "consumer_test";
        return loader_.createSharedInstance(std::string{"rmcs_core::controller::chassis::"} + name);
    }
    pluginlib::ClassLoader<Component> loader_{"rmcs_executor", "rmcs_executor::Component"};
};

TEST_F(RlConsumersTest, WheelLegMappingInvalidPolicyHoldAndReset) {
    using State = rmcs_core::controller::chassis::WheelLegControlState;
    Signals source, sink;
    Component::OutputInterface<int> state;
    Component::OutputInterface<std::size_t> reset;
    source.register_output("/chassis/control_state", state, static_cast<int>(State::kRl));
    source.register_output("/chassis/reset_count", reset, std::size_t{0});
    auto& valid = source.output("/wheel_leg/rl/valid", 1.0);
    source.output("/wheel_leg/rl/healthy", 1.0);
    constexpr std::array<const char*, 4> names{"left_hip", "left_knee", "right_hip", "right_knee"};
    constexpr std::array<double, 4> nominal{0.42, -0.13742282595254576, -0.42, 0.13741557625658019};
    for (std::size_t i = 0; i < names.size(); ++i) {
        const auto joint = std::string{"/wheel_leg/"} + names[i] + "_joint";
        source.output(joint + "/angle", 0.1 * (i + 1));
        source.output(std::string{"/wheel_leg/rl/action/"} + names[i], 1.0);
        sink.input(joint + "/control_angle");
    }
    source.output("/wheel_leg/rl/action/left_wheel", 2.0);
    source.output("/wheel_leg/rl/action/right_wheel", -2.0);
    sink.input("/wheel_leg/left_wheel/control_velocity");
    sink.input("/wheel_leg/right_wheel/control_velocity");
    auto consumer = create("WheelLegRlConsumer");
    std::array<Component*, 3> graph{&source, consumer.get(), &sink};
    rmcs_executor::Executor::pair(graph);
    consumer->update();
    for (std::size_t i = 0; i < names.size(); ++i)
        EXPECT_NEAR(
            sink.read(std::string{"/wheel_leg/"} + names[i] + "_joint/control_angle"),
            nominal[i] + 0.25, 1e-12);
    EXPECT_EQ(sink.read("/wheel_leg/left_wheel/control_velocity"), 20.0);
    EXPECT_EQ(sink.read("/wheel_leg/right_wheel/control_velocity"), -20.0);
    valid = 0.0;
    consumer->update();
    EXPECT_EQ(sink.read("/wheel_leg/left_hip_joint/control_angle"), 0.1);
    EXPECT_EQ(sink.read("/wheel_leg/left_wheel/control_velocity"), 0.0);
    *state = static_cast<int>(State::kUrdfZero);
    consumer->update();
    EXPECT_EQ(sink.read("/wheel_leg/left_hip_joint/control_angle"), 0.0);
    *state = static_cast<int>(State::kCalibratedZero);
    consumer->update();
    EXPECT_EQ(sink.read("/wheel_leg/left_hip_joint/control_angle"), -1.6);
    EXPECT_EQ(sink.read("/wheel_leg/right_knee_joint/control_angle"), 2.93);
    ++*reset;
    consumer->update();
    EXPECT_EQ(sink.read("/wheel_leg/left_hip_joint/control_angle"), -0.08);
    EXPECT_EQ(sink.read("/wheel_leg/right_wheel/control_velocity"), 0.0);
}

TEST_F(RlConsumersTest, DeformableMapsCornersAndFallsBackOnInvalidActionOrReset) {
    Signals source, sink;
    Component::OutputInterface<bool> active;
    Component::OutputInterface<std::size_t> reset;
    source.register_output("/chassis/active_suspension/active", active, true);
    source.register_output("/chassis/deformable/reset_count", reset, std::size_t{0});
    source.output("/chassis/rl/valid", 1.0);
    source.output("/chassis/rl/healthy", 1.0);
    source.output("/chassis/deformable/rl_q_cmd", 0.3);
    source.output("/chassis/rl/calibration/high_physical_angle_rad", 1.0);
    source.output("/chassis/rl/calibration/low_physical_angle_rad", 0.0);
    source.output("/chassis/rl/calibration/q_max_rad", 1.0);
    auto& first_action = source.output("/chassis/rl/action/joint_leg_1", 0.0);
    for (std::size_t i = 1; i < 4; ++i)
        source.output("/chassis/rl/action/joint_leg_" + std::to_string(i + 1), i);
    constexpr std::array<const char*, 4> corners{
        "right_front", "left_front", "left_back", "right_back"};
    for (const auto* name : corners) {
        const auto joint = std::string{"/chassis/"} + name + "_joint";
        source.output(joint + "/physical_angle", 0.5);
        source.output(joint + "/traditional_target_physical_angle", 0.9);
        sink.input(joint + "/target_physical_angle");
        sink.input(joint + "/target_physical_velocity");
    }
    auto consumer = create("DeformableRlSuspension");
    auto arbiter = create("DeformableSuspensionArbiter");
    std::array<Component*, 4> graph{&source, consumer.get(), arbiter.get(), &sink};
    rmcs_executor::Executor::pair(graph);
    consumer->before_updating();
    arbiter->before_updating();
    consumer->update();
    arbiter->update();
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const auto joint = std::string{"/chassis/"} + corners[i] + "_joint";
        EXPECT_NEAR(sink.read(joint + "/target_physical_angle"), 0.7 - 0.15 * i, 1e-12);
        EXPECT_TRUE(std::isnan(sink.read(joint + "/target_physical_velocity")));
    }
    first_action = std::numeric_limits<double>::quiet_NaN();
    consumer->update();
    arbiter->update();
    for (const auto* name : corners)
        EXPECT_EQ(sink.read(std::string{"/chassis/"} + name + "_joint/target_physical_angle"), 0.9);
    first_action = 0.0;
    ++*reset;
    consumer->update();
    arbiter->update();
    EXPECT_EQ(sink.read("/chassis/right_front_joint/target_physical_angle"), 0.9);
}

} // namespace
