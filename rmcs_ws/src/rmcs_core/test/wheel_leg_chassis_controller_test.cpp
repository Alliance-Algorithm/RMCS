#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

// Bind the production component's registered ports without hardware or an executor.
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

#include "controller/chassis/wheel_leg_chassis_controller.cpp"

namespace rmcs_core::controller::chassis {
namespace {
class ChassisControllerTest : public ::testing::Test {
protected:
    using Switch = rmcs_msgs::Switch;
    using Ports = rmcs_executor::Executor;
    void SetUp() override { create(); }
    void TearDown() override {
        controller_.reset();
        rclcpp::shutdown();
    }
    void create(const std::vector<std::string>& parameters = {}) {
        std::vector<std::string> args{
            "wheel_leg_chassis_controller_test", "--ros-args", "--log-level", "error"};
        for (const auto& parameter : parameters) {
            args.push_back("-p");
            args.push_back(parameter);
        }
        std::vector<const char*> argv;
        for (const auto& arg : args)
            argv.push_back(arg.c_str());
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
        rmcs_executor::Component::initializing_component_name = "wheel_leg_chassis_controller";
        controller_ = std::make_unique<WheelLegChassisController>();
        Ports::bind(*controller_, "/remote/joystick/right", right_stick_);
        Ports::bind(*controller_, "/remote/joystick/left", left_stick_);
        Ports::bind(*controller_, "/remote/switch/right", right_);
        Ports::bind(*controller_, "/remote/switch/left", left_);
        Ports::bind(*controller_, "/remote/rotary_knob", knob_);
        Ports::bind(*controller_, "/remote/keyboard", keyboard_);
        Ports::bind(*controller_, "/wheel_leg/imu/quaternion", orientation_);
        Ports::require_bound_inputs(*controller_);
        controller_->before_updating();
    }
    template <typename T>
    const T& output(const std::string& name) {
        return Ports::output<T>(*controller_, "/chassis/" + name);
    }
    void step(Switch left, Switch right) {
        left_ = left;
        right_ = right;
        controller_->update();
    }
    void arm() {
        step(Switch::DOWN, Switch::DOWN);
        step(Switch::MIDDLE, Switch::MIDDLE);
        ASSERT_EQ(output<int>("control_state"), 3);
    }
    const Eigen::Vector3d& velocity() {
        return output<rmcs_description::BaseLink::DirectionVector>("control_velocity").vector;
    }
    std::unique_ptr<WheelLegChassisController> controller_;
    Eigen::Vector2d right_stick_ = Eigen::Vector2d::Zero(), left_stick_ = Eigen::Vector2d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Switch left_ = Switch::UNKNOWN, right_ = Switch::UNKNOWN;
    rmcs_msgs::Keyboard keyboard_ = rmcs_msgs::Keyboard::zero();
    double knob_ = 0.0;
};

TEST_F(ChassisControllerTest, StartupAndAsynchronousArmingKeepStateAndResetContract) {
    step(Switch::UNKNOWN, Switch::UNKNOWN);
    EXPECT_EQ(output<int>("control_state"), 0);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    EXPECT_EQ(output<std::size_t>("reset_count"), 0);
    step(Switch::DOWN, Switch::DOWN);
    step(Switch::DOWN, Switch::DOWN);
    EXPECT_EQ(output<std::size_t>("reset_count"), 1);
    step(Switch::MIDDLE, Switch::DOWN);
    EXPECT_EQ(output<int>("control_state"), 1);
    EXPECT_TRUE(velocity().isZero());
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 3);
    step(Switch::UP, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    arm();
    EXPECT_EQ(output<std::size_t>("reset_count"), 2);
}

TEST_F(ChassisControllerTest, ResetAndUnknownRemoteClearMotionAndJumpInTheSameTick) {
    arm();
    right_stick_.y() = 1.0;
    keyboard_.v = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(velocity().x(), 2.5);
    EXPECT_TRUE(output<bool>("jump_request"));
    EXPECT_DOUBLE_EQ(output<double>("jump_apex_delta"), 0.06);
    step(Switch::UNKNOWN, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    EXPECT_TRUE(velocity().isZero());
    EXPECT_FALSE(output<bool>("jump_request"));
    EXPECT_DOUBLE_EQ(output<double>("jump_apex_delta"), 0.0);
    EXPECT_EQ(output<std::size_t>("reset_count"), 2);
}

TEST_F(ChassisControllerTest, ModeTogglesOnlyOnKeyEdgesAndResetReturnsAuto) {
    arm();
    keyboard_.c = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::SPIN_FAST);
    EXPECT_NEAR(velocity().z(), -1.8, 1e-12);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::SPIN_FAST);
    keyboard_.c = false;
    step(Switch::MIDDLE, Switch::MIDDLE);
    keyboard_.c = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::AUTO);
    step(Switch::DOWN, Switch::DOWN);
    EXPECT_TRUE(velocity().isZero());
}

TEST_F(ChassisControllerTest, FixedHeightProfileKeepsRequestedHeightConstant) {
    controller_.reset();
    rclcpp::shutdown();
    create(
        {"command_height_min:=0.305", "command_height_max:=0.305", "default_command_height:=0.305",
         "height_range:=0.0", "height_step:=0.0"});
    arm();
    left_stick_.y() = 1.0;
    knob_ = 1.0;
    keyboard_.q = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.305);
}

TEST_F(ChassisControllerTest, RejectsReversedLimitsAndInvalidDeadzone) {
    for (const auto& parameter : {"command_height_min:=0.5", "deadzone:=1.0", "heading_kp:=-1.0"}) {
        controller_.reset();
        rclcpp::shutdown();
        EXPECT_THROW(create({parameter}), std::invalid_argument);
    }
}
} // namespace
} // namespace rmcs_core::controller::chassis
