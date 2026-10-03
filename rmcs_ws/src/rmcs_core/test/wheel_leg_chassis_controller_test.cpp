#include <array>
#include <cmath>
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
        Ports::bind(*controller_, "/wheel_leg/dr16_fresh", remote_fresh_);
        Ports::bind(*controller_, "/remote/rotary_knob", knob_);
        Ports::bind(*controller_, "/remote/keyboard", keyboard_);
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
    Switch left_ = Switch::UNKNOWN, right_ = Switch::UNKNOWN;
    rmcs_msgs::Keyboard keyboard_ = rmcs_msgs::Keyboard::zero();
    double knob_ = 0.0;
    bool remote_fresh_ = true;
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

TEST_F(ChassisControllerTest, RemoteLossClearsAnArmedSessionDespiteCachedDoubleMiddle) {
    arm();
    right_stick_.x() = 1.0;
    keyboard_.v = true;
    remote_fresh_ = false;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    EXPECT_TRUE(velocity().isZero());
    EXPECT_FALSE(output<bool>("jump_request"));
    remote_fresh_ = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    arm();
}

TEST_F(ChassisControllerTest, RemoteLossBetweenDownAndMiddleRequiresAnotherFreshDown) {
    step(Switch::DOWN, Switch::DOWN);
    remote_fresh_ = false;
    step(Switch::DOWN, Switch::DOWN);
    remote_fresh_ = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    arm();
}

TEST_F(ChassisControllerTest, ResetAndUnknownRemoteClearMotionAndJumpInTheSameTick) {
    controller_.reset();
    rclcpp::shutdown();
    create({"jump_enabled:=true"});
    arm();
    right_stick_.x() = 1.0;
    keyboard_.v = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(velocity().x(), 0.5);
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
    EXPECT_DOUBLE_EQ(velocity().z(), -1.0);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::SPIN_FAST);
    keyboard_.c = false;
    step(Switch::MIDDLE, Switch::MIDDLE);
    keyboard_.c = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::AUTO);
    keyboard_.c = false;
    step(Switch::MIDDLE, Switch::MIDDLE);
    keyboard_.c = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(velocity().z(), 1.0);
    step(Switch::DOWN, Switch::DOWN);
    EXPECT_TRUE(velocity().isZero());
}

TEST_F(ChassisControllerTest, RightStickTranslatesAndLeftHorizontalStickControlsYaw) {
    arm();
    right_stick_ = {1.0, 0.0};
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isApprox(Eigen::Vector3d{0.5, 0.0, 0.0}));
    right_stick_ = {-1.0, 0.0};
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isApprox(Eigen::Vector3d{-0.5, 0.0, 0.0}));
    right_stick_ = {0.0, 1.0};
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isZero()); // The current model has no lateral command domain.
    left_stick_ = {0.0, 1.0};
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isApprox(Eigen::Vector3d{0.0, 0.0, 1.0}));
    left_stick_.y() = -1.0;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(velocity().z(), -1.0);
}

TEST_F(ChassisControllerTest, KeyboardWasdOnlyAddsTranslationAndUnsupportedModesStayAuto) {
    arm();
    keyboard_.w = true;
    right_stick_.x() = 1.0;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isApprox(Eigen::Vector3d{0.5, 0.0, 0.0}));
    right_stick_.setZero();
    keyboard_.w = false;
    keyboard_.s = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(velocity().x(), -0.5);
    keyboard_.s = false;
    keyboard_.a = true;
    keyboard_.x = true;
    keyboard_.z = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isZero());
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::AUTO);
    keyboard_.a = false;
    keyboard_.d = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isZero());
}

TEST_F(ChassisControllerTest, SpinSwitchRequiresAnArmedSessionAndTogglesOnCombinationEdges) {
    step(Switch::MIDDLE, Switch::DOWN);
    EXPECT_EQ(output<int>("control_state"), 1);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::AUTO);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
    arm();
    right_stick_ = {1.0, 1.0};
    left_stick_.y() = 1.0;
    keyboard_.w = true;
    keyboard_.a = true;
    step(Switch::MIDDLE, Switch::DOWN);
    EXPECT_EQ(output<int>("control_state"), 3);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::SPIN_FAST);
    EXPECT_TRUE(velocity().isApprox(Eigen::Vector3d{0.0, 0.0, -1.0}));
    step(Switch::MIDDLE, Switch::DOWN);
    EXPECT_TRUE(velocity().isApprox(Eigen::Vector3d{0.0, 0.0, -1.0}));
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 3);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::SPIN_FAST);
    step(Switch::MIDDLE, Switch::DOWN);
    EXPECT_EQ(output<int>("control_state"), 3);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::AUTO);
    EXPECT_DOUBLE_EQ(velocity().z(), 1.0);
    step(Switch::UP, Switch::DOWN);
    EXPECT_EQ(output<int>("control_state"), 1);
    EXPECT_TRUE(velocity().isZero());
    step(Switch::MIDDLE, Switch::DOWN);
    EXPECT_EQ(output<int>("control_state"), 1);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
}

TEST_F(ChassisControllerTest, RemoteLossInSpinClearsSessionAndMode) {
    arm();
    step(Switch::MIDDLE, Switch::DOWN);
    remote_fresh_ = false;
    step(Switch::MIDDLE, Switch::DOWN);
    EXPECT_EQ(output<int>("control_state"), 1);
    EXPECT_EQ(output<rmcs_msgs::ChassisMode>("control_mode"), rmcs_msgs::ChassisMode::AUTO);
    EXPECT_TRUE(velocity().isZero());
    remote_fresh_ = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_EQ(output<int>("control_state"), 1);
}

TEST_F(ChassisControllerTest, MotionLimitsDeadzoneAndNonfiniteInputsRemainBounded) {
    controller_.reset();
    rclcpp::shutdown();
    create(
        {"vx_max:=0.4", "vy_max:=0.3", "yaw_rate_max:=0.8", "spin_yaw_rate:=0.6",
         "angular_z_invert:=true"});
    arm();
    right_stick_ = {1.0, 1.0};
    left_stick_.y() = 5.0;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_NEAR(velocity().x(), 0.4 / std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(velocity().y(), 0.3 / std::sqrt(2.0), 1e-12);
    EXPECT_DOUBLE_EQ(velocity().z(), -0.8);
    right_stick_ = {0.04, std::numeric_limits<double>::quiet_NaN()};
    left_stick_.y() = std::numeric_limits<double>::infinity();
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isZero());
    keyboard_.c = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_TRUE(velocity().isApprox(Eigen::Vector3d{0.0, 0.0, 0.6}));
}

TEST_F(ChassisControllerTest, JumpRequestIsDisabledByDefault) {
    arm();
    keyboard_.v = true;
    keyboard_.shift = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_FALSE(output<bool>("jump_request"));
    EXPECT_DOUBLE_EQ(output<double>("jump_apex_delta"), 0.0);
}

TEST_F(ChassisControllerTest, RotaryKnobMapsAsymmetricHeightAndLeftVerticalStickHasNoEffect) {
    controller_.reset();
    rclcpp::shutdown();
    create(
        {"command_height_min:=0.23", "command_height_max:=0.43", "default_command_height:=0.305",
         "height_step:=0.01"});
    arm();
    left_stick_ = {1.0, 0.0};
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.305);
    EXPECT_DOUBLE_EQ(velocity().z(), 0.0);
    knob_ = -1.0;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.23);
    knob_ = 1.0;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.43);
    for (const double knob :
         {0.0, 0.04, -0.04, std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()}) {
        knob_ = knob;
        step(Switch::MIDDLE, Switch::MIDDLE);
        EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.305);
    }
    knob_ = 0.54; // Half of the positive travel after the 0.08 deadzone.
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.3675);
    knob_ = -0.54;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.2675);
    knob_ = 0.0;
    left_stick_ = {0.0, 1.0};
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.305);
    EXPECT_DOUBLE_EQ(velocity().z(), 1.0);
}

TEST_F(ChassisControllerTest, HeightKeysAddBoundedOffsetAndRotaryInversionReversesEndpoints) {
    controller_.reset();
    rclcpp::shutdown();
    create(
        {"command_height_min:=0.23", "command_height_max:=0.43", "default_command_height:=0.305",
         "height_step:=0.01", "height_invert:=true"});
    arm();
    knob_ = 1.0;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.23);
    knob_ = -1.0;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.43);
    knob_ = 0.0;
    keyboard_.r = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.315);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.315);
    keyboard_.r = false;
    keyboard_.f = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.305);
    keyboard_.f = false;
    for (int i = 0; i < 50; ++i) {
        keyboard_.r = true;
        step(Switch::MIDDLE, Switch::MIDDLE);
        keyboard_.r = false;
        step(Switch::MIDDLE, Switch::MIDDLE);
    }
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.43);
    keyboard_.f = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.42);
    step(Switch::DOWN, Switch::DOWN);
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.305);
}

TEST_F(ChassisControllerTest, FixedHeightProfileKeepsRequestedHeightConstant) {
    controller_.reset();
    rclcpp::shutdown();
    create(
        {"command_height_min:=0.305", "command_height_max:=0.305", "default_command_height:=0.305",
         "height_step:=0.0"});
    arm();
    left_stick_.x() = 1.0;
    knob_ = 1.0;
    keyboard_.r = true;
    step(Switch::MIDDLE, Switch::MIDDLE);
    EXPECT_DOUBLE_EQ(output<double>("control_height"), 0.305);
}

TEST_F(ChassisControllerTest, RejectsReversedLimitsAndInvalidDeadzone) {
    for (const auto& parameter :
         {"command_height_min:=0.5", "deadzone:=1.0", "vy_max:=-0.1", "spin_yaw_rate:=1.1",
          "yaw_rate_max:=0.0"}) {
        controller_.reset();
        rclcpp::shutdown();
        EXPECT_THROW(create({parameter}), std::invalid_argument);
    }
}
} // namespace
} // namespace rmcs_core::controller::chassis
