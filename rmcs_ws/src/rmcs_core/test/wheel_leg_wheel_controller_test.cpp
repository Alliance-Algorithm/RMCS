#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

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

// Run actual controller logic with synthetic ports; no hardware is constructed.
#include "identification/wheel_leg_wheel_identification_controller.cpp"

namespace rmcs_core::controller::identification {
namespace {
class WheelControllerTest : public ::testing::Test {
protected:
    using Clock = std::chrono::steady_clock;
    using Switch = rmcs_msgs::Switch;
    using Ports = rmcs_executor::Executor;
    void SetUp() override {
        const char* args[]{"wheel_test",    "--ros-args",
                           "--params-file", WHEEL_CONTROLLER_TEST_PROFILE,
                           "--log-level",   "error"};
        rclcpp::init(6, args);
        rmcs_executor::Component::initializing_component_name =
            "wheel_leg_wheel_identification_controller";
        controller = std::make_unique<WheelLegWheelIdentificationController>();
        bind("/predefined/update_count", tick);
        bind("/predefined/update_rate", rate);
        bind("/predefined/timestamp", time);
        bind("/remote/switch/left", left);
        bind("/remote/switch/right", right);
        bind("/wheel_leg/feedback_fresh", fresh);
        bind("/wheel_leg/dr16_fresh", remote);
        constexpr std::array names{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (int i = 0; i < 4; ++i) {
            const std::string prefix = std::string("/wheel_leg/") + names[i];
            bind(prefix + "/status_code", status[i]);
            bind(prefix + "/fault_code", fault[i]);
        }
        for (int i = 0; i < 2; ++i) {
            const std::string prefix = i == 0 ? "/wheel_leg/left_wheel" : "/wheel_leg/right_wheel";
            bind(prefix + "/velocity", speed[i]);
            bind(prefix + "/max_torque", max_torque[i]);
            bind(prefix + "/feedback_steady_ns", feedback_ns[i]);
        }
        Ports::require_bound_inputs(*controller);
    }
    void TearDown() override {
        controller.reset();
        rclcpp::shutdown();
    }
    template <typename T>
    void bind(const std::string& name, T& value) {
        Ports::bind(*controller, name, value);
    }
    template <typename T>
    const T& out(const std::string& name) {
        return Ports::output<T>(*controller, "/wheel_leg/" + name);
    }
    void step(int count = 1, std::size_t tick_increment = 1, int elapsed_ms = 1) {
        for (int i = 0; i < count; ++i) {
            tick += tick_increment;
            time += std::chrono::milliseconds(elapsed_ms);
            feedback_ns.fill(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    Clock::now().time_since_epoch())
                    .count());
            controller->update();
        }
    }
    void start() {
        step(510);
        left = right = Switch::MIDDLE;
        step(151);
        ASSERT_EQ(out<int>("identification/phase"), 2);
        ASSERT_TRUE(out<bool>("enable_request"));
    }
    void expect_released() {
        EXPECT_FALSE(out<bool>("enable_request"));
        EXPECT_EQ(out<int>("identification/segment_id"), -1);
        for (const auto* name :
             {"left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint",
              "left_wheel", "right_wheel"})
            EXPECT_TRUE(std::isnan(out<double>(std::string{name} + "/control_torque"))) << name;
        for (const auto* name : {"left_wheel", "right_wheel"})
            EXPECT_TRUE(std::isnan(out<double>(std::string{name} + "/control_velocity"))) << name;
    }
    std::unique_ptr<WheelLegWheelIdentificationController> controller;
    std::size_t tick = 0;
    double rate = 1000.;
    Clock::time_point time = Clock::now();
    Switch left = Switch::DOWN, right = Switch::DOWN;
    bool fresh = true, remote = true;
    std::array<int, 4> status{}, fault{};
    std::array<double, 2> speed{}, max_torque{4.936675020885547, 4.936675020885547};
    std::array<std::uint64_t, 2> feedback_ns{};
};

TEST_F(WheelControllerTest, OneKhzFeedbackUpdatesWithinFiftyHzTargetHold) {
    start();
    step(4501);
    const double target = out<double>("left_wheel/control_velocity");
    const double before = out<double>("left_wheel/control_torque");
    EXPECT_GT(target, 0.);
    speed = {.7, 7.};
    step();
    EXPECT_EQ(out<double>("left_wheel/control_velocity"), target);
    EXPECT_NEAR(
        out<double>("left_wheel/control_torque"),
        before - controller->get_parameter("wheel_velocity_kp").as_double() * .7, 1e-12);
    EXPECT_EQ(out<double>("right_wheel/control_velocity"), 0.);
    EXPECT_EQ(out<double>("right_wheel/control_torque"), 0.);
    speed[0] = -35.;
    step();
    EXPECT_NEAR(
        out<double>("left_wheel/control_torque"),
        controller->get_parameter("wheel_torque_cap").as_double(), 1e-12);
    left = right = Switch::DOWN;
    step();
    EXPECT_FALSE(out<bool>("enable_request"));
    EXPECT_TRUE(std::isnan(out<double>("left_wheel/control_torque")));
    EXPECT_TRUE(std::isnan(out<double>("right_wheel/control_torque")));
}

TEST_F(WheelControllerTest, RejectsWrongDriverReductionAndRemoteLoss) {
    start();
    max_torque[0] = 6.;
    step();
    EXPECT_EQ(out<int>("identification/failure_reason"), 6);
    EXPECT_FALSE(out<bool>("enable_request"));
}

TEST_F(WheelControllerTest, StaleRemoteReleasesWheelCurrent) {
    start();
    step(4501);
    remote = false;
    step();
    EXPECT_EQ(out<int>("identification/failure_reason"), 3);
    EXPECT_FALSE(out<bool>("enable_request"));
    EXPECT_TRUE(std::isnan(out<double>("left_wheel/control_torque")));
}

TEST_F(WheelControllerTest, TrueCoastThenRightSideAndBothAreSelected) {
    const WheelProbePlan plan{{{.2, .5, 1.}, {2., 5., 10., 20., 30.}, 1., 2., 4., 1., 12., 12., 2}};
    start();
    int elapsed_ms = 0;
    bool coast = false, right_seen = false, both_seen = false;
    for (const auto& segment : plan.segments()) {
        bool selected = (!coast && segment.label == "left_zero_current_release")
                     || (!right_seen && segment.label == "right_low_entry")
                     || (!both_seen && segment.label == "both_api_opposite_entry");
        if (!selected)
            continue;
        const int target_ms = static_cast<int>(std::round((segment.start_s + .5) * 1000));
        step(target_ms - elapsed_ms);
        elapsed_ms = target_ms;
        speed = {1., 1.};
        step();
        ++elapsed_ms;
        ASSERT_EQ(out<int>("identification/phase"), 2);
        if (segment.coast) {
            coast = true;
            EXPECT_EQ(out<double>("left_wheel/control_torque"), 0.);
            EXPECT_EQ(out<double>("right_wheel/control_torque"), 0.);
        } else if (segment.axis_scale[0] == 0.) {
            right_seen = true;
            EXPECT_EQ(out<double>("left_wheel/control_torque"), 0.);
            EXPECT_GT(out<double>("right_wheel/control_velocity"), 0.);
            EXPECT_NEAR(
                out<double>("right_wheel/control_torque"),
                controller->get_parameter("wheel_velocity_kp").as_double()
                    * (out<double>("right_wheel/control_velocity") - 1.),
                1e-12);
        } else {
            both_seen = true;
            EXPECT_GT(out<double>("left_wheel/control_velocity"), 0.);
            EXPECT_LT(out<double>("right_wheel/control_velocity"), 0.);
        }
    }
    EXPECT_TRUE(coast && right_seen && both_seen);
    step(static_cast<int>(std::round(plan.duration() * 1000)) + 1 - elapsed_ms);
    EXPECT_EQ(out<int>("identification/phase"), 3);
    EXPECT_FALSE(out<bool>("enable_request"));
}

struct WheelClockFault {
    std::size_t tick_increment;
    int elapsed_ms;
    const char* name;
};

class WheelClockTest
    : public WheelControllerTest
    , public ::testing::WithParamInterface<WheelClockFault> {};

TEST_P(WheelClockTest, FaultRequiresFreshDownAndNewDwellBeforeRestart) {
    start();
    step(4501);
    ASSERT_GT(std::abs(out<double>("left_wheel/control_torque")), 0.0);
    const auto fault = GetParam();
    step(1, fault.tick_increment, fault.elapsed_ms);
    ASSERT_EQ(out<int>("identification/phase"), -1);
    ASSERT_EQ(out<int>("identification/failure_reason"), 2);
    expect_released();
    step();
    EXPECT_EQ(out<int>("identification/phase"), -1);
    expect_released();

    left = right = Switch::DOWN;
    remote = false;
    step();
    EXPECT_EQ(out<int>("identification/phase"), -1);
    EXPECT_EQ(out<std::uint32_t>("identification/repetition_id"), 0u);
    expect_released();

    // Reset has priority over the broken executor clock, invalid update rate
    // and stale motor feedback, but still requires a fresh operator request.
    remote = true;
    fresh = false;
    rate = std::numeric_limits<double>::quiet_NaN();
    step(1, 8, -100);
    ASSERT_EQ(out<int>("identification/phase"), 0);
    EXPECT_EQ(out<int>("identification/failure_reason"), 0);
    EXPECT_EQ(out<std::uint32_t>("identification/repetition_id"), 1u);
    expect_released();

    rate = 1000.0;
    fresh = true;
    step(100);
    left = right = Switch::MIDDLE;
    step();
    EXPECT_EQ(out<int>("identification/phase"), -1);
    EXPECT_EQ(out<int>("identification/failure_reason"), 3);
    expect_released();

    left = right = Switch::DOWN;
    start();
    EXPECT_EQ(out<int>("identification/failure_reason"), 0);
    EXPECT_EQ(out<std::uint32_t>("identification/repetition_id"), 2u);
}

TEST_P(WheelClockTest, BothDownPreemptsClockFaultAndPreservesStopSample) {
    start();
    const auto fault = GetParam();
    left = right = Switch::DOWN;
    fresh = false;
    step(1, fault.tick_increment, fault.elapsed_ms);
    EXPECT_EQ(out<int>("identification/phase"), -1);
    EXPECT_EQ(out<int>("identification/failure_reason"), 3);
    expect_released();

    step();
    EXPECT_EQ(out<int>("identification/phase"), 0);
    EXPECT_EQ(out<int>("identification/failure_reason"), 0);
    EXPECT_EQ(out<std::uint32_t>("identification/repetition_id"), 1u);
    expect_released();

    fresh = true;
    start();
}

INSTANTIATE_TEST_SUITE_P(
    ClockDiscontinuities, WheelClockTest,
    ::testing::Values(
        WheelClockFault{8, 1, "TickJump"}, WheelClockFault{1, -100, "ClockRollback"},
        WheelClockFault{1, 21, "LongInterval"}),
    [](const ::testing::TestParamInfo<WheelClockFault>& info) { return info.param.name; });

} // namespace
} // namespace rmcs_core::controller::identification
