// Production plugins and executor; sensor source is memory-only and never opens a board.
#include "controller/chassis/wheel_leg/policy/selection.hpp"
#include "executor.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <thread>
namespace {
using namespace std::chrono_literals;
using namespace rmcs_core::controller::chassis;
using Component = rmcs_executor::Component;
class Sensors final : public Component {
public:
    explicit Sensors(std::atomic<int>& phase)
        : phase_(phase) {
        for (int i = 0; i < 6; ++i) {
            const auto path = std::string{"/wheel_leg/"} + kMotorNames[i];
            register_output(
                path + "/angle", q_[i], i < 4 ? kMotorOffsets[i] - kNominalPosition[i] : 0.);
            register_output(path + "/velocity", dq_[i], 0.);
            register_output(path + "/max_torque", limit_[i], i < 4 ? 40. : 4.5);
            if (i < 4) {
                register_output(path + "/status_code", status_[i], 0);
                register_output(path + "/fault_code", fault_[i], 0);
            }
        }
        register_output("/wheel_leg/imu/quaternion", orientation_, Eigen::Quaterniond::Identity());
        register_output("/wheel_leg/imu/angular_velocity", gyro_, Eigen::Vector3d::Zero());
        register_output("/wheel_leg/feedback_fresh", fresh_, true);
        register_output("/wheel_leg/dr16_fresh", remote_, true);
        register_output("/wheel_leg/dm_control_ready", ready_, false);
        register_output("/wheel_leg/simulation", simulation_, true);
        register_output("/remote/switch/left", left_, rmcs_msgs::Switch::DOWN);
        register_output("/remote/switch/right", right_, rmcs_msgs::Switch::DOWN);
        register_output("/remote/joystick/left", stick_left_, Eigen::Vector2d::Zero());
        register_output("/remote/joystick/right", stick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/keyboard", keyboard_, rmcs_msgs::Keyboard::zero());
    }
    void update() override {
        const int phase = phase_.load();
        using S = rmcs_msgs::Switch;
        *left_ = phase == 0 || phase == 5 ? S::DOWN : phase == 6 ? S::UNKNOWN : S::MIDDLE;
        *right_ = phase == 0 || phase == 5 ? S::DOWN
                : phase == 6               ? S::UNKNOWN
                : phase == 3               ? S::UP
                : phase == 4               ? S::DOWN
                                           : S::MIDDLE;
        *remote_ = phase != 6;
        *fresh_ = phase != 7;
        *ready_ = phase >= 2 && phase != 5 && phase != 6;
        seen_phase = phase;
        if (phase == 8 && !gap_done.exchange(true))
            std::this_thread::sleep_for(80ms);
    }
    std::atomic<bool> gap_done{false};
    std::atomic<int> seen_phase{-1};

private:
    std::atomic<int>& phase_;
    std::array<OutputInterface<double>, 6> q_, dq_, limit_;
    std::array<OutputInterface<int>, 4> status_, fault_;
    OutputInterface<Eigen::Quaterniond> orientation_;
    OutputInterface<Eigen::Vector3d> gyro_;
    OutputInterface<Eigen::Vector2d> stick_left_, stick_right_;
    OutputInterface<rmcs_msgs::Switch> left_, right_;
    OutputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> fresh_, remote_, ready_, simulation_;
};
// A controlled inference boundary exercises the real controller's failure and PD scheduling.
class ManualActions final : public Component {
public:
    ManualActions(std::atomic<int>& phase, bool wheelbipe)
        : phase_(phase) {
        const auto& p = wheelbipe ? kWheelbipe : kProneMotion;
        for (int i = 0; i < 6; ++i) {
            const auto path = std::string{"/wheel_leg/rl/raw/"} + kMotorNames[i];
            register_output(path, raw_[i], 0.);
            register_output(path + "/scale", scale_[i], 1.);
            register_output(
                path + "/clip_min", minimum_[i], -std::numeric_limits<double>::infinity());
            register_output(
                path + "/clip_max", maximum_[i], std::numeric_limits<double>::infinity());
        }
        register_output("/wheel_leg/rl/valid", valid_, 1.);
        register_output("/wheel_leg/rl/healthy", healthy_, 1.);
        register_output("/wheel_leg/rl/action_seq", sequence_, std::size_t{0});
        register_output("/wheel_leg/rl/action_steady_ns", stamp_, std::uint64_t{0});
        register_output("/wheel_leg/rl/policy_rate", rate_, p.policy_hz);
        register_output("/wheel_leg/rl/model_spec_sha256", model_, std::string{kModelSpecSha256});
        register_output(
            "/wheel_leg/rl/observation_contract_sha256", observation_,
            std::string{p.observation_sha});
    }
    void update() override {
        ++*sequence_;
        *stamp_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
        for (int i = 0; i < 6; ++i)
            *raw_[i] = 1e-5 * static_cast<double>(*sequence_) + 1e-4 * i;
        *healthy_ = phase_ == 9 ? 0. : 1.;
    }

private:
    std::atomic<int>& phase_;
    std::array<OutputInterface<double>, 6> raw_, scale_, minimum_, maximum_;
    OutputInterface<double> valid_, healthy_, rate_;
    OutputInterface<std::size_t> sequence_;
    OutputInterface<std::uint64_t> stamp_;
    OutputInterface<std::string> model_, observation_;
};
class Observer final : public Component {
public:
    explicit Observer(bool rl)
        : rl_(rl) {
        register_input("/wheel_leg/enable_request", enabled_);
        register_input("/chassis/control_enable", armed_);
        register_input("/chassis/reset_count", reset_);
        register_input(rl ? "/wheel_leg/rl/fault_latched" : "/wheel_leg/control/fault", fault_);
        for (int i = 0; i < 6; ++i)
            register_input(
                std::string{"/wheel_leg/"} + kMotorNames[i] + "/control_torque", torque_[i]);
        if (rl) {
            register_input("/wheel_leg/rl/input/frame_sequence", sequence_);
            register_input("/wheel_leg/rl/input/frame", frame_);
            register_input("/wheel_leg/rl/applied_action", applied_);
            register_input("/wheel_leg/rl/action_applied", has_action_);
            register_input("/predefined/update_count", tick_);
        }
    }
    void update() override {
        bool zero = true;
        for (int i = 0; i < 6; ++i) {
            zero &= *torque_[i] == 0.;
            if (!std::isfinite(*torque_[i]) || std::abs(*torque_[i]) > (i < 4 ? 40. : 4.5) + 1e-9)
                invalid = true;
        }
        all_zero = zero;
        enabled = *enabled_;
        armed = *armed_;
        fault = *fault_;
        reset = *reset_;
        if (rl_ && !*has_action_)
            last_pd_tick_.reset();
        if (rl_ && *has_action_ && (!last_pd_tick_ || previous_action_ != *applied_)) {
            if (last_pd_tick_ && *tick_ - *last_pd_tick_ != 5)
                pd_clock_error = true;
            last_pd_tick_ = *tick_;
            previous_action_ = *applied_;
        }
        if (rl_ && *sequence_ != 0 && *sequence_ != last_sequence_) {
            if (last_frame_tick_) {
                const auto delta = *tick_ - *last_frame_tick_;
                if (delta != 10 && delta != 20)
                    irregular_clock = true;
            }
            last_frame_tick_ = *tick_;
            last_sequence_ = *sequence_;
            ++frames;
            if (*has_action_)
                for (int i = 0; i < 6; ++i) {
                    const int j = wheelbipe ? std::array{0, 2, 1, 3, 4, 5}[i] : i;
                    if (std::abs(
                            (*frame_)[22 + i]
                            - std::clamp(static_cast<float>((*applied_)[j]), -100.F, 100.F))
                        > 1e-6)
                        history_error = true;
                }
        }
        if (rl_ && *sequence_ == 0) {
            last_sequence_ = 0;
            last_frame_tick_.reset();
        }
    }
    bool wheelbipe = false;
    std::atomic<bool> pd_clock_error{false};
    std::atomic<bool> all_zero{true}, enabled{false}, armed{false}, fault{false}, invalid{false},
        history_error{false}, irregular_clock{false};
    std::atomic<std::size_t> frames{0}, reset{0};

private:
    bool rl_;
    InputInterface<bool> enabled_, armed_, fault_, has_action_;
    InputInterface<std::size_t> reset_, sequence_, tick_;
    InputInterface<std::vector<double>> frame_;
    InputInterface<JointVector> applied_;
    std::array<InputInterface<double>, 6> torque_;
    std::size_t last_sequence_ = 0;
    std::optional<std::size_t> last_frame_tick_, last_pd_tick_;
    JointVector previous_action_{};
};
bool await(const std::function<bool()>& check) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
        if (check())
            return true;
        std::this_thread::sleep_for(2ms);
    }
    return check();
}
TEST(WheelLegIntegration, LoadsRealPluginsAndHandlesArmStopDisconnectTimeoutAndReset) {
    const std::string profile = std::getenv("RMCS_WHEEL_LEG_PROFILE")
                                  ? std::getenv("RMCS_WHEEL_LEG_PROFILE")
                                  : RMCS_PROFILE;
    const bool manual = std::getenv("RMCS_MANUAL_INFERENCE") != nullptr;
    const bool rl = profile.find("traditional") == std::string::npos;
    rclcpp::executors::SingleThreadedExecutor ros;
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    std::atomic<int> phase{0};
    Component::initializing_component_name = "sensors";
    auto sensors = std::make_shared<Sensors>(phase);
    Component::initializing_component_name = "observer";
    auto observe = std::make_shared<Observer>(rl);
    observe->wheelbipe = profile.find("wheelbipe") != std::string::npos;
    auto set_phase = [&](int next) {
        phase = next;
        EXPECT_TRUE(await([&] { return sensors->seen_phase == next; }));
    };
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        executor.add_component(sensors);
        executor.add_component(observe);
        auto add = [&](const char* type, const char* name) {
            Component::initializing_component_name = name;
            executor.add_component(loader.createSharedInstance(type));
        };
        add("rmcs_core::controller::chassis::WheelLegChassisController",
            "wheel_leg_chassis_controller");
        add("rmcs_core::controller::chassis::WheelLegStateEstimator", "wheel_leg_state_estimator");
        if (rl) {
            add("rmcs_core::controller::chassis::WheelLegRlController", "wheel_leg_rl_controller");
            add("rmcs_core::controller::chassis::WheelLegObservation", "wheel_leg_observation");
            if (manual) {
                Component::initializing_component_name = "manual_actions";
                executor.add_component(std::make_shared<ManualActions>(phase, observe->wheelbipe));
            } else
                add("rmcs_rl::RlBridge", "rl_bridge");
        } else
            add("rmcs_core::controller::chassis::WheelLegController", "wheel_leg_controller");
        add("rmcs_core::broadcaster::ValueBroadcaster", "value_broadcaster");
        ASSERT_NO_THROW(executor.start());
        EXPECT_TRUE(await([&] { return observe->reset > 0; }));
        EXPECT_TRUE(observe->all_zero);
        set_phase(1);
        EXPECT_TRUE(await([&] { return observe->armed.load(); }));
        EXPECT_TRUE(observe->all_zero);
        set_phase(2);
        EXPECT_TRUE(await([&] { return observe->enabled.load(); }));
        if (rl) {
            EXPECT_TRUE(await([&] { return observe->frames >= 6; }));
        }
        std::this_thread::sleep_for(50ms);
        EXPECT_FALSE(observe->fault);
        EXPECT_FALSE(observe->invalid);
        if (!rl) {
            set_phase(3);
            std::this_thread::sleep_for(40ms);
            EXPECT_FALSE(observe->fault);
        }
        set_phase(5);
        EXPECT_TRUE(
            await([&] { return !observe->armed && observe->all_zero && !observe->enabled; }));
        set_phase(2);
        EXPECT_TRUE(await([&] { return observe->armed && observe->enabled; }));
        set_phase(6);
        EXPECT_TRUE(await([&] { return !observe->armed && observe->all_zero; }));
        // A middle command after disconnect cannot arm without an explicit double-DOWN reset.
        set_phase(2);
        std::this_thread::sleep_for(50ms);
        EXPECT_FALSE(observe->armed);
        set_phase(5);
        EXPECT_FALSE(observe->armed);
        std::this_thread::sleep_for(5ms);
        set_phase(2);
        EXPECT_TRUE(await([&] { return observe->armed && observe->enabled; }));
        set_phase(7);
        EXPECT_TRUE(
            await([&] { return observe->fault && observe->all_zero && !observe->enabled; }));
        set_phase(5);
        EXPECT_TRUE(await([&] { return !observe->fault && !observe->armed; }));
        std::this_thread::sleep_for(5ms);
        set_phase(2);
        EXPECT_TRUE(await([&] { return observe->armed && observe->enabled; }));
        if (rl) {
            const auto frames = observe->frames.load();
            EXPECT_TRUE(await([&] { return observe->frames > frames + 3; }));
            if (manual) {
                EXPECT_FALSE(observe->pd_clock_error);
                EXPECT_FALSE(observe->irregular_clock);
                set_phase(9);
                EXPECT_TRUE(await([&] { return observe->fault && observe->all_zero; }));
            } else {
                set_phase(8);
                EXPECT_TRUE(await(
                    [&] { return sensors->gap_done && observe->fault && observe->all_zero; }));
            }
            EXPECT_FALSE(observe->history_error);
            // A reset deliberately restarts the observation phase; active windows keep their
            // selected cadence.
        }
        rclcpp::shutdown();
    }
    EXPECT_FALSE(observe->invalid);
}
} // namespace
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    const char* profile = std::getenv("RMCS_WHEEL_LEG_PROFILE");
    if (!profile)
        profile = RMCS_PROFILE;
    const char* args[]{"wheel_leg_integration", "--ros-args", "--params-file", profile};
    rclcpp::init(4, args);
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
