// Offline executor test: production plugins, memory sensors, no board or CAN.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <thread>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/chassis/wheel_leg/self_righting.hpp"
#include "executor.hpp"
#include "hardware/wheel_leg_imu.hpp"

namespace {
using namespace std::chrono_literals;
using Component = rmcs_executor::Component;
using Switch = rmcs_msgs::Switch;
using Vec3 = Eigen::Vector3d;
using namespace rmcs_core::controller::chassis::wheel_leg;
constexpr std::array kAxes{"left_hip_joint",   "left_knee_joint", "right_hip_joint",
                           "right_knee_joint", "left_wheel",      "right_wheel"};

struct Monitor {
    std::atomic<bool> enable{false}, active{false}, ready{false}, servo{false};
    std::atomic<bool> unsafe{false};
    std::atomic<int> fault{0}, route{0};
    std::atomic<double> failure{0.}, length{0.}, command_v{0.}, command_yaw{0.};
    std::atomic<std::uint64_t> frames{0};
    std::array<std::atomic<double>, 6> effort{};
};

class Sensors final : public Component {
public:
    explicit Sensors(std::atomic<int>& phase, const Eigen::Quaterniond& orientation)
        : phase_(phase) {
        const double knee = recovery_v202::stand_knee;
        const auto& table = recovery_v202::geometry;
        const auto upper =
            std::lower_bound(table.begin(), table.end(), knee, [](const auto& row, double value) {
                return row[0] < value;
            });
        const auto& a = *(upper - 1);
        const auto& b = *upper;
        const double weight = (knee - a[0]) / (b[0] - a[0]);
        const double orbit = recovery_v202::stand_orbit * std::numbers::pi / 180.;
        const std::array zero{1.6, 3.0282761689183353, -1.6, -3.0283102010168};
        for (std::size_t i = 0; i < 6; ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + kAxes[i];
            const double cad = i < 4 ? std::lerp(a[i + 1], b[i + 1], weight)
                                           + (i < 2 ? -orbit + 2. * std::numbers::pi
                                                    : orbit - 4. * std::numbers::pi)
                                     : 0.;
            register_output(prefix + "/angle", q_[i], i < 4 ? zero[i] - cad : 0.);
            register_output(prefix + "/velocity", dq_[i], 0.);
            register_output(prefix + "/max_torque", limit_[i], i < 4 ? 40. : 4.5);
            register_output(prefix + "/feedback_steady_ns", stamp_[i], std::uint64_t{0});
        }
        register_output("/wheel_leg/imu/quaternion", orientation_, orientation);
        register_output("/wheel_leg/imu/angular_velocity", omega_, Vec3::Zero().eval());
        register_output("/wheel_leg/imu/acceleration", acceleration_, Vec3{0., 0., 9.80665});
        register_output("/wheel_leg/imu/last_steady_ns", imu_time_, std::uint64_t{0});
        register_output(
            "/wheel_leg/imu/acceleration_steady_ns", acceleration_time_, std::uint64_t{0});
        register_output("/wheel_leg/feedback_fresh", fresh_, true);
        register_output("/wheel_leg/dm_control_ready", dm_ready_, false);
        register_output("/wheel_leg/dr16_fresh", remote_fresh_, true);
        register_output("/remote/switch/left", left_, Switch::DOWN);
        register_output("/remote/switch/right", right_, Switch::DOWN);
        register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d{1., 1.});
        register_output("/remote/joystick/right", joystick_right_, Eigen::Vector2d{1., 1.});
        register_output("/remote/rotary_knob", knob_, 1.);
        for (std::size_t i = 0; i < 3; ++i) {
            register_output(
                std::string{"/wheel_leg/imu/sensor_gravity/"} + "xyz"[i], gravity_[i],
                i == 2 ? -1. : 0.);
            register_output(std::string{"/wheel_leg/imu/sensor_gyro/"} + "xyz"[i], gyro_[i], 0.);
        }
    }

    void update() override {
        const int phase = phase_.load();
        *left_ = phase == 0 || phase == 5 ? Switch::DOWN : Switch::MIDDLE;
        *right_ = phase == 0 || phase == 5 ? Switch::DOWN
                : phase == 1               ? Switch::MIDDLE
                                           : Switch::UP;
        *remote_fresh_ = phase != 6;
        *dm_ready_ = phase >= 3 && phase != 5;
        *fresh_ = true;
        const auto now =
            static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::steady_clock::now().time_since_epoch())
                                           .count());
        for (std::size_t i = 0; i < 6; ++i) {
            if (phase != 4 || i != 0)
                *stamp_[i] = now;
            *dq_[i] = phase == 7 && i == 0 ? 4. : 0.;
        }
        *imu_time_ = *acceleration_time_ = now;
    }

private:
    std::atomic<int>& phase_;
    std::array<OutputInterface<double>, 6> q_, dq_, limit_;
    std::array<OutputInterface<std::uint64_t>, 6> stamp_;
    std::array<OutputInterface<double>, 3> gravity_, gyro_;
    OutputInterface<Eigen::Quaterniond> orientation_;
    OutputInterface<Vec3> omega_, acceleration_;
    OutputInterface<std::uint64_t> imu_time_, acceleration_time_;
    OutputInterface<bool> fresh_, dm_ready_, remote_fresh_;
    OutputInterface<Switch> left_, right_;
    OutputInterface<Eigen::Vector2d> joystick_left_, joystick_right_;
    OutputInterface<double> knob_;
};

class Observe final : public Component {
public:
    explicit Observe(Monitor& monitor)
        : monitor_(monitor) {
        register_input("/wheel_leg/enable_request", enable_);
        register_input("/wheelleg/recovery/active", active_);
        register_input("/wheelleg/recovery/ready", ready_);
        register_input("/wheelleg/recovery/fault", fault_);
        register_input("/wheelleg/traditional/failure", failure_);
        register_input("/wheelleg/reference/leg_length", length_);
        register_input("/wheelleg/traditional/command_velocity", command_v_);
        register_input("/wheelleg/traditional/command_yaw_rate", command_yaw_);
        for (std::size_t i = 0; i < 6; ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + kAxes[i];
            register_input(prefix + "/control_torque", effort_[i]);
        }
    }
    void update() override {
        monitor_.enable = *enable_;
        monitor_.active = *active_;
        monitor_.ready = *ready_;
        monitor_.fault = *fault_;
        monitor_.route = 0;
        monitor_.failure = *failure_;
        monitor_.length = *length_;
        monitor_.command_v = *command_v_;
        monitor_.command_yaw = *command_yaw_;
        monitor_.servo = false;
        for (std::size_t i = 0; i < 6; ++i) {
            monitor_.effort[i] = *effort_[i];
            if (!std::isfinite(*effort_[i]) || (i >= 4 && std::abs(*effort_[i]) > 4.5 + 1e-9))
                monitor_.unsafe = true;
            if (i < 4 && std::abs(*effort_[i]) > 40. + 1e-9)
                monitor_.unsafe = true;
        }
        ++monitor_.frames;
    }

private:
    Monitor& monitor_;
    InputInterface<bool> enable_, active_, ready_;
    InputInterface<std::uint8_t> fault_, route_;
    InputInterface<double> failure_, length_, command_v_, command_yaw_;
    std::array<InputInterface<double>, 6> effort_;
};

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
void await(const std::function<bool()>& condition, const char* message) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition())
            return;
        std::this_thread::sleep_for(2ms);
    }
    throw std::runtime_error(message);
}
bool zero(const Monitor& monitor) {
    for (const auto& value : monitor.effort)
        if (value.load() != 0.)
            return false;
    return true;
}
struct StopRos {
    ~StopRos() { rclcpp::shutdown(); }
};
} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::Node hardware{
            "wheel_leg",
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)};
        rmcs_core::hardware::device::Bmi088Ekf imu{
            rmcs_core::hardware::device::Bmi088Ekf::Config{
                .body_to_sensor = Eigen::AngleAxisd{std::numbers::pi, Vec3::UnitY()}.toRotationMatrix()}};
        require(
            imu.push_accelerometer_sample(0, 0, -5461, {}),
            "Raw mounted IMU failed initialization");
        require(imu.snapshot().has_value(), "Mapped attitude unavailable");
        require(
            (imu.snapshot()->orientation.conjugate() * -Vec3::UnitZ())
                .isApprox(-Vec3::UnitZ(), 1e-9),
            "Raw -Z did not become upright base_link");
        rclcpp::executors::SingleThreadedExecutor ros;
        pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
        std::atomic<int> phase{0};
        Monitor monitor;
        Component::initializing_component_name = "mock_hardware";
        auto sensors = std::make_shared<Sensors>(phase, imu.snapshot()->orientation);
        Component::initializing_component_name = "wheelleg_self_righting";
        auto recovery = loader.createSharedInstance(
            "rmcs_core::controller::wheelleg::WheelLegSelfRightingController");
        Component::initializing_component_name = "wheelleg_traditional";
        auto traditional = loader.createSharedInstance(
            "rmcs_core::controller::wheelleg::WheelLegTraditionalController");
        Component::initializing_component_name = "value_broadcaster";
        auto broadcaster = loader.createSharedInstance("rmcs_core::broadcaster::ValueBroadcaster");
        Component::initializing_component_name = "observe";
        auto observe = std::make_shared<Observe>(monitor);
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        StopRos stop;
        executor.add_component(sensors);
        executor.add_component(recovery);
        executor.add_component(traditional);
        executor.add_component(broadcaster);
        executor.add_component(observe);
        executor.start();
        std::this_thread::sleep_for(1100ms);
        require(!monitor.enable && zero(monitor), "Double DOWN did not keep all outputs zero");
        phase = 1;
        await(
            [&] { return monitor.active && monitor.enable; },
            "Recovery/enable handshake never acquired ownership");
        std::this_thread::sleep_for(30ms);
        require(zero(monitor), "Both MIDDLE moved before wheel permission");
        phase = 2;
        std::this_thread::sleep_for(30ms);
        require(zero(monitor) && !monitor.ready, "Recovery moved before DM ready");
        phase = 3;
        std::this_thread::sleep_for(30ms);
        require(!monitor.servo, "Recovery unexpectedly enabled the removed velocity servo");
        require(monitor.active && !monitor.servo,
                "Native recovery did not retain pure-torque ownership");
        require(
            monitor.command_v == 0. && monitor.command_yaw == 0.,
            "Standing profile accepted joystick motion");
        require(!monitor.unsafe, "Nonfinite or unbounded output escaped the sole actuator host");
        phase = 4;
        await(
            [&] { return monitor.failure == 12. && !monitor.enable && zero(monitor); },
            "Stale axis feedback did not latch a stop");
        phase = 3;
        std::this_thread::sleep_for(30ms);
        require(
            !monitor.enable && zero(monitor), "Restored feedback silently rearmed a latched fault");
        phase = 5;
        std::this_thread::sleep_for(1100ms);
        require(
            !monitor.enable && monitor.failure == 0. && monitor.fault == 0 && zero(monitor),
            "Double DOWN did not reset both controllers");
        phase = 1;
        await(
            [&] { return monitor.enable && monitor.active; },
            "Fresh reset did not permit a new attempt");
        phase = 6;
        await([&] { return !monitor.enable && zero(monitor); }, "Remote loss retained torque");
        require(!monitor.unsafe, "An unsafe output was observed");
        std::cout << "PASS: mounted IMU, plugin/YAML pairing, DM readiness, pure-torque recovery, "
                      "zero commands, stale/reset/remote interlocks; "
                     "no hardware opened.\n";
    } catch (const std::exception& error) {
        std::cerr << "Traditional commissioning integration failed: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}
