#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <numbers>
#include <print>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/imu_diagnostics.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/device/tof.hpp"
#include "hardware/util/feedback_freshness.hpp"
#include "hardware/util/feedback_monitor.hpp"
#include "hardware/util/status_monitor.hpp"

namespace rmcs_core::hardware {

class WheelLeg
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLeg()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_output("/tf", tf_);
        command_component_->register_input("/remote/switch/left", gimbal_switch_left_);
        command_component_->register_input("/remote/switch/right", gimbal_switch_right_);
        command_component_->register_input(
            "/chassis/control_enable", gimbal_control_enable_, false);

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        chassis_board_ = std::make_unique<ChassisBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());
        gimbal_board_ = std::make_unique<GimbalBoard>(
            *this, *command_component_, get_parameter("board_serial_gimbal_board").as_string());
    }

    void update() override {
        chassis_board_->update();
        gimbal_board_->update();
        remote_control_->update();
        chassis_board_->update_feedback(*this);
        gimbal_board_->update_feedback(*this);
    }

private:
    class ChassisBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        ChassisBoard(
            WheelLeg& wheel_leg, rmcs_executor::Component& command, std::string_view board_serial)
            : tf_(wheel_leg.tf_)
            , chassis_left_wheel_motor_(wheel_leg, command, "/wheel_leg/left_wheel")
            , chassis_right_wheel_motor_(wheel_leg, command, "/wheel_leg/right_wheel")
            , chassis_left_hip_motor_(wheel_leg, command, "/wheel_leg/left_hip_joint")
            , chassis_right_hip_motor_(wheel_leg, command, "/wheel_leg/right_hip_joint")
            , chassis_left_knee_motor_(wheel_leg, command, "/wheel_leg/left_knee_joint")
            , chassis_right_knee_motor_(wheel_leg, command, "/wheel_leg/right_knee_joint")
            , gimbal_yaw_motor_(wheel_leg, command, "/gimbal/yaw")
            , gimbal_bullet_feeder_(wheel_leg, command, "/gimbal/bullet_feeder")
            , supercap_(wheel_leg, command)
            , left_tof_(wheel_leg, "/chassis/tof/left")
            , right_tof_(wheel_leg, "/chassis/tof/right")
            // Chassis sensor reads (-x, y, -z): Ry(pi).
            , bmi088_(
                  device::Bmi088Ekf::Config{
                      .body_to_sensor =
                          Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitY()}
                              .toRotationMatrix()}) {
            using namespace device;

            wheel_leg.register_output(
                "/wheel_leg/imu/acceleration", imu_acceleration_, Eigen::Vector3d::Zero());

            // Stock wheel gearboxes removed, 15.62 external reduction.
            chassis_left_wheel_motor_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 1}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(15.62));
            chassis_right_wheel_motor_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 2}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(15.62));

            const auto arming_ms = wheel_leg.get_parameter_or<double>("dm_arming_timeout_ms", 100.);
            const auto feedback_ms =
                wheel_leg.get_parameter_or<double>("motor_feedback_timeout_ms", 20.);
            if (!std::isfinite(arming_ms) || arming_ms < 1. || arming_ms > 1000.
                || !std::isfinite(feedback_ms) || feedback_ms < 1. || feedback_ms > 100.)
                throw std::invalid_argument("Invalid wheel-leg drive timeout");
            const auto arming_timeout = std::chrono::milliseconds{static_cast<long>(arming_ms)};
            motor_feedback_timeout_ns_ = static_cast<std::uint64_t>(feedback_ms * 1.e6);
            chassis_left_hip_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 1, 1}
                    .set_arming_timeout(arming_timeout)
                    .set_reversed()
                    .enable_multi_turn_angle());
            chassis_right_hip_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 2, 2}
                    .set_arming_timeout(arming_timeout)
                    .set_reversed()
                    .enable_multi_turn_angle());
            chassis_left_knee_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 1, 1}
                    .set_arming_timeout(arming_timeout)
                    .set_reversed()
                    .enable_multi_turn_angle());
            chassis_right_knee_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 2, 2}
                    .set_arming_timeout(arming_timeout)
                    .set_reversed()
                    .enable_multi_turn_angle());

            auto zero_point = int{0};
            wheel_leg.get_parameter("yaw_motor_zero_point", zero_point);
            gimbal_yaw_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(zero_point));
            // External 27:1 feeder reducer.
            gimbal_bullet_feeder_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 3}.set_reduction_ratio(27.));
            left_tof_.configure(Tof::Config{}.set_id(5));
            right_tof_.configure(Tof::Config{}.set_id(6));

            wheel_leg.register_output(
                "/wheel_leg/imu/quaternion", imu_quaternion_, Eigen::Quaterniond::Identity());
            wheel_leg.register_output(
                "/wheel_leg/imu/angular_velocity", imu_angular_velocity_, Eigen::Vector3d::Zero());
            const std::array<std::string, 6> joints{"left_hip_joint",  "left_knee_joint",
                                                    "right_hip_joint", "right_knee_joint",
                                                    "left_wheel",      "right_wheel"};
            for (std::size_t i = 0; i < joints.size(); ++i)
                wheel_leg.register_output(
                    "/wheel_leg/" + joints[i] + "/feedback_steady_ns", motor_feedback_ns_[i],
                    std::uint64_t{0});
            wheel_leg.register_output(
                "/wheel_leg/imu/last_steady_ns", imu_feedback_ns_, std::uint64_t{0});
            wheel_leg.register_output("/wheel_leg/control_ready", control_ready_, false);
            wheel_leg.register_output("/wheel_leg/control_ready_blockers", ready_blockers_, 0);
            wheel_leg.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            wheel_leg.remote_control_->register_dr16(&dr16_);
            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial, options);
            board_->start_transmit().uart_config(Spec::kUarts.kUart0, {.baudrate = 115200});
        }

        auto status() const -> std::vector<std::string> {
            auto lines = monitor_.text();
            const auto feedback = gimbal_yaw_motor_.status1();
            lines.push_back(
                std::format(
                    "yaw drive={} status1=0x{:02X} motor_state=0x{:02X} error_state=0x{:02X}",
                    device::LkMotorDriveScheduler::phase_name(gimbal_yaw_motor_.drive_phase()),
                    feedback.command, feedback.motor_state, feedback.error_state));
            return lines;
        }
        int64_t yaw_raw_angle() const { return gimbal_yaw_motor_.last_raw_angle(); }
        void update_feedback(rclcpp::Node& node) { feedback_.update(node); }

        void update() {
            chassis_left_wheel_motor_.update_status();
            chassis_right_wheel_motor_.update_status();
            chassis_left_hip_motor_.update_status();
            chassis_right_hip_motor_.update_status();
            chassis_left_knee_motor_.update_status();
            chassis_right_knee_motor_.update_status();

            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
            gimbal_bullet_feeder_.update_status();
            supercap_.update_status();
            left_tof_.update_status();
            right_tof_.update_status();
            // RemoteControl disables the timeout of a lone DR16; re-enable it before each update
            // so a lost remote still resets the chassis.
            dr16_.set_timeout_enabled(true);
            dr16_.update_status();

            for (std::size_t i = 0; i < motor_feedback_ns_.size(); ++i)
                *motor_feedback_ns_[i] = motor_received_ns_[i].load(std::memory_order_relaxed);
            *imu_feedback_ns_ = imu_received_ns_.load(std::memory_order_relaxed);
            if (const auto snapshot = bmi088_.snapshot()) {
                *imu_quaternion_ = snapshot->orientation;
                *imu_angular_velocity_ = snapshot->gyro_body;
            }
            *imu_acceleration_ = Eigen::Vector3d{
                acceleration_[0].load(std::memory_order_relaxed),
                acceleration_[1].load(std::memory_order_relaxed),
                acceleration_[2].load(std::memory_order_relaxed)};
            update_control_ready();
        }

        void command_update(bool gimbal_requested) {
            using namespace device;

            auto builder = board_->start_transmit();
            builder.can_transmit(
                Spec::kCans.kCan0,
                {.can_id = 0x200,
                 .can_data =
                     CanPacket8{
                         chassis_left_wheel_motor_.generate_command(),
                         chassis_right_wheel_motor_.generate_command(),
                         gimbal_bullet_feeder_.generate_command(), CanPacket8::PaddingQuarter{}}
                         .as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan0,
                {.can_id = 0x1FE,
                 .can_data =
                     CanPacket8{
                         CanPacket8::PaddingQuarter{}, CanPacket8::PaddingQuarter{},
                         CanPacket8::PaddingQuarter{}, supercap_.generate_command()}
                         .as_bytes()});

            builder.can_transmit(
                Spec::kCans.kCan1,
                {.can_id = chassis_left_hip_motor_.send_id(),
                 .can_data = chassis_left_hip_motor_.generate_command().as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan1,
                {.can_id = chassis_right_hip_motor_.send_id(),
                 .can_data = chassis_right_hip_motor_.generate_command().as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan2,
                {.can_id = chassis_left_knee_motor_.send_id(),
                 .can_data = chassis_left_knee_motor_.generate_command().as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan2,
                {.can_id = chassis_right_knee_motor_.send_id(),
                 .can_data = chassis_right_knee_motor_.generate_command().as_bytes()});
            if (auto packet = gimbal_yaw_motor_.generate_drive_command(gimbal_requested))
                builder.can_transmit(
                    Spec::kCans.kCan3, {.can_id = 0x141, .can_data = packet->as_bytes()});
        }

    private:
        void update_control_ready() {
            constexpr std::uint64_t kImuTimeout = 20'000'000;
            const auto now = steady_ns();
            int blockers = 0;
            for (std::size_t i = 0; i < motor_feedback_ns_.size(); ++i)
                if (!feedback_fresh(now, *motor_feedback_ns_[i], motor_feedback_timeout_ns_))
                    blockers |= 1 << i;
            if (!feedback_fresh(now, *imu_feedback_ns_, kImuTimeout))
                blockers |= 1 << 6;
            const std::array enabled{
                chassis_left_hip_motor_.enabled(), chassis_left_knee_motor_.enabled(),
                chassis_right_hip_motor_.enabled(), chassis_right_knee_motor_.enabled()};
            for (std::size_t i = 0; i < enabled.size(); ++i)
                if (!enabled[i])
                    blockers |= 1 << (7 + i);
            *ready_blockers_ = blockers;
            *control_ready_ = blockers == 0;
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            const auto can_id = data.can_id;

            if (can == Spec::kCans.kCan0) {
                if (can_id >= 0x201 && can_id <= 0x203)
                    feedback_.tick(can_id - 0x201);
                if (can_id == 0x201)
                    store_motor_status(chassis_left_wheel_motor_, data.can_data, 4);
                else if (can_id == 0x202)
                    store_motor_status(chassis_right_wheel_motor_, data.can_data, 5);
                else if (can_id == 0x203)
                    gimbal_bullet_feeder_.store_status(data.can_data);
                else if (can_id == 0x300)
                    supercap_.store_status(data.can_data);
                monitor_.tick("Chassis::Can0", can_id);
            } else if (can == Spec::kCans.kCan1) {
                if (can_id == chassis_left_hip_motor_.master_id()) {
                    feedback_.tick(3);
                    store_motor_status(chassis_left_hip_motor_, data.can_data, 0);
                } else if (can_id == chassis_right_hip_motor_.master_id()) {
                    feedback_.tick(4);
                    store_motor_status(chassis_right_hip_motor_, data.can_data, 2);
                } else if (can_id == left_tof_.recv_id()) {
                    left_tof_.store_status(data.can_data);
                } else if (can_id == right_tof_.recv_id()) {
                    right_tof_.store_status(data.can_data);
                }
                monitor_.tick("Chassis::Can1", can_id);
            } else if (can == Spec::kCans.kCan2) {
                if (can_id == chassis_left_knee_motor_.master_id()) {
                    feedback_.tick(5);
                    store_motor_status(chassis_left_knee_motor_, data.can_data, 1);
                } else if (can_id == chassis_right_knee_motor_.master_id()) {
                    feedback_.tick(6);
                    store_motor_status(chassis_right_knee_motor_, data.can_data, 3);
                }
                monitor_.tick("Chassis::Can2", can_id);
            } else if (can == Spec::kCans.kCan3) {
                if (can_id == 0x141) {
                    feedback_.tick(7);
                    gimbal_yaw_motor_.store_status(data.can_data);
                }
                monitor_.tick("Chassis::Can3", can_id);
            }
        }

        void store_motor_status(auto& motor, std::span<const std::byte> data, std::size_t index) {
            if constexpr (std::is_same_v<decltype(motor.store_status(data)), bool>) {
                if (!motor.store_status(data))
                    return;
            } else {
                motor.store_status(data);
            }
            motor_received_ns_[index].store(steady_ns(), std::memory_order_relaxed);
        }

        static std::uint64_t steady_ns() {
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                monitor_.tick("Chassis::Dbus", "Dr16");
            } else if (uart == Spec::kUarts.kUart0) {
                const std::byte* ptr = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&ptr](std::byte* storage) noexcept { *storage = *ptr++; },
                    data.uart_data.size());
                monitor_.tick("Chassis::Uart0", "Referee");
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            // BMI088 +/-6 g range, transformed by the chassis Ry(pi) mounting.
            Eigen::Vector3d acceleration =
                Eigen::Vector3d{
                    static_cast<double>(data.x), static_cast<double>(data.y),
                    static_cast<double>(data.z)}
                * (6.0 * 9.81 / 32767.0);
            acceleration.x() = -acceleration.x();
            acceleration.z() = -acceleration.z();
            for (int i = 0; i < 3; ++i)
                acceleration_[i].store(acceleration[i], std::memory_order_relaxed);
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
            monitor_.tick("Chassis::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            imu_received_ns_.store(steady_ns(), std::memory_order_relaxed);
            monitor_.tick("Chassis::Imu", "Gyr");
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::DjiMotor chassis_left_wheel_motor_, chassis_right_wheel_motor_;
        device::DmMotor chassis_left_hip_motor_, chassis_right_hip_motor_;
        device::DmMotor chassis_left_knee_motor_, chassis_right_knee_motor_;
        OutputInterface<bool> control_ready_;
        OutputInterface<int> ready_blockers_;
        std::uint64_t motor_feedback_timeout_ns_ = 20'000'000;
        std::array<OutputInterface<std::uint64_t>, 6> motor_feedback_ns_;
        std::array<std::atomic<std::uint64_t>, 6> motor_received_ns_{};
        OutputInterface<std::uint64_t> imu_feedback_ns_;
        std::atomic<std::uint64_t> imu_received_ns_{0};

        device::LkMotor gimbal_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;
        device::Supercap supercap_;
        device::Tof left_tof_, right_tof_;
        device::Dr16 dr16_;

        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        OutputInterface<Eigen::Quaterniond> imu_quaternion_;
        OutputInterface<Eigen::Vector3d> imu_angular_velocity_;
        OutputInterface<Eigen::Vector3d> imu_acceleration_;
        std::array<std::atomic<double>, 3> acceleration_{};

        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};

        FeedbackMonitor<8> feedback_{
            {"left wheel M3508 [Chassis CAN0 0x201]", "right wheel M3508 [Chassis CAN0 0x202]",
             "bullet feeder M3508 [Chassis CAN0 0x203]", "left hip DM [Chassis CAN1 ID1]",
             "right hip DM [Chassis CAN1 ID2]", "left knee DM [Chassis CAN2 ID1]",
             "right knee DM [Chassis CAN2 ID2]", "gimbal yaw [Chassis CAN3 0x141]"}};
        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    class GimbalBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        GimbalBoard(
            WheelLeg& wheel_leg, rmcs_executor::Component& command, std::string_view board_serial)
            : tf_(wheel_leg.tf_)
            , gimbal_pitch_motor_(wheel_leg, command, "/gimbal/pitch")
            , gimbal_left_friction_(wheel_leg, command, "/gimbal/left_friction")
            , gimbal_right_friction_(wheel_leg, command, "/gimbal/right_friction")
            // Raw (x, y, z) reads as (y, -x, z): Rz(pi/2).
            , bmi088_(
                  device::Bmi088Ekf::Config{
                      .body_to_sensor =
                          Eigen::AngleAxisd{std::numbers::pi / 2., Eigen::Vector3d::UnitZ()}
                              .toRotationMatrix()}) {
            using namespace device;

            auto zero_point = int{0};
            wheel_leg.get_parameter("pitch_motor_zero_point", zero_point);
            gimbal_pitch_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(zero_point));
            gimbal_left_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 2}.set_reduction_ratio(1.).set_reversed());
            gimbal_right_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 1}.set_reduction_ratio(1.));

            wheel_leg.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_, 0.);
            wheel_leg.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_, 0.);
            wheel_leg.register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot_output_);
            wheel_leg.register_output("/gimbal/auto_aim/exposure_signal", camera_signal_output_);
            const auto diagnostics_path =
                wheel_leg.get_parameter_or<std::string>("imu_diagnostics_csv_path", "");
            if (!diagnostics_path.empty())
                imu_diagnostics_ =
                    std::make_unique<device::ImuDiagnostics>(wheel_leg, diagnostics_path);
            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial, options);

            board_->start_transmit().gpio_digital_read(
                Spec::kGpios.kUart1Rx, {
                                           .period_ms = 0,
                                           .asap = false,
                                           .rising_edge = false,
                                           .falling_edge = true,
                                           .capture_timestamp = true,
                                           .pull = librmcs::data::GpioPull::kUp,
                                       });
        }

        auto status() const -> std::vector<std::string> {
            auto lines = monitor_.text();
            const auto feedback = gimbal_pitch_motor_.status1();
            lines.push_back(
                std::format(
                    "pitch drive={} status1=0x{:02X} motor_state=0x{:02X} error_state=0x{:02X}",
                    device::LkMotorDriveScheduler::phase_name(gimbal_pitch_motor_.drive_phase()),
                    feedback.command, feedback.motor_state, feedback.error_state));
            return lines;
        }
        int64_t pitch_raw_angle() const { return gimbal_pitch_motor_.last_raw_angle(); }
        void update_feedback(rclcpp::Node& node) { feedback_.update(node); }

        void update() {
            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());
            if (const auto snapshot = bmi088_.snapshot()) {
                *gimbal_yaw_velocity_imu_ = snapshot->gyro_body.z();
                *gimbal_pitch_velocity_imu_ = snapshot->gyro_body.y();
                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    snapshot->orientation.conjugate());
            }
            if (imu_diagnostics_)
                imu_diagnostics_->update();
        }

        void command_update(bool gimbal_requested) {
            using namespace device;

            auto builder = board_->start_transmit();
            if (auto packet = gimbal_pitch_motor_.generate_drive_command(gimbal_requested))
                builder.can_transmit(
                    Spec::kCans.kCan0, {.can_id = 0x142, .can_data = packet->as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan1,
                {.can_id = 0x200,
                 .can_data =
                     CanPacket8{
                         gimbal_right_friction_.generate_command(),
                         gimbal_left_friction_.generate_command(), CanPacket8::PaddingQuarter{},
                         CanPacket8::PaddingQuarter{}}
                         .as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            const auto can_id = data.can_id;
            if (can == Spec::kCans.kCan0) {
                if (can_id == 0x142) {
                    feedback_.tick(0);
                    gimbal_pitch_motor_.store_status(data.can_data);
                }
                monitor_.tick("Gimbal::Can0", can_id);
            } else if (can == Spec::kCans.kCan1) {
                if (can_id == 0x201) {
                    feedback_.tick(1);
                    gimbal_right_friction_.store_status(data.can_data);
                } else if (can_id == 0x202) {
                    feedback_.tick(2);
                    gimbal_left_friction_.store_status(data.can_data);
                }
                monitor_.tick("Gimbal::Can1", can_id);
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
            if (imu_diagnostics_)
                imu_diagnostics_->record(
                    false, data.x, data.y, data.z, data.timestamp_quarter_us, timestamp,
                    bmi088_.snapshot());
            monitor_.tick("Gimbal::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            monitor_.tick("Gimbal::Imu", "Gyr");
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value()) {
                if (imu_diagnostics_)
                    imu_diagnostics_->record(
                        true, data.x, data.y, data.z, data.timestamp_quarter_us, timestamp,
                        std::nullopt);
                return;
            }
            device::Bmi088Ekf::Diagnostics diagnostics;
            const auto snapshot = bmi088_.try_update_with_gyroscope_sample(
                data.x, data.y, data.z, *timestamp, imu_diagnostics_ ? &diagnostics : nullptr);
            if (snapshot)
                imu_snapshot_output_.emit(*snapshot);
            if (imu_diagnostics_)
                imu_diagnostics_->record(
                    true, data.x, data.y, data.z, data.timestamp_quarter_us, timestamp, snapshot,
                    diagnostics);
        }

        void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) override {
            if (gpio != Spec::kGpios.kUart1Rx)
                return;
            if (!data.timestamp_quarter_us)
                return;

            const auto timestamp = board_clock_lifter_.lift_timestamp(*data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;

            camera_signal_output_.emit(*timestamp);
            monitor_.tick("Gimbal::CameraSync", "Active");
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_, gimbal_right_friction_;

        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        OutputInterface<double> gimbal_yaw_velocity_imu_, gimbal_pitch_velocity_imu_;
        EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_output_;
        EventOutputInterface<rmcs_msgs::BoardClock::time_point> camera_signal_output_;

        std::unique_ptr<device::ImuDiagnostics> imu_diagnostics_;
        FeedbackMonitor<3> feedback_{
            {"gimbal pitch [Gimbal CAN0 0x142]", "right friction M3508 [Gimbal CAN1 0x201]",
             "left friction M3508 [Gimbal CAN1 0x202]"}};
        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    struct CommandTransmitter : public rmcs_executor::Component {
        std::function<void()> fn;

        template <typename Fn>
        explicit CommandTransmitter(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        void update() override { fn(); }
    };

    void
        status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };
        text("    yaw_motor_zero_point: {}", chassis_board_->yaw_raw_angle());
        text("    pitch_motor_zero_point: {}", gimbal_board_->pitch_raw_angle());
        text("\nChassisBoard Status:");
        for (const auto& line : chassis_board_->status())
            text("> {}", line);
        text("\nGimbalBoard Status:");
        for (const auto& line : gimbal_board_->status())
            text("> {}", line);
        response->message = feedback_message.str();
    }

    void command_update() {
        using rmcs_msgs::Switch;
        const auto left = *gimbal_switch_left_;
        const auto right = *gimbal_switch_right_;
        // Gimbal-only configurations have no chassis enable source. Neither axis waits for
        // leg feedback or a finite torque command before sending its clear/run sequence.
        const bool gimbal_requested = left != Switch::UNKNOWN && right != Switch::UNKNOWN
                                   && !(left == Switch::DOWN && right == Switch::DOWN)
                                   && (!gimbal_control_enable_.ready() || *gimbal_control_enable_);
        chassis_board_->command_update(gimbal_requested);
        gimbal_board_->command_update(gimbal_requested);
    }

    std::shared_ptr<rmcs_executor::Component> command_component_{
        create_partner_component<CommandTransmitter>(
            get_component_name() + "_command", [this] { command_update(); })};

    OutputInterface<rmcs_description::Tf> tf_;
    InputInterface<rmcs_msgs::Switch> gimbal_switch_left_, gimbal_switch_right_;
    InputInterface<bool> gimbal_control_enable_;

    std::unique_ptr<ChassisBoard> chassis_board_;
    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLeg, rmcs_executor::Component)
