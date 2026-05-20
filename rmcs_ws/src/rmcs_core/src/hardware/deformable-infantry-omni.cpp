#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <string>
#include <tuple>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <librmcs/agent/rmcs_board_lite.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>

namespace rmcs_core::hardware {

using Clock = std::chrono::steady_clock;

class DeformableInfantryOmni
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryOmni()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_input("/predefined/timestamp", timestamp_);
        register_output("/tf", tf_);

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_, get_parameter("serial_filter_rmcs_board").as_string());
        top_board_ = std::make_unique<TopBoard>(
            *this, *command_, get_parameter("serial_filter_top_board").as_string(), true);
        imu_board_ =
            std::make_unique<ImuBoard>(*this, get_parameter("serial_filter_imu").as_string());

        // For command: remote-status
        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });
    }

    ~DeformableInfantryOmni() override = default;

    void before_updating() override { top_board_->request_hard_sync_read(); }

    void update() override {
        bottom_board_->update();
        top_board_->update();
        imu_board_->update();
    }

    void command_update() {
        const bool even = ((cmd_tick_++ & 1u) == 0u);
        bottom_board_->command_update(even);
        top_board_->command_update();
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    class Command : public rmcs_executor::Component {
    public:
        explicit Command(DeformableInfantryOmni& deformableInfantry)
            : deformableInfantry(deformableInfantry) {}

        void update() override { deformableInfantry.command_update(); }

        DeformableInfantryOmni& deformableInfantry;
    };

    class BottomBoard final : private librmcs::agent::RmcsBoardLite {
        friend class DeformableInfantryOmni;

    public:
        explicit BottomBoard(
            DeformableInfantryOmni& status, rmcs_executor::Component& command,
            const std::string& serial_filter = {})
            : RmcsBoardLite{
                  serial_filter,
                  librmcs::agent::AdvancedOptions{.dangerously_skip_version_checks = true}}
            , status_{status}
            , command_{command} {

            status.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart0_transmit(
                    {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(status.get_parameter("yaw_motor_zero_point").as_int())));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                        .set_reduction_ratio(13.0)
                        .enable_multi_turn_angle()
                        .set_reversed());

            for (auto& motor : chassis_joint_motors_)
                motor.configure(
                    device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei36}
                        .set_reversed()
                        .enable_multi_turn_angle());

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Keep the existing chassis yaw axis mapping explicit until the bottom-board IMU
                // installation is re-validated on hardware.
                return std::make_tuple(-y, x, z);
            });

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}.enable_multi_turn_angle());

            status.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            status.register_output("/chassis/imu/pitch", chassis_imu_pitch_, 0.0);
            status.register_output("/chassis/imu/roll", chassis_imu_roll_, 0.0);
            status.register_output("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, 0.0);
            status.register_output("/chassis/imu/roll_rate", chassis_imu_roll_rate_, 0.0);
            status.register_output(
                "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, kNaN);
            status.register_output(
                "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, kNaN);
            status.register_output(
                "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, kNaN);
            status.register_output(
                "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_,
                kNaN);
            status.register_output(
                "/chassis/left_front_joint/physical_velocity", left_front_joint_physical_velocity_,
                kNaN);
            status.register_output(
                "/chassis/left_back_joint/physical_velocity", left_back_joint_physical_velocity_,
                kNaN);
            status.register_output(
                "/chassis/right_back_joint/physical_velocity", right_back_joint_physical_velocity_,
                kNaN);
            status.register_output(
                "/chassis/right_front_joint/physical_velocity",
                right_front_joint_physical_velocity_, kNaN);
            status.register_output("/chassis/encoder/alpha", encoder_alpha_, kNaN);
            status.register_output("/chassis/encoder/alpha_dot", encoder_alpha_dot_, kNaN);
            status.register_output("/chassis/radius", radius_, default_radius_);

            status.get_parameter_or("debug_log_supercap", debug_log_supercap_, false);
            status.get_parameter_or("debug_log_wheel_motor", debug_log_wheel_motor_, false);
            status.get_parameter_or(
                "debug_log_deformable_joint_motor", debug_log_deformable_joint_motor_, false);
        }

        ~BottomBoard() override = default;

        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gz();
            {
                const double q0 = imu_.q0();
                const double q1 = imu_.q1();
                const double q2 = imu_.q2();
                const double q3 = imu_.q3();

                double sin_pitch = 2.0 * (q0 * q2 - q3 * q1);
                sin_pitch = std::clamp(sin_pitch, -1.0, 1.0);

                const double standard_pitch = std::asin(sin_pitch);
                const double standard_roll =
                    std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

                // Export chassis attitude using the requested convention:
                // pitch < 0 when the front is higher, roll > 0 when the left side is higher.
                *chassis_imu_pitch_ = -standard_pitch;
                *chassis_imu_roll_ = standard_roll;
                *chassis_imu_pitch_rate_ = -imu_.gy();
                *chassis_imu_roll_rate_ = imu_.gx();
            }

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_joint_motors_)
                motor.update_status();

            update_joint_physical_feedback_(
                0, left_front_joint_physical_angle_, left_front_joint_physical_velocity_);
            update_joint_physical_feedback_(
                1, left_back_joint_physical_angle_, left_back_joint_physical_velocity_);
            update_joint_physical_feedback_(
                2, right_back_joint_physical_angle_, right_back_joint_physical_velocity_);
            update_joint_physical_feedback_(
                3, right_front_joint_physical_angle_, right_front_joint_physical_velocity_);

            update_geometry_feedback_();
            if (debug_log_wheel_motor_ || debug_log_deformable_joint_motor_)
                log_chassis_feedback_once_per_second_();

            dr16_.update_status();
            gimbal_yaw_motor_.update_status();
            if (supercap_status_received_.load(std::memory_order_relaxed))
                supercap_.update_status();
            if (debug_log_supercap_)
                log_supercap_feedback_once_per_second_();
            gimbal_bullet_feeder_.update_status();

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
        }

        void command_update(bool even) {
            auto builder = start_transmit();
            if (even) {
                builder.can0_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            chassis_wheel_motors_[0].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            chassis_wheel_motors_[1].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            chassis_wheel_motors_[2].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            gimbal_bullet_feeder_.generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
                builder.can3_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            chassis_wheel_motors_[3].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x142,
                    .can_data = gimbal_yaw_motor_.generate_torque_command().as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            supercap_.generate_command(),
                        }
                            .as_bytes(),
                });
            } else {
                builder.can0_transmit({
                    .can_id = 0x141,
                    .can_data = chassis_joint_motors_[0].generate_command().as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x141,
                    .can_data = chassis_joint_motors_[1].generate_command().as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x141,
                    .can_data = chassis_joint_motors_[2].generate_command().as_bytes(),
                });
                builder.can3_transmit({
                    .can_id = 0x141,
                    .can_data = chassis_joint_motors_[3].generate_command().as_bytes(),
                });
            }
        }

    private:
        static constexpr double joint_zero_physical_angle_rad_ = 62.5 * std::numbers::pi / 180.0;
        static constexpr double chassis_radius_base_ = 0.2341741;
        static constexpr double rod_length_ = 0.150;
        static constexpr double default_radius_ = chassis_radius_base_ + rod_length_;

        DeformableInfantryOmni& status_;
        Component& command_;

        device::Bmi088 imu_{1000, 0.2, 0.0};
        device::LkMotor gimbal_yaw_motor_{status_, command_, "/gimbal/yaw"};
        device::Dr16 dr16_{status_};

        device::DjiMotor chassis_wheel_motors_[4]{
            device::DjiMotor{status_, command_, "/chassis/left_front_wheel"},
            device::DjiMotor{status_, command_, "/chassis/left_back_wheel"},
            device::DjiMotor{status_, command_, "/chassis/right_back_wheel"},
            device::DjiMotor{status_, command_, "/chassis/right_front_wheel"},
        };
        device::LkMotor chassis_joint_motors_[4]{
            device::LkMotor{status_, command_, "/chassis/left_front_joint"},
            device::LkMotor{status_, command_, "/chassis/left_back_joint"},
            device::LkMotor{status_, command_, "/chassis/right_back_joint"},
            device::LkMotor{status_, command_, "/chassis/right_front_joint"},
        };

        std::atomic<bool> wheel_status_received_[4] = {false, false, false, false};
        std::atomic<bool> joint_status_received_[4] = {false, false, false, false};
        bool debug_log_supercap_ = false;
        bool debug_log_wheel_motor_ = false;
        bool debug_log_deformable_joint_motor_ = false;
        Clock::time_point next_chassis_feedback_log_time_{Clock::now() + std::chrono::seconds(1)};
        Clock::time_point next_supercap_feedback_log_time_{Clock::now() + std::chrono::seconds(1)};
        device::Supercap supercap_{status_, command_};
        std::atomic<device::CanPacket8> latest_supercap_status_{device::CanPacket8{uint64_t{0}}};
        std::atomic<bool> supercap_status_received_{false};
        device::DjiMotor gimbal_bullet_feeder_{status_, command_, "/gimbal/bullet_feeder"};

        OutputInterface<rmcs_description::Tf>& tf_{status_.tf_};

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_imu_pitch_;
        OutputInterface<double> chassis_imu_roll_;
        OutputInterface<double> chassis_imu_pitch_rate_;
        OutputInterface<double> chassis_imu_roll_rate_;
        OutputInterface<double> left_front_joint_physical_angle_;
        OutputInterface<double> left_back_joint_physical_angle_;
        OutputInterface<double> right_back_joint_physical_angle_;
        OutputInterface<double> right_front_joint_physical_angle_;
        OutputInterface<double> left_front_joint_physical_velocity_;
        OutputInterface<double> left_back_joint_physical_velocity_;
        OutputInterface<double> right_back_joint_physical_velocity_;
        OutputInterface<double> right_front_joint_physical_velocity_;
        OutputInterface<double> encoder_alpha_;
        OutputInterface<double> encoder_alpha_dot_;
        OutputInterface<double> radius_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        void update_joint_physical_feedback_(
            size_t index, OutputInterface<double>& angle_output,
            OutputInterface<double>& velocity_output) {

            if (!joint_status_received_[index].load(std::memory_order_relaxed)) {
                *angle_output = kNaN;
                *velocity_output = kNaN;
                return;
            }

            const auto to_physical_angle = [](double motor_angle) {
                return joint_zero_physical_angle_rad_ - motor_angle;
            };
            const auto to_physical_velocity = [](double motor_velocity) { return -motor_velocity; };

            *angle_output = to_physical_angle(chassis_joint_motors_[index].angle());
            *velocity_output = to_physical_velocity(chassis_joint_motors_[index].velocity());
        }

        void update_geometry_feedback_() {
            const Eigen::Vector4d alpha_rad{
                *left_front_joint_physical_angle_, *left_back_joint_physical_angle_,
                *right_back_joint_physical_angle_, *right_front_joint_physical_angle_};
            const Eigen::Vector4d alpha_dot_rad{
                *left_front_joint_physical_velocity_, *left_back_joint_physical_velocity_,
                *right_back_joint_physical_velocity_, *right_front_joint_physical_velocity_};

            if (!alpha_rad.array().isFinite().all() || !alpha_dot_rad.array().isFinite().all()) {
                *encoder_alpha_ = kNaN;
                *encoder_alpha_dot_ = kNaN;
                *radius_ = default_radius_;
                RCLCPP_WARN_THROTTLE(
                    status_.get_logger(), *status_.get_clock(), 1000,
                    "deformable joint feedback invalid, fallback chassis radius to default %.3f m",
                    default_radius_);
                return;
            }

            *encoder_alpha_ = alpha_rad.mean();
            *encoder_alpha_dot_ = alpha_dot_rad.mean();
            *radius_ = (chassis_radius_base_ + rod_length_ * alpha_rad.array().cos()).mean();
        }

        void log_chassis_feedback_once_per_second_() {
            const auto now = Clock::now();
            if (now < next_chassis_feedback_log_time_)
                return;

            const auto wheel_rx = [this](size_t index) {
                return wheel_status_received_[index].load(std::memory_order_relaxed) ? 'Y' : 'N';
            };
            const auto joint_rx = [this](size_t index) {
                return joint_status_received_[index].load(std::memory_order_relaxed) ? 'Y' : 'N';
            };

            if (debug_log_wheel_motor_) {
                RCLCPP_INFO(
                    status_.get_logger(),
                    "[wheel motor] angle(rad) lf=% .3f lb=% .3f rb=% .3f rf=% .3f | "
                    "encoder(deg) lf=% .1f lb=% .1f rb=% .1f rf=% .1f | "
                    "rx=[%c %c %c %c]",
                    chassis_wheel_motors_[0].angle(), chassis_wheel_motors_[1].angle(),
                    chassis_wheel_motors_[2].angle(), chassis_wheel_motors_[3].angle(),
                    chassis_wheel_motors_[0].angle(), chassis_wheel_motors_[1].angle(),
                    chassis_wheel_motors_[2].angle(), chassis_wheel_motors_[3].angle(), wheel_rx(0),
                    wheel_rx(1), wheel_rx(2), wheel_rx(3));
            }

            if (debug_log_deformable_joint_motor_) {
                RCLCPP_INFO(
                    status_.get_logger(),
                    "[deformable joint motor] angle(rad) lf=% .3f lb=% .3f rb=% .3f rf=% .3f | "
                    "velocity(rad/s) lf=% .3f lb=% .3f rb=% .3f rf=% .3f | "
                    "rx=[%c %c %c %c]",
                    *left_front_joint_physical_angle_, *left_back_joint_physical_angle_,
                    *right_back_joint_physical_angle_, *right_front_joint_physical_angle_,
                    *left_front_joint_physical_velocity_, *left_back_joint_physical_velocity_,
                    *right_back_joint_physical_velocity_, *right_front_joint_physical_velocity_,
                    joint_rx(0), joint_rx(1), joint_rx(2), joint_rx(3));
            }

            next_chassis_feedback_log_time_ = now + std::chrono::seconds(1);
        }

        void log_supercap_feedback_once_per_second_() {
            const auto now = Clock::now();
            if (now < next_supercap_feedback_log_time_)
                return;

            const bool supercap_rx = supercap_status_received_.load(std::memory_order_relaxed);
            auto supercap_raw_packet = latest_supercap_status_.load(std::memory_order_relaxed);
            const auto supercap_raw_bytes = supercap_raw_packet.as_bytes();

            RCLCPP_INFO(
                status_.get_logger(),
                "[supercap] can1 rx=%c id=0x300 enabled=%d supercap_v=% .3f chassis_v=% .3f "
                "power=% .3f raw=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                supercap_rx ? 'Y' : 'N',
                supercap_rx ? (supercap_.supercap_enabled() ? 1 : 0) : -1,
                supercap_rx ? supercap_.supercap_voltage() : kNaN,
                supercap_rx ? supercap_.chassis_voltage() : kNaN,
                supercap_rx ? supercap_.chassis_power() : kNaN,
                std::to_integer<unsigned int>(supercap_raw_bytes[0]),
                std::to_integer<unsigned int>(supercap_raw_bytes[1]),
                std::to_integer<unsigned int>(supercap_raw_bytes[2]),
                std::to_integer<unsigned int>(supercap_raw_bytes[3]),
                std::to_integer<unsigned int>(supercap_raw_bytes[4]),
                std::to_integer<unsigned int>(supercap_raw_bytes[5]),
                std::to_integer<unsigned int>(supercap_raw_bytes[6]),
                std::to_integer<unsigned int>(supercap_raw_bytes[7]));

            next_supercap_feedback_log_time_ = now + std::chrono::seconds(1);
        }

        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(data.can_data);
                wheel_status_received_[0].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x141) {
                chassis_joint_motors_[0].store_status(data.can_data);
                joint_status_received_[0].store(true, std::memory_order_relaxed);
            }
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201) {
                chassis_wheel_motors_[1].store_status(data.can_data);
                wheel_status_received_[1].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x141) {
                chassis_joint_motors_[1].store_status(data.can_data);
                joint_status_received_[1].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x300) {
                if (data.can_data.size() == 8)
                    latest_supercap_status_.store(
                        device::CanPacket8{data.can_data}, std::memory_order_relaxed);
                supercap_.store_status(data.can_data);
                supercap_status_received_.store(true, std::memory_order_relaxed);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201) {
                chassis_wheel_motors_[2].store_status(data.can_data);
                wheel_status_received_[2].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x141) {
                chassis_joint_motors_[2].store_status(data.can_data);
                joint_status_received_[2].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x142) {
                gimbal_yaw_motor_.store_status(data.can_data);
            } else if (data.can_id == 0x203) {
                gimbal_bullet_feeder_.store_status(data.can_data);
            }
        }

        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201) {
                chassis_wheel_motors_[3].store_status(data.can_data);
                wheel_status_received_[3].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x141) {
                chassis_joint_motors_[3].store_status(data.can_data);
                joint_status_received_[3].store(true, std::memory_order_relaxed);
            }
        }

        void uart0_receive_callback(const librmcs::data::UartDataView& data) override {
            const std::byte* ptr = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_n(
                [&ptr](std::byte* storage) noexcept { *storage = *ptr++; }, data.uart_data.size());
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }
    };

    class ImuBoard final : private librmcs::agent::RmcsBoardLite {
        friend class DeformableInfantryOmni;

    public:
        explicit ImuBoard(DeformableInfantryOmni& status, const std::string& serial_filter = {})
            : RmcsBoardLite{
                  serial_filter,
                  librmcs::agent::AdvancedOptions{.dangerously_skip_version_checks = true}}
            , tf_{status.tf_}
            , bmi088_{1000, 0.2, 0.0} {

            status.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(x, z, -y); });
        }

        ~ImuBoard() override = default;

        void update() {
            bmi088_.update_status();
            Eigen::Quaterniond const gimbal_imu_pose{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            *gimbal_pitch_velocity_imu_ = bmi088_.gy();
        }

    private:
        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        device::Bmi088 bmi088_;
    };

    class TopBoard final : private librmcs::agent::RmcsBoardLite {
        friend class DeformableInfantryOmni;

    public:
        explicit TopBoard(
            DeformableInfantryOmni& status, Command& command, std::string serial_filter = {},
            bool has_external_imu_board = false)
            : librmcs::agent::RmcsBoardLite(
                  serial_filter,
                  librmcs::agent::AdvancedOptions{.dangerously_skip_version_checks = true})
            , has_external_imu_board_(has_external_imu_board)
            , tf_(status.tf_)
            , bmi088_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(status, command, "/gimbal/pitch")
            , gimbal_left_friction_(status, command, "/gimbal/left_friction")
            , gimbal_right_friction_(status, command, "/gimbal/right_friction")
            , scope_motor_(status, command, "/gimbal/scope") {

            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(status.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));

            scope_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}.enable_multi_turn_angle());

            status.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            if (!has_external_imu_board_)
                status.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_encoder_);

            bmi088_.set_coordinate_mapping([](double x, double y, double z) {
                // Top board BMI088 maps to gimbal frame as (-x, -y, z).
                return std::make_tuple(y, -x, z);
            });
        }

        ~TopBoard() override = default;

        void request_hard_sync_read() {
            // RMCS-lite top board variant currently has no GPIO hard-sync request
            // path.
        }

        void update() {
            bmi088_.update_status();

            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            scope_motor_.update_status();

            const double pitch_encoder_angle = gimbal_pitch_motor_.angle();

            *gimbal_yaw_velocity_bmi088_ = bmi088_.gz();
            if (!has_external_imu_board_) {
                Eigen::Quaterniond const odom_imu_to_yaw_link{
                    bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
                Eigen::Quaterniond const yaw_link_to_odom_imu = odom_imu_to_yaw_link.conjugate();
                Eigen::Quaterniond pitch_link_to_odom_imu =
                    Eigen::Quaterniond{
                        Eigen::AngleAxisd{-pitch_encoder_angle, Eigen::Vector3d::UnitY()}}
                    * yaw_link_to_odom_imu;
                pitch_link_to_odom_imu.normalize();

                *gimbal_pitch_velocity_encoder_ = gimbal_pitch_motor_.velocity();
                // The BMI088 is mounted on the yaw link. fast_tf stores PitchLink ->
                // OdomImu, so use the encoder pitch from the TF tree to move the
                // yaw-link pose back into PitchLink.
                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    pitch_link_to_odom_imu);
            }

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                pitch_encoder_angle);
        }

        void command_update() {
            auto builder = start_transmit();
            builder.can0_transmit({
                .can_id = 0x141,
                .can_data = gimbal_pitch_motor_.generate_command().as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        gimbal_left_friction_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        gimbal_right_friction_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });

            builder.can3_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        scope_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
        }

    private:
        void uart1_receive_callback(const librmcs::data::UartDataView&) override {}

        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x141)
                gimbal_pitch_motor_.store_status(data.can_data);
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x201)
                gimbal_left_friction_.store_status(data.can_data);
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x202)
                gimbal_right_friction_.store_status(data.can_data);
        }

        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x203)
                scope_motor_.store_status(data.can_data);
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        bool has_external_imu_board_ = false;
        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_encoder_;

        device::Bmi088 bmi088_;
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;
        device::DjiMotor scope_motor_;
    };

    auto status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
        -> void {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("Gimbal Status");
        text("-   Yaw: {}", bottom_board_->gimbal_yaw_motor_.last_raw_angle());
        text("- Pitch: {}", top_board_->gimbal_pitch_motor_.last_raw_angle());

        text("Chassis Status");
        constexpr auto kPosition =
            std::array<std::string_view, 4>{"left front", "left back", "right back", "right front"};
        constexpr auto kMaxLength =
            std::ranges::max_element(kPosition, {}, &std::string_view::size)->size();

        for (auto&& [index, motor] :
             std::views::zip(kPosition, bottom_board_->chassis_joint_motors_)) {
            text("- {:{}}: {}", index, kMaxLength, motor.last_raw_angle());
        }

        response->message = feedback_message.str();
    }

    OutputInterface<rmcs_description::Tf> tf_;
    InputInterface<Clock::time_point> timestamp_;

    std::unique_ptr<ImuBoard> imu_board_;
    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;

    std::shared_ptr<Command> command_;
    uint32_t cmd_tick_ = 0;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantryOmni, rmcs_executor::Component)
