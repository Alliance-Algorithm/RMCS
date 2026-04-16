#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/hard_sync_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/agent/c_board.hpp>
#include <librmcs/agent/rmcs_board.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

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
        , deformable_infantry_command_(
              create_partner_component<DeformableInfantryOmniCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_input("/predefined/timestamp", timestamp_);
        register_output("/tf", tf_);
        register_output("/gimbal/hard_sync_snapshot", hard_sync_snapshot_);

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        joints_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/joints/calibrate", rclcpp::QoS(1), [this](std_msgs::msg::Int32::UniquePtr msg) {
                joints_calibrate_subscription_callback(std::move(msg));
            });

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        rmcs_board_ = std::make_unique<BottomBoard>(
            *this, *deformable_infantry_command_,
            get_parameter("serial_filter_rmcs_board").as_string());
        top_board_ = std::make_unique<TopBoard>(
            *this, *deformable_infantry_command_,
            get_parameter("serial_filter_top_board").as_string());
    }

    ~DeformableInfantryOmni() override = default;

    void before_updating() override {
        top_board_->request_hard_sync_read();
        next_hard_sync_log_time_ = Clock::now() + std::chrono::seconds(1);
    }

    void update() override {
        rmcs_board_->update();
        top_board_->update();
        update_hard_sync_snapshot();
    }

    void command_update() {
        const bool even = ((cmd_tick_++ & 1u) == 0u);
        rmcs_board_->command_update(even);
        top_board_->command_update();
    }

private:
    class DeformableInfantryOmniCommand;
    class BottomBoard;
    class TopBoard;

    void update_hard_sync_snapshot() {
        if (!hard_sync_pending_.exchange(false, std::memory_order_relaxed))
            return;

        hard_sync_snapshot_->valid = true;
        hard_sync_snapshot_->exposure_timestamp = *timestamp_;
        hard_sync_snapshot_->qw = top_board_->bmi088_.q0();
        hard_sync_snapshot_->qx = top_board_->bmi088_.q1();
        hard_sync_snapshot_->qy = top_board_->bmi088_.q2();
        hard_sync_snapshot_->qz = top_board_->bmi088_.q3();
        ++hard_sync_snapshot_count_;

        if (*timestamp_ >= next_hard_sync_log_time_) {
            RCLCPP_INFO(
                get_logger(), "[hard sync] published %zu snapshots in the last second",
                hard_sync_snapshot_count_);
            hard_sync_snapshot_count_ = 0;
            next_hard_sync_log_time_ = *timestamp_ + std::chrono::seconds(1);
        }
    }

    void joints_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!rmcs_board_)
            return;

        RCLCPP_INFO(
            get_logger(), "New left front offset: %ld",
            rmcs_board_->chassis_joint_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "New left back offset: %ld",
            rmcs_board_->chassis_joint_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "New right back offset: %ld",
            rmcs_board_->chassis_joint_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "New right front offset: %ld",
            rmcs_board_->chassis_joint_motors_[3].calibrate_zero_point());
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!rmcs_board_ || !top_board_)
            return;

        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %ld",
            rmcs_board_->gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    class DeformableInfantryOmniCommand : public rmcs_executor::Component {
    public:
        explicit DeformableInfantryOmniCommand(DeformableInfantryOmni& deformableInfantry)
            : deformableInfantry(deformableInfantry) {}

        void update() override { deformableInfantry.command_update(); }

        DeformableInfantryOmni& deformableInfantry;
    };

    class BottomBoard final : private librmcs::agent::RmcsBoard {
    public:
        friend class DeformableInfantryOmni;

        static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

        explicit BottomBoard(
            DeformableInfantryOmni& deformableInfantry,
            DeformableInfantryOmniCommand& deformableInfantry_command,
            std::string serial_filter =
                {
        })
            : librmcs::agent::RmcsBoard(
                  serial_filter,
                  librmcs::agent::AdvancedOptions{.dangerously_skip_version_checks = true})
            , deformable_infantry_(deformableInfantry)
            , tf_(deformableInfantry.tf_)
            , imu_(1000, 0.2, 0.0)
            , gimbal_yaw_motor_(deformableInfantry, deformableInfantry_command, "/gimbal/yaw")
            , dr16_(deformableInfantry)
            , chassis_wheel_motors_{device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/left_front_wheel"}, device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/left_back_wheel"}, device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/right_back_wheel"}, device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/right_front_wheel"}},
            chassis_joint_motors_{
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/left_front_joint"},
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/left_back_joint"},
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/right_back_joint"},
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/right_front_joint"}},
            supercap_(deformableInfantry, deformableInfantry_command),
            gimbal_bullet_feeder_(
                deformableInfantry, deformableInfantry_command, "/gimbal/bullet_feeder") {

            deformableInfantry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart0_transmit({
                    .uart_data = std::span<const std::byte>{buffer, size}
                });
                return size;
            };

            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(
                        deformableInfantry.get_parameter("yaw_motor_zero_point").as_int())));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                        .set_reduction_ratio(11.0)
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
                return std::make_tuple(x, y, z);
            });

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}.enable_multi_turn_angle());

            deformableInfantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            deformableInfantry.register_output("/chassis/imu/pitch", chassis_imu_pitch_, 0.0);
            deformableInfantry.register_output("/chassis/imu/roll", chassis_imu_roll_, 0.0);
            deformableInfantry.register_output(
                "/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, 0.0);
            deformableInfantry.register_output(
                "/chassis/imu/roll_rate", chassis_imu_roll_rate_, 0.0);
            deformableInfantry.register_output(
                "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, nan_);
            deformableInfantry.register_output(
                "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, nan_);
            deformableInfantry.register_output(
                "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, nan_);
            deformableInfantry.register_output(
                "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_,
                nan_);
            deformableInfantry.register_output(
                "/chassis/left_front_joint/physical_velocity", left_front_joint_physical_velocity_,
                nan_);
            deformableInfantry.register_output(
                "/chassis/left_back_joint/physical_velocity", left_back_joint_physical_velocity_,
                nan_);
            deformableInfantry.register_output(
                "/chassis/right_back_joint/physical_velocity", right_back_joint_physical_velocity_,
                nan_);
            deformableInfantry.register_output(
                "/chassis/right_front_joint/physical_velocity",
                right_front_joint_physical_velocity_, nan_);
            deformableInfantry.register_output("/chassis/encoder/alpha", encoder_alpha_, nan_);
            deformableInfantry.register_output(
                "/chassis/encoder/alpha_dot", encoder_alpha_dot_, nan_);
            deformableInfantry.register_output("/chassis/radius", radius_, nan_);
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

                *chassis_imu_pitch_ = std::asin(sin_pitch);
                *chassis_imu_roll_ =
                    std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
                *chassis_imu_pitch_rate_ = imu_.gy();
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

            dr16_.update_status();
            gimbal_yaw_motor_.update_status();
            if (supercap_status_received_.load(std::memory_order_relaxed))
                supercap_.update_status();
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
        DeformableInfantryOmni& deformable_infantry_;

        static constexpr double joint_zero_physical_angle_rad_ = 62.5 * std::numbers::pi / 180.0;
        static constexpr double chassis_radius_base_ = 0.2341741;
        static constexpr double rod_length_ = 0.150;

        static double to_physical_angle_(double motor_angle) {
            return joint_zero_physical_angle_rad_ - motor_angle;
        }

        static double to_physical_velocity_(double motor_velocity) { return -motor_velocity; }

        void update_joint_physical_feedback_(
            size_t index, OutputInterface<double>& angle_output,
            OutputInterface<double>& velocity_output) {
            if (!joint_status_received_[index].load(std::memory_order_relaxed)) {
                *angle_output = nan_;
                *velocity_output = nan_;
                return;
            }

            *angle_output = to_physical_angle_(chassis_joint_motors_[index].angle());
            *velocity_output = to_physical_velocity_(chassis_joint_motors_[index].velocity());
        }

        void update_geometry_feedback_() {
            const Eigen::Vector4d alpha_rad{
                *left_front_joint_physical_angle_, *left_back_joint_physical_angle_,
                *right_back_joint_physical_angle_, *right_front_joint_physical_angle_};
            const Eigen::Vector4d alpha_dot_rad{
                *left_front_joint_physical_velocity_, *left_back_joint_physical_velocity_,
                *right_back_joint_physical_velocity_, *right_front_joint_physical_velocity_};

            if (!alpha_rad.array().isFinite().all() || !alpha_dot_rad.array().isFinite().all()) {
                *encoder_alpha_ = nan_;
                *encoder_alpha_dot_ = nan_;
                *radius_ = nan_;
                return;
            }

            *encoder_alpha_ = alpha_rad.mean();
            *encoder_alpha_dot_ = alpha_dot_rad.mean();
            *radius_ = (chassis_radius_base_ + rod_length_ * alpha_rad.array().cos()).mean();
        }

        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[0].store_status(data.can_data);
            else if (data.can_id == 0x141) {
                chassis_joint_motors_[0].store_status(data.can_data);
                joint_status_received_[0].store(true, std::memory_order_relaxed);
            }
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[1].store_status(data.can_data);
            else if (data.can_id == 0x141) {
                chassis_joint_motors_[1].store_status(data.can_data);
                joint_status_received_[1].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x300) {
                supercap_.store_status(data.can_data);
                supercap_status_received_.store(true, std::memory_order_relaxed);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[2].store_status(data.can_data);
            else if (data.can_id == 0x141) {
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
            if (data.can_id == 0x201)
                chassis_wheel_motors_[3].store_status(data.can_data);
            else if (data.can_id == 0x141) {
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

        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088 imu_;
        device::LkMotor gimbal_yaw_motor_;
        device::Dr16 dr16_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::LkMotor chassis_joint_motors_[4];
        std::atomic<bool> joint_status_received_[4] = {false, false, false, false};
        device::Supercap supercap_;
        std::atomic<bool> supercap_status_received_{false};
        device::DjiMotor gimbal_bullet_feeder_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

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
    };

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class DeformableInfantryOmni;

        explicit TopBoard(
            DeformableInfantryOmni& deformableInfantry,
            DeformableInfantryOmniCommand& deformableInfantry_command,
            std::string serial_filter = {})
            : librmcs::agent::CBoard(
                  serial_filter,
                  librmcs::agent::AdvancedOptions{.dangerously_skip_version_checks = true})
            , hard_sync_pending_(deformableInfantry.hard_sync_pending_)
            , tf_(deformableInfantry.tf_)
            , bmi088_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(deformableInfantry, deformableInfantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(
                  deformableInfantry, deformableInfantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(
                  deformableInfantry, deformableInfantry_command, "/gimbal/right_friction")
            , scope_motor_(deformableInfantry, deformableInfantry_command, "/gimbal/scope") {

            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            deformableInfantry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));

            scope_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}.enable_multi_turn_angle());

            deformableInfantry.register_output(
                "/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            deformableInfantry.register_output(
                "/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_encoder_);

            bmi088_.set_coordinate_mapping([](double x, double y, double z) {
                // Top board BMI088 maps to gimbal frame as (-x, -y, z).
                return std::make_tuple(y, -x, z);
            });
        }

        ~TopBoard() override = default;

        void request_hard_sync_read() {
            // RMCS-board top board variant currently has no GPIO hard-sync request path.
        }

        void update() {
            bmi088_.update_status();

            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            scope_motor_.update_status();

            const double pitch_encoder_angle = gimbal_pitch_motor_.angle();
            Eigen::Quaterniond const odom_imu_to_yaw_link{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
            Eigen::Quaterniond const yaw_link_to_odom_imu = odom_imu_to_yaw_link.conjugate();
            Eigen::Quaterniond pitch_link_to_odom_imu = Eigen::Quaterniond{Eigen::AngleAxisd{
                                                          -pitch_encoder_angle,
                                                          Eigen::Vector3d::UnitY()}}
                                                      * yaw_link_to_odom_imu;
            pitch_link_to_odom_imu.normalize();

            *gimbal_yaw_velocity_bmi088_ = bmi088_.gz();
            *gimbal_pitch_velocity_encoder_ = gimbal_pitch_motor_.velocity();
            // The BMI088 is mounted on the yaw link. fast_tf stores PitchLink -> OdomImu, so use
            // the encoder pitch from the TF tree to move the yaw-link pose back into PitchLink.
            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                pitch_link_to_odom_imu);

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                pitch_encoder_angle);
        }

        void command_update() {
            auto builder = start_transmit();
            builder.can1_transmit({
                .can_id = 0x141,
                .can_data = gimbal_pitch_motor_.generate_velocity_command().as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       gimbal_left_friction_.generate_command(),
                                       gimbal_right_friction_.generate_command(),
                                       scope_motor_.generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
        }

    private:
        void uart1_receive_callback(const librmcs::data::UartDataView&) override {}

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x141)
                gimbal_pitch_motor_.store_status(data.can_data);
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x201)
                gimbal_left_friction_.store_status(data.can_data);
            else if (data.can_id == 0x202)
                gimbal_right_friction_.store_status(data.can_data);
            else if (data.can_id == 0x203)
                scope_motor_.store_status(data.can_data);
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        std::atomic<bool>& hard_sync_pending_;
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_encoder_;

        device::Bmi088 bmi088_;
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;
        device::DjiMotor scope_motor_;
    };

    OutputInterface<rmcs_description::Tf> tf_;
    InputInterface<Clock::time_point> timestamp_;
    OutputInterface<rmcs_msgs::HardSyncSnapshot> hard_sync_snapshot_;
    std::atomic<bool> hard_sync_pending_{false};
    size_t hard_sync_snapshot_count_ = 0;
    Clock::time_point next_hard_sync_log_time_{};

    std::shared_ptr<DeformableInfantryOmniCommand> deformable_infantry_command_;
    std::unique_ptr<BottomBoard> rmcs_board_;
    std::unique_ptr<TopBoard> top_board_;
    uint32_t cmd_tick_ = 0;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr joints_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantryOmni, rmcs_executor::Component)
