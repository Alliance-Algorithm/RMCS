#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>

#include <eigen3/Eigen/Dense>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/agent/rmcs_board_pro.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

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
        joints_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/joints/calibrate", rclcpp::QoS(1), [this](std_msgs::msg::Int32::UniquePtr msg) {
                joints_calibrate_subscription_callback(std::move(msg));
            });

        rmcs_board_ = std::make_unique<BottomBoard>(
            *this, *deformable_infantry_command_,
            get_parameter("serial_filter_rmcs_board").as_string());
    }

    ~DeformableInfantryOmni() override = default;

    void update() override { rmcs_board_->update(); }

    void command_update() { rmcs_board_->command_update(); }

private:
    class DeformableInfantryOmniCommand;
    class BottomBoard;

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

    class DeformableInfantryOmniCommand : public rmcs_executor::Component {
    public:
        explicit DeformableInfantryOmniCommand(DeformableInfantryOmni& deformable_infantry)
            : deformable_infantry(deformable_infantry) {}

        void update() override { deformable_infantry.command_update(); }

        DeformableInfantryOmni& deformable_infantry;
    };

    class BottomBoard final : private librmcs::agent::RmcsBoardPro {
    public:
        friend class DeformableInfantryOmni;

        static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

        explicit BottomBoard(
            DeformableInfantryOmni& deformable_infantry,
            DeformableInfantryOmniCommand& deformable_infantry_command,
            std::string serial_filter =
                {
        })
            : librmcs::agent::RmcsBoardPro(
                  serial_filter,
                  librmcs::agent::AdvancedOptions{.dangerously_skip_version_checks = true})
            , deformable_infantry_(deformable_infantry)
            , imu_(1000, 0.2, 0.0)
            , dr16_(deformable_infantry)
            , chassis_wheel_motors_{device::DjiMotor{deformable_infantry, deformable_infantry_command, "/chassis/left_front_wheel"}, device::DjiMotor{deformable_infantry, deformable_infantry_command, "/chassis/left_back_wheel"}, device::DjiMotor{deformable_infantry, deformable_infantry_command, "/chassis/right_back_wheel"}, device::DjiMotor{deformable_infantry, deformable_infantry_command, "/chassis/right_front_wheel"}},
            chassis_joint_motors_{
                device::LkMotor{
                    deformable_infantry, deformable_infantry_command, "/chassis/left_front_joint"},
                device::LkMotor{
                    deformable_infantry, deformable_infantry_command, "/chassis/left_back_joint"},
                device::LkMotor{
                    deformable_infantry, deformable_infantry_command, "/chassis/right_back_joint"},
                device::LkMotor{
                    deformable_infantry, deformable_infantry_command,
                    "/chassis/right_front_joint"}},
            supercap_(deformable_infantry, deformable_infantry_command) {

            deformable_infantry.register_output("/referee/serial", referee_serial_);
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

            deformable_infantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            deformable_infantry.register_output(
                "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, nan_);
            deformable_infantry.register_output(
                "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, nan_);
            deformable_infantry.register_output(
                "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, nan_);
            deformable_infantry.register_output(
                "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_,
                nan_);
            deformable_infantry.register_output(
                "/chassis/left_front_joint/physical_velocity", left_front_joint_physical_velocity_,
                nan_);
            deformable_infantry.register_output(
                "/chassis/left_back_joint/physical_velocity", left_back_joint_physical_velocity_,
                nan_);
            deformable_infantry.register_output(
                "/chassis/right_back_joint/physical_velocity", right_back_joint_physical_velocity_,
                nan_);
            deformable_infantry.register_output(
                "/chassis/right_front_joint/physical_velocity",
                right_front_joint_physical_velocity_, nan_);
            deformable_infantry.register_output("/chassis/encoder/alpha", encoder_alpha_, nan_);
            deformable_infantry.register_output(
                "/chassis/encoder/alpha_dot", encoder_alpha_dot_, nan_);
            deformable_infantry.register_output("/chassis/radius", radius_, nan_);
        }

        ~BottomBoard() override = default;

        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gz();

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
            if (supercap_status_received_.load(std::memory_order_relaxed))
                supercap_.update_status();
        }

        void command_update() {
            auto builder = start_transmit();

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
                                       device::CanPacket8::PaddingQuarter{},
                                       supercap_.generate_command(),
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
            } else if (data.can_id == 0x300) {
                supercap_.store_status(data.can_data);
                supercap_status_received_.store(true, std::memory_order_relaxed);
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

        void uart2_receive_callback(const librmcs::data::UartDataView& data) override {
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

        device::Bmi088 imu_;
        device::Dr16 dr16_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::LkMotor chassis_joint_motors_[4];
        std::atomic<bool> joint_status_received_[4] = {false, false, false, false};
        device::Supercap supercap_;
        std::atomic<bool> supercap_status_received_{false};

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
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

    std::shared_ptr<DeformableInfantryOmniCommand> deformable_infantry_command_;
    std::unique_ptr<BottomBoard> rmcs_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr joints_calibrate_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantryOmni, rmcs_executor::Component)
