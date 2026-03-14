#include <cstddef>
#include <cstdint>
#include <memory>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/agent/c_board.hpp>
#include <librmcs/agent/rmcs_board.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/utility/ring_buffer.hpp"

namespace rmcs_core::hardware {

class DeformableInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , deformable_infantry_command_(
              create_partner_component<DeformableInfantryCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        steers_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/steers/calibrate", rclcpp::QoS(1),
            [this](std_msgs::msg::Int32::UniquePtr msg) {
                steers_calibrate_subscription_callback(std::move(msg));
            });

        joints_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/joints/calibrate", rclcpp::QoS(1),
            [this](std_msgs::msg::Int32::UniquePtr msg) {
                joints_calibrate_subscription_callback(std::move(msg));
            });

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        rmcs_board_ = std::make_unique<CombinedBoard>(
            *this, *deformable_infantry_command_,
            get_parameter("serial_filter_rmcs_board").as_string());

        top_board_ = std::make_unique<TopBoard>(
            *this, *deformable_infantry_command_,
            get_parameter("serial_filter_top_board").as_string());
    }

    ~DeformableInfantry() override = default;

    void update() override {
        rmcs_board_->update();
        top_board_->update();
    }

    void command_update() {
        const bool even = ((cmd_tick_++ & 1u) == 0u);
        rmcs_board_->command_update(even);
        top_board_->command_update();
    }

private:
    class DeformableInfantryCommand;
    class CombinedBoard;
    class TopBoard;

    void steers_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!rmcs_board_)
            return;

        RCLCPP_INFO(get_logger(), "New left front offset: %d",
                    rmcs_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(get_logger(), "New left back offset: %d",
                    rmcs_board_->chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(get_logger(), "New right back offset: %d",
                    rmcs_board_->chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(get_logger(), "New right front offset: %d",
                    rmcs_board_->chassis_steer_motors_[3].calibrate_zero_point());
    }

    void joints_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!rmcs_board_)
            return;

        RCLCPP_INFO(get_logger(), "New left front offset: %f",
                    rmcs_board_->chassis_joint_motors_[0].encoder_angle());
        RCLCPP_INFO(get_logger(), "New left back offset: %f",
                    rmcs_board_->chassis_joint_motors_[1].encoder_angle());
        RCLCPP_INFO(get_logger(), "New right back offset: %f",
                    rmcs_board_->chassis_joint_motors_[2].encoder_angle());
        RCLCPP_INFO(get_logger(), "New right front offset: %f",
                    rmcs_board_->chassis_joint_motors_[3].encoder_angle());
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %ld",
            rmcs_board_->gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    class DeformableInfantryCommand : public rmcs_executor::Component {
    public:
        explicit DeformableInfantryCommand(DeformableInfantry& deformableInfantry)
            : deformableInfantry(deformableInfantry) {}

        void update() override { deformableInfantry.command_update(); }

        DeformableInfantry& deformableInfantry;
    };

    class CombinedBoard final : private librmcs::agent::RmcsBoard {
    public:
        friend class DeformableInfantry;

        explicit CombinedBoard(
            DeformableInfantry& deformableInfantry,
            DeformableInfantryCommand& deformableInfantry_command,
            std::string serial_filter = {})
            : librmcs::agent::RmcsBoard(serial_filter)
            , tf_(deformableInfantry.tf_)
            , imu_(1000, 0.2, 0.0)
            , gimbal_yaw_motor_(deformableInfantry, deformableInfantry_command, "/gimbal/yaw")
            , dr16_(deformableInfantry)
            , chassis_wheel_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_front_wheel"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_back_wheel"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_back_wheel"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_front_wheel"}}
            , chassis_steer_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_front_steering"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_back_steering"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_back_steering"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_front_steering"}}
            , chassis_joint_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_front_joint"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_back_joint"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_back_joint"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_front_joint"}}
            , supercap_(deformableInfantry, deformableInfantry_command)
            , gimbal_bullet_feeder_(
                  deformableInfantry, deformableInfantry_command, "/gimbal/bullet_feeder") {

            deformableInfantry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart2_transmit(
                    {.uart_data = std::span<const std::byte>{buffer, size}});
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
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                        .set_reduction_ratio(1.0)
                        .set_reversed()
                        .enable_multi_turn_angle());

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}.enable_multi_turn_angle());

            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());

            deformableInfantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        ~CombinedBoard() override = default;

        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gy();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();
            for (auto& motor : chassis_joint_motors_)
                motor.update_status();

            dr16_.update_status();
            gimbal_yaw_motor_.update_status();
            supercap_.update_status();
            gimbal_bullet_feeder_.update_status();

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
        }

        void command_update(bool even) {
            auto builder = start_transmit();
            if (even) {
                builder.can0_transmit({
                    .can_id = 0x1FE,
                    .can_data = device::CanPacket8{
                        chassis_steer_motors_[0].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data = device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        chassis_steer_motors_[1].generate_command(),
                    }.as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x1FE,
                    .can_data = device::CanPacket8{
                        chassis_steer_motors_[2].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
                builder.can3_transmit({
                    .can_id = 0x1FE,
                    .can_data = device::CanPacket8{
                        chassis_steer_motors_[3].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
            } else {
                builder.can0_transmit({
                    .can_id = 0x200,
                    .can_data = device::CanPacket8{
                        chassis_wheel_motors_[0].generate_command(),
                        chassis_joint_motors_[0].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x200,
                    .can_data = device::CanPacket8{
                        chassis_wheel_motors_[1].generate_command(),
                        chassis_joint_motors_[1].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x200,
                    .can_data = device::CanPacket8{
                        chassis_wheel_motors_[2].generate_command(),
                        chassis_joint_motors_[2].generate_command(),
                        gimbal_bullet_feeder_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
                builder.can3_transmit({
                    .can_id = 0x200,
                    .can_data = device::CanPacket8{
                        chassis_wheel_motors_[3].generate_command(),
                        chassis_joint_motors_[3].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x142,
                    .can_data = gimbal_yaw_motor_
                                    .generate_velocity_command(
                                        gimbal_yaw_motor_.control_velocity() - imu_.gz())
                                    .as_bytes(),
                });
            }
        }

    private:
        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[0].store_status(data.can_data);
            else if (data.can_id == 0x202)
                chassis_joint_motors_[0].store_status(data.can_data);
            else if (data.can_id == 0x205)
                chassis_steer_motors_[0].store_status(data.can_data);
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[1].store_status(data.can_data);
            else if (data.can_id == 0x202)
                chassis_joint_motors_[1].store_status(data.can_data);
            else if (data.can_id == 0x208)
                chassis_steer_motors_[1].store_status(data.can_data);
            else if (data.can_id == 0x142)
                gimbal_yaw_motor_.store_status(data.can_data);
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[2].store_status(data.can_data);
            else if (data.can_id == 0x202)
                chassis_joint_motors_[2].store_status(data.can_data);
            else if (data.can_id == 0x203)
                gimbal_bullet_feeder_.store_status(data.can_data);
            else if (data.can_id == 0x205)
                chassis_steer_motors_[2].store_status(data.can_data);
            // Supercap mapping is reserved on CAN2 for now.
        }

        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[3].store_status(data.can_data);
            else if (data.can_id == 0x202)
                chassis_joint_motors_[3].store_status(data.can_data);
            else if (data.can_id == 0x205)
                chassis_steer_motors_[3].store_status(data.can_data);
        }

        void uart2_receive_callback(const librmcs::data::UartDataView& data) override {
            const std::byte* ptr = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_multi(
                [&ptr](std::byte* storage) { *storage = *ptr++; }, data.uart_data.size());
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
        device::DjiMotor chassis_steer_motors_[4];
        device::DjiMotor chassis_joint_motors_[4];
        device::Supercap supercap_;
        device::DjiMotor gimbal_bullet_feeder_;

        utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
    };

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class DeformableInfantry;

        explicit TopBoard(
            DeformableInfantry& deformableInfantry,
            DeformableInfantryCommand& deformableInfantry_command,
            std::string serial_filter = {})
            : CBoard(serial_filter)
            , tf_(deformableInfantry.tf_)
            , bmi088_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(deformableInfantry, deformableInfantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(
                  deformableInfantry, deformableInfantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(
                  deformableInfantry, deformableInfantry_command, "/gimbal/right_friction")
            , scope_motor(deformableInfantry, deformableInfantry_command, "/gimbal/scope") {

            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(
                        deformableInfantry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reduction_ratio(1.)
                .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.));

            scope_motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}.enable_multi_turn_angle());

            deformableInfantry.register_output(
                "/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            deformableInfantry.register_output(
                "/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_);

            bmi088_.set_coordinate_mapping_tilted(
                /*roll_rad=*/0.0 * std::numbers::pi / 180,
                /*pitch_rad=*/27.73 * std::numbers::pi / 180,
                /*yaw_rad=*/0 * std::numbers::pi / 180);
        }

        ~TopBoard() = default;

        void update() {
            bmi088_.update_status();
            Eigen::Quaterniond gimbal_bmi088_pose{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};

            *gimbal_yaw_velocity_bmi088_   = bmi088_.gz();
            *gimbal_pitch_velocity_bmi088_ = bmi088_.gy();

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_bmi088_pose.conjugate());

            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            scope_motor.update_status();

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());
        }

        void command_update() {
            auto builder = start_transmit();
            builder.can1_transmit({
                .can_id = 0x141,
                .can_data = gimbal_pitch_motor_.generate_command().as_bytes(),
            });
            builder.can2_transmit({
                .can_id = 0x200,
                .can_data = device::CanPacket8{
                    gimbal_left_friction_.generate_command(),
                    device::CanPacket8::PaddingQuarter{},
                    scope_motor.generate_command(),
                    gimbal_right_friction_.generate_command(),
                }.as_bytes(),
            });
        }

    private:
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
            else if (data.can_id == 0x204)
                gimbal_right_friction_.store_status(data.can_data);
            else if (data.can_id == 0x203)
                scope_motor.store_status(data.can_data);
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_bmi088_;

        device::Bmi088 bmi088_;
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;
        device::DjiMotor scope_motor;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    std::shared_ptr<DeformableInfantryCommand> deformable_infantry_command_;
    std::unique_ptr<CombinedBoard> rmcs_board_;
    std::unique_ptr<TopBoard> top_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr joints_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    uint32_t cmd_tick_ = 0;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantry, rmcs_executor::Component)
