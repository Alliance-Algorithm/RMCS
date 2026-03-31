#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/hard_sync_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/agent/rmcs_board.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

class DeformableInfantryV2
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryV2()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , deformable_infantry_command_(
              create_partner_component<DeformableInfantryV2Command>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
\
        register_output("/chassis/power", chassis_power_, 0.0);
        register_output("/chassis/voltage", chassis_voltage_, 0.0);

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});
        tf_->set_transform<PitchLink, OdomImu>(Eigen::Quaterniond::Identity());
        tf_->set_state<GimbalCenterLink, YawLink>(0.0);
        tf_->set_state<YawLink, PitchLink>(0.0);

        steers_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/steers/calibrate", rclcpp::QoS(1), [this](std_msgs::msg::Int32::UniquePtr msg) {
                steers_calibrate_subscription_callback(std::move(msg));
            });

        joints_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/joints/calibrate", rclcpp::QoS(1), [this](std_msgs::msg::Int32::UniquePtr msg) {
                joints_calibrate_subscription_callback(std::move(msg));
            });

        rmcs_board_ = std::make_unique<BottomBoard>(
            *this, *deformable_infantry_command_,
            get_parameter("serial_filter_rmcs_board").as_string());
    }

    ~DeformableInfantryV2() override = default;

    void update() override {
        rmcs_board_->update();
    }

    void command_update() {
        const bool even = ((cmd_tick_++ & 1u) == 0u);
        rmcs_board_->command_update(even);
    }

private:
    class DeformableInfantryV2Command;
    class BottomBoard;

    void steers_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!rmcs_board_)
            return;

        RCLCPP_INFO(
            get_logger(), "New left front offset: %d",
            rmcs_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "New left back offset: %d",
            rmcs_board_->chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "New right back offset: %d",
            rmcs_board_->chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "New right front offset: %d",
            rmcs_board_->chassis_steer_motors_[3].calibrate_zero_point());
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

    class DeformableInfantryV2Command : public rmcs_executor::Component {
    public:
        explicit DeformableInfantryV2Command(DeformableInfantryV2& deformableInfantry)
            : deformableInfantry(deformableInfantry) {}

        void update() override { deformableInfantry.command_update(); }

        DeformableInfantryV2& deformableInfantry;
    };

    class BottomBoard final : private librmcs::agent::RmcsBoard {
    public:
        friend class DeformableInfantryV2;

        explicit BottomBoard(
            DeformableInfantryV2& deformableInfantry,
            DeformableInfantryV2Command& deformableInfantry_command,
            std::string serial_filter = {})
            : librmcs::agent::RmcsBoard(serial_filter)
            , imu_(1000, 0.2, 0.0)
            , dr16_(deformableInfantry)
            , chassis_wheel_motors_{device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/left_front_wheel"}, device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/left_back_wheel"}, device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/right_back_wheel"}, device::DjiMotor{deformableInfantry, deformableInfantry_command, "/chassis/right_front_wheel"}},
            chassis_steer_motors_{
                device::DjiMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/left_front_steering"},
                device::DjiMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/left_back_steering"},
                device::DjiMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/right_back_steering"},
                device::DjiMotor{
                    deformableInfantry, deformableInfantry_command,
                    "/chassis/right_front_steering"}},
            chassis_joint_motors_{
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/left_front_joint"},
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/left_back_joint"},
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/right_back_joint"},
                device::LkMotor{
                    deformableInfantry, deformableInfantry_command, "/chassis/right_front_joint"}} {

            deformableInfantry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart2_transmit({
                    .uart_data = std::span<const std::byte>{buffer, size}
                });
                return size;
            };

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                        .set_reduction_ratio(11.0)
                        .enable_multi_turn_angle()
                        .set_reversed());

            // V2: LK MG5010 i36 direct-drive joint motors, built-in encoder zero point
            for (auto& motor : chassis_joint_motors_)
                motor.configure(
                    device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei36}
                        .set_reversed()
                        .enable_multi_turn_angle());

            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            deformableInfantry.get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            deformableInfantry.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            deformableInfantry.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            deformableInfantry.get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());

            deformableInfantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        ~BottomBoard() override = default;

        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gz();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();
            for (auto& motor : chassis_joint_motors_)
                motor.update_status();

            dr16_.update_status();
        }

        void command_update(bool even) {
            auto builder = start_transmit();
            if (even) {
                // Steer motors: same as V1
                builder.can0_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           chassis_steer_motors_[0].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           chassis_steer_motors_[1].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           chassis_steer_motors_[2].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
                builder.can3_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           chassis_steer_motors_[3].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
            } else {
                // V2: Wheel DJI frames (wheel only, no joint packed in)
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

                // V2: Joint LK motors — individual CAN frames
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
        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[0].store_status(data.can_data);
            else if (data.can_id == 0x141)
                chassis_joint_motors_[0].store_status(data.can_data);
            else if (data.can_id == 0x205)
                chassis_steer_motors_[0].store_status(data.can_data);
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[1].store_status(data.can_data);
            else if (data.can_id == 0x141)
                chassis_joint_motors_[1].store_status(data.can_data);
            else if (data.can_id == 0x205)
                chassis_steer_motors_[1].store_status(data.can_data);
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[2].store_status(data.can_data);
            else if (data.can_id == 0x141)
                chassis_joint_motors_[2].store_status(data.can_data);
            else if (data.can_id == 0x205)
                chassis_steer_motors_[2].store_status(data.can_data);
        }

        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201)
                chassis_wheel_motors_[3].store_status(data.can_data);
            else if (data.can_id == 0x141)
                chassis_joint_motors_[3].store_status(data.can_data);
            else if (data.can_id == 0x205)
                chassis_steer_motors_[3].store_status(data.can_data);
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
        device::DjiMotor chassis_steer_motors_[4];
        device::LkMotor chassis_joint_motors_[4];

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
    };


    OutputInterface<rmcs_description::Tf> tf_;

    OutputInterface<double> chassis_power_;
    OutputInterface<double> chassis_voltage_;

    std::shared_ptr<DeformableInfantryV2Command> deformable_infantry_command_;
    std::unique_ptr<BottomBoard> rmcs_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr joints_calibrate_subscription_;

    uint32_t cmd_tick_ = 0;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantryV2, rmcs_executor::Component)
