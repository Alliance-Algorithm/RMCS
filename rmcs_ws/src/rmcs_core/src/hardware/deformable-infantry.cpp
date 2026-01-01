#include <array>
#include <memory>
#include <thread>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"
#include "librmcs/client/cboard.hpp"

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

        left_board_ = std::make_unique<LeftBoard>(
            *this, *deformable_infantry_command_,
            static_cast<int>(get_parameter("usb_pid_left_board").as_int()));

        right_board_ = std::make_unique<RightBoard>(
            *this, *deformable_infantry_command_,
            static_cast<int>(get_parameter("usb_pid_right_board").as_int()));
    }

    ~DeformableInfantry() override = default;

    void update() override {
        left_board_->update();
        right_board_->update();
    }

    void command_update() {
        left_board_->command_update();
        right_board_->command_update();
    }

private:
    class DeformableInfantryCommand;
    class LeftBoard;
    class RightBoard;

    void steers_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!left_board_ || !right_board_)
            return;

        RCLCPP_INFO(get_logger(), "New left front offset: %d",
                    left_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(get_logger(), "New left back offset: %d",
                    left_board_->chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(get_logger(), "New right back offset: %d",
                    right_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(get_logger(), "New right front offset: %d",
                    right_board_->chassis_steer_motors_[1].calibrate_zero_point());
    }

    void joints_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!left_board_ || !right_board_)
            return;

        RCLCPP_INFO(get_logger(), "New left front offset: %f",
                    left_board_->chassis_joint_motors_[0].encoder_angle());
        RCLCPP_INFO(get_logger(), "New left back offset: %f",
                    left_board_->chassis_joint_motors_[1].encoder_angle());
        RCLCPP_INFO(get_logger(), "New right back offset: %f",
                    right_board_->chassis_joint_motors_[0].encoder_angle());
        RCLCPP_INFO(get_logger(), "New right front offset: %f",
                    right_board_->chassis_joint_motors_[1].encoder_angle());
    }

    class DeformableInfantryCommand : public rmcs_executor::Component {
    public:
        explicit DeformableInfantryCommand(DeformableInfantry& deformableInfantry)
            : deformableInfantry(deformableInfantry) {}

        void update() override { deformableInfantry.command_update(); }

        DeformableInfantry& deformableInfantry;
    };

    class LeftBoard final : private librmcs::client::CBoard {
    public:
        friend class DeformableInfantry;

        explicit LeftBoard(
            DeformableInfantry& deformableInfantry,
            DeformableInfantryCommand& deformableInfantry_command,
            int usb_pid = -1)
            : CBoard(usb_pid)
            , imu_(1000, 0.2, 0.0)
            , dr16_(deformableInfantry)
            , chassis_wheel_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_front_wheel"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_back_wheel"}}
            , chassis_steer_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_front_steering"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_back_steering"}}
            , chassis_joint_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_back_joint"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/left_front_joint"}}
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            deformableInfantry.register_output("/referee/serial", referee_serial_);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle()
                        .set_reversed());

            for (auto& motor : chassis_joint_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                        .set_reduction_ratio(1.0)
                        .set_reversed()
                        .enable_multi_turn_angle());

            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());

            deformableInfantry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        ~LeftBoard() override {
            stop_handling_events();
            if (event_thread_.joinable()) {
                event_thread_.join();
            }
        }

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
        }

        void command_update() {
            uint16_t can_commands[4];

            can_commands[0] = chassis_wheel_motors_[0].generate_command();
            can_commands[1] = chassis_joint_motors_[0].generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = chassis_steer_motors_[0].generate_command();
            can_commands[1] = 0;
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = chassis_wheel_motors_[1].generate_command();
            can_commands[1] = chassis_joint_motors_[1].generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = 0;
            can_commands[1] = chassis_steer_motors_[1].generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));
            transmit_buffer_.trigger_transmission();
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
                return;

            if (can_id == 0x201) chassis_wheel_motors_[0].store_status(can_data);
            else if (can_id == 0x202) chassis_joint_motors_[0].store_status(can_data);
            else if (can_id == 0x205) chassis_steer_motors_[0].store_status(can_data);
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
                return;

            if (can_id == 0x201) chassis_wheel_motors_[1].store_status(can_data);
            else if (can_id == 0x202) chassis_joint_motors_[1].store_status(can_data);
            else if (can_id == 0x206) chassis_steer_motors_[1].store_status(can_data);
        }

        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            referee_ring_buffer_receive_.emplace_back_multi(
                [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

    private:
        device::Bmi088 imu_;
        device::Dr16 dr16_;
        device::DjiMotor chassis_wheel_motors_[2];
        device::DjiMotor chassis_steer_motors_[2];
        device::DjiMotor chassis_joint_motors_[2];
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
    };

    class RightBoard final : private librmcs::client::CBoard {
    public:
        friend class DeformableInfantry;

        explicit RightBoard(
            DeformableInfantry& deformableInfantry,
            DeformableInfantryCommand& deformableInfantry_command,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , chassis_wheel_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_back_wheel"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_front_wheel"}}
            , chassis_steer_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_back_steering"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_front_steering"}}
            , chassis_joint_motors_{
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_back_joint"},
                  device::DjiMotor{deformableInfantry, deformableInfantry_command,
                                   "/chassis/right_front_joint"}}
            , transmit_buffer_(*this, 32)
            , supercap_(deformableInfantry, deformableInfantry_command)
            , event_thread_([this]() { handle_events(); }) {

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle()
                        .set_reversed());

            for (auto& motor : chassis_joint_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                        .set_reduction_ratio(1.0)
                        .set_reversed()
                        .enable_multi_turn_angle());

            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(
                        deformableInfantry.get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
        }

        ~RightBoard() final {
            stop_handling_events();
            if (event_thread_.joinable()) {
                event_thread_.join();
            }
        }

        void update() {
            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();
            for (auto& motor : chassis_joint_motors_)
                motor.update_status();
            supercap_.update_status();
        }

        void command_update() {
            uint16_t can_commands[4];

            can_commands[0] = chassis_wheel_motors_[0].generate_command();
            can_commands[1] = chassis_joint_motors_[0].generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = chassis_steer_motors_[0].generate_command();
            can_commands[1] = 0;
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = chassis_wheel_motors_[1].generate_command();
            can_commands[1] = chassis_joint_motors_[1].generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = 0;
            can_commands[1] = chassis_steer_motors_[1].generate_command();
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

            uint16_t cap_cmd[4] = {0, 0, 0, static_cast<uint16_t>(supercap_.generate_command())};
            transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(cap_cmd));

            transmit_buffer_.trigger_transmission();
        }

        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
                return;

            if (can_id == 0x201) chassis_wheel_motors_[0].store_status(can_data);
            else if (can_id == 0x202) chassis_joint_motors_[0].store_status(can_data);
            else if (can_id == 0x205) chassis_steer_motors_[0].store_status(can_data);
            else if (can_id == 0x300) supercap_.store_status(can_data);
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
                return;

            if (can_id == 0x201) chassis_wheel_motors_[1].store_status(can_data);
            else if (can_id == 0x202) chassis_joint_motors_[1].store_status(can_data);
            else if (can_id == 0x206) chassis_steer_motors_[1].store_status(can_data);
        }

    private:
        device::DjiMotor chassis_wheel_motors_[2];
        device::DjiMotor chassis_steer_motors_[2];
        device::DjiMotor chassis_joint_motors_[2];
        TransmitBuffer transmit_buffer_;
        device::Supercap supercap_;
        std::thread event_thread_;
    };

    std::shared_ptr<DeformableInfantryCommand> deformable_infantry_command_;
    std::unique_ptr<LeftBoard> left_board_;
    std::unique_ptr<RightBoard> right_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr joints_calibrate_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantry, rmcs_executor::Component)
