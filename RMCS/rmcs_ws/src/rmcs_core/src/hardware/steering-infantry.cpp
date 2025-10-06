
#include <memory>

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

class SteeringInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SteeringInfantry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<SteeringInfantryCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;
        register_output("/tf", tf_);

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
        steers_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/steers/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                steers_calibrate_subscription_callback(std::move(msg));
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_top_board").as_int()));

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));

        using namespace rmcs_description;
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.06603, 0.0, 0.082});
    }
    ~SteeringInfantry() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();
    }

    void command_update() {

        top_board_->command_update();
        bottom_board_->command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %ld",
            bottom_board_->gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    void steers_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left front offset: %d",
            bottom_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left back offset: %d",
            bottom_board_->chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right back offset: %d",
            bottom_board_->chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right front offset: %d",
            bottom_board_->chassis_steer_motors_[3].calibrate_zero_point());
    }
    class SteeringInfantryCommand : public rmcs_executor::Component {
    public:
        explicit SteeringInfantryCommand(SteeringInfantry& steeringInfantry)
            : steeringInfantry(steeringInfantry) {}

        void update() override { steeringInfantry.command_update(); }

        SteeringInfantry& steeringInfantry;
    };

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class SteeringInfantry;
        explicit TopBoard(
            SteeringInfantry& steeringInfantry, SteeringInfantryCommand& steeringInfantry_command,
            int usb_pid = -1)
            : CBoard(usb_pid)
            , tf_(steeringInfantry.tf_)
            , bmi088_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(steeringInfantry, steeringInfantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(
                  steeringInfantry, steeringInfantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(
                  steeringInfantry, steeringInfantry_command, "/gimbal/right_friction")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {
            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::MG4010E_I10}.set_encoder_zero_point(
                    static_cast<int>(
                        steeringInfantry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.));
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());

            steeringInfantry.register_output(
                "/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            steeringInfantry.register_output(
                "/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_);

            bmi088_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise
                // use a matrix.

                // Eigen::AngleAxisd pitch_link_to_bmi088_link{
                //     std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_bmi088_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(x, y, z);
            });
        }
        ~TopBoard() {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {
            bmi088_.update_status();
            Eigen::Quaterniond gimbal_bmi088_pose{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_bmi088_pose.conjugate());

            *gimbal_yaw_velocity_bmi088_ = bmi088_.gz();
            *gimbal_pitch_velocity_bmi088_ = bmi088_.gy();

            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());
        }

        void command_update() {
            uint16_t can_commands[4];
            can_commands[2] = gimbal_left_friction_.generate_command();
            can_commands[3] = gimbal_right_friction_.generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            transmit_buffer_.add_can2_transmission(
                0x142, gimbal_pitch_motor_.generate_velocity_command(
                           gimbal_pitch_motor_.control_velocity()));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x203) {
                gimbal_left_friction_.store_status(can_data);
            } else if (can_id == 0x204) {
                gimbal_right_friction_.store_status(can_data);
            }
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x142)
                gimbal_pitch_motor_.store_status(can_data);
        }
        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_accelerometer_status(x, y, z);
        }
        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_gyroscope_status(x, y, z);
        }
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_bmi088_;

        device::Bmi088 bmi088_;
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };
    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class SteeringInfantry;

        explicit BottomBoard(
            SteeringInfantry& steeringInfantry, SteeringInfantryCommand& steeringInfantry_command,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(1000, 0.2, 0.0)
            , tf_(steeringInfantry.tf_)
            , dr16_(steeringInfantry)
            , gimbal_yaw_motor_(steeringInfantry, steeringInfantry_command, "/gimbal/yaw")
            , gimbal_bullet_feeder_(
                  steeringInfantry, steeringInfantry_command, "/gimbal/bullet_feeder")
            , chassis_wheel_motors_(
                  {steeringInfantry, steeringInfantry_command, "/chassis/left_front_wheel"},
                  {steeringInfantry, steeringInfantry_command, "/chassis/left_back_wheel"},
                  {steeringInfantry, steeringInfantry_command, "/chassis/right_back_wheel"},
                  {steeringInfantry, steeringInfantry_command, "/chassis/right_front_wheel"})
            , chassis_steer_motors_(
                  {steeringInfantry, steeringInfantry_command, "/chassis/left_front_steering"},
                  {steeringInfantry, steeringInfantry_command, "/chassis/left_back_steering"},
                  {steeringInfantry, steeringInfantry_command, "/chassis/right_back_steering"},
                  {steeringInfantry, steeringInfantry_command, "/chassis/right_front_steering"})
            , supercap_(steeringInfantry, steeringInfantry_command)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            steeringInfantry.register_output("/referee/serial", referee_serial_);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };
            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::MG4010E_I10}.set_encoder_zero_point(
                    static_cast<int>(
                        steeringInfantry.get_parameter("yaw_motor_zero_point").as_int())));

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M3508}

                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle()
                        .set_reversed());
            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steeringInfantry.get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steeringInfantry.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steeringInfantry.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steeringInfantry.get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            steeringInfantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }
        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gy();
            supercap_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();

            dr16_.update_status();
            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());

            gimbal_bullet_feeder_.update_status();
        }
        void command_update() {
            uint16_t batch_commands[4];
            for (int i = 0; i < 4; i++)
                batch_commands[i] = chassis_wheel_motors_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));
            transmit_buffer_.add_can1_transmission(0x1FF, gimbal_bullet_feeder_.generate_command());
            if (can_transmission_mode) {
                batch_commands[3] = supercap_.generate_command();
                transmit_buffer_.add_can1_transmission(
                    0x1FE, std::bit_cast<uint64_t>(batch_commands));
                transmit_buffer_.trigger_transmission();
                for (int i = 0; i < 4; i++) {
                    batch_commands[i] = chassis_steer_motors_[i].generate_command();
                }
                transmit_buffer_.add_can2_transmission(
                    0x1FE, std::bit_cast<uint64_t>(batch_commands));
                transmit_buffer_.trigger_transmission();
            } else {
                transmit_buffer_.add_can2_transmission(
                    0x142, gimbal_yaw_motor_.generate_velocity_command(
                               gimbal_yaw_motor_.control_velocity() - imu_.gy()));
                transmit_buffer_.trigger_transmission();
            }
            can_transmission_mode = !can_transmission_mode;
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x201)
                chassis_wheel_motors_[0].store_status(can_data);
            else if (can_id == 0x202)
                chassis_wheel_motors_[1].store_status(can_data);
            else if (can_id == 0x203)
                chassis_wheel_motors_[2].store_status(can_data);
            else if (can_id == 0x204)
                chassis_wheel_motors_[3].store_status(can_data);
            else if (can_id == 0x205)
                gimbal_bullet_feeder_.store_status(can_data);
            else if (can_id == 0x300)
                supercap_.store_status(can_data);
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x142)
                gimbal_yaw_motor_.store_status(can_data);
            else if (can_id == 0x205)
                chassis_steer_motors_[0].store_status(can_data);
            else if (can_id == 0x206)
                chassis_steer_motors_[1].store_status(can_data);
            else if (can_id == 0x207)
                chassis_steer_motors_[2].store_status(can_data);
            else if (can_id == 0x208)
                chassis_steer_motors_[3].store_status(can_data);
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
        bool can_transmission_mode = true;
        device::Bmi088 imu_;
        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<bool> powermeter_control_enabled_;
        OutputInterface<double> powermeter_charge_power_limit_;

        device::Dr16 dr16_;
        device::LkMotor gimbal_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::DjiMotor chassis_steer_motors_[4];
        device::Supercap supercap_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        OutputInterface<double> chassis_yaw_velocity_imu_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    std::shared_ptr<SteeringInfantryCommand> command_component_;
    std::shared_ptr<TopBoard> top_board_;
    std::shared_ptr<BottomBoard> bottom_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SteeringInfantry, rmcs_executor::Component)