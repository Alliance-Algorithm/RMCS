#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/client/cboard.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

class Hero
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Hero()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<HeroCommand>(get_component_name() + "_command", *this))
        , top_board_(
              *this, *command_component_,
              static_cast<int>(get_parameter("usb_pid_top_board").as_int()))
        , bottom_board_(
              *this, *command_component_,
              static_cast<int>(get_parameter("usb_pid_bottom_board").as_int())) {

        register_output("/tf", tf_);
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::ImuLink>(
            Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitZ()});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
    }

    ~Hero() override = default;

    void update() override {
        top_board_.update();
        bottom_board_.update();
    }

    void command_update() {
        top_board_.command_update();
        bottom_board_.command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %ld",
            bottom_board_.gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_.gimbal_pitch_motor_.calibrate_zero_point());
    }

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    class HeroCommand : public rmcs_executor::Component {
    public:
        explicit HeroCommand(Hero& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

        Hero& hero_;
    };
    std::shared_ptr<HeroCommand> command_component_;

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class Hero;
        explicit TopBoard(Hero& hero, HeroCommand& hero_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , bmi088_(1000, 0.2, 0.0)
            , tf_(hero.tf_)
            , gimbal_pitch_motor_(hero, hero_command, "/gimbal/pitch")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {
            (void)hero;
            (void)hero_command;

            hero.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            hero.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}.set_encoder_zero_point(
                    static_cast<int>(hero.get_parameter("pitch_motor_zero_point").as_int())));
        }

        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            bmi088_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
            tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());
            *gimbal_yaw_velocity_imu_   = bmi088_.gz();
            *gimbal_pitch_velocity_imu_ = -bmi088_.gy();

            gimbal_pitch_motor_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());
        }

        void command_update() {
            transmit_buffer_.add_can2_transmission(0x141, gimbal_pitch_motor_.generate_command());

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141) {
                gimbal_pitch_motor_.store_status(can_data);
            }
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 bmi088_;
        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        device::LkMotor gimbal_pitch_motor_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    } top_board_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class Hero;
        explicit BottomBoard(Hero& hero, HeroCommand& hero_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , bmi088_(1000, 0.2, 0.0)
            , tf_(hero.tf_)
            , dr16_(hero)
            , chassis_wheel_motors_(
                  {hero, hero_command, "/chassis/left_front_wheel"},
                  {hero, hero_command, "/chassis/left_back_wheel"},
                  {hero, hero_command, "/chassis/right_back_wheel"},
                  {hero, hero_command, "/chassis/right_front_wheel"})
            , gimbal_yaw_motor_(hero, hero_command, "/gimbal/yaw")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                                    .enable_multi_turn_angle());
            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}.set_encoder_zero_point(
                    static_cast<int>(hero.get_parameter("yaw_motor_zero_point").as_int())));
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            bmi088_.update_status();

            dr16_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();

            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
        }

        void command_update() {
            uint16_t can_commands[4];

            for (int i = 0; i < 4; i++)
                can_commands[i] = chassis_wheel_motors_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            // Use the chassis angular velocity as feedforward input for yaw velocity control.
            // This approach currently works only on Hero, as it utilizes motor angular velocity
            // instead of gyro angular velocity for closed-loop control.
            transmit_buffer_.add_can2_transmission(
                0x141, gimbal_yaw_motor_.generate_velocity_command(
                           gimbal_yaw_motor_.control_velocity() - bmi088_.gz()));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                auto& motor = chassis_wheel_motors_[0];
                motor.store_status(can_data);
            } else if (can_id == 0x202) {
                auto& motor = chassis_wheel_motors_[1];
                motor.store_status(can_data);
            } else if (can_id == 0x203) {
                auto& motor = chassis_wheel_motors_[2];
                motor.store_status(can_data);
            } else if (can_id == 0x204) {
                auto& motor = chassis_wheel_motors_[3];
                motor.store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141) {
                gimbal_yaw_motor_.store_status(can_data);
            }
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 bmi088_;
        OutputInterface<rmcs_description::Tf>& tf_;

        device::Dr16 dr16_;

        device::DjiMotor chassis_wheel_motors_[4];

        device::LkMotor gimbal_yaw_motor_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    } bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Hero, rmcs_executor::Component)