#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/client/cboard.hpp>

#include "hardware/device/benewake.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/gy614.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

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
        using namespace rmcs_description;

        auto cboard_init_q_w = get_parameter("cboard_init_q_w").as_double();
        auto cboard_init_q_x = get_parameter("cboard_init_q_x").as_double();
        auto cboard_init_q_y = get_parameter("cboard_init_q_y").as_double();
        auto cboard_init_q_z = get_parameter("cboard_init_q_z").as_double();

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, ImuLink>(
            Eigen::Quaterniond{cboard_init_q_w, cboard_init_q_x, cboard_init_q_y, cboard_init_q_z});

        auto camera_q_w = get_parameter("camera_q_w").as_double();
        auto camera_q_x = get_parameter("camera_q_x").as_double();
        auto camera_q_y = get_parameter("camera_q_y").as_double();
        auto camera_q_z = get_parameter("camera_q_z").as_double();
        auto camera_t_x = get_parameter("camera_t_x").as_double();
        auto camera_t_y = get_parameter("camera_t_y").as_double();
        auto camera_t_z = get_parameter("camera_t_z").as_double();

        auto camera_q = Eigen::Quaterniond{camera_q_w, camera_q_x, camera_q_y, camera_q_z};
        auto camera_t = Eigen::Vector3d{camera_t_x, camera_t_y, camera_t_z};
        auto iso      = Eigen::Isometry3d::Identity();
        iso.rotate(camera_q);
        iso.pretranslate(camera_t);
        tf_->set_transform<PitchLink, CameraLink>(iso);

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
            , benewake_(hero)
            , gy614_(hero)
            , tf_(hero.tf_)
            , gimbal_pitch_motor_(
                  hero, hero_command, "/gimbal/pitch",
                  device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}
                      .set_encoder_zero_point(
                          static_cast<int>(hero.get_parameter("pitch_motor_zero_point").as_int())))
            , gimbal_friction_wheels_(
                  {hero, hero_command, "/gimbal/first_left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.)},
                  {hero, hero_command, "/gimbal/second_left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.)},
                  {hero, hero_command, "/gimbal/first_right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()},
                  {hero, hero_command, "/gimbal/second_right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()})
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            hero.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            hero.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        }

        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            bmi088_.update_status();
            gy614_.update();
            benewake_.update();

            Eigen::Quaterniond gimbal_imu_pose{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
            tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());
            *gimbal_yaw_velocity_imu_   = bmi088_.gz();
            *gimbal_pitch_velocity_imu_ = -bmi088_.gy();

            gimbal_pitch_motor_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            for (auto& motor : gimbal_friction_wheels_)
                motor.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];
            for (int i = 0; i < 4; i++)
                batch_commands[i] = gimbal_friction_wheels_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.add_can2_transmission(0x141, gimbal_pitch_motor_.generate_torque_command(
                gimbal_pitch_motor_.control_velocity()));
    

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                gimbal_friction_wheels_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                gimbal_friction_wheels_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                gimbal_friction_wheels_[2].store_status(can_data);
            } else if (can_id == 0x204) {
                gimbal_friction_wheels_[3].store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141) {
                gimbal_pitch_motor_.store_status(can_data);
            }
        }

        void uart1_receive_callback(const std::byte* data, uint8_t length) override {
            benewake_.store_status(data, length);
        }

        void uart2_receive_callback(const std::byte* data, uint8_t length) override {
            gy614_.store_status(data, length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 bmi088_;
        device::Benewake benewake_;
        device::Gy614 gy614_;
        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        device::LkMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_friction_wheels_[4];

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
                  {hero, hero_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {hero, hero_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {hero, hero_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {hero, hero_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}})
            , supercap_(hero, hero_command)
            , gimbal_yaw_motor_(
                  hero, hero_command, "/gimbal/yaw",
                  device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}
                      .set_encoder_zero_point(
                          static_cast<int>(hero.get_parameter("yaw_motor_zero_point").as_int())))
            , gimbal_bullet_feeder_(
                  hero, hero_command, "/gimbal/bullet_feeder",
                  device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reversed())
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            hero.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };
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

            supercap_.update_status();

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

            batch_commands[0] = gimbal_bullet_feeder_.generate_command();
            batch_commands[1] = 0;
            batch_commands[2] = 0;
            batch_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x1FF, std::bit_cast<uint64_t>(batch_commands));

            // Use the chassis angular velocity as feedforward input for yaw velocity control.
            // This approach currently works only on Hero, as it utilizes motor angular velocity
            // instead of gyro angular velocity for closed-loop control.
            transmit_buffer_.add_can2_transmission(
                0x141, gimbal_yaw_motor_.generate_velocity_command(
                           gimbal_yaw_motor_.control_velocity() - bmi088_.gz()));

            batch_commands[0] = 0;
            batch_commands[1] = 0;
            batch_commands[2] = 0;
            batch_commands[3] = supercap_.generate_command();
            transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                chassis_wheel_motors_[2].store_status(can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors_[3].store_status(can_data);
            } else if (can_id == 0x205) {
                gimbal_bullet_feeder_.store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141) {
                gimbal_yaw_motor_.store_status(can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(can_data);
            }
        }

        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            referee_ring_buffer_receive_.emplace_back_multi(
                [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
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
        device::Supercap supercap_;

        device::LkMotor gimbal_yaw_motor_;

        device::DjiMotor gimbal_bullet_feeder_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    } bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Hero, rmcs_executor::Component)