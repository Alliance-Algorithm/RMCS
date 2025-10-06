#include <atomic>
#include <memory>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/benewake.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/gy614.hpp"
#include "hardware/device/hipnuc.hpp"
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
              create_partner_component<HeroCommand>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.16, 0.0, 0.15});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~Hero() override = default;

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
            , tf_(hero.tf_)
            , imu_(1000, 0.2, 0.0)
            , gy614_(hero, "/friction_wheels/temperature")
            , benewake_(hero, "/gimbal/auto_aim/laser_distance")
            , gimbal_pitch_motor_(
                  hero, hero_command, "/gimbal/pitch",
                  device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}
                      .set_encoder_zero_point(
                          static_cast<int>(hero.get_parameter("pitch_motor_zero_point").as_int())))
            // TODO: Bad CAN ID sequence, needs to be adjusted.
            , gimbal_friction_wheels_(
                  {hero, hero_command, "/gimbal/second_left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.)},
                  {hero, hero_command, "/gimbal/second_right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()},
                  {hero, hero_command, "/gimbal/first_left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.)},
                  {hero, hero_command, "/gimbal/first_right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()})
            , gimbal_scope_motor_(
                  hero, hero_command, "/gimbal/scope",
                  device::DjiMotor::Config{device::DjiMotor::Type::M2006})
            , gimbal_player_viewer_motor_(
                  hero, hero_command, "/gimbal/player_viewer",
                  device::LkMotor::Config{device::LkMotor::Type::MG4005E_I10})
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                //     std::numbers::pi, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(x, y, z);
            });

            hero.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            hero.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            external_imu_thread_ = std::jthread([this, &hero](const std::stop_token& stop_token) {
                external_imu_thread_main(
                    stop_token, hero.get_parameter("external_imu_port").as_string(),
                    hero.get_logger());
            });
        }

        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
            external_imu_thread_.request_stop();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            if (external_imu_available_.load(std::memory_order::relaxed)) {
                external_imu_.update_status();
                gimbal_imu_pose.slerp(0.001, external_imu_.quaternion());
                imu_.q0() = gimbal_imu_pose.w();
                imu_.q1() = gimbal_imu_pose.x();
                imu_.q2() = gimbal_imu_pose.y();
                imu_.q3() = gimbal_imu_pose.z();
            }

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            gy614_.update_status();
            benewake_.update_status();

            *gimbal_yaw_velocity_imu_ = imu_.gz();
            *gimbal_pitch_velocity_imu_ = imu_.gy();

            gimbal_pitch_motor_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            for (auto& motor : gimbal_friction_wheels_)
                motor.update_status();

            gimbal_player_viewer_motor_.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];
            for (int i = 0; i < 4; i++)
                batch_commands[i] = gimbal_friction_wheels_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.add_can2_transmission(0x142, gimbal_pitch_motor_.generate_command());

            batch_commands[0] = gimbal_scope_motor_.generate_command();
            batch_commands[1] = 0;
            batch_commands[2] = 0;
            batch_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x1ff, std::bit_cast<uint64_t>(batch_commands));

            transmit_buffer_.add_can2_transmission(
                0x141, gimbal_player_viewer_motor_.generate_angle_command(
                           gimbal_player_viewer_motor_.control_angle()));

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

            if (can_id == 0x142) {
                gimbal_pitch_motor_.store_status(can_data);
            } else if (can_id == 0x141) {
                gimbal_player_viewer_motor_.store_status(can_data);
            }
        }

        void uart1_receive_callback(const std::byte* data, uint8_t length) override {
            benewake_.store_status(data, length);
        }

        void uart2_receive_callback(const std::byte* data, uint8_t length) override {
            gy614_.store_status(data, length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        void external_imu_thread_main(
            const std::stop_token& stop_token, const std::string& port_name,
            const rclcpp::Logger& logger) {
            try {
                serial::Serial serial{port_name, 115200, serial::Timeout::simpleTimeout(10)};
                rmcs_utility::FpsCounter fps_counter;

                while (!stop_token.stop_requested()) {
                    if (external_imu_.store_status<uint8_t>(serial) && fps_counter.count()) {
                        bool available = fps_counter.fps() > 350.0;
                        if (!available)
                            RCLCPP_WARN(logger, "External IMU low FPS: %.2f", fps_counter.fps());
                        else if (!external_imu_available_.load(std::memory_order::relaxed))
                            RCLCPP_INFO(
                                logger, "External IMU now available with FPS: %.2f",
                                fps_counter.fps());
                        external_imu_available_.store(available, std::memory_order::relaxed);
                    }
                }
            } catch (const std::exception& e) {
                external_imu_available_.store(false, std::memory_order::relaxed);
                RCLCPP_ERROR(logger, "Exception in external IMU thread: %s", e.what());
            }
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088 imu_;
        device::Gy614 gy614_;
        device::Benewake benewake_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        device::LkMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_friction_wheels_[4];

        device::DjiMotor gimbal_scope_motor_;
        device::LkMotor gimbal_player_viewer_motor_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

        rmcs_core::hardware::device::Hipnuc external_imu_;
        std::atomic<bool> external_imu_available_ = false;
        std::jthread external_imu_thread_;
    };

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class Hero;
        explicit BottomBoard(Hero& hero, HeroCommand& hero_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(1000, 0.2, 0.0)
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
                  device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}
                      .set_reversed()
                      .enable_multi_turn_angle())
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

            hero.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            dr16_.update_status();

            *chassis_yaw_velocity_imu_ = imu_.gz();

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

            transmit_buffer_.add_can1_transmission(
                0x141, gimbal_bullet_feeder_.generate_torque_command(
                           gimbal_bullet_feeder_.control_torque()));

            transmit_buffer_.add_can2_transmission(0x141, gimbal_yaw_motor_.generate_command());

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
            } else if (can_id == 0x141) {
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
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 imu_;
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> chassis_yaw_velocity_imu_;

        device::Dr16 dr16_;

        device::DjiMotor chassis_wheel_motors_[4];
        device::Supercap supercap_;

        device::LkMotor gimbal_yaw_motor_;

        device::LkMotor gimbal_bullet_feeder_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Hero, rmcs_executor::Component)