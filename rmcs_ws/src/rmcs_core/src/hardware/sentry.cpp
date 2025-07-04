#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"
#include "librmcs/client/cboard.hpp"
#include "rmcs_utility/navigation_util.hpp"

#include <game_stage.hpp>
#include <robot_id.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial_interface.hpp>

#include <memory>

namespace rmcs_core::hardware {

class Sentry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Sentry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<SentryCommand>(get_component_name() + "_command", *this)) {
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
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.03403, 0.0, 0.08285});
    }
    ~Sentry() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();

        // 右摇杆由中位向上拨时，发送录制雷达消息的请求
        // 当配置文件 (slam.yaml) 选择取消录制时，则不会进行录制
        {
            const auto switch_status = bottom_board_->switch_right();

            using namespace rmcs_msgs;
            if (last_switch_right_ == Switch::MIDDLE && switch_status == Switch::UP)
                rmcs_utility::switch_record(*this, true);

            if (last_switch_right_ == Switch::UP && switch_status == Switch::MIDDLE)
                rmcs_utility::switch_record(*this, false);

            last_switch_right_ = switch_status;
        }
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
            get_logger(), "[steer calibration] New lf offset: %d",
            bottom_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New lb offset: %d",
            bottom_board_->chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New rb offset: %d",
            bottom_board_->chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New rf offset: %d",
            bottom_board_->chassis_steer_motors_[3].calibrate_zero_point());
    }

    // 裁判系统等信息更新在 Sentry 中，故使用新组件来接收消息，防止循环引用
    class SentryCommand : public rmcs_executor::Component {
    public:
        explicit SentryCommand(Sentry& sentry)
            : sentry_(sentry) {
            register_input("/referee/game/stage", game_stage_, false);
            register_input("/referee/id", robot_id_, false);
        }

        void update() override {
            sentry_.command_update();

            // 比赛进入裁判系统自检阶段时，重置 SLAM 并进行重定位
            do {
                using namespace rmcs_msgs;
                using namespace rmcs_utility;

                if (!game_stage_.ready() || !robot_id_.ready())
                    break;

                const auto is_entry_game = bool{
                    *game_stage_ == GameStage::REFEREE_CHECK && last_game_stage_ != *game_stage_};
                if (is_entry_game) {
                    update_robot_side(sentry_, robot_id_->color());
                    switch_record(sentry_, true);
                    initialize_navigation(sentry_);
                    RCLCPP_INFO(
                        sentry_.get_logger(),
                        "Entry game, initialize navigation now and start lidar recorder");
                }

                const auto is_exit_game =
                    bool{*game_stage_ == GameStage::SETTLING && last_game_stage_ != *game_stage_};
                if (is_exit_game) {
                    switch_record(sentry_, false);
                    RCLCPP_INFO(sentry_.get_logger(), "Exit game, stop lidar recorder");
                }

                const auto is_unknown_status =
                    bool{*game_stage_ == GameStage::UNKNOWN && last_game_stage_ != *game_stage_};
                if (is_unknown_status) {
                    switch_record(sentry_, false);
                    RCLCPP_INFO(sentry_.get_logger(), "Unknown status, stop lidar recorder");
                }

                last_game_stage_ = *game_stage_;
            } while (false);
        }

    private:
        Sentry& sentry_;

        InputInterface<rmcs_msgs::GameStage> game_stage_;
        rmcs_msgs::GameStage last_game_stage_;

        InputInterface<rmcs_msgs::RobotId> robot_id_;
    };

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class Sentry;
        explicit TopBoard(Sentry& sentry, SentryCommand& sentry_command, int usb_pid = -1)
            : CBoard(usb_pid)
            , tf_(sentry.tf_)
            , bmi088_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(sentry, sentry_command, "/gimbal/pitch")
            , gimbal_left_friction_(sentry, sentry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(sentry, sentry_command, "/gimbal/right_friction")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {
            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::MG4010E_I10}.set_encoder_zero_point(
                    static_cast<int>(sentry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.));

            sentry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            sentry.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_);

            bmi088_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise
                // use a matrix.

                // Eigen::AngleAxisd pitch_link_to_bmi088_link{
                //     std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_bmi088_link * Eigen::Vector3d{1,
                // 2, 3}; std::cout << mapping << std::endl;

                return std::make_tuple(-x, -y, z);
            });
        }
        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {
            bmi088_.update_status();
            Eigen::Quaterniond gimbal_bmi088_pose{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_bmi088_pose.conjugate());

            *gimbal_yaw_velocity_bmi088_   = bmi088_.gz();
            *gimbal_pitch_velocity_bmi088_ = bmi088_.gy();

            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            // RCLCPP_INFO(rclcpp::get_logger("a"), "%f", gimbal_left_friction_.velocity());

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());
        }

        void command_update() {
            uint16_t can_commands[4];
            can_commands[0] = gimbal_left_friction_.generate_command();
            can_commands[1] = gimbal_right_friction_.generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            transmit_buffer_.add_can1_transmission(
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

            if (can_id == 0x201) {
                gimbal_left_friction_.store_status(can_data);
            } else if (can_id == 0x202) {
                gimbal_right_friction_.store_status(can_data);
            } else {
                gimbal_pitch_motor_.store_status(can_data);
            }
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            (void)can_id;
            (void)can_data;
            (void)is_extended_can_id;
            (void)is_remote_transmission;
            (void)can_data_length;
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
        friend class Sentry;

        explicit BottomBoard(Sentry& sentry, SentryCommand& sentry_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(1000, 0.2, 0.0)
            , tf_(sentry.tf_)
            , dr16_(sentry)
            , gimbal_yaw_motor_(sentry, sentry_command, "/gimbal/yaw")
            , gimbal_bullet_feeder_(sentry, sentry_command, "/gimbal/bullet_feeder")
            , chassis_wheel_motors_(
                  {sentry, sentry_command, "/chassis/left_front_wheel"},
                  {sentry, sentry_command, "/chassis/left_back_wheel"},
                  {sentry, sentry_command, "/chassis/right_back_wheel"},
                  {sentry, sentry_command, "/chassis/right_front_wheel"})
            , chassis_steer_motors_(
                  {sentry, sentry_command, "/chassis/left_front_steering"},
                  {sentry, sentry_command, "/chassis/left_back_steering"},
                  {sentry, sentry_command, "/chassis/right_back_steering"},
                  {sentry, sentry_command, "/chassis/right_front_steering"})
            , supercap_(sentry, sentry_command)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            sentry.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };
            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}.set_encoder_zero_point(
                    static_cast<int>(sentry.get_parameter("yaw_motor_zero_point").as_int())));

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                        .set_reversed()
                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle());
            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            sentry.register_output("/chassis/yaw/velocity", chassis_yaw_velocity);
        }
        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {

            imu_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_) {
                motor.update_status();
                // RCLCPP_INFO(rclcpp::get_logger("hello"), "%f\n", motor.angle());
            }

            dr16_.update_status();
            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());

            gimbal_bullet_feeder_.update_status();
            *chassis_yaw_velocity = imu_.gz();

            supercap_.update_status();
        }

        void command_update() {
            uint16_t batch_commands[4];

            for (int i = 0; i < 4; i++)
                batch_commands[i] = chassis_wheel_motors_[i].generate_command();
            transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));
            transmit_buffer_.add_can2_transmission(0x1FF, gimbal_bullet_feeder_.generate_command());

            batch_commands[0] = 0;
            batch_commands[1] = 0;
            batch_commands[2] = 0;
            batch_commands[3] = supercap_.generate_command();
            transmit_buffer_.add_can2_transmission(0x1fe, std::bit_cast<uint64_t>(batch_commands));

            for (int i = 0; i < 4; i++) {
                batch_commands[i] = chassis_steer_motors_[i].generate_command();
            }
            // std::cerr << chassis_steer_motors_[1].generate_command() << std::endl;

            transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(batch_commands));
            static uint8_t yaw_count = 0;
            yaw_count                = (yaw_count + 1) % 2;
            if (yaw_count % 2)
                transmit_buffer_.add_can1_transmission(
                    0x142,
                    gimbal_yaw_motor_.generate_torque_command(gimbal_yaw_motor_.control_torque()));

            transmit_buffer_.trigger_transmission();
        }

        rmcs_msgs::Switch switch_right() const { return dr16_.switch_right(); }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        void can2_receive_callback(
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
            } else if (can_id == 0x205) {
                auto& motor = gimbal_bullet_feeder_;
                motor.store_status(can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(can_data);
            }
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x205) {
                auto& motor = chassis_steer_motors_[0];
                motor.store_status(can_data);
            } else if (can_id == 0x206) {
                auto& motor = chassis_steer_motors_[1];
                motor.store_status(can_data);
            } else if (can_id == 0x207) {
                auto& motor = chassis_steer_motors_[2];
                motor.store_status(can_data);
            } else if (can_id == 0x208) {
                auto& motor = chassis_steer_motors_[3];
                motor.store_status(can_data);
            } else if (can_id == 0x142) {
                auto& motor = gimbal_yaw_motor_;
                motor.store_status(can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(can_data);
            }
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
        OutputInterface<rmcs_description::Tf>& tf_;

        device::Dr16 dr16_;
        device::LkMotor gimbal_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::DjiMotor chassis_steer_motors_[4];
        device::Supercap supercap_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        OutputInterface<double> chassis_yaw_velocity;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    std::shared_ptr<SentryCommand> command_component_;
    std::shared_ptr<TopBoard> top_board_;
    std::shared_ptr<BottomBoard> bottom_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Sentry, rmcs_executor::Component)
