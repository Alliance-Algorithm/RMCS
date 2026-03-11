#include <cstddef>
#include <cstring>
#include <memory>
#include <span>
#include <string_view>
#include <tuple>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <rmcs_utility/tick_timer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/benewake.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class SteeringHero
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SteeringHero()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<SteeringHeroCommand>(
                  get_component_name() + "_command", *this)) {

        register_output("/tf", tf_);
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.16, 0.0, 0.15});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());

        temperature_logging_timer_.reset(1000);
    }

    SteeringHero(const SteeringHero&) = delete;
    SteeringHero& operator=(const SteeringHero&) = delete;
    SteeringHero(SteeringHero&&) = delete;
    SteeringHero& operator=(SteeringHero&&) = delete;

    ~SteeringHero() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();

        if (temperature_logging_timer_.tick()) {
            temperature_logging_timer_.reset(1000);
            RCLCPP_INFO(
                get_logger(),
                "Temperature: pitch: %.1f, top_yaw: %.1f, bottom_yaw: %.1f, feeder: %.1f",
                top_board_->gimbal_pitch_motor_.temperature(),
                top_board_->gimbal_top_yaw_motor_.temperature(),
                bottom_board_->gimbal_bottom_yaw_motor_.temperature(),
                top_board_->gimbal_bullet_feeder_.temperature());
        }
    }

    void command_update() {
        top_board_->command_update();
        bottom_board_->command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New bottom yaw offset: %ld",
            bottom_board_->gimbal_bottom_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New top yaw offset: %ld",
            top_board_->gimbal_top_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New viewer offset: %ld",
            top_board_->gimbal_player_viewer_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left front steering offset: %d",
            bottom_board_->chassis_steering_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left back steering offset: %d",
            bottom_board_->chassis_steering_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right back steering offset: %d",
            bottom_board_->chassis_steering_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right front steering offset: %d",
            bottom_board_->chassis_steering_motors_[3].calibrate_zero_point());
    }

    class SteeringHeroCommand : public rmcs_executor::Component {
    public:
        explicit SteeringHeroCommand(SteeringHero& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

    private:
        SteeringHero& hero_;
    };
    std::shared_ptr<SteeringHeroCommand> command_component_;

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class SteeringHero;
        explicit TopBoard(
            SteeringHero& hero, SteeringHeroCommand& hero_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , tf_(hero.tf_)
            , imu_(1000, 0.2, 0.0)
            , benewake_(hero, "/gimbal/auto_aim/laser_distance")
            , gimbal_top_yaw_motor_(
                  hero, hero_command, "/gimbal/top_yaw",
                  device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}
                      .set_encoder_zero_point(
                          static_cast<int>(
                              hero.get_parameter("top_yaw_motor_zero_point").as_int())))
            , gimbal_pitch_motor_(
                  hero, hero_command, "/gimbal/pitch",
                  device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}
                      .set_encoder_zero_point(
                          static_cast<int>(hero.get_parameter("pitch_motor_zero_point").as_int())))
            , gimbal_friction_wheels_(
                  {hero, hero_command, "/gimbal/second_left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(
                       1.)},
                  {hero, hero_command, "/gimbal/first_left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(
                       1.)},
                  {hero, hero_command, "/gimbal/first_right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()},
                  {hero, hero_command, "/gimbal/second_right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                       .set_reduction_ratio(1.)
                       .set_reversed()})
            , gimbal_bullet_feeder_(
                  hero, hero_command, "/gimbal/bullet_feeder",
                  device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}
                      .set_reversed()
                      .enable_multi_turn_angle())
            , gimbal_scope_motor_(
                  hero, hero_command, "/gimbal/scope",
                  device::DjiMotor::Config{device::DjiMotor::Type::kM2006})
            , gimbal_player_viewer_motor_(
                  hero, hero_command, "/gimbal/player_viewer",
                  device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}
                      .set_encoder_zero_point(
                          static_cast<int>(hero.get_parameter("viewer_motor_zero_point").as_int()))
                      .set_reversed()) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                //     std::numbers::pi, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(-x, -y, z);
            });

            hero.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            hero.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        }

        TopBoard(const TopBoard&) = delete;
        TopBoard& operator=(const TopBoard&) = delete;
        TopBoard(TopBoard&&) = delete;
        TopBoard& operator=(TopBoard&&) = delete;

        ~TopBoard() final = default;

        void update() {
            imu_.update_status();
            const Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            benewake_.update_status();

            *gimbal_yaw_velocity_imu_ = imu_.gz();
            *gimbal_pitch_velocity_imu_ = imu_.gy();

            gimbal_top_yaw_motor_.update_status();

            gimbal_pitch_motor_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            gimbal_player_viewer_motor_.update_status();
            tf_->set_state<rmcs_description::PitchLink, rmcs_description::ViewerLink>(
                gimbal_player_viewer_motor_.angle());

            gimbal_scope_motor_.update_status();

            for (auto& motor : gimbal_friction_wheels_)
                motor.update_status();

            gimbal_bullet_feeder_.update_status();
        }

        void command_update() {
            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       gimbal_friction_wheels_[0].generate_command(),
                                       gimbal_friction_wheels_[1].generate_command(),
                                       gimbal_friction_wheels_[2].generate_command(),
                                       gimbal_friction_wheels_[3].generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x141,
                .can_data = gimbal_bullet_feeder_
                                .generate_torque_command(gimbal_bullet_feeder_.control_torque())
                                .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       gimbal_scope_motor_.generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x143,
                .can_data =
                    gimbal_player_viewer_motor_
                        .generate_velocity_command(gimbal_player_viewer_motor_.control_velocity())
                        .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x141,
                .can_data = gimbal_top_yaw_motor_.generate_command().as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x142,
                .can_data = gimbal_pitch_motor_.generate_command().as_bytes(),
            });
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            auto can_id = data.can_id;
            if (can_id == 0x201) {
                gimbal_friction_wheels_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                gimbal_friction_wheels_[1].store_status(data.can_data);
            } else if (can_id == 0x203) {
                gimbal_friction_wheels_[2].store_status(data.can_data);
            } else if (can_id == 0x204) {
                gimbal_friction_wheels_[3].store_status(data.can_data);
            } else if (can_id == 0x141) {
                gimbal_bullet_feeder_.store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            auto can_id = data.can_id;
            if (can_id == 0x141) {
                gimbal_top_yaw_motor_.store_status(data.can_data);
            } else if (can_id == 0x142) {
                gimbal_pitch_motor_.store_status(data.can_data);
            } else if (can_id == 0x143) {
                gimbal_player_viewer_motor_.store_status(data.can_data);
            } else if (can_id == 0x201) {
                gimbal_scope_motor_.store_status(data.can_data);
            }
        }

        void uart2_receive_callback(const librmcs::data::UartDataView& data) override {
            benewake_.store_status(data.uart_data.data(), data.uart_data.size());
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
        device::Benewake benewake_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        device::LkMotor gimbal_top_yaw_motor_;
        device::LkMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_friction_wheels_[4];
        device::LkMotor gimbal_bullet_feeder_;

        device::DjiMotor gimbal_scope_motor_;
        device::LkMotor gimbal_player_viewer_motor_;
    };

    class BottomBoard final : private librmcs::agent::CBoard {
    public:
        friend class SteeringHero;
        explicit BottomBoard(
            SteeringHero& hero, SteeringHeroCommand& hero_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , imu_(1000, 0.2, 0.0)
            , tf_(hero.tf_)
            , dr16_(hero)
            , chassis_steering_motors_(
                  {hero, hero_command, "/chassis/left_front_steering",
                   device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                       .set_encoder_zero_point(
                           static_cast<int>(hero.get_parameter("left_front_zero_point").as_int()))
                       .set_reversed()},
                  {hero, hero_command, "/chassis/left_back_steering",
                   device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                       .set_encoder_zero_point(
                           static_cast<int>(hero.get_parameter("left_back_zero_point").as_int()))
                       .set_reversed()},
                  {hero, hero_command, "/chassis/right_back_steering",
                   device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                       .set_encoder_zero_point(
                           static_cast<int>(hero.get_parameter("right_back_zero_point").as_int()))
                       .set_reversed()},
                  {hero, hero_command, "/chassis/right_front_steering",
                   device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                       .set_encoder_zero_point(
                           static_cast<int>(hero.get_parameter("right_front_zero_point").as_int()))
                       .set_reversed()})
            , chassis_wheel_motors_(
                  {hero, hero_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                       .set_reversed()
                       .set_reduction_ratio(2232. / 169.)},
                  {hero, hero_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                       .set_reversed()
                       .set_reduction_ratio(2232. / 169.)},
                  {hero, hero_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                       .set_reversed()
                       .set_reduction_ratio(2232. / 169.)},
                  {hero, hero_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                       .set_reversed()
                       .set_reduction_ratio(2232. / 169.)})
            , supercap_(hero, hero_command)
            , gimbal_bottom_yaw_motor_(
                  hero, hero_command, "/gimbal/bottom_yaw",
                  device::LkMotor::Config{device::LkMotor::Type::kMG6012Ei8}
                      .set_reversed()
                      .set_encoder_zero_point(
                          static_cast<int>(
                              hero.get_parameter("bottom_yaw_motor_zero_point").as_int()))) {

            hero.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart1_transmit({
                    .uart_data = std::span<const std::byte>{buffer, size}
                });
                return size;
            };

            hero.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);

            hero.register_output(
                "/chassis/powermeter/control_enable", powermeter_control_enabled_, false);
            hero.register_output(
                "/chassis/powermeter/charge_power_limit", powermeter_charge_power_limit_, 0.);
        }

        BottomBoard(const BottomBoard&) = delete;
        BottomBoard& operator=(const BottomBoard&) = delete;
        BottomBoard(BottomBoard&&) = delete;
        BottomBoard& operator=(BottomBoard&&) = delete;

        ~BottomBoard() final = default;

        void update() {
            imu_.update_status();
            dr16_.update_status();
            supercap_.update_status();

            *chassis_yaw_velocity_imu_ = imu_.gz();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steering_motors_)
                motor.update_status();
            gimbal_bottom_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_bottom_yaw_motor_.angle());
        }

        void command_update() {
            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       chassis_wheel_motors_[0].generate_command(),
                                       chassis_wheel_motors_[1].generate_command(),
                                       chassis_wheel_motors_[2].generate_command(),
                                       chassis_wheel_motors_[3].generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       supercap_.generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       chassis_steering_motors_[0].generate_command(),
                                       chassis_steering_motors_[1].generate_command(),
                                       chassis_steering_motors_[2].generate_command(),
                                       chassis_steering_motors_[3].generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x141,
                .can_data = gimbal_bottom_yaw_motor_.generate_command().as_bytes(),
            });
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            auto can_id = data.can_id;
            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x203) {
                chassis_wheel_motors_[2].store_status(data.can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors_[3].store_status(data.can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            auto can_id = data.can_id;
            if (can_id == 0x205) {
                chassis_steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x206) {
                chassis_steering_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x207) {
                chassis_steering_motors_[2].store_status(data.can_data);
            } else if (can_id == 0x208) {
                chassis_steering_motors_[3].store_status(data.can_data);
            } else if (can_id == 0x141) {
                gimbal_bottom_yaw_motor_.store_status(data.can_data);
            }
        }

        void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
            const auto* uart_data = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_n(
                [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
                data.uart_data.size());
        }

        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        device::Bmi088 imu_;
        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> chassis_yaw_velocity_imu_;

        OutputInterface<bool> powermeter_control_enabled_;
        OutputInterface<double> powermeter_charge_power_limit_;

        device::Dr16 dr16_;

        device::DjiMotor chassis_steering_motors_[4];
        device::DjiMotor chassis_wheel_motors_[4];
        device::Supercap supercap_;

        device::LkMotor gimbal_bottom_yaw_motor_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;

    rmcs_utility::TickTimer temperature_logging_timer_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SteeringHero, rmcs_executor::Component)
