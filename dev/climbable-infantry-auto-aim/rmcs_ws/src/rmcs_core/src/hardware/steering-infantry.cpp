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
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

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
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());

        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.06603, 0.0, 0.082});
    }

    SteeringInfantry(const SteeringInfantry&) = delete;
    SteeringInfantry& operator=(const SteeringInfantry&) = delete;
    SteeringInfantry(SteeringInfantry&&) = delete;
    SteeringInfantry& operator=(SteeringInfantry&&) = delete;

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
        explicit SteeringInfantryCommand(SteeringInfantry& steering_infantry)
            : steering_infantry(steering_infantry) {}

        void update() override { steering_infantry.command_update(); }

        SteeringInfantry& steering_infantry;
    };

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class SteeringInfantry;
        explicit TopBoard(
            SteeringInfantry& steering_infantry, SteeringInfantryCommand& steering_infantry_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , tf_(steering_infantry.tf_)
            , bmi088_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(steering_infantry, steering_infantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(
                  steering_infantry, steering_infantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(
                  steering_infantry, steering_infantry_command, "/gimbal/right_friction") {
            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(
                        steering_infantry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());

            steering_infantry.register_output(
                "/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            steering_infantry.register_output(
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

        TopBoard(const TopBoard&) = delete;
        TopBoard& operator=(const TopBoard&) = delete;
        TopBoard(TopBoard&&) = delete;
        TopBoard& operator=(TopBoard&&) = delete;

        ~TopBoard() override = default;

        void update() {
            bmi088_.update_status();
            const Eigen::Quaterniond gimbal_bmi088_pose{
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
            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       gimbal_left_friction_.generate_command(),
                                       gimbal_right_friction_.generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x142,
                .can_data = gimbal_pitch_motor_
                                .generate_velocity_command(gimbal_pitch_motor_.control_velocity())
                                .as_bytes(),
            });
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x203) {
                gimbal_left_friction_.store_status(data.can_data);
            } else if (can_id == 0x204) {
                gimbal_right_friction_.store_status(data.can_data);
            }
        }
        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x142)
                gimbal_pitch_motor_.store_status(data.can_data);
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
    };

    class BottomBoard final : private librmcs::agent::CBoard {
    public:
        friend class SteeringInfantry;

        explicit BottomBoard(
            SteeringInfantry& steering_infantry, SteeringInfantryCommand& steering_infantry_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , imu_(1000, 0.2, 0.0)
            , tf_(steering_infantry.tf_)
            , dr16_(steering_infantry)
            , gimbal_yaw_motor_(steering_infantry, steering_infantry_command, "/gimbal/yaw")
            , gimbal_bullet_feeder_(
                  steering_infantry, steering_infantry_command, "/gimbal/bullet_feeder")
            , chassis_wheel_motors_(
                  {steering_infantry, steering_infantry_command, "/chassis/left_front_wheel"},
                  {steering_infantry, steering_infantry_command, "/chassis/left_back_wheel"},
                  {steering_infantry, steering_infantry_command, "/chassis/right_back_wheel"},
                  {steering_infantry, steering_infantry_command, "/chassis/right_front_wheel"})
            , chassis_steer_motors_(
                  {steering_infantry, steering_infantry_command, "/chassis/left_front_steering"},
                  {steering_infantry, steering_infantry_command, "/chassis/left_back_steering"},
                  {steering_infantry, steering_infantry_command, "/chassis/right_back_steering"},
                  {steering_infantry, steering_infantry_command, "/chassis/right_front_steering"})
            , supercap_(steering_infantry, steering_infantry_command) {

            steering_infantry.register_output("/referee/serial", referee_serial_);

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
            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(
                        steering_infantry.get_parameter("yaw_motor_zero_point").as_int())));

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508}

                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle()
                        .set_reversed());
            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_infantry.get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_infantry.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_infantry.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_infantry.get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            steering_infantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        BottomBoard(const BottomBoard&) = delete;
        BottomBoard& operator=(const BottomBoard&) = delete;
        BottomBoard(BottomBoard&&) = delete;
        BottomBoard& operator=(BottomBoard&&) = delete;

        ~BottomBoard() override = default;

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
            if (can_transmission_mode_) {
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
                    .can_id = 0x1FF,
                    .can_data =
                        device::CanPacket8{
                                           gimbal_bullet_feeder_.generate_command(),
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
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           supercap_.generate_command(),
                                           }
                            .as_bytes(),
                });

                auto steer_builder = start_transmit();
                steer_builder.can2_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           chassis_steer_motors_[0].generate_command(),
                                           chassis_steer_motors_[1].generate_command(),
                                           chassis_steer_motors_[2].generate_command(),
                                           chassis_steer_motors_[3].generate_command(),
                                           }
                            .as_bytes(),
                });
            } else {
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
                    .can_id = 0x1FF,
                    .can_data =
                        device::CanPacket8{
                                           gimbal_bullet_feeder_.generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });

                builder.can2_transmit({
                    .can_id = 0x142,
                    .can_data = gimbal_yaw_motor_
                                    .generate_velocity_command(
                                        gimbal_yaw_motor_.control_velocity() - imu_.gy())
                                    .as_bytes(),
                });
            }
            can_transmission_mode_ = !can_transmission_mode_;
        }

    private:
        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x201)
                chassis_wheel_motors_[0].store_status(data.can_data);
            else if (can_id == 0x202)
                chassis_wheel_motors_[1].store_status(data.can_data);
            else if (can_id == 0x203)
                chassis_wheel_motors_[2].store_status(data.can_data);
            else if (can_id == 0x204)
                chassis_wheel_motors_[3].store_status(data.can_data);
            else if (can_id == 0x205)
                gimbal_bullet_feeder_.store_status(data.can_data);
            else if (can_id == 0x300)
                supercap_.store_status(data.can_data);
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x142)
                gimbal_yaw_motor_.store_status(data.can_data);
            else if (can_id == 0x205)
                chassis_steer_motors_[0].store_status(data.can_data);
            else if (can_id == 0x206)
                chassis_steer_motors_[1].store_status(data.can_data);
            else if (can_id == 0x207)
                chassis_steer_motors_[2].store_status(data.can_data);
            else if (can_id == 0x208)
                chassis_steer_motors_[3].store_status(data.can_data);
        }

        void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
            const auto* uart_data = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_n(
                [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
                data.uart_data.size());
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        bool can_transmission_mode_ = true;
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

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        OutputInterface<double> chassis_yaw_velocity_imu_;
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
