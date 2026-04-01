#include <atomic>
#include <cstddef>
#include <cstring>
#include <memory>
#include <span>
#include <string_view>
#include <tuple>

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

class ClimbableInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ClimbableInfantry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<ClimbableInfantryCommand>(
                  get_component_name() + "_command", *this)) {

        register_output("/tf", tf_);

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());

        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.06603, 0.0, 0.082});
    }

    ClimbableInfantry(const ClimbableInfantry&) = delete;
    ClimbableInfantry& operator=(const ClimbableInfantry&) = delete;
    ClimbableInfantry(ClimbableInfantry&&) = delete;
    ClimbableInfantry& operator=(ClimbableInfantry&&) = delete;

    ~ClimbableInfantry() override = default;

    void update() override { bottom_board_->update(); }

    void command_update() { bottom_board_->command_update(); }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
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

    class ClimbableInfantryCommand : public rmcs_executor::Component {
    public:
        explicit ClimbableInfantryCommand(ClimbableInfantry& infantry_)
            : infantry_(infantry_) {}

        void update() override { infantry_.command_update(); }

        ClimbableInfantry& infantry_;
    };
    std::shared_ptr<ClimbableInfantryCommand> command_component_;

    class BottomBoard final : private librmcs::agent::CBoard {
    public:
        friend class ClimbableInfantry;
        explicit BottomBoard(
            ClimbableInfantry& climbable_infantry,
            ClimbableInfantryCommand& climbable_infantry_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , logger_(climbable_infantry.get_logger())
            , imu_(1000, 0.2, 0.0)
            , dr16_(climbable_infantry)
            , supercap_(climbable_infantry, climbable_infantry_command)

            , chassis_front_climber_motor_(
                  {climbable_infantry, climbable_infantry_command,
                   "/chassis/climber/left_front_motor"},
                  {climbable_infantry, climbable_infantry_command,
                   "/chassis/climber/right_front_motor"})
            , chassis_back_climber_motor_(
                  {climbable_infantry, climbable_infantry_command,
                   "/chassis/climber/left_back_motor"},
                  {climbable_infantry, climbable_infantry_command,
                   "/chassis/climber/right_back_motor"})
            , chassis_steering_motors_(
                  {climbable_infantry, climbable_infantry_command, "/chassis/left_front_steering"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/left_back_steering"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_back_steering"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_front_steering"})
            , chassis_wheel_motors_(
                  {climbable_infantry, climbable_infantry_command, "/chassis/left_front_wheel"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/left_back_wheel"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_back_wheel"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_front_wheel"}) {

            /*  舵  */
            chassis_steering_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            climbable_infantry.get_parameter("left_front_zero_point").as_int()))
                    .set_reversed());
            chassis_steering_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            climbable_infantry.get_parameter("left_back_zero_point").as_int()))
                    .set_reversed());
            chassis_steering_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            climbable_infantry.get_parameter("right_back_zero_point").as_int()))
                    .set_reversed());
            chassis_steering_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            climbable_infantry.get_parameter("right_front_zero_point").as_int()))
                    .set_reversed());
            /*  轮  */
            chassis_wheel_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            chassis_wheel_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            chassis_wheel_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            chassis_wheel_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            /*  爬  */

            chassis_front_climber_motor_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(19.));
            chassis_front_climber_motor_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
            chassis_back_climber_motor_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(19.));
            chassis_back_climber_motor_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(19.));
            climbable_infantry.register_output("/referee/serial", referee_serial_);
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

            climbable_infantry.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            climbable_infantry.register_output("/chassis/pitch_imu", chassis_pitch_imu_, 0.0);
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
            *chassis_pitch_imu_ = -std::asin(2.0 * (imu_.q0() * imu_.q2() - imu_.q3() * imu_.q1()));

            RCLCPP_INFO(
                logger_, "[chassis calibration] left front steering offset: %d",
                chassis_steering_motors_[0].calibrate_zero_point());
            RCLCPP_INFO(
                logger_, "[chassis calibration] left back steering offset: %d",
                chassis_steering_motors_[1].calibrate_zero_point());
            RCLCPP_INFO(
                logger_, "[chassis calibration] right back steering offset: %d",
                chassis_steering_motors_[2].calibrate_zero_point());
            RCLCPP_INFO(
                logger_, "[chassis calibration] right front steering offset: %d",
                chassis_steering_motors_[3].calibrate_zero_point());

            // chassis_front_climber_motor_[0].update_status();
            // chassis_front_climber_motor_[1].update_status();
            // chassis_back_climber_motor_[0].update_status();
            // chassis_back_climber_motor_[1].update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steering_motors_)
                motor.update_status();
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

            // builder.can2_transmit({
            //     .can_id = 0x200,
            //     .can_data =
            //         device::CanPacket8{
            //                            chassis_back_climber_motor_[0].generate_command(),
            //                            chassis_back_climber_motor_[1].generate_command(),
            //                            chassis_front_climber_motor_[0].generate_command(),
            //                            chassis_front_climber_motor_[1].generate_command(),
            //                            }
            //             .as_bytes(),
            // });
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
            }
            if (can_id == 0x205) {
                chassis_steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x206) {
                chassis_steering_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x207) {
                chassis_steering_motors_[2].store_status(data.can_data);
            } else if (can_id == 0x208) {
                chassis_steering_motors_[3].store_status(data.can_data);
            }

            //  else if (can_id == 0x300) {
            //     supercap_.store_status(data.can_data);
            // }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;

            if (can_id == 0x203) {
                chassis_front_climber_motor_[0].store_status(data.can_data);
            } else if (can_id == 0x204) {
                chassis_front_climber_motor_[1].store_status(data.can_data);
            } else if (can_id == 0x201) {
                chassis_back_climber_motor_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                chassis_back_climber_motor_[1].store_status(data.can_data);
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

        rclcpp::Logger logger_;

        short enable = 0;

        device::Bmi088 imu_;

        device::Dr16 dr16_;
        device::Supercap supercap_;

        device::DjiMotor chassis_front_climber_motor_[2];
        device::DjiMotor chassis_back_climber_motor_[2];
        device::DjiMotor chassis_steering_motors_[4];
        device::DjiMotor chassis_wheel_motors_[4];

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_imu_;
        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::shared_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ClimbableInfantry, rmcs_executor::Component)
