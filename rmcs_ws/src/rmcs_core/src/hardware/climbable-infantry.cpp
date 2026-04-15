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

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());
        bottom_board_one_ = std::make_unique<BottomBoard_one>(
            *this, *command_component_, *top_board_,
            get_parameter("board_serial_bottom_board_one").as_string());

        bottom_board_two_ = std::make_unique<BottomBoard_two>(
            *this, *command_component_, get_parameter("board_serial_bottom_board_two").as_string());

        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.06603, 0.0, 0.082});
    }

    ClimbableInfantry(const ClimbableInfantry&) = delete;
    ClimbableInfantry& operator=(const ClimbableInfantry&) = delete;
    ClimbableInfantry(ClimbableInfantry&&) = delete;
    ClimbableInfantry& operator=(ClimbableInfantry&&) = delete;

    ~ClimbableInfantry() override = default;

    void update() override {
        top_board_->update();
        bottom_board_one_->update();
        bottom_board_two_->update();
    }

    void command_update() {
        top_board_->command_update();
        bottom_board_one_->command_update();
        bottom_board_two_->command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %ld",
            bottom_board_one_->gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New bullet feeder offset: %ld",
            static_cast<long>(bottom_board_two_->gimbal_bullet_feeder_.calibrate_zero_point()));
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left front steering offset: %d",
            bottom_board_one_->chassis_steering_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left back steering offset: %d",
            bottom_board_two_->chassis_steering_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right back steering offset: %d",
            bottom_board_two_->chassis_steering_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right front steering offset: %d",
            bottom_board_one_->chassis_steering_motors_[1].calibrate_zero_point());
    }

    class ClimbableInfantryCommand : public rmcs_executor::Component {
    public:
        explicit ClimbableInfantryCommand(ClimbableInfantry& infantry_)
            : infantry_(infantry_) {}

        void update() override { infantry_.command_update(); }

        ClimbableInfantry& infantry_;
    };
    std::shared_ptr<ClimbableInfantryCommand> command_component_;

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class ClimbableInfantry;
        explicit TopBoard(
            ClimbableInfantry& climbable_infantry,
            ClimbableInfantryCommand& climbable_infantry_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , tf_(climbable_infantry.tf_)
            , imu_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(climbable_infantry, climbable_infantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(
                  climbable_infantry, climbable_infantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(
                  climbable_infantry, climbable_infantry_command, "/gimbal/right_friction") {

            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(
                        climbable_infantry.get_parameter("pitch_motor_zero_point").as_int())));
            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());

            climbable_infantry.register_output(
                "/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            climbable_infantry.register_output(
                "/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            imu_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, z); });
        }

        TopBoard(const TopBoard&) = delete;
        TopBoard& operator=(const TopBoard&) = delete;
        TopBoard(TopBoard&&) = delete;
        TopBoard& operator=(TopBoard&&) = delete;

        ~TopBoard() final = default;

        double yaw_imu_velocity() const { return imu_.gz(); }

        void update() {
            imu_.update_status();
            const Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            *gimbal_yaw_velocity_imu_ = imu_.gz();
            *gimbal_pitch_velocity_imu_ = imu_.gy();

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
                                       gimbal_left_friction_.generate_command(),
                                       gimbal_right_friction_.generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            builder.can1_transmit({
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
                gimbal_left_friction_.store_status(data.can_data);
            } else if (can_id == 0x202) {
                gimbal_right_friction_.store_status(data.can_data);
            } else if (can_id == 0x142) {
                gimbal_pitch_motor_.store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
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
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
    };

    class BottomBoard_one final : private librmcs::agent::CBoard {
    public:
        friend class ClimbableInfantry;
        explicit BottomBoard_one(
            ClimbableInfantry& climbable_infantry,
            ClimbableInfantryCommand& climbable_infantry_command, TopBoard& top_board,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , top_board_(top_board)
            , logger_(climbable_infantry.get_logger())
            , imu_(1000, 0.2, 0.0)
            , dr16_(climbable_infantry)
            , supercap_(climbable_infantry, climbable_infantry_command)
            , gimbal_yaw_motor_(climbable_infantry, climbable_infantry_command, "/gimbal/yaw")
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
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_front_steering"})
            , chassis_wheel_motors_(
                  {climbable_infantry, climbable_infantry_command, "/chassis/left_front_wheel"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_front_wheel"}) {

            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(
                        climbable_infantry.get_parameter("yaw_motor_zero_point").as_int())));

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
                            climbable_infantry.get_parameter("right_front_zero_point").as_int()))
                    .set_reversed());

            chassis_wheel_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            chassis_wheel_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));

            chassis_front_climber_motor_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(19.));
            chassis_front_climber_motor_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
            chassis_back_climber_motor_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(19.)
                    .set_reversed());
            chassis_back_climber_motor_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
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

        BottomBoard_one(const BottomBoard_one&) = delete;
        BottomBoard_one& operator=(const BottomBoard_one&) = delete;
        BottomBoard_one(BottomBoard_one&&) = delete;
        BottomBoard_one& operator=(BottomBoard_one&&) = delete;

        ~BottomBoard_one() final = default;

        void update() {
            imu_.update_status();
            dr16_.update_status();
            supercap_.update_status();

            *chassis_yaw_velocity_imu_ = imu_.gz();
            *chassis_pitch_imu_ = -std::asin(2.0 * (imu_.q0() * imu_.q2() - imu_.q3() * imu_.q1()));

            // RCLCPP_INFO(
            //     logger_, "[chassis calibration] left front steering offset: %lf",
            //     *chassis_pitch_imu_);
            // RCLCPP_INFO(
            //     logger_, "[chassis calibration] right front steering offset: %d",
            //     chassis_steering_motors_[1].calibrate_zero_point());

            chassis_front_climber_motor_[0].update_status();
            chassis_front_climber_motor_[1].update_status();
            chassis_back_climber_motor_[0].update_status();
            chassis_back_climber_motor_[1].update_status();
            gimbal_yaw_motor_.update_status();

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
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_wheel_motors_[1].generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       chassis_steering_motors_[0].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_steering_motors_[1].generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x141,
                .can_data = gimbal_yaw_motor_.generate_command().as_bytes(),
            });

            // builder.can1_transmit({
            //     .can_id = 0x1FE,
            //     .can_data =
            //         device::CanPacket8{
            //                            device::CanPacket8::PaddingQuarter{},
            //                            device::CanPacket8::PaddingQuarter{},
            //                            device::CanPacket8::PaddingQuarter{},
            //                            supercap_.generate_command(),
            //                            }
            //             .as_bytes(),
            // });

            builder.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       chassis_back_climber_motor_[0].generate_command(),
                                       chassis_back_climber_motor_[1].generate_command(),
                                       chassis_front_climber_motor_[0].generate_command(),
                                       chassis_front_climber_motor_[1].generate_command(),
                                       }
                        .as_bytes(),
            });
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;

            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x205) {
                chassis_steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x208) {
                chassis_steering_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(data.can_data);
            }
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
            } else if (can_id == 0x141) {
                gimbal_yaw_motor_.store_status(data.can_data);
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

        TopBoard& top_board_;
        rclcpp::Logger logger_;

        device::Bmi088 imu_;
        device::Dr16 dr16_;
        device::Supercap supercap_;
        device::LkMotor gimbal_yaw_motor_;

        device::DjiMotor chassis_front_climber_motor_[2];
        device::DjiMotor chassis_back_climber_motor_[2];
        device::DjiMotor chassis_steering_motors_[2];
        device::DjiMotor chassis_wheel_motors_[2];

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_imu_;
    };

    class BottomBoard_two final : private librmcs::agent::CBoard {
    public:
        friend class ClimbableInfantry;
        explicit BottomBoard_two(
            ClimbableInfantry& climbable_infantry,
            ClimbableInfantryCommand& climbable_infantry_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , logger_(climbable_infantry.get_logger())
            , gimbal_bullet_feeder_(
                  climbable_infantry, climbable_infantry_command, "/gimbal/bullet_feeder")
            , chassis_steering_motors_(
                  {climbable_infantry, climbable_infantry_command, "/chassis/left_back_steering"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_back_steering"})
            , chassis_wheel_motors_(
                  {climbable_infantry, climbable_infantry_command, "/chassis/left_back_wheel"},
                  {climbable_infantry, climbable_infantry_command, "/chassis/right_back_wheel"}) {

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            climbable_infantry.get_parameter("bullet_feeder_motor_zero_point")
                                .as_int()))
                    .set_reversed()
                    .enable_multi_turn_angle());

            chassis_steering_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            climbable_infantry.get_parameter("left_back_zero_point").as_int()))
                    .set_reversed());
            chassis_steering_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            climbable_infantry.get_parameter("right_back_zero_point").as_int()))
                    .set_reversed());

            chassis_wheel_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            chassis_wheel_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
        }

        BottomBoard_two(const BottomBoard_two&) = delete;
        BottomBoard_two& operator=(const BottomBoard_two&) = delete;
        BottomBoard_two(BottomBoard_two&&) = delete;
        BottomBoard_two& operator=(BottomBoard_two&&) = delete;

        ~BottomBoard_two() final = default;

        void update() {
            // RCLCPP_INFO(
            //     logger_, "[chassis calibration] left back steering offset: %d",
            //     chassis_steering_motors_[0].calibrate_zero_point());
            // RCLCPP_INFO(
            //     logger_, "[chassis calibration] right back steering offset: %d",
            //     chassis_steering_motors_[1].calibrate_zero_point());

            gimbal_bullet_feeder_.update_status();
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
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_wheel_motors_[0].generate_command(),
                                       chassis_wheel_motors_[1].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_steering_motors_[0].generate_command(),
                                       chassis_steering_motors_[1].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            builder.can2_transmit({
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
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;

            if (can_id == 0x202) {
                chassis_wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x203) {
                chassis_wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x206) {
                chassis_steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x207) {
                chassis_steering_motors_[1].store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;

            if (can_id == 0x205) {
                gimbal_bullet_feeder_.store_status(data.can_data);
            }
        }

        rclcpp::Logger logger_;

        device::DjiMotor gimbal_bullet_feeder_;
        device::DjiMotor chassis_steering_motors_[2];
        device::DjiMotor chassis_wheel_motors_[2];
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::shared_ptr<TopBoard> top_board_;
    std::shared_ptr<BottomBoard_one> bottom_board_one_;
    std::shared_ptr<BottomBoard_two> bottom_board_two_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ClimbableInfantry, rmcs_executor::Component)
