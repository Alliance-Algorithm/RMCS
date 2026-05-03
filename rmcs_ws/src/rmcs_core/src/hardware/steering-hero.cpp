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

class SteeringHero
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SteeringHero()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<SteeringHeroCommand>(
                  get_component_name() + "_command", *this)) {

        register_output("/tf", tf_);

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());

        bottom_board_one_ = std::make_unique<BottomBoard_one>(
            *this, *command_component_, get_parameter("board_serial_bottom_board_one").as_string());

        bottom_board_two_ = std::make_unique<BottomBoard_two>(
            *this, *command_component_, get_parameter("board_serial_bottom_board_two").as_string());

        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.06603, 0.0, 0.082});
    }

    SteeringHero(const SteeringHero&) = delete;
    SteeringHero& operator=(const SteeringHero&) = delete;
    SteeringHero(SteeringHero&&) = delete;
    SteeringHero& operator=(SteeringHero&&) = delete;

    ~SteeringHero() override = default;

    void update() override {
        top_board_->update();
        bottom_board_one_->update();
        bottom_board_two_->update();

        tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
            bottom_board_two_->gimbal_bottom_yaw_motor_.angle()
            + top_board_->gimbal_top_yaw_motor_.angle());
        tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
            top_board_->gimbal_pitch_motor_.angle());
    }

    void command_update() {
        top_board_->command_update();
        bottom_board_one_->command_update();
        bottom_board_two_->command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New bottom yaw offset: %ld",
            bottom_board_two_->gimbal_bottom_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New top yaw offset: %ld",
            top_board_->gimbal_top_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New bullet feeder offset: %ld",
            top_board_->gimbal_bullet_feeder_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left front steering offset: %d",
            bottom_board_one_->chassis_steering_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right front steering offset: %d",
            bottom_board_one_->chassis_steering_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left back steering offset: %d",
            bottom_board_two_->chassis_steering_motors2_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right back steering offset: %d",
            bottom_board_two_->chassis_steering_motors2_[1].calibrate_zero_point());
    }

    class SteeringHeroCommand : public rmcs_executor::Component {
    public:
        explicit SteeringHeroCommand(SteeringHero& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

        SteeringHero& hero_;
    };
    std::shared_ptr<SteeringHeroCommand> command_component_;

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class SteeringHero;
        explicit TopBoard(
            SteeringHero& steering_hero, SteeringHeroCommand& steering_hero_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , logger_(steering_hero.get_logger())
            , tf_(steering_hero.tf_)
            , imu_(1000, 0.2, 0.0)
            , gimbal_top_yaw_motor_(steering_hero, steering_hero_command, "/gimbal/top_yaw")
            , gimbal_pitch_motor_(steering_hero, steering_hero_command, "/gimbal/pitch")
            , gimbal_friction_wheels_(
                  {steering_hero, steering_hero_command, "/gimbal/first_left_friction"},
                  {steering_hero, steering_hero_command, "/gimbal/first_right_friction"},
                  {steering_hero, steering_hero_command, "/gimbal/second_left_friction"},
                  {steering_hero, steering_hero_command, "/gimbal/second_right_friction"})
            , gimbal_bullet_feeder_(steering_hero, steering_hero_command, "/gimbal/bullet_feeder")
            , putter_motor_(steering_hero, steering_hero_command, "/gimbal/putter") {

            gimbal_top_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}.set_encoder_zero_point(
                    static_cast<int>(
                        steering_hero.get_parameter("top_yaw_motor_zero_point").as_int())));
            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("pitch_motor_zero_point").as_int()))
                    .enable_multi_turn_angle());
            gimbal_friction_wheels_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));
            gimbal_friction_wheels_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(1.));
            gimbal_friction_wheels_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));
            gimbal_friction_wheels_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(1.));
            gimbal_bullet_feeder_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("bullet_feeder_motor_zero_point").as_int()))
                    .set_reversed()
                    .enable_multi_turn_angle());
            putter_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.)
                    .enable_multi_turn_angle());

            steering_hero.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
            steering_hero.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);

            steering_hero.register_output(
                "/gimbal/photoelectric_sensor", photoelectric_sensor_status_, false);
            steering_hero.register_output(
                "/gimbal/grayscale_sensor", grayscale_sensor_status_, false);

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise
                // use a matrix.

                // Eigen::AngleAxisd pitch_link_to_bmi088_link{
                //     std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_bmi088_link * Eigen::Vector3d{1, 2,
                // 3}; std::cout << mapping << std::endl;
                return std::make_tuple(x, y, z);
            });
        }

        TopBoard(const TopBoard&) = delete;
        TopBoard& operator=(const TopBoard&) = delete;
        TopBoard(TopBoard&&) = delete;
        TopBoard& operator=(TopBoard&&) = delete;

        ~TopBoard() final = default;

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            *gimbal_yaw_velocity_imu_ = imu_.gz();
            *gimbal_pitch_velocity_imu_ = imu_.gy();

            gimbal_top_yaw_motor_.update_status();
            gimbal_pitch_motor_.update_status();

            for (auto& motor : gimbal_friction_wheels_)
                motor.update_status();

            gimbal_bullet_feeder_.update_status();
            putter_motor_.update_status();

            if (last_camera_capturer_trigger_timestamp_ != *camera_capturer_trigger_timestamp_)
                *camera_capturer_trigger_ = true;
            last_camera_capturer_trigger_timestamp_ = *camera_capturer_trigger_timestamp_;

            *photoelectric_sensor_status_ = photoelectric_sensor_status_atomic.load();
            *grayscale_sensor_status_ = grayscale_sensor_status_atomic.load();
        }

        void command_update() {
            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        gimbal_friction_wheels_[3].generate_command(),
                        gimbal_friction_wheels_[1].generate_command(),
                        gimbal_friction_wheels_[2].generate_command(),
                        gimbal_friction_wheels_[0].generate_command(),
                    }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x1FF,
                .can_data =
                    device::CanPacket8{
                        putter_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x141,
                .can_data = gimbal_bullet_feeder_.generate_torque_command().as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x141,
                .can_data = gimbal_top_yaw_motor_.generate_command().as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x142,
                .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes(),
            });

            builder.gpio_digital_read({
                .channel = 7,
                .period_ms = 20,
                .pull = librmcs::data::GpioPull::kUp,
            });

            builder.gpio_digital_read({
                .channel = 5,
                .period_ms = 20,
                .pull = librmcs::data::GpioPull::kUp,
            });
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x204) {
                gimbal_friction_wheels_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                gimbal_friction_wheels_[1].store_status(data.can_data);
            } else if (can_id == 0x203) {
                gimbal_friction_wheels_[2].store_status(data.can_data);
            } else if (can_id == 0x201) {
                gimbal_friction_wheels_[3].store_status(data.can_data);
            } else if (can_id == 0x205) {
                putter_motor_.store_status(data.can_data);
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
            }
        }

        void gpio_digital_read_result_callback(
            const librmcs::data::GpioDigitalDataView& data) override {
            if (data.channel == 7) {
                photoelectric_sensor_status_atomic.store(!data.high);
            } else if (data.channel == 5) {
                grayscale_sensor_status_atomic.store(!data.high);
            }
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        rclcpp::Logger logger_;
        OutputInterface<rmcs_description::Tf>& tf_;

        std::time_t last_camera_capturer_trigger_timestamp_{0};

        device::Bmi088 imu_;
        device::LkMotor gimbal_top_yaw_motor_;
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_friction_wheels_[4];
        device::LkMotor gimbal_bullet_feeder_;
        device::DjiMotor putter_motor_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
        OutputInterface<bool> photoelectric_sensor_status_;
        OutputInterface<bool> grayscale_sensor_status_;
        OutputInterface<bool> camera_capturer_trigger_;
        OutputInterface<std::time_t> camera_capturer_trigger_timestamp_;
        std::atomic<bool> photoelectric_sensor_status_atomic{false};
        std::atomic<bool> grayscale_sensor_status_atomic{false};
    };

    class BottomBoard_one final : private librmcs::agent::CBoard {
    public:
        friend class SteeringHero;
        explicit BottomBoard_one(
            SteeringHero& steering_hero, SteeringHeroCommand& steering_hero_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , logger_(steering_hero.get_logger())
            , imu_(1000, 0.2, 0.0)
            , chassis_front_climber_motor_(
                  {steering_hero, steering_hero_command, "/chassis/climber/left_front_motor"},
                  {steering_hero, steering_hero_command, "/chassis/climber/right_front_motor"})
            , chassis_back_climber_motor_(
                  {steering_hero, steering_hero_command, "/chassis/climber/left_back_motor"},
                  {steering_hero, steering_hero_command, "/chassis/climber/right_back_motor"})
            , chassis_steering_motors_(
                  {steering_hero, steering_hero_command, "/chassis/left_front_steering"},
                  {steering_hero, steering_hero_command, "/chassis/right_front_steering"})
            , chassis_wheel_motors_(
                  {steering_hero, steering_hero_command, "/chassis/left_front_wheel"},
                  {steering_hero, steering_hero_command, "/chassis/right_front_wheel"}) {
            //

            chassis_steering_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("left_front_zero_point").as_int()))
                    .set_reversed());
            chassis_steering_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("right_front_zero_point").as_int()))
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
                    .set_reduction_ratio(19.));
            chassis_back_climber_motor_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(19.));

            steering_hero.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            steering_hero.register_output("/chassis/pitch_imu", chassis_pitch_imu_, 0.0);
        }
        BottomBoard_one(const BottomBoard_one&) = delete;
        BottomBoard_one& operator=(const BottomBoard_one&) = delete;
        BottomBoard_one(BottomBoard_one&&) = delete;
        BottomBoard_one& operator=(BottomBoard_one&&) = delete;

        ~BottomBoard_one() final = default;

        void update() {
            imu_.update_status();

            *chassis_yaw_velocity_imu_ = imu_.gz();
            *chassis_pitch_imu_ = -std::asin(2.0 * (imu_.q0() * imu_.q2() - imu_.q3() * imu_.q1()));

            chassis_front_climber_motor_[0].update_status();
            chassis_front_climber_motor_[1].update_status();
            chassis_back_climber_motor_[0].update_status();
            chassis_back_climber_motor_[1].update_status();

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
                        chassis_wheel_motors_[1].generate_command(),
                        chassis_wheel_motors_[0].generate_command(),
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
                        chassis_steering_motors_[0].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        chassis_steering_motors_[1].generate_command(),
                    }
                        .as_bytes(),
            });

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
                chassis_wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x206) {
                chassis_steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x208) {
                chassis_steering_motors_[1].store_status(data.can_data);
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
            }
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        rclcpp::Logger logger_;

        device::Bmi088 imu_;
        device::DjiMotor chassis_front_climber_motor_[2];
        device::DjiMotor chassis_back_climber_motor_[2];
        device::DjiMotor chassis_steering_motors_[2];
        device::DjiMotor chassis_wheel_motors_[2];

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_imu_;
        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;
    };

    class BottomBoard_two final : private librmcs::agent::CBoard {
    public:
        friend class SteeringHero;
        explicit BottomBoard_two(
            SteeringHero& steering_hero, SteeringHeroCommand& steering_hero_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , logger_(steering_hero.get_logger())
            , imu_(1000, 0.2, 0.0)
            , dr16_(steering_hero)
            , supercap_(steering_hero, steering_hero_command)
            , chassis_steering_motors2_(
                  {steering_hero, steering_hero_command, "/chassis/left_back_steering"},
                  {steering_hero, steering_hero_command, "/chassis/right_back_steering"})
            , chassis_wheel_motors2_(
                  {steering_hero, steering_hero_command, "/chassis/left_back_wheel"},
                  {steering_hero, steering_hero_command, "/chassis/right_back_wheel"})
            , gimbal_bottom_yaw_motor_(steering_hero, steering_hero_command, "/gimbal/bottom_yaw") {
            chassis_steering_motors2_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("left_back_zero_point").as_int()))
                    .set_reversed());
            chassis_steering_motors2_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("right_back_zero_point").as_int()))
                    .set_reversed());
            chassis_wheel_motors2_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            chassis_wheel_motors2_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(2232. / 169.));
            gimbal_bottom_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG6012Ei8}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("bottom_yaw_motor_zero_point").as_int())));
            steering_hero.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart1_transmit(
                    {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };
            steering_hero.register_output(
                "/chassis/powermeter/control_enable", powermeter_control_enabled_, false);
            steering_hero.register_output(
                "/chassis/powermeter/charge_power_limit", powermeter_charge_power_limit_, 0.);
        }

        BottomBoard_two(const BottomBoard_two&) = delete;
        BottomBoard_two& operator=(const BottomBoard_two&) = delete;
        BottomBoard_two(BottomBoard_two&&) = delete;
        BottomBoard_two& operator=(BottomBoard_two&&) = delete;

        ~BottomBoard_two() final = default;

        void update() {
            imu_.update_status();
            dr16_.update_status();
            supercap_.update_status();

            for (auto& motor : chassis_wheel_motors2_)
                motor.update_status();
            for (auto& motor : chassis_steering_motors2_)
                motor.update_status();

            gimbal_bottom_yaw_motor_.update_status();
        }

        void command_update() {

            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        chassis_wheel_motors2_[1].generate_command(),
                        chassis_wheel_motors2_[0].generate_command(),
                    }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                        chassis_steering_motors2_[0].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        chassis_steering_motors2_[1].generate_command(),
                        supercap_.generate_command(),
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

            if (can_id == 0x203) {
                chassis_wheel_motors2_[1].store_status(data.can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors2_[0].store_status(data.can_data);
            } else if (can_id == 0x205) {
                chassis_steering_motors2_[0].store_status(data.can_data);
            } else if (can_id == 0x207) {
                chassis_steering_motors2_[1].store_status(data.can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;

            if (can_id == 0x141) {
                gimbal_bottom_yaw_motor_.store_status(data.can_data);
            }
        }

        rclcpp::Logger logger_;

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

        OutputInterface<bool> powermeter_control_enabled_;
        OutputInterface<double> powermeter_charge_power_limit_;

        device::Dr16 dr16_;
        device::Supercap supercap_;

        device::DjiMotor chassis_steering_motors2_[2];
        device::DjiMotor chassis_wheel_motors2_[2];
        device::LkMotor gimbal_bottom_yaw_motor_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::shared_ptr<TopBoard> top_board_;
    std::shared_ptr<BottomBoard_one> bottom_board_one_;
    std::shared_ptr<BottomBoard_two> bottom_board_two_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SteeringHero, rmcs_executor::Component)
