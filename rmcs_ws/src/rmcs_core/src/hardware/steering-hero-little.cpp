#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <librmcs/agent/c_board.hpp>
#include <librmcs/agent/rmcs_board_lite.hpp>
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

class CanReceiveRateCounter {
public:
    explicit CanReceiveRateCounter(rclcpp::Logger logger, std::string_view channel_name)
        : logger_(logger)
        , channel_name_(channel_name) {}

    void record(std::uint32_t can_id) {
        const auto now = Clock::now();

        std::lock_guard lock{mutex_};
        auto& status = statuses_[can_id];
        ++status.receive_count;
        status.last_receive_time = now;

        report_if_due(now);
    }

    void report_if_due() {
        const auto now = Clock::now();

        std::lock_guard lock{mutex_};
        report_if_due(now);
    }

private:
    using Clock = std::chrono::steady_clock;

    struct Status {
        std::size_t receive_count{0};
        Clock::time_point last_receive_time{};
    };

    void report_if_due(Clock::time_point now) {
        if (statuses_.empty())
            return;

        if (last_report_time_ == Clock::time_point{}) {
            last_report_time_ = now;
            return;
        }

        const auto elapsed = now - last_report_time_;
        if (elapsed < kReportInterval)
            return;

        const auto elapsed_seconds = std::chrono::duration<double>(elapsed).count();
        for (auto& [can_id, status] : statuses_) {
            const bool attached = now - status.last_receive_time <= kMissTimeout;
            RCLCPP_INFO(
                logger_, "[can rx] %.*s id=0x%03X rate=%.1fHz status=%s",
                static_cast<int>(channel_name_.size()), channel_name_.data(),
                static_cast<unsigned int>(can_id),
                static_cast<double>(status.receive_count) / elapsed_seconds,
                attached ? "attach" : "miss");
            status.receive_count = 0;
        }

        last_report_time_ = now;
    }

    static constexpr std::chrono::milliseconds kReportInterval{1000};
    static constexpr std::chrono::milliseconds kMissTimeout{1000};

    rclcpp::Logger logger_;
    std::string_view channel_name_;
    std::mutex mutex_;
    Clock::time_point last_report_time_{};
    std::map<std::uint32_t, Status> statuses_;
};

class SteeringHeroLittle
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SteeringHeroLittle()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<SteeringHeroLittleCommand>(
                  get_component_name() + "_command", *this)) {

        register_output("/tf", tf_);

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("serial_bottom_rmcs_board").as_string());

        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.06603, 0.0, 0.082});
    }

    SteeringHeroLittle(const SteeringHeroLittle&) = delete;
    SteeringHeroLittle& operator=(const SteeringHeroLittle&) = delete;
    SteeringHeroLittle(SteeringHeroLittle&&) = delete;
    SteeringHeroLittle& operator=(SteeringHeroLittle&&) = delete;

    ~SteeringHeroLittle() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();

        tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
            bottom_board_->gimbal_bottom_yaw_motor_.angle()
            + top_board_->gimbal_top_yaw_motor_.angle());
        tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
            top_board_->gimbal_pitch_motor_.angle());
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
            get_logger(), "[gimbal calibration] New bullet feeder offset: %ld",
            top_board_->gimbal_bullet_feeder_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left front steering offset: %d",
            bottom_board_->chassis_steering_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right front steering offset: %d",
            bottom_board_->chassis_steering_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] left back steering offset: %d",
            bottom_board_->chassis_steering_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] right back steering offset: %d",
            bottom_board_->chassis_steering_motors_[3].calibrate_zero_point());
    }

    class SteeringHeroLittleCommand : public rmcs_executor::Component {
    public:
        explicit SteeringHeroLittleCommand(SteeringHeroLittle& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

        SteeringHeroLittle& hero_;
    };
    std::shared_ptr<SteeringHeroLittleCommand> command_component_;

    class TopBoard final : private librmcs::agent::RmcsBoardLite {
    public:
        friend class SteeringHeroLittle;
        explicit TopBoard(
            SteeringHeroLittle& steering_hero, SteeringHeroLittleCommand& steering_hero_command,
            std::string_view board_serial = {})
            : librmcs::agent::RmcsBoardLite(board_serial)
            , logger_(steering_hero.get_logger())
            , tf_(steering_hero.tf_)
            , imu_(1000, 0.2, 0.0)
            , gimbal_top_yaw_motor_(steering_hero, steering_hero_command, "/gimbal/top_yaw")
            , gimbal_pitch_motor_(steering_hero, steering_hero_command, "/gimbal/pitch")
            , gimbal_friction_wheels_(
                  {steering_hero, steering_hero_command, "/gimbal/first_right_friction"},
                  {steering_hero, steering_hero_command, "/gimbal/first_left_friction"},
                  {steering_hero, steering_hero_command, "/gimbal/second_right_friction"},
                  {steering_hero, steering_hero_command, "/gimbal/second_left_friction"})
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
            steering_hero.register_output(
                "/auto_aim/image_capturer/timestamp", camera_capturer_trigger_timestamp_, 0);
            steering_hero.register_output(
                "/auto_aim/image_capturer/trigger", camera_capturer_trigger_, 0);

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

            for (unsigned int i = 0; i < 8; i++) {
                builder.can0_transmit({
                    .can_id = 0x141 + i,
                    .can_data = gimbal_top_yaw_motor_.generate_command().as_bytes(),
                });
            }

            // builder.can0_transmit({
            //     .can_id = 0x141,
            //     .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes(),
            // });

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

            builder.can3_transmit({
                .can_id = 0x142,
                .can_data = gimbal_bullet_feeder_.generate_torque_command().as_bytes(),
            });

            // builder.gpio_digital_read({
            //     .channel = 7,
            //     .period_ms = 20,
            //     .pull = librmcs::data::GpioPull::kUp,
            // });

            // builder.gpio_digital_read({
            //     .channel = 5,
            //     .period_ms = 20,
            //     .pull = librmcs::data::GpioPull::kUp,
            // });
        }

    private:
        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            // RCLCPP_INFO(logger_, "can_id: %x", can_id);
            // if (can_id == 0x142) {
            //     gimbal_top_yaw_motor_.store_status(data.can_data);
            //     RCLCPP_INFO(logger_, "1");
            // } else if (can_id == 0x141) {
            //     gimbal_pitch_motor_.store_status(data.can_data);
            // }
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            // RCLCPP_INFO(logger_, "can_id:%x", can_id);
            if (can_id == 0x201) {
                gimbal_friction_wheels_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                gimbal_friction_wheels_[1].store_status(data.can_data);
            } else if (can_id == 0x203) {
                gimbal_friction_wheels_[2].store_status(data.can_data);
            } else if (can_id == 0x204) {
                gimbal_friction_wheels_[3].store_status(data.can_data);
            } else if (can_id == 0x205) {
                putter_motor_.store_status(data.can_data);
            }
        }

        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x142) {
                gimbal_bullet_feeder_.store_status(data.can_data);
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

    class BottomBoard final : private librmcs::agent::RmcsBoardLite {
    public:
        friend class SteeringHeroLittle;
        explicit BottomBoard(
            SteeringHeroLittle& steering_hero, SteeringHeroLittleCommand& steering_hero_command,
            std::string_view board_serial = {})
            : librmcs::agent::RmcsBoardLite(
                  board_serial, {.dangerously_skip_version_checks = false})
            , logger_(steering_hero.get_logger())
            // , can0_receive_rate_counter_(logger_, "bottom/can0")
            // , can1_receive_rate_counter_(logger_, "bottom/can1")
            // , can2_receive_rate_counter_(logger_, "bottom/can2")
            // , can3_receive_rate_counter_(logger_, "bottom/can3")
            , imu_(1000, 0.2, 0.0)
            , dr16_(steering_hero)
            , supercap_(steering_hero, steering_hero_command)
            , chassis_steering_motors_(
                  {steering_hero, steering_hero_command, "/chassis/left_front_steering"},
                  {steering_hero, steering_hero_command, "/chassis/right_front_steering"},
                  {steering_hero, steering_hero_command, "/chassis/left_back_steering"},
                  {steering_hero, steering_hero_command, "/chassis/right_back_steering"})
            , chassis_wheel_motors_(
                  {steering_hero, steering_hero_command, "/chassis/left_front_wheel"},
                  {steering_hero, steering_hero_command, "/chassis/right_front_wheel"},
                  {steering_hero, steering_hero_command, "/chassis/left_back_wheel"},
                  {steering_hero, steering_hero_command, "/chassis/right_back_wheel"})
            , chassis_front_climber_motor_(
                  {steering_hero, steering_hero_command, "/chassis/climber/left_front_motor"},
                  {steering_hero, steering_hero_command, "/chassis/climber/right_front_motor"})
            , chassis_back_climber_motor_(
                  {steering_hero, steering_hero_command, "/chassis/climber/left_back_motor"},
                  {steering_hero, steering_hero_command, "/chassis/climber/right_back_motor"})
            , gimbal_bottom_yaw_motor_(steering_hero, steering_hero_command, "/gimbal/bottom_yaw") {
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
            chassis_steering_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("left_back_zero_point").as_int()))
                    .set_reversed());
            chassis_steering_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            steering_hero.get_parameter("right_back_zero_point").as_int()))
                    .set_reversed());

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
                start_transmit().uart1_transmit({
                    .uart_data = std::span<const std::byte>{buffer, size}
                });
                return size;
            };
            steering_hero.register_output(
                "/chassis/powermeter/control_enable", powermeter_control_enabled_, false);
            steering_hero.register_output(
                "/chassis/powermeter/charge_power_limit", powermeter_charge_power_limit_, 0.);

            steering_hero.register_output(
                "/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            steering_hero.register_output("/chassis/pitch_imu", chassis_pitch_imu_, 0.0);
        }

        BottomBoard(const BottomBoard&) = delete;
        BottomBoard& operator=(const BottomBoard&) = delete;
        BottomBoard(BottomBoard&&) = delete;
        BottomBoard& operator=(BottomBoard&&) = delete;

        ~BottomBoard() final = default;

        void update() {
            // can0_receive_rate_counter_.report_if_due();
            // can1_receive_rate_counter_.report_if_due();
            // can2_receive_rate_counter_.report_if_due();
            // can3_receive_rate_counter_.report_if_due();

            imu_.update_status();
            dr16_.update_status();
            supercap_.update_status();

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

            gimbal_bottom_yaw_motor_.update_status();
        }

        void command_update() {
            auto builder = start_transmit();

            builder.can0_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       chassis_wheel_motors_[0].generate_command(),
                                       chassis_wheel_motors_[1].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            builder.can0_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       chassis_steering_motors_[1].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_steering_motors_[0].generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
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
                                       chassis_steering_motors_[3].generate_command(),
                                       chassis_steering_motors_[2].generate_command(),
                                       supercap_.generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can3_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_back_climber_motor_[1].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_back_climber_motor_[0].generate_command(),
                                       }
                        .as_bytes(),
            });

            builder.can3_transmit({
                .can_id = 0x141,
                .can_data = gimbal_bottom_yaw_motor_.generate_command().as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       chassis_front_climber_motor_[0].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_front_climber_motor_[1].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
        }

    private:
        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            // can0_receive_rate_counter_.record(can_id);
            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x205) {
                chassis_steering_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x208) {
                chassis_steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(data.can_data);
            }
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            // can1_receive_rate_counter_.record(can_id);
            if (can_id == 0x203) {
                chassis_wheel_motors_[2].store_status(data.can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors_[3].store_status(data.can_data);
            } else if (can_id == 0x207) {
                chassis_steering_motors_[2].store_status(data.can_data);
            } else if (can_id == 0x206) {
                chassis_steering_motors_[3].store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            auto can_id = data.can_id;
            // can2_receive_rate_counter_.record(can_id);
            if (can_id == 0x201) {
                chassis_front_climber_motor_[0].store_status(data.can_data);
            } else if (can_id == 0x203) {
                chassis_front_climber_motor_[1].store_status(data.can_data);
            }
        }

        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            // can3_receive_rate_counter_.record(can_id);
            if (can_id == 0x202) {
                chassis_back_climber_motor_[0].store_status(data.can_data);
            } else if (can_id == 0x204) {
                chassis_back_climber_motor_[1].store_status(data.can_data);
            } else if (can_id == 0x141) {
                gimbal_bottom_yaw_motor_.store_status(data.can_data);
            }
        }

        void uart0_receive_callback(const librmcs::data::UartDataView& data) override {
            const std::byte* ptr = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_n(
                [&ptr](std::byte* storage) noexcept { *storage = *ptr++; }, data.uart_data.size());
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
        // CanReceiveRateCounter can0_receive_rate_counter_;
        // CanReceiveRateCounter can1_receive_rate_counter_;
        // CanReceiveRateCounter can2_receive_rate_counter_;
        // CanReceiveRateCounter can3_receive_rate_counter_;

        device::Bmi088 imu_;
        device::Dr16 dr16_;
        device::Supercap supercap_;

        device::DjiMotor chassis_steering_motors_[4];
        device::DjiMotor chassis_wheel_motors_[4];
        device::DjiMotor chassis_front_climber_motor_[2];
        device::DjiMotor chassis_back_climber_motor_[2];
        device::LkMotor gimbal_bottom_yaw_motor_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};

        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        OutputInterface<bool> powermeter_control_enabled_;
        OutputInterface<double> powermeter_charge_power_limit_;
        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_imu_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    std::shared_ptr<TopBoard> top_board_;
    std::shared_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SteeringHeroLittle, rmcs_executor::Component)
