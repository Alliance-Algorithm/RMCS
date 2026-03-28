#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <format>
#include <memory>
#include <numbers>
#include <print>
#include <ranges>
#include <span>
#include <sstream>
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
#include <rmcs_msgs/hard_sync_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

using Clock = std::chrono::steady_clock;

namespace {

double wrap_single_turn(double angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi_v<double>);
}

} // namespace

class Sentry
    : public rmcs_executor::Component
    , public rclcpp::Node {
    class SentryCommand;
    class GimbalBoard;
    class TopBoard;
    class BottomBoard;

public:
    Sentry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<SentryCommand>(get_component_name() + "_command", *this)) {
        register_input("/predefined/timestamp", timestamp_);
        register_output("/tf", tf_);
        register_output("/gimbal/hard_sync_snapshot", hard_sync_snapshot_);

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
        steers_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/steers/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                steers_calibrate_subscription_callback(std::move(msg));
            });

        // For command: remote-status
        status_service_ = Node::create_service<std_srvs::srv::Trigger>(
            "/rmcs/service/robot_status",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr&,
                const std_srvs::srv::Trigger::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());

        gimbal_board_ = std::make_unique<GimbalBoard>(
            *this, get_parameter("board_serial_gimbal_board").as_string());

        tf_->set_transform<rmcs_description::BottomYawLink, rmcs_description::TopYawLink>(
            Eigen::Translation3d{0.08, 0.0, 0.0});
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.07128, 0.0, 0.0481});
    }

    Sentry(const Sentry&) = delete;
    Sentry& operator=(const Sentry&) = delete;
    Sentry(Sentry&&) = delete;
    Sentry& operator=(Sentry&&) = delete;

    ~Sentry() override = default;

    void before_updating() override {
        gimbal_board_->begin_hard_sync_capture();
        next_hard_sync_log_time_ = Clock::now() + std::chrono::seconds(1);
    }

    void update() override {
        top_board_->update();
        bottom_board_->update();
        gimbal_board_->update();
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomGimbalImu>(
            gimbal_board_->imu_pose().conjugate());
        update_hard_sync_snapshot();
    }

    void command_update() {
        top_board_->command_update();
        bottom_board_->command_update();
    }

private:
    auto status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
        -> void {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("Gimbal Status");
        text("-  Bottom Yaw: {}", bottom_board_->gimbal_bottom_yaw_motor_.last_raw_angle());
        text("-     Top Yaw: {}", top_board_->gimbal_top_yaw_motor_.last_raw_angle());
        text("- Pitch Angle: {}", top_board_->gimbal_pitch_motor_.last_raw_angle());

        text("Chassis Status");
        constexpr auto position =
            std::array<std::string_view, 4>{"right back", "right front", "left front", "left back"};
        constexpr auto max_length =
            std::ranges::max_element(position, {}, &std::string_view::size)->size();

        for (auto&& [index, motor] :
             std::views::zip(position, bottom_board_->chassis_steer_motors_)) {
            text("- {:{}}: {}", index, max_length, motor.last_raw_angle());
        }

        response->message = feedback_message.str();
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New yaw offset: %ld",
            bottom_board_->gimbal_bottom_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New top yaw offset: %ld",
            top_board_->gimbal_top_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[gimbal calibration] New pitch offset: %ld",
            top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    void steers_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left front offset: %d",
            bottom_board_->chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left back offset: %d",
            bottom_board_->chassis_steer_motors_[3].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right back offset: %d",
            bottom_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right front offset: %d",
            bottom_board_->chassis_steer_motors_[1].calibrate_zero_point());
    }

    void update_hard_sync_snapshot() {
        if (!hard_sync_pending_.exchange(false, std::memory_order_relaxed))
            return;

        hard_sync_snapshot_->valid = true;
        hard_sync_snapshot_->exposure_timestamp = *timestamp_;
        const Eigen::Quaterniond gimbal_board_imu_pose = gimbal_board_->imu_pose();
        hard_sync_snapshot_->qw = gimbal_board_imu_pose.w();
        hard_sync_snapshot_->qx = gimbal_board_imu_pose.x();
        hard_sync_snapshot_->qy = gimbal_board_imu_pose.y();
        hard_sync_snapshot_->qz = gimbal_board_imu_pose.z();
        ++hard_sync_snapshot_count_;

        if (*timestamp_ >= next_hard_sync_log_time_) {
            RCLCPP_INFO(
                get_logger(), "[hard sync] published %zu snapshots in the last second",
                hard_sync_snapshot_count_);
            hard_sync_snapshot_count_ = 0;
            next_hard_sync_log_time_ = *timestamp_ + std::chrono::seconds(1);
        }
    }

    class SentryCommand : public rmcs_executor::Component {
    public:
        explicit SentryCommand(Sentry& sentry)
            : sentry(sentry) {}

        void update() override { sentry.command_update(); }

        Sentry& sentry;
    };

    class GimbalBoard final : private librmcs::agent::CBoard {
    public:
        explicit GimbalBoard(Sentry& sentry, std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , sentry_(sentry) {
            bmi088_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise
                // use a matrix.

                // Eigen::AngleAxisd pitch_link_to_bmi088_link{
                //     std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_bmi088_link *
                // Eigen::Vector3d{1, 2, 3}; std::cout << mapping << std::endl;

                return std::make_tuple(y, -x, z);
            });
        }

        GimbalBoard(const GimbalBoard&) = delete;
        GimbalBoard& operator=(const GimbalBoard&) = delete;
        GimbalBoard(GimbalBoard&&) = delete;
        GimbalBoard& operator=(GimbalBoard&&) = delete;

        ~GimbalBoard() override = default;

        void begin_hard_sync_capture() {
            start_transmit().gpio_digital_read({
                .channel = 7,
                .falling_edge = true,
                .pull = librmcs::data::GpioPull::kUp,
            });
        }

        void update() {
            bmi088_.update_status();
            imu_pose_ = Eigen::Quaterniond{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        }

        auto imu_pose() const -> Eigen::Quaterniond { return imu_pose_; }

    private:
        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        void gpio_digital_read_result_callback(
            const librmcs::data::GpioDigitalDataView& data) override {
            if (data.channel == 7 && !data.high)
                sentry_.hard_sync_pending_.store(true, std::memory_order_relaxed);
        }

        Sentry& sentry_;
        device::Bmi088 bmi088_{1000, 0.2, 0.0};
        Eigen::Quaterniond imu_pose_ = Eigen::Quaterniond::Identity();
    };

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class Sentry;
        explicit TopBoard(
            Sentry& sentry, SentryCommand& sentry_command, std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , tf_(sentry.tf_)
            , bmi088_(1000, 0.2, 0.0)
            , gimbal_pitch_motor_(sentry, sentry_command, "/gimbal/pitch")
            , gimbal_top_yaw_motor_(sentry, sentry_command, "/gimbal/top_yaw")
            , gimbal_bullet_feeder_(sentry, sentry_command, "/gimbal/bullet_feeder")
            , gimbal_left_friction_(sentry, sentry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(sentry, sentry_command, "/gimbal/right_friction") {
            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(sentry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_top_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(sentry.get_parameter("top_yaw_motor_zero_point").as_int())));

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.)
                    .set_reversed());

            sentry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            sentry.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_);

            bmi088_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise
                // use a matrix.

                // Eigen::AngleAxisd pitch_link_to_bmi088_link{
                //     std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_bmi088_link *
                // Eigen::Vector3d{1, 2, 3}; std::cout << mapping << std::endl;

                return std::make_tuple(-x, -y, z);
            });
        }

        TopBoard(const TopBoard&) = delete;
        TopBoard& operator=(const TopBoard&) = delete;
        TopBoard(TopBoard&&) = delete;
        TopBoard& operator=(TopBoard&&) = delete;

        ~TopBoard() override = default;

        void update() {
            gimbal_top_yaw_motor_.update_status();
            gimbal_pitch_motor_.update_status();

            const auto pitch_angle = wrap_single_turn(gimbal_pitch_motor_.angle());

            bmi088_.update_status();
            const Eigen::Quaterniond gimbal_bmi088_pose{
                bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};

            tf_->set_transform<rmcs_description::BottomYawLink, rmcs_description::OdomImu>(
                gimbal_bmi088_pose.conjugate());

            *gimbal_yaw_velocity_bmi088_ = bmi088_.gz();
            *gimbal_pitch_velocity_bmi088_ = bmi088_.gy();

            gimbal_bullet_feeder_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            // std::cerr << "top_yaw: " << gimbal_top_yaw_motor_.angle() << '\n';
            tf_->set_state<rmcs_description::BottomYawLink, rmcs_description::TopYawLink>(
                gimbal_top_yaw_motor_.angle());
            tf_->set_state<rmcs_description::TopYawLink, rmcs_description::PitchLink>(pitch_angle);
        }

        void command_update() {
            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        gimbal_right_friction_.generate_command(),
                        gimbal_left_friction_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        gimbal_bullet_feeder_.generate_command(),
                    }
                        .as_bytes(),
            });

            builder.can1_transmit({
                .can_id = 0x141,
                .can_data = gimbal_top_yaw_motor_.generate_torque_command().as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x141,
                .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes(),
            });
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x202) {
                gimbal_left_friction_.store_status(data.can_data);
            } else if (can_id == 0x201) {
                gimbal_right_friction_.store_status(data.can_data);
            } else if (can_id == 0x204) {
                gimbal_bullet_feeder_.store_status(data.can_data);
            } else if (can_id == 0x141) {
                gimbal_top_yaw_motor_.store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x141)
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
        device::LkMotor gimbal_top_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;

        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;
    };

    class BottomBoard final : private librmcs::agent::CBoard {
    public:
        friend class Sentry;

        explicit BottomBoard(
            Sentry& sentry, SentryCommand& sentry_command, std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , imu_(1000, 0.2, 0.0)
            , tf_(sentry.tf_)
            , dr16_(sentry)
            , gimbal_bottom_yaw_motor_(sentry, sentry_command, "/gimbal/bottom_yaw")
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
            , supercap_(sentry, sentry_command) {
            sentry.register_output("/referee/serial", referee_serial_);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart1_transmit(
                    {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            gimbal_bottom_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG6012Ei8}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            sentry.get_parameter("bottom_yaw_motor_zero_point").as_int())));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle()
                        .set_reversed());
            chassis_steer_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            chassis_steer_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(sentry.get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
            sentry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        BottomBoard(const BottomBoard&) = delete;
        BottomBoard& operator=(const BottomBoard&) = delete;
        BottomBoard(BottomBoard&&) = delete;
        BottomBoard& operator=(BottomBoard&&) = delete;

        ~BottomBoard() override = default;

        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gz();
            supercap_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();

            dr16_.update_status();
            gimbal_bottom_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_bottom_yaw_motor_.angle());
        }

        void command_update() {
            auto builder = start_transmit();
            builder.can1_transmit({
                .can_id = 0x141,
                .can_data = gimbal_bottom_yaw_motor_.generate_command().as_bytes(),
            });

            if (can_transmission_mode_) {
                builder
                    .can1_transmit({
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[1].generate_command(),
                                chassis_wheel_motors_[0].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    })
                    .can2_transmit({
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                device::CanPacket8::PaddingQuarter{},
                                chassis_wheel_motors_[2].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                chassis_wheel_motors_[3].generate_command(),
                            }
                                .as_bytes(),
                    });
            } else {
                builder
                    .can1_transmit({
                        .can_id = 0x1FE,
                        .can_data =
                            device::CanPacket8{
                                chassis_steer_motors_[1].generate_command(),
                                chassis_steer_motors_[0].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    })
                    .can2_transmit({
                        .can_id = 0x1FE,
                        .can_data =
                            device::CanPacket8{
                                chassis_steer_motors_[2].generate_command(),
                                chassis_steer_motors_[3].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                supercap_.generate_command(),
                            }
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
                chassis_wheel_motors_[1].store_status(data.can_data);
            else if (can_id == 0x202)
                chassis_wheel_motors_[0].store_status(data.can_data);
            else if (can_id == 0x205)
                chassis_steer_motors_[1].store_status(data.can_data);
            else if (can_id == 0x206)
                chassis_steer_motors_[0].store_status(data.can_data);
            else if (can_id == 0x141)
                gimbal_bottom_yaw_motor_.store_status(data.can_data);
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x202)
                chassis_wheel_motors_[2].store_status(data.can_data);
            else if (can_id == 0x204)
                chassis_wheel_motors_[3].store_status(data.can_data);
            else if (can_id == 0x205)
                chassis_steer_motors_[2].store_status(data.can_data);
            else if (can_id == 0x206)
                chassis_steer_motors_[3].store_status(data.can_data);
            else if (can_id == 0x300)
                supercap_.store_status(data.can_data);
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

        device::Dr16 dr16_;
        device::LkMotor gimbal_bottom_yaw_motor_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::DjiMotor chassis_steer_motors_[4];
        device::Supercap supercap_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        OutputInterface<double> chassis_yaw_velocity_imu_;
    };

    InputInterface<Clock::time_point> timestamp_;
    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<rmcs_msgs::HardSyncSnapshot> hard_sync_snapshot_;
    std::atomic<bool> hard_sync_pending_{false};
    size_t hard_sync_snapshot_count_ = 0;
    Clock::time_point next_hard_sync_log_time_{};

    std::shared_ptr<SentryCommand> command_component_;
    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Sentry, rmcs_executor::Component)
