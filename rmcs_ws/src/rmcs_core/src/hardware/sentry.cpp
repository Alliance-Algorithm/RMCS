#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

#include <ranges>

#include <eigen3/Eigen/Geometry>
#include <librmcs/agent/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rmcs_core::hardware {

class Sentry
    : public rmcs_executor::Component
    , public rclcpp::Node {

    static constexpr auto kPosition =
        std::array{"left_front", "left_back", "right_back", "right_front"};

public:
    Sentry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

        register_input("/predefined/timestamp", timestamp_);

        register_output("/tf", tf_);
        register_output("/auto_aim/camera_transform", camera_transform_);
        register_output("/auto_aim/barrel_direction", barrel_direction_);
        register_output(
            "/auto_aim/yaw_velocity", yaw_velocity_, std::numeric_limits<double>::quiet_NaN());

        // For command: remote-status
        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        gimbal_board_ = std::make_unique<GimbalBoard>(
            *this, *command_component_, get_parameter("board_serial_gimbal_board").as_string());

        chassis_board_ = std::make_unique<ChassisBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());

        tf_->set_transform<rmcs_description::BottomYawLink, rmcs_description::TopYawLink>(
            Eigen::Translation3d{0.08, 0.0, 0.0});
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.07128, 0.0, 0.0481});
    }

    auto update() -> void override {
        gimbal_board_->update();
        chassis_board_->update();

        using namespace rmcs_description;
        *camera_transform_ = fast_tf::lookup_transform<OdomGimbalImu, CameraLink>(*tf_);
        *barrel_direction_ = *fast_tf::cast<OdomGimbalImu>(
            PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        *yaw_velocity_ = gimbal_board_->yaw_velocity();
    }

private:
    class GimbalBoard final : private librmcs::agent::RmcsBoardLite {
    public:
        explicit GimbalBoard(
            Sentry& sentry, rmcs_executor::Component& sentry_command,
            std::string_view board_serial = {})
            : librmcs::agent::RmcsBoardLite(board_serial, {})
            , tf_(sentry.tf_)
            , gimbal_pitch_motor_(sentry, sentry_command, "/gimbal/pitch")
            , gimbal_top_yaw_motor_(sentry, sentry_command, "/gimbal/top_yaw")
            , gimbal_bullet_feeder_(sentry, sentry_command, "/gimbal/bullet_feeder")
            , gimbal_left_friction_(sentry, sentry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(sentry, sentry_command, "/gimbal/right_friction") {

            using namespace device;

            auto zero_point = int{0};
            sentry.get_parameter("pitch_motor_zero_point", zero_point);
            gimbal_pitch_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(zero_point));

            sentry.get_parameter("top_yaw_motor_zero_point", zero_point);
            gimbal_top_yaw_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(zero_point));

            gimbal_bullet_feeder_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 4}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            gimbal_left_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 2}.set_reduction_ratio(1.));
            gimbal_right_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 1}.set_reduction_ratio(1.).set_reversed());

            sentry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            sentry.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_);

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, z); });
        }

        auto yaw_velocity() const -> double { return *gimbal_yaw_velocity_bmi088_; }

        auto update() -> void {
            gimbal_bullet_feeder_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            gimbal_top_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::BottomYawLink, rmcs_description::TopYawLink>(
                gimbal_top_yaw_motor_.angle());

            gimbal_pitch_motor_.update_status();
            const auto pitch_angle =
                std::remainder(gimbal_pitch_motor_.angle(), 2.0 * std::numbers::pi_v<double>);
            tf_->set_state<rmcs_description::TopYawLink, rmcs_description::PitchLink>(pitch_angle);

            bmi088_.update_status();
            const auto rotation =
                Eigen::Quaterniond{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomGimbalImu>(
                rotation.conjugate());

            *gimbal_yaw_velocity_bmi088_ = bmi088_.gz();
            *gimbal_pitch_velocity_bmi088_ = bmi088_.gy();
        }

        auto command_update() -> void {
            auto builder = start_transmit();

            builder.can0_transmit({
                .can_id = gimbal_right_friction_.send_id(),
                .can_data =
                    device::CanPacket8{
                        gimbal_right_friction_.generate_command(),
                        gimbal_left_friction_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        gimbal_bullet_feeder_.generate_command(),
                    }
                        .as_bytes(),
            });

            builder
                .can3_transmit({
                    .can_id = 0x141,
                    .can_data = gimbal_top_yaw_motor_.generate_torque_command().as_bytes(),
                })
                .can3_transmit({
                    .can_id = 0x142,
                    .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes(),
                });
        }

        auto can0_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;

            /*^^*/ gimbal_left_friction_.match_then_store_status(can_id, data.can_data)
                || gimbal_right_friction_.match_then_store_status(can_id, data.can_data)
                || gimbal_bullet_feeder_.match_then_store_status(can_id, data.can_data);
        }

        auto can3_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;

            /*^^*/ if (can_id == 0x141) {
                gimbal_top_yaw_motor_.store_status(data.can_data);
            } else if (can_id == 0x142) {
                gimbal_pitch_motor_.store_status(data.can_data);
            }
        }

        auto accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data)
            -> void override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        auto gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data)
            -> void override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_bmi088_;

        device::Bmi088 bmi088_{1000, 0.2, 0.0};
        device::LkMotor gimbal_pitch_motor_;
        device::LkMotor gimbal_top_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;

        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;
    };

    class ChassisBoard final : private librmcs::agent::RmcsBoardLite {
    public:
        explicit ChassisBoard(
            Sentry& sentry, rmcs_executor::Component& sentry_command,
            std::string_view board_serial = {})
            : RmcsBoardLite{board_serial}
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

            using namespace device;

            sentry.register_output("/referee/serial", referee_serial_);
            sentry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart0_transmit(
                    {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            const auto zero_point = sentry.get_parameter("bottom_yaw_motor_zero_point").as_int();
            gimbal_bottom_yaw_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG6012Ei8}.set_reversed().set_encoder_zero_point(
                    static_cast<int>(zero_point)));

            constexpr auto kWheelIds = std::array<std::uint8_t, 4>{2, 1, 2, 4};
            for (auto&& [motor, id] : std::views::zip(chassis_wheel_motors_, kWheelIds)) {
                motor.configure(
                    DjiMotor::Config{DjiMotor::Type::kM3508, id}
                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle()
                        .set_reversed());
            }

            constexpr auto kSteerIds = std::array<std::uint8_t, 4>{2, 1, 1, 2};
            for (auto&& [motor, name, id] :
                 std::views::zip(chassis_steer_motors_, kPosition, kSteerIds)) {
                const auto zero_point =
                    sentry.get_parameter(std::string{name} + "_zero_point").as_int();
                motor.configure(
                    DjiMotor::Config{DjiMotor::Type::kGM6020, id}
                        .set_reversed()
                        .set_encoder_zero_point(static_cast<int>(zero_point))
                        .enable_multi_turn_angle());
            }
        }

        auto update() -> void {
            imu_.update_status();
            gimbal_bottom_yaw_motor_.update_status();
            dr16_.update_status();
            supercap_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_bottom_yaw_motor_.angle());

            *chassis_yaw_velocity_imu_ = imu_.gz();
        }

        auto command_update() -> void {
            using namespace device;

            auto cache = CanPacket8{};
            auto generate = [&](const auto& motors, //
                                CanPacket8::Quarter extra = {}, std::size_t slot = 4) {
                auto slots = std::array<CanPacket8::Quarter, 4>{};
                slots.fill(CanPacket8::PaddingQuarter{});
                std::uint32_t can_id = 0;
                for (auto& motor : motors) {
                    slots[(motor.id() - 1) % 4] = motor.generate_command();
                    if (can_id == 0)
                        can_id = motor.send_id();
                }
                if (slot < 4)
                    slots[slot] = extra;
                cache = CanPacket8{slots[0], slots[1], slots[2], slots[3]};
                return librmcs::data::CanDataView{.can_id = can_id, .can_data = cache.as_bytes()};
            };

            auto supercap_package = CanPacket8{
                CanPacket8::PaddingQuarter{},
                CanPacket8::PaddingQuarter{},
                CanPacket8::PaddingQuarter{},
                supercap_.generate_command(),
            };
            auto bottom_yaw_package = gimbal_bottom_yaw_motor_.generate_command();

            start_transmit()
                .can0_transmit({.can_id = 0x141, .can_data = bottom_yaw_package.as_bytes()})

                .can1_transmit(generate(std::views::counted(chassis_wheel_motors_, 2)))
                .can2_transmit(generate(std::views::counted(chassis_wheel_motors_ + 2, 2)))

                .can1_transmit(generate(std::views::counted(chassis_steer_motors_, 2)))
                .can2_transmit(generate(std::views::counted(chassis_steer_motors_ + 2, 2)))

                .can3_transmit({.can_id = 0x1fe, .can_data = supercap_package.as_bytes()});
        }

        auto can0_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x141)
                gimbal_bottom_yaw_motor_.store_status(data.can_data);
        }
        auto can1_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            const auto can_id = data.can_id;
            const auto can_data = data.can_data;

            /*^^*/ chassis_wheel_motors_[0].match_then_store_status(can_id, can_data)
                || chassis_wheel_motors_[1].match_then_store_status(can_id, can_data)

                || chassis_steer_motors_[0].match_then_store_status(can_id, can_data)
                || chassis_steer_motors_[1].match_then_store_status(can_id, can_data);
        }
        auto can2_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            const auto can_id = data.can_id;
            const auto can_data = data.can_data;

            /*^^*/ chassis_wheel_motors_[2].match_then_store_status(can_id, can_data)
                || chassis_wheel_motors_[3].match_then_store_status(can_id, can_data)

                || chassis_steer_motors_[2].match_then_store_status(can_id, can_data)
                || chassis_steer_motors_[3].match_then_store_status(can_id, can_data);
        }
        auto can3_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (data.can_id == 0x300)
                supercap_.store_status(data.can_data);
        }

        auto dbus_receive_callback(const librmcs::data::UartDataView& data) -> void override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }
        auto uart0_receive_callback(const librmcs::data::UartDataView& data) -> void override {
            const auto* uart_data = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_n(
                [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
                data.uart_data.size());
        }

        auto accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data)
            -> void override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        auto gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data)
            -> void override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        device::Bmi088 imu_{1000, 0.2, 0.0};
        OutputInterface<rmcs_description::Tf>& tf_;

        device::Dr16 dr16_;
        device::LkMotor gimbal_bottom_yaw_motor_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::DjiMotor chassis_steer_motors_[4];
        device::Supercap supercap_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        OutputInterface<double> chassis_yaw_velocity_imu_;
    }; // namespace rmcs_core::hardware

    struct CommandTransmitter : public rmcs_executor::Component {
        std::function<void()> fn;

        template <std::invocable Fn>
        explicit CommandTransmitter(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        auto update() -> void override { fn(); }
    };

    auto status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
        -> void {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text(
            "bottom_yaw_motor_zero_point: {}",
            chassis_board_->gimbal_bottom_yaw_motor_.last_raw_angle());
        text("pitch_motor_zero_point: {}", gimbal_board_->gimbal_pitch_motor_.last_raw_angle());
        text("top_yaw_motor_zero_point: {}", gimbal_board_->gimbal_top_yaw_motor_.last_raw_angle());

        text("");
        for (auto&& [index, motor] :
             std::views::zip(kPosition, chassis_board_->chassis_steer_motors_)) {
            text("{}_zero_point: {}", index, motor.last_raw_angle());
        }
        response->message = feedback_message.str();
    }

    auto command_update() -> void {
        gimbal_board_->command_update();
        chassis_board_->command_update();
    }
    std::shared_ptr<rmcs_executor::Component> command_component_{
        create_partner_component<CommandTransmitter>(
            get_component_name() + "_command", [this] { command_update(); })};

    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    OutputInterface<rmcs_description::Tf> tf_;

    OutputInterface<Eigen::Isometry3d> camera_transform_;
    OutputInterface<Eigen::Vector3d> barrel_direction_;
    OutputInterface<double> yaw_velocity_;

    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<ChassisBoard> chassis_board_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Sentry, rmcs_executor::Component)
