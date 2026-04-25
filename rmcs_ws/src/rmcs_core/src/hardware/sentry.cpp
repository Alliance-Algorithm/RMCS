#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

#include <ranges>

#include <eigen3/Eigen/Geometry>
#include <librmcs/agent/c_board.hpp>
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
public:
    Sentry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

        register_input("/predefined/timestamp", timestamp_);
        register_output("/tf", tf_);

        // For command: remote-status
        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());

        gimbal_board_ =
            std::make_unique<GimbalBoard>(get_parameter("board_serial_gimbal_board").as_string());

        tf_->set_transform<rmcs_description::BottomYawLink, rmcs_description::TopYawLink>(
            Eigen::Translation3d{0.08, 0.0, 0.0});
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::CameraLink>(
            Eigen::Translation3d{0.07128, 0.0, 0.0481});
    }

    auto update() -> void override {
        top_board_->update();
        bottom_board_->update();
        gimbal_board_->update();
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomGimbalImu>(
            gimbal_board_->imu_pose().conjugate());
    }

private:
    class GimbalBoard final : private librmcs::agent::CBoard {
    public:
        explicit GimbalBoard(std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial) {
            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(y, -x, z); });
        }

        GimbalBoard(const GimbalBoard&) = delete;
        GimbalBoard& operator=(const GimbalBoard&) = delete;
        GimbalBoard(GimbalBoard&&) = delete;
        GimbalBoard& operator=(GimbalBoard&&) = delete;

        ~GimbalBoard() override = default;

        auto update() -> void {
            bmi088_.update_status();
            imu_pose_ = Eigen::Quaterniond{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        }

        auto imu_pose() const -> Eigen::Quaterniond { return imu_pose_; }

    private:
        auto accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data)
            -> void override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        auto gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data)
            -> void override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        device::Bmi088 bmi088_{1000, 0.2, 0.0};
        Eigen::Quaterniond imu_pose_ = Eigen::Quaterniond::Identity();
    };

    class TopBoard final : private librmcs::agent::RmcsBoardLite {
        friend class Sentry;

    public:
        explicit TopBoard(
            Sentry& sentry, rmcs_executor::Component& sentry_command,
            std::string_view board_serial = {}, librmcs::agent::AdvancedOptions options = {})
            : librmcs::agent::RmcsBoardLite(board_serial, options)
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

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, z); });
        }

        auto update() -> void {
            gimbal_top_yaw_motor_.update_status();
            gimbal_pitch_motor_.update_status();

            const auto pitch_angle =
                std::remainder(gimbal_pitch_motor_.angle(), 2.0 * std::numbers::pi_v<double>);

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

            tf_->set_state<rmcs_description::BottomYawLink, rmcs_description::TopYawLink>(
                gimbal_top_yaw_motor_.angle());
            tf_->set_state<rmcs_description::TopYawLink, rmcs_description::PitchLink>(pitch_angle);
        }

        auto command_update() -> void {
            auto builder = start_transmit();

            builder.can0_transmit({
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

            builder.can3_transmit({
                .can_id = 0x141,
                .can_data = gimbal_top_yaw_motor_.generate_torque_command().as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x141,
                .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes(),
            });
        }

    private:
        auto can0_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x202) {
                gimbal_left_friction_.store_status(data.can_data);
            } else if (can_id == 0x201) {
                gimbal_right_friction_.store_status(data.can_data);
            } else if (can_id == 0x204) {
                gimbal_bullet_feeder_.store_status(data.can_data);
            }
        }

        auto can2_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x141)
                gimbal_pitch_motor_.store_status(data.can_data);
        }

        auto can3_receive_callback(const librmcs::data::CanDataView& data) -> void override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            auto can_id = data.can_id;
            if (can_id == 0x141)
                gimbal_top_yaw_motor_.store_status(data.can_data);
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

        device::Bmi088 bmi088_;
        device::LkMotor gimbal_pitch_motor_;
        device::LkMotor gimbal_top_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;

        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;
    };

    class BottomBoard final : private librmcs::agent::CBoard {
        friend class Sentry;

    public:
        explicit BottomBoard(
            Sentry& sentry, rmcs_executor::Component& sentry_command,
            std::string_view board_serial = {})
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

            const auto zero_point = sentry.get_parameter("bottom_yaw_motor_zero_point").as_int();
            gimbal_bottom_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG6012Ei8}
                    .set_reversed()
                    .set_encoder_zero_point(static_cast<int>(zero_point)));

            for (auto& motor : chassis_wheel_motors_) {
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                        .set_reduction_ratio(11.)
                        .enable_multi_turn_angle()
                        .set_reversed());
            }

            constexpr auto kSteerNames = std::array{
                "right_back_zero_point",
                "right_front_zero_point",
                "left_front_zero_point",
                "left_back_zero_point",
            };
            for (auto&& [motor, name] : std::views::zip(chassis_steer_motors_, kSteerNames)) {
                const auto zero_point = sentry.get_parameter(name).as_int();
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                        .set_reversed()
                        .set_encoder_zero_point(static_cast<int>(zero_point))
                        .enable_multi_turn_angle());
            }

            sentry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
        }

        auto update() -> void {
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

        auto command_update() -> void {
            using namespace device;

            auto builder = start_transmit();
            builder.can1_transmit({
                .can_id = 0x141,
                .can_data = gimbal_bottom_yaw_motor_.generate_command().as_bytes(),
            });

            auto cache = CanPacket8{};
            auto generate = [&](std::uint32_t id, std::ranges::range auto& motors, auto... args) {
                auto command = [&]<typename T>(T arg) {
                    if constexpr (std::same_as<CanPacket8::Quarter, T>) {
                        return arg;
                    } else {
                        const auto valid = arg >= 0 && arg < 4;
                        return valid ? motors[arg].generate_command()
                                     : CanPacket8::PaddingQuarter{};
                    }
                };
                cache = CanPacket8{command(args)...};
                return librmcs::data::CanDataView{.can_id = id, .can_data = cache.as_bytes()};
            };

            if (can_transmission_mode_) {
                builder.can1_transmit(generate(0x200, chassis_wheel_motors_, 1, 0, -1, -1))
                    .can2_transmit(generate(0x200, chassis_wheel_motors_, -1, 2, -1, 3));
            } else {
                builder.can1_transmit(generate(0x1FE, chassis_steer_motors_, 1, 0, -1, -1))
                    .can2_transmit(generate(
                        0x1FE, chassis_steer_motors_, 2, 3, -1, supercap_.generate_command()));
            }
            can_transmission_mode_ = !can_transmission_mode_;
        }

    private:
        auto dbus_receive_callback(const librmcs::data::UartDataView& data) -> void override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        auto can1_receive_callback(const librmcs::data::CanDataView& data) -> void override {
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

        auto can2_receive_callback(const librmcs::data::CanDataView& data) -> void override {
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

        auto uart1_receive_callback(const librmcs::data::UartDataView& data) -> void override {
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

        text("Gimbal Status");
        text("-  Bottom Yaw: {}", bottom_board_->gimbal_bottom_yaw_motor_.last_raw_angle());
        text("-     Top Yaw: {}", top_board_->gimbal_top_yaw_motor_.last_raw_angle());
        text("- Pitch Angle: {}", top_board_->gimbal_pitch_motor_.last_raw_angle());

        text("Chassis Status");
        constexpr auto kPosition =
            std::array<std::string_view, 4>{"right back", "right front", "left front", "left back"};
        constexpr auto kMaxLength =
            std::ranges::max_element(kPosition, {}, &std::string_view::size)->size();

        for (auto&& [index, motor] :
             std::views::zip(kPosition, bottom_board_->chassis_steer_motors_)) {
            text("- {:{}}: {}", index, kMaxLength, motor.last_raw_angle());
        }

        response->message = feedback_message.str();
    }

    auto command_update() -> void {
        top_board_->command_update();
        bottom_board_->command_update();
    }
    std::shared_ptr<rmcs_executor::Component> command_component_{
        create_partner_component<CommandTransmitter>(
            get_component_name() + "_command", [this] { command_update(); })};

    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    OutputInterface<rmcs_description::Tf> tf_;

    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Sentry, rmcs_executor::Component)
