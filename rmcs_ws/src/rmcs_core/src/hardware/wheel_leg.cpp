#include <format>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/device/tof.hpp"
#include "hardware/util/status_monitor.hpp"

namespace rmcs_core::hardware {

class WheelLeg
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLeg()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_output("/tf", tf_);

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        chassis_board_ = std::make_unique<ChassisBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string());
        gimbal_board_ = std::make_unique<GimbalBoard>(
            *this, *command_component_, get_parameter("board_serial_gimbal_board").as_string());
    }

    void update() override {
        chassis_board_->update();
        gimbal_board_->update();
        remote_control_->update();
    }

private:
    class ChassisBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        ChassisBoard(
            WheelLeg& wheel_leg, rmcs_executor::Component& command, std::string_view board_serial)
            : tf_(wheel_leg.tf_)
            , chassis_left_wheel_motor_(wheel_leg, command, "/wheel_leg/left_wheel")
            , chassis_right_wheel_motor_(wheel_leg, command, "/wheel_leg/right_wheel")
            , chassis_left_hip_motor_(wheel_leg, command, "/wheel_leg/left_hip_joint")
            , chassis_right_hip_motor_(wheel_leg, command, "/wheel_leg/right_hip_joint")
            , chassis_left_knee_motor_(wheel_leg, command, "/wheel_leg/left_knee_joint")
            , chassis_right_knee_motor_(wheel_leg, command, "/wheel_leg/right_knee_joint")
            , gimbal_yaw_motor_(wheel_leg, command, "/gimbal/yaw")
            , gimbal_bullet_feeder_(wheel_leg, command, "/gimbal/bullet_feeder")
            , supercap_(wheel_leg, command)
            , left_tof_(wheel_leg, "/chassis/tof/left")
            , right_tof_(wheel_leg, "/chassis/tof/right")
            // Chassis sensor reads (-x, y, -z): Ry(pi).
            , bmi088_(
                  device::Bmi088Ekf::Config{
                      .body_to_sensor =
                          Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitY()}
                              .toRotationMatrix()}) {
            using namespace device;

            // Stock wheel gearboxes removed, 15.62 external reduction.
            chassis_left_wheel_motor_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 1}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(15.62));
            chassis_right_wheel_motor_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 2}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(15.62));

            chassis_left_hip_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 1, 1}
                    .set_reversed()
                    .enable_multi_turn_angle());
            chassis_right_hip_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 2, 2}
                    .set_reversed()
                    .enable_multi_turn_angle());
            chassis_left_knee_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 1, 1}
                    .set_reversed()
                    .enable_multi_turn_angle());
            chassis_right_knee_motor_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009, 2, 2}
                    .set_reversed()
                    .enable_multi_turn_angle());

            auto zero_point = int{0};
            wheel_leg.get_parameter("yaw_motor_zero_point", zero_point);
            gimbal_yaw_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(zero_point));
            // External 27:1 feeder reducer.
            gimbal_bullet_feeder_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 3}.set_reduction_ratio(27.));
            left_tof_.configure(Tof::Config{}.set_id(5));
            right_tof_.configure(Tof::Config{}.set_id(6));

            wheel_leg.register_output(
                "/wheel_leg/imu/quaternion", imu_quaternion_, Eigen::Quaterniond::Identity());
            wheel_leg.register_output(
                "/wheel_leg/imu/angular_velocity", imu_angular_velocity_, Eigen::Vector3d::Zero());
            wheel_leg.register_output("/wheel_leg/dm_control_ready", dm_control_ready_, false);
            wheel_leg.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            wheel_leg.remote_control_->register_dr16(&dr16_);
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);
            board_->start_transmit().uart_config(Spec::kUarts.kUart0, {.baudrate = 115200});
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }
        int yaw_raw_angle() const { return gimbal_yaw_motor_.last_raw_angle(); }

        void update() {
            chassis_left_wheel_motor_.update_status();
            chassis_right_wheel_motor_.update_status();
            chassis_left_hip_motor_.update_status();
            chassis_right_hip_motor_.update_status();
            chassis_left_knee_motor_.update_status();
            chassis_right_knee_motor_.update_status();
            *dm_control_ready_ =
                chassis_left_hip_motor_.enabled() && chassis_right_hip_motor_.enabled()
                && chassis_left_knee_motor_.enabled() && chassis_right_knee_motor_.enabled();

            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
            gimbal_bullet_feeder_.update_status();
            supercap_.update_status();
            left_tof_.update_status();
            right_tof_.update_status();
            // This robot has a single DR16; keep its timeout enabled.
            dr16_.set_timeout_enabled(true);
            dr16_.update_status();

            if (const auto snapshot = bmi088_.snapshot()) {
                *imu_quaternion_ = snapshot->orientation;
                *imu_angular_velocity_ = snapshot->gyro_body;
            }
        }

        void command_update() {
            using namespace device;

            auto builder = board_->start_transmit();
            builder.can_transmit(
                Spec::kCans.kCan0,
                {.can_id = 0x200,
                 .can_data =
                     CanPacket8{
                         chassis_left_wheel_motor_.generate_command(),
                         chassis_right_wheel_motor_.generate_command(),
                         gimbal_bullet_feeder_.generate_command(), CanPacket8::PaddingQuarter{}}
                         .as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan0,
                {.can_id = 0x1FE,
                 .can_data =
                     CanPacket8{
                         CanPacket8::PaddingQuarter{}, CanPacket8::PaddingQuarter{},
                         CanPacket8::PaddingQuarter{}, supercap_.generate_command()}
                         .as_bytes()});

            builder.can_transmit(
                Spec::kCans.kCan1,
                {.can_id = chassis_left_hip_motor_.send_id(),
                 .can_data = chassis_left_hip_motor_.generate_command().as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan1,
                {.can_id = chassis_right_hip_motor_.send_id(),
                 .can_data = chassis_right_hip_motor_.generate_command().as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan2,
                {.can_id = chassis_left_knee_motor_.send_id(),
                 .can_data = chassis_left_knee_motor_.generate_command().as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan2,
                {.can_id = chassis_right_knee_motor_.send_id(),
                 .can_data = chassis_right_knee_motor_.generate_command().as_bytes()});
            builder.can_transmit(
                Spec::kCans.kCan3,
                {.can_id = 0x141,
                 .can_data = gimbal_yaw_motor_.generate_torque_command().as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            const auto can_id = data.can_id;

            if (can == Spec::kCans.kCan0) {
                if (can_id == 0x201)
                    chassis_left_wheel_motor_.store_status(data.can_data);
                else if (can_id == 0x202)
                    chassis_right_wheel_motor_.store_status(data.can_data);
                else if (can_id == 0x203)
                    gimbal_bullet_feeder_.store_status(data.can_data);
                else if (can_id == 0x300)
                    supercap_.store_status(data.can_data);
                monitor_.tick("Chassis::Can0", can_id);
            } else if (can == Spec::kCans.kCan1) {
                if (can_id == chassis_left_hip_motor_.master_id())
                    chassis_left_hip_motor_.store_status(data.can_data);
                else if (can_id == chassis_right_hip_motor_.master_id())
                    chassis_right_hip_motor_.store_status(data.can_data);
                else if (can_id == left_tof_.recv_id())
                    left_tof_.store_status(data.can_data);
                else if (can_id == right_tof_.recv_id())
                    right_tof_.store_status(data.can_data);
                monitor_.tick("Chassis::Can1", can_id);
            } else if (can == Spec::kCans.kCan2) {
                if (can_id == chassis_left_knee_motor_.master_id())
                    chassis_left_knee_motor_.store_status(data.can_data);
                else if (can_id == chassis_right_knee_motor_.master_id())
                    chassis_right_knee_motor_.store_status(data.can_data);
                monitor_.tick("Chassis::Can2", can_id);
            } else if (can == Spec::kCans.kCan3) {
                if (can_id == 0x141)
                    gimbal_yaw_motor_.store_status(data.can_data);
                monitor_.tick("Chassis::Can3", can_id);
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                monitor_.tick("Chassis::Dbus", "Dr16");
            } else if (uart == Spec::kUarts.kUart0) {
                const std::byte* ptr = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&ptr](std::byte* storage) noexcept { *storage = *ptr++; },
                    data.uart_data.size());
                monitor_.tick("Chassis::Uart0", "Referee");
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
            monitor_.tick("Chassis::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            monitor_.tick("Chassis::Imu", "Gyr");
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::DjiMotor chassis_left_wheel_motor_, chassis_right_wheel_motor_;
        device::DmMotor chassis_left_hip_motor_, chassis_right_hip_motor_;
        device::DmMotor chassis_left_knee_motor_, chassis_right_knee_motor_;
        OutputInterface<bool> dm_control_ready_;

        device::LkMotor gimbal_yaw_motor_;
        device::DjiMotor gimbal_bullet_feeder_;
        device::Supercap supercap_;
        device::Tof left_tof_, right_tof_;
        device::Dr16 dr16_;

        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        OutputInterface<Eigen::Quaterniond> imu_quaternion_;
        OutputInterface<Eigen::Vector3d> imu_angular_velocity_;

        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};

        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    class GimbalBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        GimbalBoard(
            WheelLeg& wheel_leg, rmcs_executor::Component& command, std::string_view board_serial)
            : tf_(wheel_leg.tf_)
            , gimbal_pitch_motor_(wheel_leg, command, "/gimbal/pitch")
            , gimbal_left_friction_(wheel_leg, command, "/gimbal/left_friction")
            , gimbal_right_friction_(wheel_leg, command, "/gimbal/right_friction")
            // Raw (x, y, z) reads as (y, -x, z): Rz(pi/2).
            , bmi088_(
                  device::Bmi088Ekf::Config{
                      .body_to_sensor =
                          Eigen::AngleAxisd{std::numbers::pi / 2., Eigen::Vector3d::UnitZ()}
                              .toRotationMatrix()}) {
            using namespace device;

            auto zero_point = int{0};
            wheel_leg.get_parameter("pitch_motor_zero_point", zero_point);
            gimbal_pitch_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(zero_point));
            gimbal_left_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 2}.set_reduction_ratio(1.).set_reversed());
            gimbal_right_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508, 1}.set_reduction_ratio(1.));

            wheel_leg.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_, 0.);
            wheel_leg.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_, 0.);
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }
        int pitch_raw_angle() const { return gimbal_pitch_motor_.last_raw_angle(); }

        void update() {
            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());
            if (const auto snapshot = bmi088_.snapshot()) {
                *gimbal_yaw_velocity_imu_ = snapshot->gyro_body.z();
                *gimbal_pitch_velocity_imu_ = snapshot->gyro_body.y();
                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    snapshot->orientation.conjugate());
            }
        }

        void command_update() const {
            using namespace device;

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan0,
                    {.can_id = 0x142,
                     .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes()})
                .can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = 0x200,
                     .can_data =
                         CanPacket8{
                             gimbal_right_friction_.generate_command(),
                             gimbal_left_friction_.generate_command(), CanPacket8::PaddingQuarter{},
                             CanPacket8::PaddingQuarter{}}
                             .as_bytes()});
        }

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            const auto can_id = data.can_id;
            if (can == Spec::kCans.kCan0) {
                if (can_id == 0x142)
                    gimbal_pitch_motor_.store_status(data.can_data);
                monitor_.tick("Gimbal::Can0", can_id);
            } else if (can == Spec::kCans.kCan1) {
                if (can_id == 0x201)
                    gimbal_right_friction_.store_status(data.can_data);
                else if (can_id == 0x202)
                    gimbal_left_friction_.store_status(data.can_data);
                monitor_.tick("Gimbal::Can1", can_id);
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
            monitor_.tick("Gimbal::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            monitor_.tick("Gimbal::Imu", "Gyr");
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
        }

        OutputInterface<rmcs_description::Tf>& tf_;

        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_, gimbal_right_friction_;

        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        OutputInterface<double> gimbal_yaw_velocity_imu_, gimbal_pitch_velocity_imu_;

        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    struct CommandTransmitter : public rmcs_executor::Component {
        std::function<void()> fn;

        template <typename Fn>
        explicit CommandTransmitter(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        void update() override { fn(); }
    };

    void
        status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };
        text("    yaw_motor_zero_point: {}", chassis_board_->yaw_raw_angle());
        text("    pitch_motor_zero_point: {}", gimbal_board_->pitch_raw_angle());
        text("\nChassisBoard Status:");
        for (const auto& line : chassis_board_->status())
            text("> {}", line);
        text("\nGimbalBoard Status:");
        for (const auto& line : gimbal_board_->status())
            text("> {}", line);
        response->message = feedback_message.str();
    }

    void command_update() {
        chassis_board_->command_update();
        gimbal_board_->command_update();
    }

    std::shared_ptr<rmcs_executor::Component> command_component_{
        create_partner_component<CommandTransmitter>(
            get_component_name() + "_command", [this] { command_update(); })};

    OutputInterface<rmcs_description::Tf> tf_;

    std::unique_ptr<ChassisBoard> chassis_board_;
    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLeg, rmcs_executor::Component)
