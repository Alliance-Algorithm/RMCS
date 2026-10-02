#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <ranges>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tunnel_sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/util/status_monitor.hpp"

namespace rmcs_core::hardware {

class FoldableSentry
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    FoldableSentry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

        constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();

        register_output("/tf", tf_);
        register_output("/auto_aim/camera_transform", camera_transform_);
        register_output("/auto_aim/barrel_direction", barrel_direction_);
        register_output("/auto_aim/yaw_velocity", yaw_velocity_, kNaN);

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        gimbal_board_ = std::make_unique<GimbalBoard>(
            *this, *command_component_, get_parameter("board_serial_gimbal_board").as_string());

        chassis_board_ = std::make_unique<ChassisBoard>(
            *this, *command_component_, get_parameter("board_serial_chassis_board").as_string());

        using namespace rmcs_description::tunnel_sentry;

        // 占位
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.0, 0.0, 0.0});

        // 占位
        tf_->set_transform<BottomYawLink, RollLink>(Eigen::Translation3d{0.0, 0.0, 0.0});
        tf_->set_transform<RollLink, TopYawLink>(Eigen::Translation3d{0.0, 0.0, 0.0});
        tf_->set_transform<TopYawLink, PitchLink>(Eigen::Translation3d{0.0, 0.0, 0.0});

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });
    }

    void update() override {
        gimbal_board_->update();
        chassis_board_->update();
        remote_control_->update();

        using namespace rmcs_description::tunnel_sentry;
        *camera_transform_ = fast_tf::lookup_transform<OdomGimbalImu, CameraLink>(*tf_);
        *barrel_direction_ = *fast_tf::cast<OdomGimbalImu>(
            PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        *yaw_velocity_ = gimbal_board_->yaw_velocity();

        // 打印日志供确定 roll_folded_angle roll_unfold_angle top_yaw_folded_angle pitch_folded_angle
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200, "roll %f pitch %f yaw %f",
            gimbal_board_->gimbal_roll_motor_.angle(),
            gimbal_board_->gimbal_pitch_motor_.angle(),
            gimbal_board_->gimbal_top_yaw_motor_.angle());
    }

private:
    class GimbalBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit GimbalBoard(
            FoldableSentry& sentry, rmcs_executor::Component& sentry_command,
            std::string_view board_serial = {})
            : tf_(sentry.tf_)
            , bmi088_{device::Bmi088Ekf::Config{
                  .body_to_sensor =
                      Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitZ()}
                          .toRotationMatrix()}}
            , gimbal_roll_motor_(sentry, sentry_command, "/gimbal/roll")
            , gimbal_top_yaw_motor_(sentry, sentry_command, "/gimbal/top_yaw")
            , gimbal_pitch_motor_(sentry, sentry_command, "/gimbal/pitch")
            , gimbal_bullet_feeder_(sentry, sentry_command, "/gimbal/bullet_feeder")
            , gimbal_top_friction_(sentry, sentry_command, "/gimbal/top_friction")
            , gimbal_left_friction_(sentry, sentry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(sentry, sentry_command, "/gimbal/right_friction") {

            using namespace device;

            auto zero_point = int{0};
            sentry.get_parameter("roll_motor_zero_point", zero_point);
            gimbal_roll_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG5010Ei10}.set_encoder_zero_point(zero_point));

            sentry.get_parameter("top_yaw_motor_zero_point", zero_point);
            gimbal_top_yaw_motor_.configure(
                DjiMotor::Config{DjiMotor::Type::kGM6020, 1}.set_encoder_zero_point(zero_point));

            sentry.get_parameter("pitch_motor_zero_point", zero_point);
            gimbal_pitch_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(zero_point));

            gimbal_bullet_feeder_.configure(
                DjiMotor::Config{DjiMotor::Type::kM2006, 1}
                    .set_reversed()
                    .set_reduction_ratio(36.0));

            gimbal_top_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM2006, 2}.set_reduction_ratio(1.0));
            gimbal_left_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM2006, 3}.set_reduction_ratio(1.0));
            gimbal_right_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM2006, 4}
                    .set_reduction_ratio(1.0)
                    .set_reversed());

            // bmi088_.configure{};

            sentry.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_, 0.0);
            sentry.register_output(
                "/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_, 0.0);
            sentry.register_output("/gimbal/auto_aim/exposure_signal", camera_signal_output_);
            sentry.register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot_output_);

            // 扫频测试用 top_yaw 力矩覆盖通道，未接/NaN 时回退到云台控制器输出。
            sentry_command.register_input(
                "/gimbal/top_yaw/control_torque_test", gimbal_top_yaw_test_torque_, false);

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);
            board_->start_transmit().gpio_digital_read(
                Spec::kGpios.kUart1Rx, {
                                           .period_ms = 0,
                                           .asap = false,
                                           .rising_edge = false,
                                           .falling_edge = true,
                                           .capture_timestamp = true,
                                           .pull = librmcs::data::GpioPull::kUp,
                                       });
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        auto yaw_velocity() const -> double { return *gimbal_yaw_velocity_bmi088_; }

        void update() {
            using namespace rmcs_description::tunnel_sentry;

            gimbal_bullet_feeder_.update_status();
            gimbal_top_friction_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            gimbal_roll_motor_.update_status();
            tf_->set_state<BottomYawLink, RollLink>(gimbal_roll_motor_.angle());

            gimbal_top_yaw_motor_.update_status();
            tf_->set_state<RollLink, TopYawLink>(gimbal_top_yaw_motor_.angle());

            gimbal_pitch_motor_.update_status();
            const auto pitch_angle =
                std::remainder(gimbal_pitch_motor_.angle(), 2.0 * std::numbers::pi);
            tf_->set_state<TopYawLink, PitchLink>(pitch_angle);

            if (const auto snapshot = bmi088_.snapshot()) {
                tf_->set_transform<PitchLink, OdomGimbalImu>(snapshot->orientation.conjugate());
                *gimbal_yaw_velocity_bmi088_ = snapshot->gyro_body.z();
                *gimbal_pitch_velocity_bmi088_ = snapshot->gyro_body.y();
            }
        }

        void command_update() const {
            using namespace device;

            const auto top_yaw_command =
                gimbal_top_yaw_test_torque_.ready() && std::isfinite(*gimbal_top_yaw_test_torque_)
                    ? gimbal_top_yaw_motor_.generate_command(*gimbal_top_yaw_test_torque_)
                    : gimbal_top_yaw_motor_.generate_command();

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan0,
                    {
                        .can_id = 0x141,
                        .can_data = gimbal_roll_motor_.generate_torque_command().as_bytes(),
                    })
                .can_transmit(
                    Spec::kCans.kCan0,
                    {
                        .can_id = 0x142,
                        .can_data = gimbal_pitch_motor_.generate_torque_command().as_bytes(),
                    })
                .can_transmit(
                    Spec::kCans.kCan1,
                    {
                        .can_id = 0x1FE,
                        .can_data =
                            CanPacket8{
                                top_yaw_command,
                                CanPacket8::PaddingQuarter{},
                                CanPacket8::PaddingQuarter{},
                                CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    })
                .can_transmit(
                    Spec::kCans.kCan2,
                    {
                        .can_id = gimbal_right_friction_.send_id(),
                        .can_data =
                            CanPacket8{
                                CanPacket8::PaddingQuarter{},
                                gimbal_top_friction_.generate_command(),
                                gimbal_left_friction_.generate_command(),
                                gimbal_right_friction_.generate_command(),
                            }
                                .as_bytes(),
                    })
                .can_transmit(
                    Spec::kCans.kCan3,
                    {
                        .can_id = gimbal_bullet_feeder_.send_id(),
                        .can_data =
                            CanPacket8{
                                gimbal_bullet_feeder_.generate_command(),
                                CanPacket8::PaddingQuarter{},
                                CanPacket8::PaddingQuarter{},
                                CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            const auto& can_id = data.can_id;
            const auto& can_data = data.can_data;

            if (can == Spec::kCans.kCan0) {
                if (can_id == 0x141) {
                    gimbal_roll_motor_.store_status(can_data);
                } else if (can_id == 0x142) {
                    gimbal_pitch_motor_.store_status(can_data);
                }

                monitor_.tick("Gimbal::Can0", can_id);
            } else if (can == Spec::kCans.kCan1) {
                if (can_id == 0x205) {
                    gimbal_top_yaw_motor_.store_status(can_data);
                }

                monitor_.tick("Gimbal::Can1", can_id);
            } else if (can == Spec::kCans.kCan2) {
                if (can_id == 0x202) {
                    gimbal_top_friction_.store_status(can_data);
                } else if (can_id == 0x203) {
                    gimbal_left_friction_.store_status(can_data);
                } else if (can_id == 0x204) {
                    gimbal_right_friction_.store_status(can_data);
                }

                monitor_.tick("Gimbal::Can2", can_id);
            } else if (can == Spec::kCans.kCan3) {
                if (can_id == 0x201) {
                    gimbal_bullet_feeder_.store_status(can_data);
                }

                monitor_.tick("Gimbal::Can3", can_id);
            }
        }

        void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) override {
            if (!data.timestamp_quarter_us)
                return;

            if (gpio == Spec::kGpios.kUart1Rx) {
                if (data.high)
                    return;

                const auto timestamp =
                    board_clock_lifter_.lift_timestamp(*data.timestamp_quarter_us);
                if (!timestamp.has_value())
                    return;

                camera_signal_output_.emit(*timestamp);
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

            auto snapshot =
                bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
            if (!snapshot)
                return;

            imu_snapshot_output_.emit(*snapshot);
        }

        OutputInterface<rmcs_description::tunnel_sentry::Tf>& tf_;

        device::Bmi088Ekf bmi088_;
        device::LkMotor gimbal_roll_motor_;
        device::DjiMotor gimbal_top_yaw_motor_;
        device::LkMotor gimbal_pitch_motor_;

        device::DjiMotor gimbal_bullet_feeder_;

        device::DjiMotor gimbal_top_friction_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        device::BoardClockLifter board_clock_lifter_;

        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_bmi088_;

        EventOutputInterface<rmcs_msgs::BoardClock::time_point> camera_signal_output_;
        EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_output_;

        InputInterface<double> gimbal_top_yaw_test_torque_;

        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    class ChassisBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit ChassisBoard(
            FoldableSentry& sentry, rmcs_executor::Component& sentry_command,
            std::string_view board_serial = {})
            : tf_(sentry.tf_)
            , dr16_{}
            , gimbal_bottom_yaw_motor_(sentry, sentry_command, "/gimbal/bottom_yaw")
            , chassis_wheel_motors_(
                  {sentry, sentry_command, "/chassis/left_front_wheel"},
                  {sentry, sentry_command, "/chassis/left_back_wheel"},
                  {sentry, sentry_command, "/chassis/right_back_wheel"},
                  {sentry, sentry_command, "/chassis/right_front_wheel"})
            , supercap_(sentry, sentry_command) {

            using namespace device;

            sentry.register_output("/referee/serial", referee_serial_);
            sentry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0.0);
            sentry.register_output("/chassis/pitch_imu", chassis_pitch_imu_, 0.0);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            const auto zero_point = sentry.get_parameter("bottom_yaw_motor_zero_point").as_int();
            gimbal_bottom_yaw_motor_.configure(
                LkMotor::Config{LkMotor::Type::kMG6012Ei8}.set_reversed().set_encoder_zero_point(
                    static_cast<int>(zero_point)));

            constexpr auto kWheelIds = std::array<std::uint8_t, 4>{1, 2, 3, 4};
            for (auto&& [motor, id] : std::views::zip(chassis_wheel_motors_, kWheelIds)) {
                motor.configure(
                    DjiMotor::Config{DjiMotor::Type::kM3508, id}
                        .set_reduction_ratio(13.0)
                        .enable_multi_turn_angle()
                        .set_reversed());
            }

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);

            sentry.remote_control_->register_dr16(&dr16_);
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        void update() {
            using namespace rmcs_description::tunnel_sentry;

            gimbal_bottom_yaw_motor_.update_status();
            tf_->set_state<GimbalCenterLink, YawLink>(gimbal_bottom_yaw_motor_.angle());

            dr16_.update_status();
            supercap_.update_status();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            tf_->set_state<BaseLink, LeftFrontWheelLink>(chassis_wheel_motors_[0].angle());
            tf_->set_state<BaseLink, LeftBackWheelLink>(chassis_wheel_motors_[1].angle());
            tf_->set_state<BaseLink, RightBackWheelLink>(chassis_wheel_motors_[2].angle());
            tf_->set_state<BaseLink, RightFrontWheelLink>(chassis_wheel_motors_[3].angle());

            if (const auto snapshot = bmi088_.snapshot()) {
                const auto& q = snapshot->orientation;
                *chassis_pitch_imu_ = -std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
                *chassis_yaw_velocity_imu_ = snapshot->gyro_body.z();
            }
        }

        void command_update() {
            using namespace device;

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan0,
                    {
                        .can_id = chassis_wheel_motors_[0].send_id(),
                        .can_data =
                            CanPacket8{
                                chassis_wheel_motors_[0].generate_command(),
                                chassis_wheel_motors_[1].generate_command(),
                                chassis_wheel_motors_[2].generate_command(),
                                chassis_wheel_motors_[3].generate_command(),
                            }
                                .as_bytes(),
                    })
                .can_transmit(
                    Spec::kCans.kCan1,
                    {
                        .can_id = 0x1FE,
                        .can_data =
                            CanPacket8{
                                CanPacket8::PaddingQuarter{},
                                CanPacket8::PaddingQuarter{},
                                CanPacket8::PaddingQuarter{},
                                supercap_.generate_command(),
                            }
                                .as_bytes(),
                    })
                .can_transmit(
                    Spec::kCans.kCan2,
                    {
                        .can_id = 0x141,
                        .can_data = gimbal_bottom_yaw_motor_.generate_command().as_bytes(),
                    });
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            const auto& can_id = data.can_id;
            const auto& can_data = data.can_data;

            if (can == Spec::kCans.kCan0) {
                /*^^*/ chassis_wheel_motors_[0].match_then_store_status(can_id, can_data)
                    || chassis_wheel_motors_[1].match_then_store_status(can_id, can_data)
                    || chassis_wheel_motors_[2].match_then_store_status(can_id, can_data)
                    || chassis_wheel_motors_[3].match_then_store_status(can_id, can_data);

                monitor_.tick("Chassis::Can0", can_id);
            } else if (can == Spec::kCans.kCan1) {
                if (can_id == 0x300) {
                    supercap_.store_status(can_data);
                }

                monitor_.tick("Chassis::Can1", can_id);
            } else if (can == Spec::kCans.kCan2) {
                if (can_id == 0x141) {
                    gimbal_bottom_yaw_motor_.store_status(can_data);
                }

                monitor_.tick("Chassis::Can2", can_id);
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                monitor_.tick("Chassis::Dbus", "Active");
            } else if (uart == Spec::kUarts.kUart0) {
                const auto* uart_data = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
                    data.uart_data.size());
                monitor_.tick("Chassis::Uart0", "Active");
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

        OutputInterface<rmcs_description::tunnel_sentry::Tf>& tf_;

        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        device::Dr16 dr16_;
        device::LkMotor gimbal_bottom_yaw_motor_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::Supercap supercap_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_imu_;

        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    void
        status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text(
            "    bottom_yaw_motor_zero_point: {}",
            chassis_board_->gimbal_bottom_yaw_motor_.last_raw_angle());
        text("    pitch_motor_zero_point: {}", gimbal_board_->gimbal_pitch_motor_.last_raw_angle());
        text(
            "    top_yaw_motor_zero_point: {}",
            gimbal_board_->gimbal_top_yaw_motor_.last_raw_angle());
        text(
            "    roll_motor_zero_point: {}",
            gimbal_board_->gimbal_roll_motor_.last_raw_angle());

        text("\nGimbalBoard Status:");
        for (const auto& line : gimbal_board_->status()) {
            text("> {}", line);
        }

        text("\nChassisBoard Status:");
        for (const auto& line : chassis_board_->status()) {
            text("> {}", line);
        }

        response->message = feedback_message.str();
    }

    struct CommandTransmitter : public rmcs_executor::Component {
        std::function<void()> fn;

        template <std::invocable Fn>
        explicit CommandTransmitter(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        void update() override { fn(); }
    };

    void command_update() {
        gimbal_board_->command_update();
        chassis_board_->command_update();
    }

    std::shared_ptr<rmcs_executor::Component> command_component_{
        create_partner_component<CommandTransmitter>(
            get_component_name() + "_command", [this] { command_update(); })};

    OutputInterface<rmcs_description::tunnel_sentry::Tf> tf_;

    OutputInterface<Eigen::Isometry3d> camera_transform_;
    OutputInterface<Eigen::Vector3d> barrel_direction_;
    OutputInterface<double> yaw_velocity_;

    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<ChassisBoard> chassis_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::FoldableSentry, rmcs_executor::Component)
