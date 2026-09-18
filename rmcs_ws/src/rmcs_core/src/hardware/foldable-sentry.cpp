#include <cmath>
#include <concepts>
#include <functional>
#include <memory>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <string_view>
#include <utility>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tunnel_sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
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

        register_output("/tf", tf_);

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        gimbal_board_ = std::make_unique<GimbalBoard>(
            *this, *command_component_, get_parameter("board_serial").as_string());

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });
    }

    void update() override {
        gimbal_board_->update();
        remote_control_->update();

        // 打印日志供确定 roll_folded_angle roll_unfold_angle top_yaw_folded_angle pitch_folded_angle
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200, "roll %f pitch %f yaw %f",
            gimbal_board_->gimbal_roll_motor_.angle(), gimbal_board_->gimbal_pitch_motor_.angle(),
            gimbal_board_->gimbal_top_yaw_motor_.angle());
    }

private:
    class GimbalBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit GimbalBoard(
            FoldableSentry& sentry, rmcs_executor::Component& sentry_command,
            std::string_view board_serial = {})
            : tf_(sentry.tf_)
            , dr16_{}
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
                DjiMotor::Config{DjiMotor::Type::kM2006, 2}.set_reduction_ratio(1.));
            gimbal_left_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM2006, 3}.set_reduction_ratio(1.));
            gimbal_right_friction_.configure(
                DjiMotor::Config{DjiMotor::Type::kM2006, 4}.set_reduction_ratio(1.).set_reversed());

            // 折叠云台测试没有 bottom yaw 电机和底盘 IMU，
            // 注册常量 0 输出供 foldable-gimbal-controller 配对使用。
            sentry.register_output("/gimbal/bottom_yaw/angle", gimbal_bottom_yaw_angle_, 0.0);
            sentry.register_output("/gimbal/bottom_yaw/velocity", gimbal_bottom_yaw_velocity_, 0.0);
            sentry.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0.0);

            // 扫频测试用 top_yaw 力矩覆盖通道，未接/NaN 时回退到云台控制器输出。
            sentry_command.register_input(
                "/gimbal/top_yaw/control_torque_test", gimbal_top_yaw_test_torque_, false);

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);

            sentry.remote_control_->register_dr16(&dr16_);
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        void update() {
            using namespace rmcs_description::tunnel_sentry;

            dr16_.update_status();

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
                    Spec::kCans.kCan1, {
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
                            device::CanPacket8{
                                gimbal_right_friction_.generate_command(),
                                gimbal_left_friction_.generate_command(),
                                gimbal_top_friction_.generate_command(),
                                gimbal_bullet_feeder_.generate_command(),
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
                if(data.can_id == 0x202) {
                    gimbal_top_friction_.store_status(data.can_data);
                } else if (data.can_id == 0x203) {
                    gimbal_left_friction_.store_status(data.can_data);
                } else if (data.can_id == 0x204) {
                    gimbal_right_friction_.store_status(data.can_data);
                } else if (data.can_id == 0x201) {
                    gimbal_bullet_feeder_.store_status(data.can_data);
                }

                monitor_.tick("Gimbal::Can2", can_id);
            }

            
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                monitor_.tick("Gimbal::Dbus", "Active");
            }
        }

        OutputInterface<rmcs_description::tunnel_sentry::Tf>& tf_;

        device::Dr16 dr16_;
        device::LkMotor gimbal_roll_motor_;
        device::DjiMotor gimbal_top_yaw_motor_;
        device::LkMotor gimbal_pitch_motor_;

        OutputInterface<double> gimbal_bottom_yaw_angle_;
        OutputInterface<double> gimbal_bottom_yaw_velocity_;
        OutputInterface<double> chassis_yaw_velocity_imu_;

        device::DjiMotor gimbal_bullet_feeder_;

        device::DjiMotor gimbal_top_friction_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        InputInterface<double> gimbal_top_yaw_test_torque_;

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

        response->message = feedback_message.str();
    }

    struct CommandTransmitter : public rmcs_executor::Component {
        std::function<void()> fn;

        template <std::invocable Fn>
        explicit CommandTransmitter(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        void update() override { fn(); }
    };

    void command_update() { gimbal_board_->command_update(); }

    std::shared_ptr<rmcs_executor::Component> command_component_{
        create_partner_component<CommandTransmitter>(
            get_component_name() + "_command", [this] { command_update(); })};

    OutputInterface<rmcs_description::tunnel_sentry::Tf> tf_;

    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::FoldableSentry, rmcs_executor::Component)
