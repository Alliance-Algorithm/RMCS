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

class Test
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    Test()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        gimbal_board_ = std::make_unique<GimbalBoard>(
            *this, *command_component_, get_parameter("board_serial_gimbal_board").as_string());

        register_output("/gimbal/friction_ready", friction_ready_, true);
        register_output("/gimbal/friction_jammed", friction_jammed_, false);
        register_output("/gimbal/bullet_fired", bullet_fired_, false);
        register_output("/gimbal/control_bullet_allowance/limited_by_heat",control_bullet_allowance_,int64_t{3});
    }

    void update() override {
        gimbal_board_->update();
        remote_control_->update();

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Bullet feeder: angle=%.3f rad, velocity=%.3f rad/s, torque=%.3f N*m,commond_torque=%.3f",
            gimbal_board_->gimbal_bullet_feeder_.angle(),
            gimbal_board_->gimbal_bullet_feeder_.velocity(),
            gimbal_board_->gimbal_bullet_feeder_.torque(),
            gimbal_board_->gimbal_bullet_feeder_.control_torque());
    }

private:
    class GimbalBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit GimbalBoard(
            Test& test, rmcs_executor::Component& test_command,
            std::string_view board_serial = {})
            : dr16_{}
            , gimbal_bullet_feeder_(test, test_command, "/gimbal/bullet_feeder"){

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 1}
                    .set_reduction_ratio(36.0));

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);

            test.remote_control_->register_dr16(&dr16_);
            }

        void update() {
            using namespace rmcs_description::tunnel_sentry;

            gimbal_bullet_feeder_.update_status();

            dr16_.update_status();
        }

        void command_update() const {
            using namespace device;

            board_->start_transmit()
                .can_transmit(
                    Spec::kCans.kCan1,
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

            if (can == Spec::kCans.kCan1) {
                if (can_id == 0x201) {
                    gimbal_bullet_feeder_.store_status(can_data);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
            }
        }

        device::Dr16 dr16_;

        device::DjiMotor gimbal_bullet_feeder_;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

    };

    struct CommandTransmitter : public rmcs_executor::Component {
        std::function<void()> fn;

        template <std::invocable Fn>
        explicit CommandTransmitter(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        void update() override { fn(); }
    };

    void command_update() {
        gimbal_board_->command_update();
    }

    OutputInterface<bool> friction_ready_;
    OutputInterface<bool> friction_jammed_;
    OutputInterface<bool> bullet_fired_;

    OutputInterface<int64_t> control_bullet_allowance_;

    std::shared_ptr<rmcs_executor::Component> command_component_{
        create_partner_component<CommandTransmitter>(
            get_component_name() + "_command", [this] { command_update(); })};

    std::unique_ptr<GimbalBoard> gimbal_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Test, rmcs_executor::Component)
