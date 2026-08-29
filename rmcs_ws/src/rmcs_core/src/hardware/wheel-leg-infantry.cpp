#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/util/status_monitor.hpp"

namespace rmcs_core::hardware {

using Clock = std::chrono::steady_clock;

/// 轮腿步兵硬件组件（照抄 deformable-infantry-omni 模式）
///
/// 机器人：6 DOF 轮腿
///   髋 ×4：达妙 DM-J8009-2EC（MIT 模式，每电机一帧，帧 ID = CAN ID）
///   轮 ×2：DJI M3508 + 减速箱（0x200 双电机帧）
///   板：rmcs_board_lite（CAN0-3 透传 + 板载 BMI088 + DBus UART）
///
/// 接口合同（与 wheel_leg_rl_controller 配对）：
///   输出：/wheel_leg/{rf0,rf1,r_wheel,lf0,lf1,l_wheel}/{angle,velocity,torque,...}
///         /wheel_leg/imu/quaternion（Eigen::Quaterniond，世界→机体，wxyz）
///         /wheel_leg/imu/angular_velocity（Eigen::Vector3d，机体系 rad/s）
///   输入：/wheel_leg/{...}/control_torque（DmMotor/DjiMotor 在 Command partner 上注册）
///
/// CAN 通道分配（须与实车走线一致）：
///   Can0：髋 rf0 + 轮 0x200 帧     Can1：髋 rf1
///   Can2：髋 lf0                   Can3：髋 lf1
class WheelLegInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit WheelLegInfantry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        std::string serial_filter;
        get_parameter_or("serial_filter", serial_filter, std::string{});
        board_ = std::make_unique<Board>(*this, *command_, serial_filter);

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });
    }

    ~WheelLegInfantry() override = default;

    void before_updating() override { board_->request_hard_sync_read(); }

    void update() override {
        board_->update();
        remote_control_->update();
    }

    void command_update() { board_->command_update(); }

private:
    static constexpr std::size_t kHipCount = 4;  // rf0, rf1, lf0, lf1（训练 DOF 序腿部分）
    static constexpr std::size_t kWheelCount = 2; // r_wheel, l_wheel

    class Command : public rmcs_executor::Component {
    public:
        explicit Command(WheelLegInfantry& wheelLegInfantry)
            : wheelLegInfantry(wheelLegInfantry) {}

        void update() override { wheelLegInfantry.command_update(); }

        WheelLegInfantry& wheelLegInfantry;
    };

    struct Board final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit Board(
            WheelLegInfantry& status, rmcs_executor::Component& command,
            const std::string& serial_filter = {})
            : status_{status}
            , command_{command}
            , imu_(1000, 0.2, 0.0)
            , hip_motors_{
                  device::DmMotor{status, command, "/wheel_leg/rf0"},
                  device::DmMotor{status, command, "/wheel_leg/rf1"},
                  device::DmMotor{status, command, "/wheel_leg/lf0"},
                  device::DmMotor{status, command, "/wheel_leg/lf1"},
              }
            , wheel_motors_{
                  device::DjiMotor{status, command, "/wheel_leg/r_wheel"},
                  device::DjiMotor{status, command, "/wheel_leg/l_wheel"},
              } {

            // ---- 髋电机配置（达妙 MIT）----
            const auto hip_ids =
                read_uint8_array(status, "hip_motor_id", std::array<std::uint8_t, 4>{1, 2, 3, 4});
            const auto hip_feedback_ids = read_uint8_array(
                status, "hip_motor_feedback_id", std::array<std::uint8_t, 4>{0x11, 0x12, 0x13, 0x14});
            const auto hip_reversed = read_bool_array(
                status, "hip_reversed", std::array<bool, 4>{false, false, false, false});
            const auto hip_angle_bias = read_double_array(
                status, "hip_angle_bias", std::array<double, 4>{0.0, 0.0, 0.0, 0.0});

            double hip_position_max = 12.5;
            double hip_velocity_max = 45.0;
            double hip_torque_max = 54.0;
            double hip_control_torque_max = 40.0;
            status.get_parameter_or("hip_position_max", hip_position_max, 12.5);
            status.get_parameter_or("hip_velocity_max", hip_velocity_max, 45.0);
            status.get_parameter_or("hip_torque_max", hip_torque_max, 54.0);
            status.get_parameter_or("hip_control_torque_max", hip_control_torque_max, 40.0);

            for (std::size_t i = 0; i < kHipCount; ++i) {
                hip_motors_[i].configure(
                    device::DmMotor::Config{device::DmMotor::Type::kDM8009}
                        .set_id(hip_ids[i])
                        .set_feedback_id(hip_feedback_ids[i])
                        .set_reversed(hip_reversed[i])
                        .set_angle_bias(hip_angle_bias[i])
                        .set_limits(hip_position_max, hip_velocity_max, hip_torque_max)
                        .set_control_torque_max(hip_control_torque_max));
            }

            // ---- 轮电机配置（DJI M3508 + 减速箱）----
            const auto wheel_ids =
                read_uint8_array(status, "wheel_motor_id", std::array<std::uint8_t, 2>{1, 2});
            const auto wheel_reversed =
                read_bool_array(status, "wheel_reversed", std::array<bool, 2>{false, false});
            double wheel_reduction_ratio = 16.33;
            status.get_parameter_or("wheel_reduction_ratio", wheel_reduction_ratio, 16.33);

            for (std::size_t i = 0; i < kWheelCount; ++i) {
                auto config =
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508, wheel_ids[i]}
                        .set_reduction_ratio(wheel_reduction_ratio)
                        .enable_multi_turn_angle();
                if (wheel_reversed[i])
                    config.set_reversed();
                wheel_motors_[i].configure(config);
            }

            // ---- IMU 输出（RL 控制器合同）----
            // 板载 BMI088 Mahony 四元数（w,x,y,z），约定世界→机体；
            // 实车 IMU 安装方向需在装车后用 set_coordinate_mapping 对齐机体坐标系。
            status.register_output(
                "/wheel_leg/imu/quaternion", imu_quaternion_output_, Eigen::Quaterniond::Identity());
            status.register_output(
                "/wheel_leg/imu/angular_velocity", imu_angular_velocity_output_,
                Eigen::Vector3d::Zero());

            status.get_parameter_or("debug_log_motor", debug_log_motor_, false);

            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

            status_.remote_control_->register_dr16(&dr16_);
        }

        ~Board() override = default;

        void request_hard_sync_read() {
            // rmcs_board_lite 无 GPIO hard-sync 请求路径（与 deformable TopBoard 一致）
        }

        void update() {
            imu_.update_status();
            for (auto& motor : hip_motors_)
                motor.update_status();
            for (auto& motor : wheel_motors_)
                motor.update_status();
            dr16_.update_status();

            publish_imu_();
            if (debug_log_motor_)
                log_motor_feedback_once_per_second_();
        }

        void command_update() {
            auto builder = board_->start_transmit();

            // 髋：每电机一帧（MIT，帧 ID = 电机 CAN ID）
            for (std::size_t i = 0; i < kHipCount; ++i) {
                builder.can_transmit(
                    hip_can(i), //
                    {
                        .can_id = hip_motors_[i].send_id(),
                        .can_data = hip_motors_[i].generate_command().as_bytes(),
                    });
            }
            // 轮：DJI 0x200 双电机帧（r, l）
            builder.can_transmit(
                Spec::kCans.kCan0, //
                {
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            wheel_motors_[0].generate_command(),
                            wheel_motors_[1].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            for (std::size_t i = 0; i < kHipCount; ++i) {
                if (hip_motors_[i].match_then_store_status(data.can_id, data.can_data)) {
                    hip_status_received_[i].store(true, std::memory_order_relaxed);
                    monitor_.tick("Board::Can", data.can_id);
                    return;
                }
            }
            for (std::size_t i = 0; i < kWheelCount; ++i) {
                if (wheel_motors_[i].match_then_store_status(data.can_id, data.can_data)) {
                    wheel_status_received_[i].store(true, std::memory_order_relaxed);
                    monitor_.tick("Board::Can", data.can_id);
                    return;
                }
            }
            (void)can;
            monitor_.tick("Board::Can", data.can_id);
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                monitor_.tick("Board::Dbus", "Active");
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
            monitor_.tick("Board::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
            monitor_.tick("Board::Imu", "Gyr");
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        void publish_imu_() {
            // Bmi088 的 q0..q3 为 Mahony 四元数（w,x,y,z），约定：世界→机体
            *imu_quaternion_output_ =
                Eigen::Quaterniond(imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()).normalized();
            *imu_angular_velocity_output_ = Eigen::Vector3d(imu_.gx(), imu_.gy(), imu_.gz());
        }

        void log_motor_feedback_once_per_second_() {
            const auto now = Clock::now();
            if (now < next_motor_log_time_)
                return;

            std::string hip_rx;
            for (std::size_t i = 0; i < kHipCount; ++i) {
                hip_rx.push_back(hip_status_received_[i].load(std::memory_order_relaxed) ? 'Y' : 'N');
                hip_rx.push_back(' ');
            }
            std::string wheel_rx;
            for (std::size_t i = 0; i < kWheelCount; ++i) {
                wheel_rx.push_back(
                    wheel_status_received_[i].load(std::memory_order_relaxed) ? 'Y' : 'N');
                wheel_rx.push_back(' ');
            }

            RCLCPP_INFO(
                status_.get_logger(),
                "[wheel-leg motor] hip angle(rad) rf0=% .3f rf1=% .3f lf0=% .3f lf1=% .3f | "
                "wheel vel(rad/s) r=% .3f l=% .3f | hip fault=%d/%d/%d/%d | rx hip=[%s] wheel=[%s]",
                hip_motors_[0].angle(), hip_motors_[1].angle(), hip_motors_[2].angle(),
                hip_motors_[3].angle(), wheel_motors_[0].velocity(), wheel_motors_[1].velocity(),
                hip_motors_[0].fault_code(), hip_motors_[1].fault_code(),
                hip_motors_[2].fault_code(), hip_motors_[3].fault_code(), hip_rx.c_str(),
                wheel_rx.c_str());

            next_motor_log_time_ = now + std::chrono::seconds(1);
        }

        template <std::size_t N>
        static auto read_uint8_array(
            rclcpp::Node& node, const std::string& name,
            const std::array<std::uint8_t, N>& defaults) -> std::array<std::uint8_t, N> {
            std::vector<std::int64_t> values;
            if (!node.get_parameter(name, values) || values.size() != N) {
                RCLCPP_WARN(
                    node.get_logger(), "Parameter '%s' missing or wrong size, using default",
                    name.c_str());
                return defaults;
            }
            std::array<std::uint8_t, N> result{};
            for (std::size_t i = 0; i < N; ++i)
                result[i] = static_cast<std::uint8_t>(values[i]);
            return result;
        }

        template <std::size_t N>
        static auto read_bool_array(
            rclcpp::Node& node, const std::string& name,
            const std::array<bool, N>& defaults) -> std::array<bool, N> {
            std::vector<bool> values;
            if (!node.get_parameter(name, values) || values.size() != N) {
                RCLCPP_WARN(
                    node.get_logger(), "Parameter '%s' missing or wrong size, using default",
                    name.c_str());
                return defaults;
            }
            std::array<bool, N> result{};
            for (std::size_t i = 0; i < N; ++i)
                result[i] = values[i];
            return result;
        }

        template <std::size_t N>
        static auto read_double_array(
            rclcpp::Node& node, const std::string& name,
            const std::array<double, N>& defaults) -> std::array<double, N> {
            std::vector<double> values;
            if (!node.get_parameter(name, values) || values.size() != N) {
                RCLCPP_WARN(
                    node.get_logger(), "Parameter '%s' missing or wrong size, using default",
                    name.c_str());
                return defaults;
            }
            std::array<double, N> result{};
            for (std::size_t i = 0; i < N; ++i)
                result[i] = values[i];
            return result;
        }

        // 髋电机 CAN 通道分配（与实车走线一致）
        static const Spec::Can& hip_can(std::size_t index) {
            switch (index) {
            case 0: return Spec::kCans.kCan0;
            case 1: return Spec::kCans.kCan1;
            case 2: return Spec::kCans.kCan2;
            default: return Spec::kCans.kCan3;
            }
        }

        WheelLegInfantry& status_;
        rmcs_executor::Component& command_;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

        // 接口
        rmcs_executor::Component::OutputInterface<Eigen::Quaterniond> imu_quaternion_output_;
        rmcs_executor::Component::OutputInterface<Eigen::Vector3d> imu_angular_velocity_output_;

        // 设备
        device::Bmi088 imu_;
        device::Dr16 dr16_{};
        std::array<device::DmMotor, kHipCount> hip_motors_;
        std::array<device::DjiMotor, kWheelCount> wheel_motors_;

        std::array<std::atomic<bool>, kHipCount> hip_status_received_{};
        std::array<std::atomic<bool>, kWheelCount> wheel_status_received_{};

        bool debug_log_motor_ = false;
        Clock::time_point next_motor_log_time_{Clock::now() + std::chrono::seconds(1)};

        StatusMonitor monitor_{};
    };

    auto status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
        -> void {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("Hip motors (DM-J8009-2EC, MIT):");
        for (std::size_t i = 0; i < kHipCount; ++i) {
            text(
                "  hip[{}] can_id={:#x} feedback_id={:#x} angle={: .3f} rad vel={: .3f} rad/s "
                "torque={: .3f} Nm fault={:#x} rx={}",
                i, board_->hip_motors_[i].send_id(), board_->hip_motors_[i].feedback_id(),
                board_->hip_motors_[i].angle(), board_->hip_motors_[i].velocity(),
                board_->hip_motors_[i].torque(), board_->hip_motors_[i].fault_code(),
                board_->hip_status_received_[i].load(std::memory_order_relaxed) ? "Y" : "N");
        }
        text("Wheel motors (M3508):");
        for (std::size_t i = 0; i < kWheelCount; ++i) {
            text(
                "  wheel[{}] angle={: .3f} rad vel={: .3f} rad/s torque={: .3f} Nm rx={}",
                i, board_->wheel_motors_[i].angle(), board_->wheel_motors_[i].velocity(),
                board_->wheel_motors_[i].torque(),
                board_->wheel_status_received_[i].load(std::memory_order_relaxed) ? "Y" : "N");
        }

        text("\nBoard Status:");
        for (const auto& line : board_->status())
            text("> {}", line);

        response->message = feedback_message.str();
    }

    std::unique_ptr<Board> board_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::shared_ptr<Command> command_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantry, rmcs_executor::Component)
