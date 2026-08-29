#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <sstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

/// LK MG4010Ei10 单电机测试组件（rmcs_board_lite + can0 + LK ID=1，DR16 遥控控制）
///
/// 测试目标：
///   1. 电机可用性：板子连通 → 启动(0x88) → 反馈帧接收 → 扭矩/速度/位置下发
///   2. 参数校验：LK 4010i10 扭矩常数 Kt=0.07 Nm/A（电机侧）、减速比 10:1、
///      输出侧约 0.7 Nm/A、峰值扭矩 4.5 Nm（见 lk_motor.hpp 的 Type::kMG4010Ei10 分支）；
///      驱动用该 Kt 做电流↔扭矩换算，反馈扭矩即 Kt×电流×减速比，可实测核对。
///
/// LK 协议（CAN 标准帧，命令与反馈同帧 ID）：
///   帧 ID = 0x140 + 电机 ID（本组件 motor_id=1 → 0x141）
///   0x88 启动（使能） / 0xA1 扭矩电流 / 0xA2 速度 / 0xA3(+0xA4) 绝对角度+限速
///   失能 = 0xA1 电流 0（持续下发零扭矩）
///
/// DR16 遥控映射（SWA=左开关，SWD=右开关）：
///   双下（左 DWN + 右 DWN）       → 电机失能（0xA1 电流 0）
///   其余任意组合                  → 电机启动/使能（0x88）
///   双中（左 MID + 右 MID）       → 位置速度模式：目标角度 = 左摇杆X × position_scale，
///                                   限速 = |左摇杆Y| × velocity_scale（0xA3）
///   左上右中（左 UP + 右 MID）    → 速度模式：v_des = 右摇杆Y × velocity_scale，
///                                   扭矩限幅 = velocity_torque_limit（0xA2）
///   左中右上（左 MID + 右 UP）    → 扭矩模式（MIT）：t_ff = 右摇杆Y × torque_scale（0xA1）
///   其他组合                      → 使能，保持上一模式
///   遥控丢失（>500ms 无帧）       → 自动失能（安全保护）
///
/// 输出（LkMotor 驱动注册）：/dm_motor_test/motor/{angle, raw_angle, velocity,
///                              torque, temperature, max_torque}
/// 服务：/rmcs/service/dm_motor_test_status（Trigger）—— 一次查看全部状态与参数
/// 日志：每秒打印开关/摇杆/模式/下发中间值(angle,v_des,t_ff,limits)/命令帧字节/反馈值
class Dm_motor_test
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    Dm_motor_test()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , motor_(*this, *command_, "/dm_motor_test/motor") {

        // ---- 参数 ----
        std::string board_serial;
        get_parameter_or("board_serial", board_serial, std::string{});
        int motor_id = 1;
        int encoder_zero_point = 0;
        get_parameter_or("motor_id", motor_id, 1);
        get_parameter_or("encoder_zero_point", encoder_zero_point, 0);
        get_parameter_or("motor_reversed", reversed_, false);
        get_parameter_or("multi_turn_angle", multi_turn_angle_, true);
        get_parameter_or("position_scale", position_scale_, 3.0);
        get_parameter_or("velocity_scale", velocity_scale_, 5.0);
        get_parameter_or("torque_scale", torque_scale_, 4.5);
        get_parameter_or("velocity_torque_limit", velocity_torque_limit_, 4.5);
        get_parameter_or("log_rate", log_rate_, 1.0);

        // 防御：log_rate<=0 会导致日志退化为每周期打印
        if (log_rate_ <= 0.0)
            log_rate_ = 1.0;

        can_id_ = 0x140u + static_cast<std::uint32_t>(motor_id);

        auto config = device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                          .set_encoder_zero_point(encoder_zero_point);
        if (reversed_)
            config.set_reversed();
        if (multi_turn_angle_)
            config.enable_multi_turn_angle();
        motor_.configure(config);

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/dm_motor_test_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        // 遥控超时保护：>500ms 无 DR16 帧 → valid=false → 自动失能
        dr16_.set_timeout_enabled(true);

        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);

        RCLCPP_INFO(
            logger_,
            "[dm motor test] LK MG4010Ei10 id=%u can_id=0x%03X | Kt=0.07 Nm/A(电机侧) "
            "gear=10:1 max_torque=%.2f Nm",
            static_cast<unsigned>(motor_id), static_cast<unsigned>(can_id_), motor_.max_torque());
    }

    Dm_motor_test(const Dm_motor_test&) = delete;
    Dm_motor_test& operator=(const Dm_motor_test&) = delete;
    Dm_motor_test(Dm_motor_test&&) = delete;
    Dm_motor_test& operator=(Dm_motor_test&&) = delete;

    ~Dm_motor_test() override = default;

    void update() override {
        update_remote_control_();
        motor_.update_status();
        log_motor_feedback_once_per_second_();
    }

    void command_update() {
        auto builder = board_->start_transmit();

        // 一次性系统命令：启动 0x88（使能）/ 失能 0xA1 电流 0
        const auto system_command =
            pending_system_command_.exchange(kCommandNone, std::memory_order_relaxed);
        if (system_command == kCommandStartup) {
            builder.can_transmit(
                Spec::kCans.kCan0, //
                {.can_id = can_id_,
                 .can_data = device::LkMotor::generate_startup_command().as_bytes()});
            RCLCPP_INFO(
                logger_, "[dm motor test] sent startup(enable) 0x88 to id=%u on can0",
                static_cast<unsigned>(can_id_ - 0x140u));
        } else if (system_command == kCommandDisable) {
            builder.can_transmit(
                Spec::kCans.kCan0, //
                {.can_id = can_id_,
                 .can_data = device::LkMotor::generate_disable_command().as_bytes()});
            RCLCPP_INFO(
                logger_, "[dm motor test] sent disable(0xA1/current=0) to id=%u on can0",
                static_cast<unsigned>(can_id_ - 0x140u));
        }

        // 模式命令（内容由当前模式决定）
        // 注意：packet 必须保持为局部变量（as_bytes 的 span 指向它，且 as_bytes 非 const）
        auto packet = build_mode_command_();
        const auto bytes = packet.as_bytes();
        std::copy(bytes.begin(), bytes.end(), last_command_bytes_.begin());
        builder.can_transmit(
            Spec::kCans.kCan0, //
            {.can_id = can_id_, .can_data = bytes});
    }

private:
    using Clock = std::chrono::steady_clock;

    enum class ControlMode : std::uint8_t {
        kDisabled,
        kMit,
        kVelocity,
        kPositionVelocity,
    };

    static constexpr std::uint8_t kCommandNone = 0xFF;
    static constexpr std::uint8_t kCommandStartup = 0x88;
    static constexpr std::uint8_t kCommandDisable = 0xA1;

    static const char* mode_name(ControlMode mode) {
        switch (mode) {
        case ControlMode::kPositionVelocity: return "POS_VEL";
        case ControlMode::kVelocity: return "VEL";
        case ControlMode::kMit: return "MIT";
        case ControlMode::kDisabled: return "DISABLED";
        }
        return "?";
    }

    static const char* switch_name(rmcs_msgs::Switch sw) {
        switch (sw) {
        case rmcs_msgs::Switch::UP: return "UP";
        case rmcs_msgs::Switch::MIDDLE: return "MID";
        case rmcs_msgs::Switch::DOWN: return "DWN";
        default: return "UNK";
        }
    }

    class Command : public rmcs_executor::Component {
    public:
        explicit Command(Dm_motor_test& dm_motor_test)
            : dm_motor_test_(dm_motor_test) {}

        void update() override { dm_motor_test_.command_update(); }

    private:
        Dm_motor_test& dm_motor_test_;
    };

    void set_mode(ControlMode mode) {
        if (mode_ == mode)
            return;
        mode_ = mode;
        RCLCPP_INFO(logger_, "[dm motor test] mode → %s", mode_name(mode_));
    }

    /// DR16 → 使能/失能/模式 状态机（每周期在 update() 中调用）
    void update_remote_control_() {
        dr16_.update_status();

        // 遥控丢失（>500ms 无帧）：安全失能
        if (!dr16_.valid()) {
            if (enabled_) {
                pending_system_command_.store(kCommandDisable, std::memory_order_relaxed);
                enabled_ = false;
                RCLCPP_WARN(logger_, "[dm motor test] DR16 lost — motor disabled");
            }
            set_mode(ControlMode::kDisabled);
            return;
        }
        if (!remote_connected_) {
            remote_connected_ = true;
            RCLCPP_INFO(logger_, "[dm motor test] DR16 connected");
        }

        const auto left = dr16_.switch_left();
        const auto right = dr16_.switch_right();
        last_left_ = dr16_.joystick_left();
        last_right_ = dr16_.joystick_right();
        last_rotary_knob_ = dr16_.rotary_knob();

        // 双下 → 失能；其余 → 使能
        const bool both_down =
            left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN;
        if (both_down) {
            if (enabled_) {
                pending_system_command_.store(kCommandDisable, std::memory_order_relaxed);
                enabled_ = false;
                RCLCPP_INFO(logger_, "[dm motor test] switch both-down → disable");
            }
            set_mode(ControlMode::kDisabled);
            return;
        }
        if (!enabled_) {
            pending_system_command_.store(kCommandStartup, std::memory_order_relaxed);
            enabled_ = true;
            RCLCPP_INFO(logger_, "[dm motor test] switch not both-down → enable(startup)");
        }

        // 模式映射
        if (left == rmcs_msgs::Switch::MIDDLE && right == rmcs_msgs::Switch::MIDDLE) {
            set_mode(ControlMode::kPositionVelocity);
        } else if (left == rmcs_msgs::Switch::UP && right == rmcs_msgs::Switch::MIDDLE) {
            set_mode(ControlMode::kVelocity);
        } else if (left == rmcs_msgs::Switch::MIDDLE && right == rmcs_msgs::Switch::UP) {
            set_mode(ControlMode::kMit);
        }
        // 其他合法组合：使能，保持上一模式
    }

    /// 按当前模式生成 LK 命令帧，并记录下发中间值（供日志/服务）
    device::CanPacket8 build_mode_command_() {
        switch (mode_) {
        case ControlMode::kPositionVelocity:
            cmd_angle_ = last_left_.x() * position_scale_;
            cmd_velocity_limit_ = std::abs(last_left_.y()) * velocity_scale_;
            cmd_v_des_ = 0.0;
            cmd_t_ff_ = 0.0;
            cmd_torque_limit_ = 0.0;
            return motor_.generate_angle_command(cmd_angle_, cmd_velocity_limit_);
        case ControlMode::kVelocity:
            cmd_v_des_ = last_right_.y() * velocity_scale_;
            cmd_torque_limit_ = velocity_torque_limit_;
            cmd_angle_ = 0.0;
            cmd_velocity_limit_ = 0.0;
            cmd_t_ff_ = 0.0;
            return motor_.generate_velocity_command(cmd_v_des_, cmd_torque_limit_);
        case ControlMode::kMit:
            cmd_t_ff_ = last_right_.y() * torque_scale_;
            cmd_angle_ = 0.0;
            cmd_v_des_ = 0.0;
            cmd_velocity_limit_ = 0.0;
            cmd_torque_limit_ = 0.0;
            return motor_.generate_torque_command(cmd_t_ff_);
        case ControlMode::kDisabled:
        default:
            cmd_angle_ = 0.0;
            cmd_v_des_ = 0.0;
            cmd_t_ff_ = 0.0;
            cmd_velocity_limit_ = 0.0;
            cmd_torque_limit_ = 0.0;
            return motor_.generate_torque_command(0.0); // 失能期间持续下发零扭矩
        }
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        if (data.can_id == can_id_) {
            motor_.store_status(data.can_data);
            status_received_.store(true, std::memory_order_relaxed);
            return;
        }
        (void)can;
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void log_motor_feedback_once_per_second_() {
        const auto now = Clock::now();
        if (now < next_log_time_)
            return;
        if (log_rate_ > 0.0)
            next_log_time_ =
                now + std::chrono::milliseconds{static_cast<std::int64_t>(1000.0 / log_rate_)};

        std::string cmd_hex;
        for (const std::byte byte : last_command_bytes_)
            cmd_hex += std::format("{:02X} ", std::to_integer<unsigned>(byte));

        const auto message = std::format(
            "[dm motor test] dr16={} swL={} swR={} mode={} en={}\n"
            "    sticks: L=({:+.2f},{:+.2f}) R=({:+.2f},{:+.2f}) knob={:+.2f}\n"
            "    cmd:    angle={:+.3f} rad v_des={:+.3f} rad/s t_ff={:+.3f} Nm "
            "vel_limit={:+.3f} rad/s tq_limit={:+.3f} Nm\n"
            "    fb:     angle={:+.3f} rad raw={} vel={:+.3f} rad/s torque={:+.3f} Nm "
            "temp={:.1f} C max_torque={:.2f} Nm rx={}\n"
            "    frame:  [{}]",
            dr16_.valid() ? 'Y' : 'N', switch_name(dr16_.switch_left()),
            switch_name(dr16_.switch_right()), mode_name(mode_), enabled_ ? 'Y' : 'N',
            last_left_.x(), last_left_.y(), last_right_.x(), last_right_.y(), last_rotary_knob_,
            cmd_angle_, cmd_v_des_, cmd_t_ff_, cmd_velocity_limit_, cmd_torque_limit_,
            motor_.angle(), motor_.last_raw_angle(), motor_.velocity(), motor_.torque(),
            motor_.temperature(), motor_.max_torque(),
            status_received_.load(std::memory_order_relaxed) ? "Y" : "N", cmd_hex);
        RCLCPP_INFO(logger_, "%s", message.c_str());
    }

    void status_service_callback(const std_srvs::srv::Trigger::Response::SharedPtr& response) {
        std::string cmd_hex;
        for (const std::byte byte : last_command_bytes_)
            cmd_hex += std::format("{:02X} ", std::to_integer<unsigned>(byte));

        auto message = std::ostringstream{};
        message << "DmMotor test (LK MG4010Ei10, can0, DR16):\n"
                << "  dr16=" << (dr16_.valid() ? "valid" : "LOST")
                << " swL=" << switch_name(dr16_.switch_left())
                << " swR=" << switch_name(dr16_.switch_right())
                << " mode=" << mode_name(mode_) << " enabled=" << (enabled_ ? "Y" : "N") << '\n'
                << "  sticks: L=(" << last_left_.x() << ", " << last_left_.y()
                << ") R=(" << last_right_.x() << ", " << last_right_.y() << ")\n"
                << "  can_id=0x" << std::hex << can_id_ << std::dec << '\n'
                << "  cmd: angle=" << cmd_angle_ << " v_des=" << cmd_v_des_
                << " t_ff=" << cmd_t_ff_ << " vel_limit=" << cmd_velocity_limit_
                << " tq_limit=" << cmd_torque_limit_ << '\n'
                << "  fb: angle=" << motor_.angle() << " raw=" << motor_.last_raw_angle()
                << " vel=" << motor_.velocity() << " torque=" << motor_.torque()
                << " temp=" << motor_.temperature() << " max_torque=" << motor_.max_torque()
                << " rx=" << (status_received_.load(std::memory_order_relaxed) ? "Y" : "N") << '\n'
                << "  frame: [" << cmd_hex << "]\n"
                << "  params: Kt=0.07 Nm/A(电机侧) gear=10:1 position_scale=" << position_scale_
                << " velocity_scale=" << velocity_scale_ << " torque_scale=" << torque_scale_
                << " velocity_torque_limit=" << velocity_torque_limit_ << " reversed="
                << (reversed_ ? "true" : "false") << " multi_turn="
                << (multi_turn_angle_ ? "true" : "false");
        response->success = true;
        response->message = message.str();
    }

private:
    rclcpp::Logger logger_;
    std::shared_ptr<Command> command_;
    device::LkMotor motor_;
    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    device::Dr16 dr16_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;

    std::atomic<std::uint8_t> pending_system_command_{kCommandNone};
    std::atomic<bool> status_received_{false};

    std::uint32_t can_id_ = 0x141;

    ControlMode mode_ = ControlMode::kDisabled;
    bool enabled_ = false;
    bool remote_connected_ = false;
    Eigen::Vector2d last_left_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_right_ = Eigen::Vector2d::Zero();
    double last_rotary_knob_ = 0.0;

    // 最近一帧下发的中间值（供日志/状态服务）
    double cmd_angle_ = 0.0;
    double cmd_v_des_ = 0.0;
    double cmd_t_ff_ = 0.0;
    double cmd_velocity_limit_ = 0.0;
    double cmd_torque_limit_ = 0.0;
    std::array<std::byte, 8> last_command_bytes_{};

    // 参数
    bool reversed_ = false;
    bool multi_turn_angle_ = true;
    double position_scale_ = 3.0;
    double velocity_scale_ = 5.0;
    double torque_scale_ = 4.5;
    double velocity_torque_limit_ = 4.5;
    double log_rate_ = 1.0;

    Clock::time_point next_log_time_{Clock::now()};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Dm_motor_test, rmcs_executor::Component)
