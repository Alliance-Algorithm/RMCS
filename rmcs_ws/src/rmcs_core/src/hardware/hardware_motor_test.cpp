#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

namespace {

enum class MotorFamily : std::uint8_t {
    kDm8009,
    kLk4010i10,
    kGm6020,
    kM3508,
};

const char* can_name(int index) {
    switch (index) {
    case 0: return "can0";
    case 1: return "can1";
    case 2: return "can2";
    case 3: return "can3";
    default: return "can?";
    }
}

// 从 yaml 里解析的家族名转枚举；返回 false 表示未知类型。
bool family_from_name(std::string_view name, MotorFamily& out) {
    if (name == "DM8009") {
        out = MotorFamily::kDm8009;
        return true;
    }
    if (name == "LK4010i10" || name == "LK") {
        out = MotorFamily::kLk4010i10;
        return true;
    }
    if (name == "GM6020") {
        out = MotorFamily::kGm6020;
        return true;
    }
    if (name == "M3508") {
        out = MotorFamily::kM3508;
        return true;
    }
    return false;
}

class MotorBackend {
public:
    virtual ~MotorBackend() = default;

    virtual const char* family_name() const = 0;
    virtual bool supports_extended_modes() const = 0;
    virtual std::uint32_t tx_can_id() const = 0;
    virtual std::uint32_t rx_can_id() const = 0;

    virtual void update_status() = 0;
    virtual bool store_feedback(std::uint32_t can_id, std::span<const std::byte> can_data) = 0;

    virtual device::CanPacket8 startup_command() = 0;
    virtual device::CanPacket8 disable_command() = 0;
    virtual device::CanPacket8 torque_command(double torque) = 0;
    virtual device::CanPacket8 position_command(double angle, double velocity_limit) {
        (void)angle;
        (void)velocity_limit;
        return torque_command(0.0);
    }
    virtual device::CanPacket8 velocity_command(double velocity, double torque_limit) {
        (void)velocity;
        (void)torque_limit;
        return torque_command(0.0);
    }

    virtual double angle() const = 0;
    virtual double velocity() const = 0;
    virtual double torque() const = 0;
    virtual double temperature() const = 0;
    virtual double max_torque() const = 0;
    virtual std::string feedback_summary() const = 0;
};

class Dm8009Backend final : public MotorBackend {
public:
    Dm8009Backend(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& prefix, int motor_id, int feedback_id, bool reversed,
        double angle_bias)
        : motor_(status_component, command_component, prefix) {
        auto config = device::DmMotor::Config{device::DmMotor::Type::kDM8009}
                          .set_id(static_cast<std::uint8_t>(motor_id))
                          .set_feedback_id(static_cast<std::uint8_t>(feedback_id))
                          .set_angle_bias(angle_bias);
        if (reversed)
            config.set_reversed();
        motor_.configure(config);
        tx_can_id_ = motor_.send_id();
        rx_can_id_ = motor_.feedback_id();
    }

    const char* family_name() const override { return "DM8009"; }
    bool supports_extended_modes() const override { return false; }
    std::uint32_t tx_can_id() const override { return tx_can_id_; }
    std::uint32_t rx_can_id() const override { return rx_can_id_; }

    void update_status() override { motor_.update_status(); }

    bool store_feedback(std::uint32_t can_id, std::span<const std::byte> can_data) override {
        return motor_.match_then_store_status(can_id, can_data);
    }

    device::CanPacket8 startup_command() override { return motor_.enable_command(); }
    device::CanPacket8 disable_command() override { return motor_.disable_command(); }
    device::CanPacket8 torque_command(double torque) override { return motor_.generate_command(torque); }

    double angle() const override { return motor_.angle(); }
    double velocity() const override { return motor_.velocity(); }
    double torque() const override { return motor_.torque(); }
    double temperature() const override { return motor_.temperature_rotor(); }
    double max_torque() const override { return motor_.max_torque(); }

    std::string feedback_summary() const override {
        return std::format(
            "temp_mos={:.1f} temp_rotor={:.1f} fault={}", motor_.temperature_mos(),
            motor_.temperature_rotor(), motor_.fault_code());
    }

private:
    device::DmMotor motor_;
    std::uint32_t tx_can_id_ = 0;
    std::uint32_t rx_can_id_ = 0;
};

class Lk4010Backend final : public MotorBackend {
public:
    Lk4010Backend(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& prefix, int motor_id, int encoder_zero_point, bool reversed,
        bool multi_turn_angle)
        : motor_(status_component, command_component, prefix) {
        auto config = device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                          .set_encoder_zero_point(encoder_zero_point);
        if (reversed)
            config.set_reversed();
        if (multi_turn_angle)
            config.enable_multi_turn_angle();
        motor_.configure(config);
        tx_can_id_ = 0x140u + static_cast<std::uint32_t>(motor_id);
        rx_can_id_ = tx_can_id_; // LK command and feedback share the same CAN ID
    }

    const char* family_name() const override { return "LK4010i10"; }
    bool supports_extended_modes() const override { return true; }
    std::uint32_t tx_can_id() const override { return tx_can_id_; }
    std::uint32_t rx_can_id() const override { return rx_can_id_; }

    void update_status() override { motor_.update_status(); }

    bool store_feedback(std::uint32_t can_id, std::span<const std::byte> can_data) override {
        if (can_id != rx_can_id_ || can_data.size() != 8)
            return false;
        motor_.store_status(can_data);
        return true;
    }

    device::CanPacket8 startup_command() override { return motor_.generate_startup_command(); }
    device::CanPacket8 disable_command() override { return motor_.generate_disable_command(); }
    device::CanPacket8 torque_command(double torque) override {
        return motor_.generate_torque_command(torque);
    }
    device::CanPacket8 position_command(double angle, double velocity_limit) override {
        return motor_.generate_angle_command(angle, velocity_limit);
    }
    device::CanPacket8 velocity_command(double velocity, double torque_limit) override {
        return motor_.generate_velocity_command(velocity, torque_limit);
    }

    double angle() const override { return motor_.angle(); }
    double velocity() const override { return motor_.velocity(); }
    double torque() const override { return motor_.torque(); }
    double temperature() const override { return motor_.temperature(); }
    double max_torque() const override { return motor_.max_torque(); }

    std::string feedback_summary() const override {
        return std::format("raw={} temp={:.1f}", motor_.last_raw_angle(), motor_.temperature());
    }

private:
    device::LkMotor motor_;
    std::uint32_t tx_can_id_ = 0;
    std::uint32_t rx_can_id_ = 0;
};

class DjiBackend final : public MotorBackend {
public:
    DjiBackend(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& prefix, device::DjiMotor::Type motor_type, int motor_id,
        int encoder_zero_point, bool reversed, bool multi_turn_angle)
        : motor_(status_component, command_component, prefix) {
        status_component.register_output(prefix + "/control_torque", control_torque_output_, 0.0);

        auto config = device::DjiMotor::Config{motor_type, static_cast<std::uint8_t>(motor_id)}
                          .set_encoder_zero_point(encoder_zero_point);
        if (reversed)
            config.set_reversed();
        if (multi_turn_angle)
            config.enable_multi_turn_angle();
        motor_.configure(config);
        tx_can_id_ = device::DjiMotor::send_id(motor_type, static_cast<std::uint8_t>(motor_id));
        rx_can_id_ = device::DjiMotor::recv_id(motor_type, static_cast<std::uint8_t>(motor_id));
        family_name_ = motor_type == device::DjiMotor::Type::kGM6020 ? "GM6020" : "M3508";
    }

    const char* family_name() const override { return family_name_; }
    bool supports_extended_modes() const override { return false; }
    std::uint32_t tx_can_id() const override { return tx_can_id_; }
    std::uint32_t rx_can_id() const override { return rx_can_id_; }

    void update_status() override { motor_.update_status(); }

    bool store_feedback(std::uint32_t can_id, std::span<const std::byte> can_data) override {
        return motor_.match_then_store_status(can_id, can_data);
    }

    device::CanPacket8 startup_command() override { return torque_command(0.0); }
    device::CanPacket8 disable_command() override { return torque_command(0.0); }
    device::CanPacket8 torque_command(double torque) override {
        *control_torque_output_ = torque;
        device::CanPacket8 packet{0};
        packet << motor_;
        return packet;
    }

    double angle() const override { return motor_.angle(); }
    double velocity() const override { return motor_.velocity(); }
    double torque() const override { return motor_.torque(); }
    double temperature() const override { return motor_.temperature(); }
    double max_torque() const override { return motor_.max_torque(); }

    std::string feedback_summary() const override {
        return std::format("raw={} temp={:.1f}", motor_.last_raw_angle(), motor_.temperature());
    }

private:
    device::DjiMotor motor_;
    rmcs_executor::Component::OutputInterface<double> control_torque_output_;
    const char* family_name_ = "M3508";
    std::uint32_t tx_can_id_ = 0;
    std::uint32_t rx_can_id_ = 0;
};

} // namespace

/// 多通道电机硬件测试入口。
///
/// 每个 CAN 口一个通道，类型由 yaml 的 can_ports 显式指定：
///   can_ports: ["0:DM8009", "1:LK4010i10", "2:GM6020", "3:M3508"]
/// 四个口同一份指令广播发送，电机 ID 统一由 motor_id 指定（默认 1）。
///
/// LK 支持位置/速度/扭矩三种遥控模式；DM8009 / DJI 6020 / DJI 3508 恒走扭矩测试。
class HardwareMotorTest
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    HardwareMotorTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        std::string board_serial;
        get_parameter_or("board_serial", board_serial, std::string{});

        int motor_id = 1;
        int feedback_id = 1;
        int encoder_zero_point = 0;
        get_parameter_or("motor_id", motor_id, 1);
        get_parameter_or("feedback_id", feedback_id, motor_id);
        get_parameter_or("encoder_zero_point", encoder_zero_point, 0);
        get_parameter_or("angle_bias", angle_bias_, 0.0);
        get_parameter_or("motor_reversed", reversed_, false);
        get_parameter_or("multi_turn_angle", multi_turn_angle_, true);
        get_parameter_or("position_scale", position_scale_, 3.0);
        get_parameter_or("velocity_scale", velocity_scale_, 5.0);
        get_parameter_or("torque_scale", torque_scale_, 4.5);
        get_parameter_or("velocity_torque_limit", velocity_torque_limit_, 4.5);
        get_parameter_or("log_rate", log_rate_, 1.0);

        if (log_rate_ <= 0.0)
            log_rate_ = 1.0;

        motor_id_ = motor_id;

        // 每个 CAN 口一个条目："can_index:FAMILY"，例如 "0:DM8009"。
        // 缺省四个口按固定映射 0=DM8009 1=LK4010i10 2=GM6020 3=M3508。
        std::vector<std::string> can_ports;
        get_parameter_or("can_ports", can_ports, std::vector<std::string>{});
        if (can_ports.empty())
            can_ports = default_can_ports_();

        for (const auto& entry : can_ports) {
            const auto colon = entry.find(':');
            if (colon == std::string::npos || colon == 0 || colon + 1 >= entry.size()) {
                RCLCPP_WARN(
                    logger_, "[hardware motor test] invalid can_ports entry '%s' (expected "
                             "'can_index:FAMILY'), skipped",
                    entry.c_str());
                continue;
            }
            int index = 0;
            try {
                index = std::stoi(entry.substr(0, colon));
            } catch (const std::exception&) {
                RCLCPP_WARN(
                    logger_, "[hardware motor test] invalid can_index in '%s', skipped",
                    entry.c_str());
                continue;
            }
            if (index < 0 || index > 3) {
                RCLCPP_WARN(
                    logger_, "[hardware motor test] can_index=%d out of range 0..3, skipped", index);
                continue;
            }
            MotorFamily family = MotorFamily::kDm8009;
            if (!family_from_name(entry.substr(colon + 1), family)) {
                RCLCPP_WARN(
                    logger_, "[hardware motor test] unknown family '%s' on can%d, skipped",
                    entry.substr(colon + 1).c_str(), index);
                continue;
            }
            auto& ch = channels_.emplace_back();
            ch.can_index = index;
            ch.can = &Spec::kCans[index];
            ch.family = family;
            // 每个口独立前缀，避免不同通道的输出端口名冲突。
            const auto prefix = std::format("/hardware_motor_test/motor/can{}", index);
            ch.backend = create_backend(family, prefix, motor_id, feedback_id, encoder_zero_point);
        }

        if (channels_.empty()) {
            throw std::runtime_error{"[hardware motor test] no valid can_ports configured"};
        }

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/hardware_motor_test_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        dr16_.set_timeout_enabled(true);
        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);

        for (const auto& ch : channels_) {
            RCLCPP_INFO(
                logger_, "[hardware motor test] %s: family=%s tx=0x%03X rx=0x%03X id=%d",
                can_name(ch.can_index), ch.backend->family_name(),
                static_cast<unsigned>(ch.backend->tx_can_id()),
                static_cast<unsigned>(ch.backend->rx_can_id()), motor_id);
        }
    }

    HardwareMotorTest(const HardwareMotorTest&) = delete;
    HardwareMotorTest& operator=(const HardwareMotorTest&) = delete;
    HardwareMotorTest(HardwareMotorTest&&) = delete;
    HardwareMotorTest& operator=(HardwareMotorTest&&) = delete;

    ~HardwareMotorTest() override = default;

    void update() override {
        update_remote_control_();
        for (auto& ch : channels_)
            ch.backend->update_status();
        log_motor_feedback_once_per_second_();
    }

    void command_update() {
        auto builder = board_->start_transmit();

        const auto system_command = pending_system_command_.exchange(kCommandNone, std::memory_order_relaxed);
        for (auto& ch : channels_) {
            if (system_command == kCommandStartup) {
                builder.can_transmit(
                    *ch.can,
                    {.can_id = ch.backend->tx_can_id(), .can_data = ch.backend->startup_command().as_bytes()});
                RCLCPP_INFO(
                    logger_, "[hardware motor test] sent startup(enable) to %s on %s",
                    ch.backend->family_name(), can_name(ch.can_index));
            } else if (system_command == kCommandDisable) {
                builder.can_transmit(
                    *ch.can,
                    {.can_id = ch.backend->tx_can_id(), .can_data = ch.backend->disable_command().as_bytes()});
                RCLCPP_INFO(
                    logger_, "[hardware motor test] sent disable to %s on %s",
                    ch.backend->family_name(), can_name(ch.can_index));
            }

            const auto packet = build_mode_command_(*ch.backend);
            const auto bytes = packet.as_bytes();
            std::copy(bytes.begin(), bytes.end(), last_command_bytes_.begin());
            builder.can_transmit(*ch.can, {.can_id = ch.backend->tx_can_id(), .can_data = bytes});
        }
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
        explicit Command(HardwareMotorTest& hardware_motor_test)
            : hardware_motor_test_(hardware_motor_test) {}

        void update() override { hardware_motor_test_.command_update(); }

    private:
        HardwareMotorTest& hardware_motor_test_;
    };

    void set_mode(ControlMode mode) {
        if (mode_ == mode)
            return;
        mode_ = mode;
        RCLCPP_INFO(logger_, "[hardware motor test] mode -> %s", mode_name(mode_));
    }

    static std::vector<std::string> default_can_ports_() {
        return {"0:DM8009", "1:LK4010i10", "2:GM6020", "3:M3508"};
    }

    std::unique_ptr<MotorBackend> create_backend(
        MotorFamily family, const std::string& prefix, int motor_id, int feedback_id,
        int encoder_zero_point) {
        switch (family) {
        case MotorFamily::kDm8009:
            return std::make_unique<Dm8009Backend>(
                *this, *command_, prefix, motor_id, feedback_id, reversed_, angle_bias_);
        case MotorFamily::kLk4010i10:
            return std::make_unique<Lk4010Backend>(
                *this, *command_, prefix, motor_id, encoder_zero_point, reversed_,
                multi_turn_angle_);
        case MotorFamily::kGm6020:
            return std::make_unique<DjiBackend>(
                *this, *command_, prefix, device::DjiMotor::Type::kGM6020, motor_id,
                encoder_zero_point, reversed_, multi_turn_angle_);
        case MotorFamily::kM3508:
            return std::make_unique<DjiBackend>(
                *this, *command_, prefix, device::DjiMotor::Type::kM3508, motor_id,
                encoder_zero_point, reversed_, multi_turn_angle_);
        }
        return nullptr;
    }

    void update_remote_control_() {
        dr16_.update_status();

        if (!dr16_.valid()) {
            if (enabled_) {
                pending_system_command_.store(kCommandDisable, std::memory_order_relaxed);
                enabled_ = false;
                RCLCPP_WARN(logger_, "[hardware motor test] DR16 lost - motor disabled");
            }
            set_mode(ControlMode::kDisabled);
            return;
        }

        if (!remote_connected_) {
            remote_connected_ = true;
            RCLCPP_INFO(logger_, "[hardware motor test] DR16 connected");
        }

        const auto left = dr16_.switch_left();
        const auto right = dr16_.switch_right();
        last_left_ = dr16_.joystick_left();
        last_right_ = dr16_.joystick_right();
        last_rotary_knob_ = dr16_.rotary_knob();

        const bool both_down = left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN;
        if (both_down) {
            if (enabled_) {
                pending_system_command_.store(kCommandDisable, std::memory_order_relaxed);
                enabled_ = false;
                RCLCPP_INFO(logger_, "[hardware motor test] switch both-down -> disable");
            }
            set_mode(ControlMode::kDisabled);
            return;
        }

        if (!enabled_) {
            pending_system_command_.store(kCommandStartup, std::memory_order_relaxed);
            enabled_ = true;
            RCLCPP_INFO(logger_, "[hardware motor test] switch not both-down -> enable(startup)");
        }

        // 右摇杆 Y 恒为扭矩指令，供非扩展型(DM8009/DJI)通道做 MIT 扭矩输出。
        cmd_t_ff_ = last_right_.y() * torque_scale_;

        // 开关组合只决定扩展型(LK)通道的模式；非扩展型通道在 build_mode_command_ 里恒走扭矩。
        if (left == rmcs_msgs::Switch::MIDDLE && right == rmcs_msgs::Switch::MIDDLE) {
            set_mode(ControlMode::kPositionVelocity);
        } else if (left == rmcs_msgs::Switch::UP && right == rmcs_msgs::Switch::MIDDLE) {
            set_mode(ControlMode::kVelocity);
        } else if (left == rmcs_msgs::Switch::MIDDLE && right == rmcs_msgs::Switch::UP) {
            set_mode(ControlMode::kMit);
        } else if (mode_ == ControlMode::kDisabled) {
            // 非双下但未匹配到指定组合时，默认进扭矩(MIT)模式，让摇杆立刻有反应。
            set_mode(ControlMode::kMit);
        }
    }

    device::CanPacket8 build_mode_command_(MotorBackend& backend) {
        if (!backend.supports_extended_modes()) {
            cmd_angle_ = 0.0;
            cmd_v_des_ = 0.0;
            cmd_velocity_limit_ = 0.0;
            cmd_torque_limit_ = 0.0;
            if (mode_ == ControlMode::kDisabled) {
                cmd_t_ff_ = 0.0;
                return backend.torque_command(0.0);
            }
            return backend.torque_command(cmd_t_ff_);
        }

        switch (mode_) {
        case ControlMode::kPositionVelocity:
            cmd_angle_ = last_left_.x() * position_scale_;
            cmd_velocity_limit_ = std::abs(last_left_.y()) * velocity_scale_;
            cmd_v_des_ = 0.0;
            cmd_t_ff_ = 0.0;
            cmd_torque_limit_ = 0.0;
            return backend.position_command(cmd_angle_, cmd_velocity_limit_);
        case ControlMode::kVelocity:
            cmd_v_des_ = last_right_.y() * velocity_scale_;
            cmd_torque_limit_ = velocity_torque_limit_;
            cmd_angle_ = 0.0;
            cmd_velocity_limit_ = 0.0;
            cmd_t_ff_ = 0.0;
            return backend.velocity_command(cmd_v_des_, cmd_torque_limit_);
        case ControlMode::kMit:
            cmd_t_ff_ = last_right_.y() * torque_scale_;
            cmd_angle_ = 0.0;
            cmd_v_des_ = 0.0;
            cmd_velocity_limit_ = 0.0;
            cmd_torque_limit_ = 0.0;
            return backend.torque_command(cmd_t_ff_);
        case ControlMode::kDisabled:
        default:
            cmd_angle_ = 0.0;
            cmd_v_des_ = 0.0;
            cmd_t_ff_ = 0.0;
            cmd_velocity_limit_ = 0.0;
            cmd_torque_limit_ = 0.0;
            return backend.torque_command(0.0);
        }
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        for (auto& ch : channels_) {
            if (can != *ch.can)
                continue;
            if (ch.backend->store_feedback(data.can_id, data.can_data))
                ch.received = true;
            break;
        }
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
            next_log_time_ = now + std::chrono::milliseconds{static_cast<std::int64_t>(1000.0 / log_rate_)};

        std::string cmd_hex;
        for (const std::byte byte : last_command_bytes_)
            cmd_hex += std::format("{:02X} ", std::to_integer<unsigned>(byte));

        auto message = std::format(
            "[hardware motor test] dr16={} swL={} swR={} mode={} en={}\n"
            "    sticks: L=({:+.2f},{:+.2f}) R=({:+.2f},{:+.2f}) knob={:+.2f}\n"
            "    cmd:    angle={:+.3f} rad v_des={:+.3f} rad/s t_ff={:+.3f} Nm "
            "vel_limit={:+.3f} rad/s tq_limit={:+.3f} Nm\n"
            "    frame:  [{}]",
            dr16_.valid() ? 'Y' : 'N', switch_name(dr16_.switch_left()),
            switch_name(dr16_.switch_right()), mode_name(mode_), enabled_ ? 'Y' : 'N', last_left_.x(),
            last_left_.y(), last_right_.x(), last_right_.y(), last_rotary_knob_, cmd_angle_, cmd_v_des_,
            cmd_t_ff_, cmd_velocity_limit_, cmd_torque_limit_, cmd_hex);
        for (const auto& ch : channels_) {
            message += std::format(
                "\n    {} [{}]: angle={:+.3f} vel={:+.3f} torque={:+.3f} temp={:.1f} "
                "max_torque={:.2f} rx={} extra={}",
                can_name(ch.can_index), ch.backend->family_name(), ch.backend->angle(),
                ch.backend->velocity(), ch.backend->torque(), ch.backend->temperature(),
                ch.backend->max_torque(), ch.received ? "Y" : "N", ch.backend->feedback_summary());
        }
        RCLCPP_INFO(logger_, "%s", message.c_str());
    }

    void status_service_callback(const std_srvs::srv::Trigger::Response::SharedPtr& response) {
        std::string cmd_hex;
        for (const std::byte byte : last_command_bytes_)
            cmd_hex += std::format("{:02X} ", std::to_integer<unsigned>(byte));

        auto message = std::ostringstream{};
        message << "HardwareMotorTest (" << channels_.size() << " CAN channels, DR16):\n"
                << "  dr16=" << (dr16_.valid() ? "valid" : "LOST") << " swL="
                << switch_name(dr16_.switch_left()) << " swR=" << switch_name(dr16_.switch_right())
                << " mode=" << mode_name(mode_) << " enabled=" << (enabled_ ? "Y" : "N") << '\n'
                << "  sticks: L=(" << last_left_.x() << ", " << last_left_.y() << ") R=("
                << last_right_.x() << ", " << last_right_.y() << ")\n"
                << "  cmd: angle=" << cmd_angle_ << " v_des=" << cmd_v_des_ << " t_ff="
                << cmd_t_ff_ << " vel_limit=" << cmd_velocity_limit_ << " tq_limit="
                << cmd_torque_limit_ << '\n';
        for (const auto& ch : channels_) {
            message << "  can" << ch.can_index << " [" << ch.backend->family_name()
                    << "]: tx=0x" << std::hex << ch.backend->tx_can_id() << std::dec << " rx=0x"
                    << std::hex << ch.backend->rx_can_id() << std::dec << " angle=" << ch.backend->angle()
                    << " vel=" << ch.backend->velocity() << " torque=" << ch.backend->torque()
                    << " temp=" << ch.backend->temperature() << " max_torque="
                    << ch.backend->max_torque() << " rx=" << (ch.received ? "Y" : "N") << " extra="
                    << ch.backend->feedback_summary() << '\n';
        }
        message << "  frame: [" << cmd_hex << "]\n"
                << "  params: motor_id=" << motor_id_ << " reversed=" << (reversed_ ? "true" : "false")
                << " multi_turn=" << (multi_turn_angle_ ? "true" : "false") << " angle_bias="
                << angle_bias_ << " position_scale=" << position_scale_ << " velocity_scale="
                << velocity_scale_ << " torque_scale=" << torque_scale_
                << " velocity_torque_limit=" << velocity_torque_limit_;
        response->success = true;
        response->message = message.str();
    }

private:
    struct Channel {
        int can_index = 0;
        const Spec::Can* can = nullptr;
        MotorFamily family = MotorFamily::kDm8009;
        std::unique_ptr<MotorBackend> backend;
        bool received = false;
    };

    rclcpp::Logger logger_;
    std::shared_ptr<Command> command_;
    std::vector<Channel> channels_;
    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    device::Dr16 dr16_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;

    std::atomic<std::uint8_t> pending_system_command_{kCommandNone};

    int motor_id_ = 1;
    ControlMode mode_ = ControlMode::kDisabled;
    bool enabled_ = false;
    bool remote_connected_ = false;
    Eigen::Vector2d last_left_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_right_ = Eigen::Vector2d::Zero();
    double last_rotary_knob_ = 0.0;

    double cmd_angle_ = 0.0;
    double cmd_v_des_ = 0.0;
    double cmd_t_ff_ = 0.0;
    double cmd_velocity_limit_ = 0.0;
    double cmd_torque_limit_ = 0.0;
    std::array<std::byte, 8> last_command_bytes_{};

    bool reversed_ = false;
    bool multi_turn_angle_ = true;
    double angle_bias_ = 0.0;
    double position_scale_ = 3.0;
    double velocity_scale_ = 5.0;
    double torque_scale_ = 4.5;
    double velocity_torque_limit_ = 4.5;
    double log_rate_ = 1.0;

    Clock::time_point next_log_time_{Clock::now()};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::HardwareMotorTest, rmcs_executor::Component)
