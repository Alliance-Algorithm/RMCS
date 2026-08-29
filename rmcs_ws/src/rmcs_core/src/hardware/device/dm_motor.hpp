#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <span>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

/// 达妙科技 DM-J8009-2EC 减速电机驱动（CAN@1Mbps, 标准帧, MIT 模式）
///
/// 协议依据（权威来源）：
///   《DM-J8009-2EC 减速电机使用说明书 V1.0》(2023.10.15) —— 本机随电机附带
///   《调试助手使用说明书(达妙驱动控制协议)》 —— 使能/失能/设零/清错命令字节来源
///
/// 电机参数（说明书 V1.0）：
///   额定 24V（24-48V），额定 20A / 峰值 50A，额定扭矩 20Nm / 峰值 40Nm，
///   额定转速 100rpm@24V / 200rpm@48V，减速比 9:1，编码器 14 位（单圈绝对，输出轴）
///
/// MIT 控制帧（帧 ID = 电机 CAN ID）：
///   D[0..1] p_des[15:0]  |  D[2..3] v_des[11:0]|Kp[11:8]  |  D[4] Kp[7:0]
///   D[5..6] Kd[11:0]|t_ff[11:8]  |  D[7] t_ff[7:0]
///   定标：位置 16 位、速度 12 位、扭矩 12 位，按 P_MAX/V_MAX/T_MAX 线性映射
///   （范围由调试助手设定，本驱动必须与电机内寄存器一致）
///   Kp ∈ [0, 500]，Kd ∈ [0, 5]（固定范围）
///   注意：对位置进行控制时 kd 不能为 0（说明书警告）
///
/// 反馈帧（帧 ID = MST_ID，调试助手设置，默认 0）：
///   D[0] = ID|ERR<<4（ID 取电机 CAN_ID 低 8 位，实际为低 4 位；ERR 故障码）
///   D[1..2] POS[15:0] | D[3..4] VEL[11:0] | D[4..5] T[11:0] | D[6] T_MOS | D[7] T_Rotor
///   ERR：8 超压 / 9 欠压 / A 过流 / B MOS 过温 / C 线圈过温 / D 通讯丢失 / E 过载
///
/// 系统命令（帧 ID = 电机 CAN ID，D[0..6]=0xFF，D[7]=命令字节；来自达妙驱动控制协议）：
///   0xFC 使能 / 0xFD 失能 / 0xFE 设零位 / 0xFB 清错
///
/// 用法（模式 A，与训练合同一致）：PC 侧做 PD，本驱动只下发 kp=kd=0 的纯扭矩
/// （t_ff = control_torque）；模式 B（可选）：直接下发 p_des/v_des/kp/kd 让电机内环 PD。
class DmMotor {
public:
    enum class Type : uint8_t { kDM8009 }; // 如需其他型号在此扩展

    struct Config {
        explicit Config(Type motor_type)
            : motor_type(motor_type) {}

        Config& set_id(std::uint8_t id) { return this->id = id, *this; }
        Config& set_feedback_id(std::uint8_t feedback_id) {
            return this->feedback_id = feedback_id, *this;
        }
        Config& set_reversed() { return reversed = true, *this; }
        Config& set_reversed(bool value) { return reversed = value, *this; }
        /// angle_bias：电机角 → 策略角的偏置，策略角 = sign*(电机角 − angle_bias)（参考 XYEGA 做法）
        Config& set_angle_bias(double angle_bias) { return this->angle_bias = angle_bias, *this; }
        /// 设置 MIT 定标范围，必须与电机内寄存器（调试助手设定）一致
        Config& set_limits(double position_max, double velocity_max, double torque_max) {
            return this->position_max = position_max, this->velocity_max = velocity_max,
                   this->torque_max = torque_max, *this;
        }
        Config& set_control_torque_max(double control_torque_max) {
            return this->control_torque_max = control_torque_max, *this;
        }

        Type motor_type;
        std::uint8_t id = 1;             // 电机 CAN ID（MIT 命令帧 ID；建议 1..15）
        std::uint8_t feedback_id = 0;    // 反馈帧 ID（MST_ID，调试助手设置，默认 0）
        bool reversed = false;
        double angle_bias = 0.0;         // rad
        double position_max = 12.5;      // P_MAX [rad]，须与电机寄存器一致
        double velocity_max = 45.0;      // V_MAX [rad/s]，须与电机寄存器一致
        double torque_max = 54.0;        // T_MAX [Nm]，须与电机寄存器一致
        double control_torque_max = 40.0; // 输出力矩限幅（峰值 40Nm）
    };

    static constexpr double kKpMax = 500.0;
    static constexpr double kKdMax = 5.0;

    // ---- 系统命令字节（达妙驱动控制协议；说明书 V1.0 未列出，装车前用调试助手确认）----
    static constexpr std::uint8_t kCommandEnable = 0xFC;
    static constexpr std::uint8_t kCommandDisable = 0xFD;
    static constexpr std::uint8_t kCommandSetZero = 0xFE;
    static constexpr std::uint8_t kCommandClearError = 0xFB;

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : status_component_(status_component)
        , command_component_(command_component) {
        status_component_.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component_.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component_.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component_.register_output(
            name_prefix + "/temperature_mos", temperature_mos_output_, 0.0);
        status_component_.register_output(
            name_prefix + "/temperature_rotor", temperature_rotor_output_, 0.0);
        status_component_.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        status_component_.register_output(name_prefix + "/fault_code", fault_code_output_, 0);

        // 模式 A：PC 侧 PD，纯扭矩下发
        command_component_.register_input(name_prefix + "/control_torque", control_torque_, false);
        // 模式 B（可选）：电机内环 PD 透传
        command_component_.register_input(name_prefix + "/control_angle", control_angle_, false);
        command_component_.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);
        command_component_.register_input(name_prefix + "/control_kp", control_kp_, false);
        command_component_.register_input(name_prefix + "/control_kd", control_kd_, false);
    }

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DmMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;
    DmMotor(DmMotor&&) = delete;
    DmMotor& operator=(DmMotor&&) = delete;

    ~DmMotor() = default;

    void configure(const Config& config) {
        type_ = config.motor_type;
        id_ = config.id;
        feedback_id_ = config.feedback_id;
        reversed_ = config.reversed;
        angle_bias_ = config.angle_bias;
        position_max_ = config.position_max;
        velocity_max_ = config.velocity_max;
        torque_max_ = config.torque_max;
        control_torque_max_ = config.control_torque_max;

        *max_torque_output_ = control_torque_max_;
        fault_code_ = 0;
    }

    // ---- 命令帧生成 ----

    /// 模式 A：纯扭矩（kp=kd=0, p_des=v_des=0, t_ff=τ）
    CanPacket8 generate_command() const { return generate_command(control_torque()); }

    CanPacket8 generate_command(double control_torque) const {
        if (std::isnan(control_torque) || std::isinf(control_torque))
            control_torque = 0.0;
        control_torque = std::clamp(control_torque, -control_torque_max_, control_torque_max_);

        const auto pos_u = float_to_uint(0.0, -position_max_, position_max_, 16);
        const auto vel_u = float_to_uint(0.0, -velocity_max_, velocity_max_, 12);
        const auto kp_u = float_to_uint(0.0, 0.0, kKpMax, 12);
        const auto kd_u = float_to_uint(0.0, 0.0, kKdMax, 12);
        const auto tff_u = float_to_uint(control_torque, -torque_max_, torque_max_, 12);
        return pack_mit(pos_u, vel_u, kp_u, kd_u, tff_u);
    }

    /// 模式 B：电机内环 PD（p_des/v_des 单位 rad/rad/s；kp/kd 为物理增益）
    CanPacket8 generate_command_pd(
        double p_des, double v_des, double kp, double kd, double t_ff) const {
        if (!std::isfinite(p_des) || !std::isfinite(v_des) || !std::isfinite(t_ff))
            return CanPacket8{0};
        p_des = std::clamp(p_des, -position_max_, position_max_);
        v_des = std::clamp(v_des, -velocity_max_, velocity_max_);
        kp = std::clamp(kp, 0.0, kKpMax);
        kd = std::clamp(kd, 0.0, kKdMax);
        t_ff = std::clamp(t_ff, -torque_max_, torque_max_);

        const auto pos_u = float_to_uint(p_des, -position_max_, position_max_, 16);
        const auto vel_u = float_to_uint(v_des, -velocity_max_, velocity_max_, 12);
        const auto kp_u = float_to_uint(kp, 0.0, kKpMax, 12);
        const auto kd_u = float_to_uint(kd, 0.0, kKdMax, 12);
        const auto tff_u = float_to_uint(t_ff, -torque_max_, torque_max_, 12);
        return pack_mit(pos_u, vel_u, kp_u, kd_u, tff_u);
    }

    // ---- 系统命令 ----

    CanPacket8 enable_command() const { return system_command(kCommandEnable); }
    CanPacket8 disable_command() const { return system_command(kCommandDisable); }
    CanPacket8 set_zero_command() const { return system_command(kCommandSetZero); }
    CanPacket8 clear_error_command() const { return system_command(kCommandClearError); }

    // ---- 反馈接收与状态更新 ----

    bool match_then_store_status(std::uint32_t can_id, std::span<const std::byte> can_data) {
        if (can_id != feedback_id_)
            return false;
        if (can_data.size() != 8)
            return false;
        // D[0] 低 4 位为电机 ID（说明书："ID 取 CAN_ID 的低 8 位"，但帧内仅 4 位可用）
        const auto d0 = static_cast<std::uint8_t>(can_data[0]);
        if ((d0 & 0x0F) != (id_ & 0x0F))
            return false;
        can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
        return true;
    }

    void update_status() {
        auto packet = can_data_.load(std::memory_order_relaxed);
        const auto bytes = packet.as_bytes();

        const auto d0 = static_cast<std::uint8_t>(bytes[0]);
        fault_code_ = static_cast<int>(d0 >> 4);

        const auto pos_u = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(static_cast<std::uint8_t>(bytes[1])) << 8)
            | static_cast<std::uint8_t>(bytes[2]));
        const auto vel_u = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(static_cast<std::uint8_t>(bytes[3])) << 4)
            | (static_cast<std::uint8_t>(bytes[4]) >> 4));
        const auto tff_u = static_cast<std::uint16_t>(
            ((static_cast<std::uint8_t>(bytes[4]) & 0x0F) << 8)
            | static_cast<std::uint8_t>(bytes[5]));

        const double sign = reversed_ ? -1.0 : 1.0;
        const double raw_angle = uint_to_float(pos_u, -position_max_, position_max_, 16);
        const double raw_velocity = uint_to_float(vel_u, -velocity_max_, velocity_max_, 12);
        const double raw_torque = uint_to_float(tff_u, -torque_max_, torque_max_, 12);

        angle_ = sign * (raw_angle - angle_bias_);
        velocity_ = sign * raw_velocity;
        torque_ = sign * raw_torque;

        temperature_mos_ = static_cast<double>(static_cast<std::uint8_t>(bytes[6]));
        temperature_rotor_ = static_cast<double>(static_cast<std::uint8_t>(bytes[7]));

        *angle_output_ = angle_;
        *velocity_output_ = velocity_;
        *torque_output_ = torque_;
        *temperature_mos_output_ = temperature_mos_;
        *temperature_rotor_output_ = temperature_rotor_;
        *fault_code_output_ = fault_code_;
    }

    // ---- 查询 ----

    double control_torque() const {
        if (control_torque_.ready())
            return *control_torque_;
        return 0.0;
    }

    std::uint8_t id() const noexcept { return id_; }
    /// MIT 命令帧 ID == 电机 CAN ID
    std::uint32_t send_id() const noexcept { return id_; }
    std::uint32_t feedback_id() const noexcept { return feedback_id_; }
    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return control_torque_max_; }
    double temperature_mos() const { return temperature_mos_; }
    double temperature_rotor() const { return temperature_rotor_; }
    int fault_code() const { return fault_code_; }

    // ---- MIT 定标 ----

    /// 线性映射：u = (x − x_min) / (x_max − x_min) * (2^N − 1)
    static std::uint16_t float_to_uint(double x, double x_min, double x_max, int bits) {
        const double span = x_max - x_min;
        if (span <= 0.0)
            return 0;
        const double max_u = static_cast<double>((std::uint32_t{1} << bits) - 1);
        const double u = std::clamp((x - x_min) / span * max_u, 0.0, max_u);
        return static_cast<std::uint16_t>(std::round(u));
    }

    /// 逆映射：x = x_min + u / (2^N − 1) * (x_max − x_min)
    static double uint_to_float(std::uint16_t u, double x_min, double x_max, int bits) {
        const double max_u = static_cast<double>((std::uint32_t{1} << bits) - 1);
        return x_min + static_cast<double>(u) / max_u * (x_max - x_min);
    }

    static CanPacket8 pack_mit(
        std::uint16_t pos_u, std::uint16_t vel_u, std::uint16_t kp_u, std::uint16_t kd_u,
        std::uint16_t tff_u) {
        std::array<std::byte, 8> bytes{};
        bytes[0] = static_cast<std::byte>((pos_u >> 8) & 0xFF);
        bytes[1] = static_cast<std::byte>(pos_u & 0xFF);
        bytes[2] = static_cast<std::byte>((vel_u >> 4) & 0xFF);
        bytes[3] = static_cast<std::byte>(((vel_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F));
        bytes[4] = static_cast<std::byte>(kp_u & 0xFF);
        bytes[5] = static_cast<std::byte>((kd_u >> 4) & 0xFF);
        bytes[6] = static_cast<std::byte>(((kd_u & 0x0F) << 4) | ((tff_u >> 8) & 0x0F));
        bytes[7] = static_cast<std::byte>(tff_u & 0xFF);
        return CanPacket8{std::span<const std::byte>(bytes)};
    }

    static CanPacket8 system_command(std::uint8_t command) {
        std::array<std::byte, 8> bytes;
        bytes.fill(static_cast<std::byte>(0xFF));
        bytes[7] = static_cast<std::byte>(command);
        return CanPacket8{std::span<const std::byte>(bytes)};
    }

private:
    Type type_ = Type::kDM8009;
    std::uint8_t id_ = 1;
    std::uint8_t feedback_id_ = 0;
    bool reversed_ = false;
    double angle_bias_ = 0.0;
    double position_max_ = 12.5;
    double velocity_max_ = 45.0;
    double torque_max_ = 54.0;
    double control_torque_max_ = 40.0;

    std::atomic<CanPacket8> can_data_{CanPacket8{0}};

    double angle_ = 0.0;
    double velocity_ = 0.0;
    double torque_ = 0.0;
    double temperature_mos_ = 0.0;
    double temperature_rotor_ = 0.0;
    int fault_code_ = 0;

    rmcs_executor::Component& status_component_;
    rmcs_executor::Component& command_component_;

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_mos_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_rotor_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;
    rmcs_executor::Component::OutputInterface<int> fault_code_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
    rmcs_executor::Component::InputInterface<double> control_angle_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_kp_;
    rmcs_executor::Component::InputInterface<double> control_kd_;
};

} // namespace rmcs_core::hardware::device
