#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

/// 达妙科技 DM-J8009-2EC 减速电机驱动（CAN@1Mbps, 标准帧, MIT 力矩模式）
///
/// 协议依据（权威来源）：
///   《DM-J8009-2EC 减速电机使用说明书 V1.0》(2023.10.15) —— 本机随电机附带
///   《调试助手使用说明书(达妙驱动控制协议) V1.4》 —— 状态码及系统命令字节来源
///
/// 电机参数（说明书 V1.0）：
///   额定 24V（24-48V），额定 20A / 峰值 50A，额定扭矩 20Nm / 峰值 40Nm，
///   额定转速 100rpm@24V / 200rpm@48V，减速比 9:1，编码器 14 位（单圈绝对，输出轴）
///
/// 说明书的三种控制模式（帧 ID 与数据格式，反馈帧三者相同）：
///   MIT 模式     帧 ID = CAN ID        D[0..1]=p_des(16b) | D[2..4]=v_des(12b)+Kp(12b)
///                                      | D[5..7]=Kd(12b)+t_ff(12b)，高位在前
///   位置速度模式  帧 ID = CAN ID + 0x100 D[0..3]=p_des、D[4..7]=v_des，float32 小端
///   速度模式     帧 ID = CAN ID + 0x200 D[0..3]=v_des，float32 小端
///
/// 本驱动使用 MIT 的力矩特例（手册"工作模式"一节：kp=0、kd=0 时给定 t_ff 即输出该扭矩）：
///   p_des、v_des、Kp、Kd 全为 0，仅 t_ff 携带力矩 τ [Nm]；Kp 量程 [0,500]、Kd 量程 [0,5]，
///   p/v/t 量程 = 电机内寄存器（P_MAX/V_MAX/T_MAX，须与 Config 定标一致）。
///   零力矩帧（非激活/失能/故障时持续发送）：80 00 80 00 00 00 08 00。
///   MIT 帧与系统命令共用帧 ID，仅 D[0..6] 全 0xFF 才解释为系统命令。
///   使用前须在调试助手将 Control Mode 设为 MIT 并写入；发送控制帧不会切换模式。
///
/// 系统命令（帧 ID = 电机 CAN ID，D[0..6]=0xFF，D[7]=命令字节；来自达妙驱动控制协议）：
///   0xFC 使能 / 0xFD 失能 / 0xFE 设零位 / 0xFB 清错
///
/// 反馈帧（帧 ID = MST_ID，调试助手设置，默认 0）：
///   D[0] = ID|STATUS<<4（ID 取电机 CAN_ID 低 8 位，实际为低 4 位）
///   D[1..2] POS[15:0] | D[3..4] VEL[11:0] | D[4..5] T[11:0] | D[6] T_MOS | D[7] T_Rotor
///   STATUS（调试协议 V1.4）：0 失能 / 1 使能 / 8 超压 / 9 欠压 / A 过流
///                             B MOS 过温 / C 线圈过温 / D 通讯丢失 / E 过载
///   注意：0/1 是使能状态，不是故障；仅 8..E 视为 fault_code。
///
/// 角度环与速度环都在控制器侧闭环（下发关节力矩 τ），本驱动只做定标、限幅与打包。
class DmMotor {
public:
    enum class Type : uint8_t { kDM8009 }; // 如需其他型号在此扩展

    struct Config {
        explicit Config(Type motor_type)
            : motor_type(motor_type) {}

        Config& set_id(std::uint8_t id) { return this->id = id, *this; }
        Config& set_feedback_id(std::uint16_t feedback_id) {
            return this->feedback_id = feedback_id, *this;
        }
        Config& set_reversed() { return reversed = true, *this; }
        Config& set_reversed(bool value) { return reversed = value, *this; }
        /// URDF相位 = wrap(sign*(电机角 − angle_offset))；reversed时raw=0对应offset。
        Config& set_angle_offset(double angle_offset) {
            return this->angle_offset = angle_offset, *this;
        }
        /// 设置反馈定标范围，必须与电机内寄存器（调试助手设定）一致
        Config& set_limits(double position_max, double velocity_max, double torque_max) {
            return this->position_max = position_max, this->velocity_max = velocity_max,
                   this->torque_max = torque_max, *this;
        }

        Type motor_type;
        std::uint8_t id = 1;           // 电机 CAN ID（MIT 帧与系统命令帧 ID = id）
        std::uint16_t feedback_id = 0; // 反馈帧 ID（MST_ID，调试助手设置，默认 0）
        bool reversed = false;
        double angle_offset = 0.0;     // rad
        double position_max = 12.5;    // P_MAX [rad]，须与电机寄存器一致
        double velocity_max = 45.0;    // V_MAX [rad/s]，反馈定标范围，须与电机寄存器一致
        double torque_max = 54.0;      // T_MAX [Nm]，须与电机寄存器一致
    };

    // ---- 系统命令字节（达妙驱动控制协议；说明书 V1.0 未列出，装车前用调试助手确认）----
    static constexpr std::uint8_t kCommandEnable = 0xFC;
    static constexpr std::uint8_t kCommandDisable = 0xFD;
    static constexpr std::uint8_t kCommandSetZero = 0xFE;
    static constexpr std::uint8_t kCommandClearError = 0xFB;

    // ---- MIT 帧 Kp/Kd 量程（手册 MIT 控制帧一节）----
    static constexpr double kMitKpMax = 500.0;
    static constexpr double kMitKdMax = 5.0;

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
        status_component_.register_output(name_prefix + "/fault_code", fault_code_output_, 0);
        status_component_.register_output(
            name_prefix + "/feedback_valid", feedback_valid_output_, false);
        status_component_.register_output(name_prefix + "/status_code", status_code_output_, 0);

        // MIT 力矩模式：角度环与速度环在控制器侧闭环，这里只下发关节力矩 τ [Nm]
        command_component_.register_input(name_prefix + "/control_torque", control_torque_, false);
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
        if (config.id == 0 || config.id > 15 || config.feedback_id > 0x7FF
            || !std::isfinite(config.angle_offset) || !std::isfinite(config.position_max)
            || !std::isfinite(config.velocity_max) || !std::isfinite(config.torque_max)
            || config.position_max <= 0.0 || config.velocity_max <= 0.0 || config.torque_max <= 0.0)
            throw std::invalid_argument("Invalid DM motor CAN ID or feedback mapping range");

        type_ = config.motor_type;
        id_ = config.id;
        feedback_id_ = config.feedback_id;
        reversed_ = config.reversed;
        angle_offset_ = config.angle_offset;
        position_max_ = config.position_max;
        velocity_max_ = config.velocity_max;
        torque_max_ = config.torque_max;

        fault_code_ = 0;
        last_feedback_ns_.store(0, std::memory_order_relaxed);
    }

    // ---- 命令帧生成 ----

    /// MIT 力矩模式：τ [Nm]，按 reversed 方向映射；T_MAX 同时用作软件后备限幅。
    /// 非有限值一律退化为零力矩帧（Kp=Kd=0 且 t_ff=0，电机不受力）。
    CanPacket8 generate_torque_command(double torque) const {
        if (!std::isfinite(torque))
            torque = 0.0;
        const double sign = reversed_ ? -1.0 : 1.0;
        const double t_motor = std::clamp(sign * torque, -torque_max_, torque_max_);
        return pack_mit(static_cast<float>(t_motor));
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
        last_feedback_ns_.store(steady_now_ns_(), std::memory_order_release);
        return true;
    }

    void update_status() {
        if (!feedback_fresh()) {
            *feedback_valid_output_ = false;
            return;
        }
        auto packet = can_data_.load(std::memory_order_relaxed);
        const auto bytes = packet.as_bytes();

        const auto d0 = static_cast<std::uint8_t>(bytes[0]);
        status_code_ = static_cast<int>(d0 >> 4);
        // 官方调试协议 V1.4：0=失能、1=使能、8..E=故障。
        fault_code_ = status_code_ <= 1 ? 0 : status_code_;

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

        // A phase, not a turn counter. Motor raw=0 and raw=2*pi describe the
        // same calibrated pose. The pair controller resolves the leg opening.
        angle_ = std::remainder(sign * (raw_angle - angle_offset_), 2.0 * std::numbers::pi);
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
        *feedback_valid_output_ = fault_code_ == 0;
        *status_code_output_ = status_code_;
    }

    // ---- 查询 ----

    bool control_torque_ready() const noexcept { return control_torque_.ready(); }
    double control_torque() const {
        if (control_torque_.ready())
            return *control_torque_;
        return 0.0;
    }

    std::uint8_t id() const noexcept { return id_; }
    /// 系统命令与 MIT 命令共用帧 ID == 电机 CAN ID
    std::uint32_t send_id() const noexcept { return id_; }
    std::uint32_t feedback_id() const noexcept { return feedback_id_; }
    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double temperature_mos() const { return temperature_mos_; }
    double temperature_rotor() const { return temperature_rotor_; }
    int fault_code() const { return fault_code_; }
    int status_code() const { return status_code_; }
    bool feedback_ready() const { return feedback_fresh() && fault_code_ == 0; }
    std::chrono::steady_clock::time_point last_feedback_time() const {
        return std::chrono::steady_clock::time_point{
            std::chrono::nanoseconds{last_feedback_ns_.load(std::memory_order_acquire)}};
    }
    double feedback_age_ms() const {
        const auto last = last_feedback_ns_.load(std::memory_order_acquire);
        return last == 0 ? -1.0 : static_cast<double>(steady_now_ns_() - last) / 1e6;
    }
    void reset_feedback_tracking() { last_feedback_ns_.store(0, std::memory_order_release); }

    // ---- 定标 ----

    /// 逆映射：x = x_min + u / (2^N − 1) * (x_max − x_min)
    static double uint_to_float(std::uint16_t u, double x_min, double x_max, int bits) {
        const double max_u = static_cast<double>((std::uint32_t{1} << bits) - 1);
        return x_min + static_cast<double>(u) / max_u * (x_max - x_min);
    }

    /// 正映射：u = round((x − x_min) / (x_max − x_min) * (2^N − 1))，结果落在 [0, 2^N − 1]
    static std::uint32_t float_to_uint(double x, double x_min, double x_max, int bits) {
        const double max_u = static_cast<double>((std::uint32_t{1} << bits) - 1);
        const double u = (std::clamp(x, x_min, x_max) - x_min) / (x_max - x_min) * max_u;
        return static_cast<std::uint32_t>(std::llround(u));
    }

    /// MIT 帧位打包（手册 MIT 控制帧，高位在前）：
    ///   D[0..1]=p_des(16b) | D[2..4]=v_des(12b)+Kp(12b) | D[5..7]=Kd(12b)+t_ff(12b)
    /// 力矩特例固定 p_des=v_des=Kp=Kd=0，仅 t_ff 非零；Kp 量程 [0,500]，Kd 量程 [0,5]。
    CanPacket8 pack_mit(float torque) const {
        const auto p = float_to_uint(0.0, -position_max_, position_max_, 16);
        const auto v = float_to_uint(0.0, -velocity_max_, velocity_max_, 12);
        const auto kp = float_to_uint(0.0, 0.0, kMitKpMax, 12);
        const auto kd = float_to_uint(0.0, 0.0, kMitKdMax, 12);
        const auto t = float_to_uint(torque, -torque_max_, torque_max_, 12);
        std::array<std::byte, 8> bytes{
            static_cast<std::byte>(p >> 8),
            static_cast<std::byte>(p & 0xFF),
            static_cast<std::byte>(v >> 4),
            static_cast<std::byte>(((v & 0xF) << 4) | (kp >> 8)),
            static_cast<std::byte>(kp & 0xFF),
            static_cast<std::byte>(kd >> 4),
            static_cast<std::byte>(((kd & 0xF) << 4) | (t >> 8)),
            static_cast<std::byte>(t & 0xFF)};
        return CanPacket8{std::span<const std::byte>(bytes)};
    }

    static CanPacket8 system_command(std::uint8_t command) {
        std::array<std::byte, 8> bytes;
        bytes.fill(static_cast<std::byte>(0xFF));
        bytes[7] = static_cast<std::byte>(command);
        return CanPacket8{std::span<const std::byte>(bytes)};
    }

private:
    static std::int64_t steady_now_ns_() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    bool feedback_fresh() const {
        const auto last = last_feedback_ns_.load(std::memory_order_acquire);
        const auto age = steady_now_ns_() - last;
        return last != 0 && age >= 0 && age <= 100'000'000;
    }

    Type type_ = Type::kDM8009;
    std::uint8_t id_ = 1;
    std::uint16_t feedback_id_ = 0;
    bool reversed_ = false;
    double angle_offset_ = 0.0;
    double position_max_ = 12.5;
    double velocity_max_ = 45.0;
    double torque_max_ = 54.0;

    std::atomic<CanPacket8> can_data_{CanPacket8{0}};
    std::atomic<std::int64_t> last_feedback_ns_{0};

    double angle_ = 0.0;
    double velocity_ = 0.0;
    double torque_ = 0.0;
    double temperature_mos_ = 0.0;
    double temperature_rotor_ = 0.0;
    int fault_code_ = 0;
    int status_code_ = 0;

    rmcs_executor::Component& status_component_;
    rmcs_executor::Component& command_component_;

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_mos_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_rotor_output_;
    rmcs_executor::Component::OutputInterface<int> fault_code_output_;
    rmcs_executor::Component::OutputInterface<bool> feedback_valid_output_;
    rmcs_executor::Component::OutputInterface<int> status_code_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device
