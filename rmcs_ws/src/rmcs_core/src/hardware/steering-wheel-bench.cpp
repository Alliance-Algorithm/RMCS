// ============================================================================
// 单个舵轮台架测试组件（硬件侧）
//
// 两个 3508（舵向 + 轮向）接在 RMCS C 板上，DR16 遥控器控制。
// 完全复用仓库已有的 device::DjiMotor / device::Dr16，不自己写协议。
//
// 运行方式（先停掉 rmcs service，否则会抢板子 USB）：
//   /rmcs_install/lib/rmcs_executor/rmcs_executor --ros-args \
//       --params-file /rmcs_install/share/rmcs_bringup/config/steering-wheel-bench.yaml
//
// 输出（给控制器/调试用）：
//   /bench/steer/angle  /bench/steer/velocity  /bench/steer/torque
//   /bench/wheel/angle  /bench/wheel/velocity  /bench/wheel/torque
//   /remote/joystick/right  /remote/switch/right
//   /bench/steer/raw_angle  /bench/wheel/raw_velocity
//
// 输入（控制指令）：
//   /bench/steer/control_torque
//   /bench/wheel/control_torque
// ============================================================================

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <format>
#include <cstdint>
#include <numbers>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/magnetic_encoder.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::hardware {

namespace {
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
double wrap_to_pi(double angle) {
    while (angle > std::numbers::pi_v<double>)
        angle -= kTwoPi;
    while (angle < -std::numbers::pi_v<double>)
        angle += kTwoPi;
    return angle;
}
} // namespace

class SteeringWheelBench
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    class Command : public Component {
    public:
        explicit Command(SteeringWheelBench& bench) : bench_(bench) {}
        void update() override {}
        SteeringWheelBench& bench_;
    };

    SteeringWheelBench()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , steer_motor_(*this, *command_, "/bench/steer")
        , wheel_motor_(*this, *command_, "/bench/wheel") {

        wheel_can_ = get_parameter_or("wheel_can", 0);
        steer_can_ = get_parameter_or("steer_can", 1);
        encoder_can_ = get_parameter_or("encoder_can", 2);
        encoder_id_ = get_parameter_or("encoder_id", 0x21);
        encoder_count_per_turn_ = get_parameter_or("encoder_count_per_turn", 65536.0);
        encoder_reversed_ = get_parameter_or("encoder_reversed", false);
        encoder_zero_offset_ = get_parameter_or("encoder_zero_offset", 0.0);
        encoder_config_.can_id = static_cast<uint32_t>(encoder_id_);
        encoder_config_.count_per_turn = encoder_count_per_turn_;
        encoder_config_.gear_ratio = get_parameter_or("encoder_gear_ratio", 1.0);
        encoder_config_.reversed = encoder_reversed_;
        encoder_config_.zero_offset = encoder_zero_offset_;
        encoder_config_.big_endian = get_parameter_or("encoder_big_endian", true);
        encoder_config_.timeout = std::chrono::milliseconds(
            static_cast<int>(get_parameter_or("encoder_timeout_ms", 100)));
        use_magnetic_encoder_ = get_parameter_or("use_magnetic_encoder", true);
        encoder_.configure(encoder_config_);

        enable_left_ = get_parameter_or("enable_left", 3);
        jog_step_rad_ = get_parameter_or("steer_jog_deg", 30.0) * std::numbers::pi_v<double> / 180.0;
        jog_threshold_ = get_parameter_or("steer_jog_threshold", 0.6);
        enable_right_ = get_parameter_or("enable_right", 3);
        const int steer_id = get_parameter_or("steer_id", 1);
        const int wheel_id = get_parameter_or("wheel_id", 2);
        // 舵和轮是两个独立的减速比：舵带一级传动（实测约 4），轮只有 3508 减速箱 19.2
        steer_reduction_ = get_parameter_or("steer_reduction", 4.0);
        wheel_reduction_ = get_parameter_or("wheel_reduction", 3591.0 / 187.0);
        const bool steer_reversed = get_parameter_or("steer_reversed", false);
        const bool wheel_reversed = get_parameter_or("wheel_reversed", false);
        const std::string serial_filter = get_parameter_or("serial_filter", std::string{});
        steer_gravity_ff_gain_ = get_parameter_or("steer_gravity_ff_gain", 0.0);
        steer_gravity_ff_phase_ = get_parameter_or("steer_gravity_ff_phase", 0.0);
        steer_mode_ = get_parameter_or("steer_mode", std::string{"position"});
        steer_deadband_ = get_parameter_or("steer_deadband", 0.02);
        steer_test_velocity_ = get_parameter_or("steer_test_velocity", 2.0);
        steer_max_velocity_ = get_parameter_or("steer_max_velocity", 3.0);
        // 速度反馈低通：舵、轮可分别配置；旧的 velocity_filter_hz 作为共同默认值
        const double filter_default = get_parameter_or("velocity_filter_hz", 0.0);
        steer_velocity_filter_hz_ =
            get_parameter_or("steer_velocity_filter_hz", filter_default);
        wheel_velocity_filter_hz_ =
            get_parameter_or("wheel_velocity_filter_hz", filter_default);
        wheel_max_velocity_ = get_parameter_or("wheel_max_velocity", 6.0);
        wheel_torque_limit_ = get_parameter_or("wheel_torque_limit", 3.0);
        steer_target_filter_hz_ = get_parameter_or("steer_target_filter_hz", 10.0);
        steer_angle_kd_ = get_parameter_or("steer_angle_kd", 0.0);
        steer_pos_step_enable_ = get_parameter_or("steer_pos_step_enable", false);
        steer_pos_step_target_ = get_parameter_or("steer_pos_step_target", 0.5);
        steer_pos_step_half_period_ = get_parameter_or("steer_pos_step_half_period", 5.0);
        steer_step_enable_ = get_parameter_or("steer_step_enable", false);
        steer_step_target_ = get_parameter_or("steer_step_target", 0.5);
        steer_step_half_period_ = get_parameter_or("steer_step_half_period", 5.0);
        wheel_step_enable_ = get_parameter_or("wheel_step_enable", false);
        wheel_step_target_ = get_parameter_or("wheel_step_target", 0.5);
        wheel_step_half_period_ = get_parameter_or("wheel_step_half_period", 5.0);

        auto steer_config =
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, static_cast<uint8_t>(steer_id)}
                .set_reduction_ratio(steer_reduction_);
        steer_config.set_encoder_zero_point(get_parameter_or("steer_zero_point", 0));
        // 必须开多圈：单圈角度每 2π/减速比（此处 90°）就回绕，位置环无法闭环
        steer_config.enable_multi_turn_angle();
        if (steer_reversed)
            steer_config.set_reversed();
        steer_motor_.configure(steer_config);

        auto wheel_config =
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, static_cast<uint8_t>(wheel_id)}
                .set_reduction_ratio(wheel_reduction_);
        if (wheel_reversed)
            wheel_config.set_reversed();
        wheel_motor_.configure(wheel_config);

        // ---- 舵：角度环（输出速度参考 rad/s）----
        steer_angle_pid_.kp = get_parameter_or("steer_angle_kp", 8.0);
        steer_angle_pid_.ki = get_parameter_or("steer_angle_ki", 0.0);
        steer_angle_pid_.kd = get_parameter_or("steer_angle_kd", 0.0);

        // ---- 舵：速度环（输出力矩 N·m）----
        steer_velocity_limit_ = get_parameter_or("steer_velocity_limit", 10.0);
        steer_torque_limit_ = get_parameter_or("steer_torque_limit", 4.0);
        steer_velocity_pid_.kp = get_parameter_or("steer_velocity_kp", 1.0);
        steer_velocity_pid_.ki = get_parameter_or("steer_velocity_ki", 0.0);
        steer_velocity_pid_.kd = get_parameter_or("steer_velocity_kd", 0.0);

        // ---- 轮：速度环（输出力矩 N·m）----
        wheel_pid_.kp = get_parameter_or("wheel_kp", 0.05);
        wheel_pid_.ki = get_parameter_or("wheel_ki", 0.0);
        wheel_pid_.kd = get_parameter_or("wheel_kd", 0.0);

        register_output("/bench/encoder/angle", encoder_angle_, 0.0);
        register_output("/bench/encoder/raw_count", encoder_raw_count_, 0.0);
        register_output("/bench/steer/raw_angle", steer_raw_angle_, 0.0);
        register_output("/bench/wheel/raw_velocity", wheel_raw_velocity_, 0.0);
        register_output("/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);

        auto options = librmcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks = true;
        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

        RCLCPP_INFO(
            get_logger(),
            "[bench] 轮:CAN%d ID%d(减比%.3f) | 舵:CAN%d ID%d(减比%.3f) | 磁编:CAN%d ID0x%X",
            wheel_can_, wheel_id, wheel_reduction_, steer_can_, steer_id, steer_reduction_,
            encoder_can_, encoder_id_);
    }

    void update() override {
        dr16_.update_status();
        steer_motor_.update_status();
        wheel_motor_.update_status();

        const auto js = dr16_.joystick_right();
        const auto sw_l = dr16_.switch_left();
        const auto sw_r = dr16_.switch_right();

        *joystick_right_ = js;
        *switch_right_ = sw_r;
        *switch_left_ = sw_l;
        *steer_raw_angle_ = steer_motor_.angle();
        *wheel_raw_velocity_ = wheel_motor_.velocity();

        // 使能：左右拨杆都在中位
        // 使能条件可配置：-1 = 忽略该拨杆（台架调试时可直接常使能）
        //   1=UP 2=DOWN 3=MIDDLE
        const bool left_ok = enable_left_ < 0 || static_cast<int>(sw_l) == enable_left_;
        const bool right_ok = enable_right_ < 0 || static_cast<int>(sw_r) == enable_right_;
        const bool enabled = left_ok && right_ok;

        // 舵角反馈：磁编看门狗有效就用磁编，超时自动退回电机自带编码器
        // use_magnetic_encoder=false 时强制只用电机自带编码器
        const bool encoder_ok = use_magnetic_encoder_ && encoder_.valid();
        const double steer_feedback = encoder_ok ? encoder_.angle() : steer_motor_.angle();

        // 磁编有效时累积齿轮比标定（编码器角度增量 vs 电机角度增量）
        if (encoder_ok)
            encoder_.feed_calibration(steer_motor_.angle() - prev_steer_angle_);
        prev_steer_angle_ = steer_motor_.angle();

        if (enabled) {
            constexpr double dt = 1e-3;
            // 速度反馈一阶低通（0 = 关闭）：抑制电调转速噪声被增益放大成力矩纹波
            if (steer_velocity_filter_hz_ > 1.0) {
                const double tau = 1.0 / (2.0 * kPi * steer_velocity_filter_hz_);
                const double alpha = dt / (tau + dt);
                steer_velocity_filt_ += alpha * (steer_motor_.velocity() - steer_velocity_filt_);
            } else {
                steer_velocity_filt_ = steer_motor_.velocity();
            }
            if (wheel_velocity_filter_hz_ > 1.0) {
                const double tau = 1.0 / (2.0 * kPi * wheel_velocity_filter_hz_);
                const double alpha = dt / (tau + dt);
                wheel_velocity_filt_ += alpha * (wheel_motor_.velocity() - wheel_velocity_filt_);
            } else {
                wheel_velocity_filt_ = wheel_motor_.velocity();
            }
            // 摇杆每推一次（越过阈值）舵角步进一个固定角度（默认 30°）
            if (std::abs(js.x()) > jog_threshold_ && std::abs(prev_js_x_) <= jog_threshold_)
                steer_target_ =
                    std::clamp(steer_target_ + (js.x() > 0 ? jog_step_rad_ : -jog_step_rad_), -kPi, kPi);
            prev_js_x_ = js.x();
            wheel_target_ = js.y() * wheel_max_velocity_;
            // 速度环阶跃测试：周期性正负跳变，不受摇杆影响
            if (wheel_step_enable_) {
                static const auto step_start = std::chrono::steady_clock::now();
                const double t =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start)
                        .count();
                const double phase = std::fmod(t, 2.0 * wheel_step_half_period_);
                wheel_target_ = phase < wheel_step_half_period_ ? wheel_step_target_
                                                                : -wheel_step_target_;
            }
            // 3508 角度是 0~2π，误差必须 wrap，否则跨零点会猛打一圈
            // 舵：两种模式
            if (steer_mode_ == "velocity") {
                // 速度模式：摇杆水平轴控制转速，松手即停（跟轮电机一样）
                steer_velocity_ref_ = js.x() * steer_max_velocity_;
                // 速度环阶跃测试：周期性正负跳变，不受摇杆影响
                if (steer_step_enable_) {
                    static const auto sstep_start = std::chrono::steady_clock::now();
                    const double st =
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - sstep_start)
                            .count();
                    const double sphase = std::fmod(st, 2.0 * steer_step_half_period_);
                    steer_velocity_ref_ = sphase < steer_step_half_period_ ? steer_step_target_
                                                                           : -steer_step_target_;
                }
            } else {
                // 位置环阶跃测试：周期性在 ±目标角 之间跳变
                if (steer_pos_step_enable_) {
                    static const auto pstep_start = std::chrono::steady_clock::now();
                    const double pt =
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - pstep_start)
                            .count();
                    const double pphase = std::fmod(pt, 2.0 * steer_pos_step_half_period_);
                    steer_target_ = pphase < steer_pos_step_half_period_ ? steer_pos_step_target_
                                                                        : -steer_pos_step_target_;
                }
                // 位置模式：目标平滑(TD) + 速度前馈 + 角度环
                const double tgt = steer_target_;
                if (steer_target_filter_hz_ > 1.0) {
                    const double tau_t = 1.0 / (2.0 * kPi * steer_target_filter_hz_);
                    const double alpha_t = dt / (tau_t + dt);
                    steer_target_smooth_ += alpha_t * (tgt - steer_target_smooth_);
                } else {
                    steer_target_smooth_ = tgt;
                }
                const double target_rate = (steer_target_smooth_ - steer_target_prev_) / dt;
                steer_target_prev_ = steer_target_smooth_;
                const double err = wrap_to_pi(steer_target_smooth_ - steer_feedback);
                steer_velocity_ref_ =
                    std::abs(err) < steer_deadband_
                        ? steer_angle_kd_ * target_rate
                        : std::clamp(
                              steer_angle_pid_.update(err) + steer_angle_kd_ * target_rate,
                              -steer_velocity_limit_, steer_velocity_limit_);
            }
            // 重力前馈：轮组重心偏离舵轴，转动时重力力矩随角度变化
            steer_gravity_ff_ =
                steer_gravity_ff_gain_ * std::sin(steer_feedback - steer_gravity_ff_phase_);
            steer_torque_ = std::clamp(
                steer_velocity_pid_.update(steer_velocity_ref_ - steer_velocity_filt_),
                -steer_torque_limit_, steer_torque_limit_) + steer_gravity_ff_;

            // 轮：只有速度环
            wheel_torque_ = std::clamp(
                wheel_pid_.update(wheel_target_ - wheel_velocity_filt_), -wheel_torque_limit_,
                wheel_torque_limit_);
        } else {
            steer_target_ = steer_feedback;
            wheel_target_ = 0.0;
            steer_angle_pid_.reset();
            steer_velocity_pid_.reset();
            wheel_pid_.reset();
            steer_velocity_ref_ = 0.0;
            steer_torque_ = 0.0;
            wheel_torque_ = 0.0;
            steer_gravity_ff_ = 0.0;
        }

        if (enabled_ != enabled) {
            RCLCPP_INFO(get_logger(), "[bench] %s", enabled ? "使能" : "停止");
            enabled_ = enabled;
        }

        send_command();

        // 舵 10 Hz 快速诊断（看符号是否一致）
        const auto steer_now = std::chrono::steady_clock::now();
        if (steer_now - last_steer_log_ >= std::chrono::milliseconds(100)) {
            last_steer_log_ = steer_now;
            const double err = wrap_to_pi(steer_target_ - steer_feedback);
            RCLCPP_INFO(
                get_logger(),
                "[舵] 角度=%.3f 目标=%.3f 误差=%+.3f v_ref=%+.3f v=%.3f 力矩=%+.2f 实测=%+.2f "
                "温度=%.0f raw=%d",
                steer_feedback, steer_target_, err, steer_velocity_ref_, steer_motor_.velocity(),
                steer_torque_, steer_motor_.torque(), steer_motor_.temperature(),
                steer_motor_.last_raw_angle());
        }

        // 1 Hz 状态打印（确认三路 CAN 是否都通）
        const auto now = std::chrono::steady_clock::now();
        if (now - last_log_ >= std::chrono::seconds(1)) {
            last_log_ = now;
            if (++wheel_log_counter_ % 50 == 0) {
                RCLCPP_INFO(
                    get_logger(), "[舵位] 目标=%+.3f 角度=%+.3f 误差=%+.3f v_ref=%+.3f v=%+.3f 力矩=%+.3f",
                    steer_target_, steer_feedback, wrap_to_pi(steer_target_ - steer_feedback),
                    steer_velocity_ref_, steer_motor_.velocity(), steer_torque_);
                RCLCPP_INFO(
                    get_logger(), "[舵速] 目标=%+.3f 实测=%+.3f 误差=%+.3f 力矩=%+.3f 实测力矩=%+.3f",
                    steer_velocity_ref_, steer_motor_.velocity(),
                    steer_velocity_ref_ - steer_motor_.velocity(), steer_torque_,
                    steer_motor_.torque());
                RCLCPP_INFO(
                    get_logger(), "[轮] 目标=%+.3f 实测=%+.3f 误差=%+.3f 力矩=%+.3f 实测力矩=%+.3f 温度=%.0f",
                    wheel_target_, wheel_motor_.velocity(), wheel_target_ - wheel_motor_.velocity(),
                    wheel_torque_, wheel_motor_.torque(), wheel_motor_.temperature());
            }

            RCLCPP_INFO(
                get_logger(),
                "[bench] 使能=%d 左%d 右%d 摇杆(%.2f,%.2f) | 舵 %.3f rad (源:%s) | 轮 目标%.2f 实测%.2f "
                "rad/s | 磁编帧 %d (通道共 %d id=0x%X) | 力矩 舵%.2f 轮%.2f",
                static_cast<int>(enabled), static_cast<int>(sw_l), static_cast<int>(sw_r),
                js.x(), js.y(), steer_feedback, encoder_ok ? "磁编" : "电机", wheel_target_,
                wheel_motor_.velocity(), encoder_.frames(), encoder_channel_frames_,
                last_encoder_id_, steer_torque_, wheel_torque_);

            RCLCPP_INFO(
                get_logger(),
                "[bench] 磁编: valid=%d frames=%d raw=%.4f 角度=%.4f | 齿轮比估计=%.4f "
                "(电机转过 %.3f rad)",
                static_cast<int>(encoder_ok), encoder_.frames(), encoder_.raw_angle(),
                encoder_.angle(), encoder_.estimated_gear_ratio(), encoder_.calibration_travel());

            std::string ids;
            {
                const auto lock = std::scoped_lock{mutex_};
                for (int c = 0; c < 4; ++c) {
                    if (seen_ids_[c].empty())
                        continue;
                    ids += std::format("CAN{}:", c);
                    for (auto id : seen_ids_[c])
                        ids += std::format("0x{:X} ", id);
                }
            }
            RCLCPP_INFO(get_logger(), "[bench] 各通道收到的 ID: %s", ids.c_str());
            RCLCPP_INFO(
                get_logger(),
                "[bench] 发出的帧: 轮 id=0x%X [%02X %02X %02X %02X %02X %02X %02X %02X] | "
                "舵 id=0x%X [%02X %02X %02X %02X %02X %02X %02X %02X]",
                wheel_motor_.send_id(), last_wheel_frame_[0], last_wheel_frame_[1],
                last_wheel_frame_[2], last_wheel_frame_[3], last_wheel_frame_[4],
                last_wheel_frame_[5], last_wheel_frame_[6], last_wheel_frame_[7],
                steer_motor_.send_id(), last_steer_frame_[0], last_steer_frame_[1],
                last_steer_frame_[2], last_steer_frame_[3], last_steer_frame_[4],
                last_steer_frame_[5], last_steer_frame_[6], last_steer_frame_[7]);
        }
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        // 记录每路 CAN 上出现过的 ID（诊断用）
        for (int i = 0; i < 4; ++i) {
            if (matches(can, i)) {
                const auto lock = std::scoped_lock{mutex_};
                seen_ids_[i].insert(data.can_id);
                break;
            }
        }

        // 按通道分发：舵、轮、磁编各自独立，避免互相错认
        if (matches(can, steer_can_))
            steer_motor_.match_then_store_status(data.can_id, data.can_data);
        else if (matches(can, wheel_can_))
            wheel_motor_.match_then_store_status(data.can_id, data.can_data);
        else if (matches(can, encoder_can_))
            handle_encoder_frame(data);
    }

    void handle_encoder_frame(const View::Can& data) {
        // 先记录该通道收到的任何帧，便于确认磁编 ID 与协议
        last_encoder_id_ = data.can_id;
        for (size_t i = 0; i < 8; ++i)
            last_raw_[i] = static_cast<uint8_t>(data.can_data[i]);
        ++encoder_channel_frames_;

        if (data.can_id != encoder_config_.can_id)
            return;

        encoder_.store_status(data.can_data);
        *encoder_raw_count_ = encoder_.raw_angle();
        *encoder_angle_ = encoder_.angle();
        if (!encoder_seen_) {
            encoder_seen_ = true;
            RCLCPP_INFO(get_logger(), "[bench] 收到磁编帧，舵角改用磁编");
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void send_command() {
        auto builder = board_->start_transmit();
        last_wheel_frame_.fill(0);
        last_steer_frame_.fill(0);

        // 轮电机：按 ID 自动放进对应 quarter（ID1→quarter0, ID2→quarter1 ...）
        device::CanPacket8 wheel_packet{};
        // 直接按 ID 写入 quarter，避免依赖 operator<< 的解析
        wheel_packet.data[(wheel_motor_.id() - 1) % 4] =
            std::bit_cast<uint16_t>(wheel_motor_.generate_command(wheel_torque_));
        for (size_t i = 0; i < 8; ++i)
            last_wheel_frame_[i] = static_cast<uint8_t>(wheel_packet.as_bytes()[i]);
        builder.can_transmit(
            descriptor(wheel_can_), librmcs::data::CanDataView{
                                         .can_id = wheel_motor_.send_id(),
                                         .can_data = wheel_packet.as_bytes(),
                                     });

        // 舵电机：同样按 ID 落 quarter
        device::CanPacket8 steer_packet{};
        steer_packet.data[(steer_motor_.id() - 1) % 4] =
            std::bit_cast<uint16_t>(steer_motor_.generate_command(steer_torque_));
        for (size_t i = 0; i < 8; ++i)
            last_steer_frame_[i] = static_cast<uint8_t>(steer_packet.as_bytes()[i]);
        builder.can_transmit(
            descriptor(steer_can_), librmcs::data::CanDataView{
                                         .can_id = steer_motor_.send_id(),
                                         .can_data = steer_packet.as_bytes(),
                                     });
    }

private:
    static auto descriptor(int index) -> const librmcs::spec::rmcs_board_lite::CanDescriptor& {
        switch (index) {
        case 0: return Spec::kCans.kCan0;
        case 2: return Spec::kCans.kCan2;
        case 3: return Spec::kCans.kCan3;
        default: return Spec::kCans.kCan1;
        }
    }

    static auto matches(const Spec::Can& can, int index) -> bool {
        switch (index) {
        case 0: return can == Spec::kCans.kCan0;
        case 1: return can == Spec::kCans.kCan1;
        case 2: return can == Spec::kCans.kCan2;
        case 3: return can == Spec::kCans.kCan3;
        default: return false;
        }
    }

    std::shared_ptr<Command> command_;
    device::DjiMotor steer_motor_;
    device::DjiMotor wheel_motor_;
    device::Dr16 dr16_;

    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kMaxSteerVelocity = 20.0;  // rad/s，角度环输出限幅
    static constexpr double kMaxSteerTorque = 20.0;    // N·m
    static constexpr double kMaxWheelTorque = 20.0;    // N·m

    int wheel_can_ = 0;
    int steer_can_ = 1;
    int encoder_can_ = 2;
    int encoder_id_ = 0x21;
    bool use_magnetic_encoder_ = true;
    device::MagneticEncoder encoder_{};
    device::MagneticEncoder::Config encoder_config_{};
    double prev_steer_angle_ = 0.0;
    double steer_velocity_limit_ = 10.0;
    double steer_torque_limit_ = 4.0;
    double steer_gravity_ff_gain_ = 0.0;
    double steer_gravity_ff_phase_ = 0.0;
    double steer_gravity_ff_ = 0.0;
    double steer_deadband_ = 0.02;
    std::string steer_mode_ = "position";
    double steer_test_velocity_ = 2.0;
    double steer_max_velocity_ = 3.0;
    double steer_velocity_filter_hz_ = 0.0;
    double wheel_velocity_filter_hz_ = 0.0;
    double steer_velocity_filt_ = 0.0;
    double wheel_velocity_filt_ = 0.0;
    double wheel_max_velocity_ = 6.0;
    double wheel_torque_limit_ = 3.0;
    bool wheel_step_enable_ = false;
    double wheel_step_target_ = 0.5;
    double steer_target_filter_hz_ = 10.0;
    double steer_target_smooth_ = 0.0;
    double steer_target_prev_ = 0.0;
    double steer_angle_kd_ = 0.0;
    bool steer_pos_step_enable_ = false;
    double steer_pos_step_target_ = 0.5;
    double steer_pos_step_half_period_ = 5.0;
    bool steer_step_enable_ = false;
    double steer_step_target_ = 0.5;
    double steer_step_half_period_ = 5.0;
    double wheel_step_half_period_ = 5.0;
    int wheel_log_counter_ = 0;
    double jog_step_rad_ = std::numbers::pi_v<double> / 6.0;  // 30°
    double jog_threshold_ = 0.6;
    double prev_js_x_ = 0.0;
    int enable_left_ = 3;
    int enable_right_ = 3;
    double encoder_count_per_turn_ = 65536.0;
    bool encoder_reversed_ = false;
    double encoder_zero_offset_ = 0.0;
    bool encoder_seen_ = false;
    int encoder_frames_ = 0;
    int encoder_channel_frames_ = 0;
    uint32_t last_encoder_id_ = 0;
    std::array<uint8_t, 8> last_raw_{};
    std::chrono::steady_clock::time_point last_log_{};
    std::chrono::steady_clock::time_point last_steer_log_{};
    OutputInterface<double> encoder_angle_;
    OutputInterface<double> encoder_raw_count_;
    double steer_reduction_ = 4.0;
    double wheel_reduction_ = 3591.0 / 187.0;

    controller::pid::PidCalculator steer_angle_pid_{};
    controller::pid::PidCalculator steer_velocity_pid_{};
    controller::pid::PidCalculator wheel_pid_{};

    bool enabled_ = false;
    double steer_target_ = 0.0;
    double wheel_target_ = 0.0;
    double steer_velocity_ref_ = 0.0;
    double steer_torque_ = 0.0;
    double wheel_torque_ = 0.0;

    OutputInterface<double> steer_raw_angle_;
    OutputInterface<double> wheel_raw_velocity_;
    OutputInterface<Eigen::Vector2d> joystick_right_;
    OutputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<rmcs_msgs::Switch> switch_left_;

    std::mutex mutex_;
    std::array<std::set<uint32_t>, 4> seen_ids_{};
    std::array<uint8_t, 8> last_wheel_frame_{};
    std::array<uint8_t, 8> last_steer_frame_{};

    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SteeringWheelBench, rmcs_executor::Component)
