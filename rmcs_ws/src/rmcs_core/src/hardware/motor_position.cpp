#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::hardware {

// 任务三：双环 PID 控制电机角度。
//
// 层叠结构（仿照云台/任务二，只是多一层角度环）：
//   外环(角度) ErrorPidController:   measurement=/motor/angle_error ->
//   control=/motor/control_velocity 内环(速度) PidController: measurement=/motor/velocity
//   -> control=/motor/control_torque （链路上用原始速度；滤波值仅在发布时输出）
//
// - 通过 topic /motor/target_angle(Float64) 发布目标角度(rad)，电机转到该位置。
// - 优弧(>180°)：目标变化时锁定一条长弧方向并解缠出终点，之后方向不再翻转，
//   误差随接近目标单调收敛，避免中间(半圈对角点)翻转导致的剧烈抖动。
class MotorPosition
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    MotorPosition()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_{create_partner_component<MotorCommand>(get_component_name() + "_command", *this)}
        , motor_{*this, *command_, "/motor"} {

        // M3508（内置齿轮减速 3591/187，编码器 8192/圈，电流模式）。
        motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                .set_reversed()
                .enable_multi_turn_angle());

        // 内环速度反馈低通滤波。
        double filter_alpha = 0.3;
        get_parameter_or("filter_alpha", filter_alpha, 0.3);
        velocity_filter_.set_alpha(filter_alpha);

        // 初始目标角度：设置则启动直接控制到该位置；否则等 topic。
        double initial_angle = std::numeric_limits<double>::quiet_NaN();
        get_parameter_or("initial_angle", initial_angle, initial_angle);
        if (std::isfinite(initial_angle)) {
            target_angle_.store(initial_angle);
            target_received_.store(true);
        }

        // 位置方形波（仅用于调双环 PID）：true 时目标角度在 square_high/square_low 间切换。
        get_parameter_or("square_wave", square_wave_, false);
        get_parameter_or("square_low", square_low_, 0.0);
        get_parameter_or("square_high", square_high_, 1.5707963);
        get_parameter_or("square_period", square_period_, 1.0); // 每半周期时长（秒）
        square_wave_start_ = Clock::now();

        // 位置序列（测试用）：true 时依次走 sequence_angles 里的角度，每个停 sequence_dwell 秒。
        get_parameter_or("sequence_wave", sequence_wave_, false);
        get_parameter_or("sequence_angles", sequence_angles_, std::vector<double>{});
        get_parameter_or("sequence_dwell", sequence_dwell_, 2.0);
        sequence_start_ = Clock::now();

        register_output("/motor/angle_error", angle_error_, 0.0);
        register_output("/motor/angle_single_turn", angle_single_turn_, 0.0);
        register_output("/motor/velocity_filtered", velocity_filtered_, 0.0);
        // 当前目标角度参考（方形波或 topic 的值），供 Foxglove 看参考轨迹。
        register_output("/motor/control_angle", control_angle_, 0.0);
        register_output("/motor/enabled", enabled_, false);

        // 发布目标角度(rad)，电机沿最短弧转到该位置。
        target_angle_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "/motor/target_angle", rclcpp::QoS{1}, [this](std_msgs::msg::Float64::UniquePtr msg) {
                target_angle_.store(msg->data);
                target_received_.store(true);
            });

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
    }

    MotorPosition(const MotorPosition&) = delete;
    MotorPosition& operator=(const MotorPosition&) = delete;
    MotorPosition(MotorPosition&&) = delete;
    MotorPosition& operator=(MotorPosition&&) = delete;

    ~MotorPosition() override = default;

    void update() override {
        motor_.update_status();
        update_control();
    }

    void command_update() {
        auto builder = board_->start_transmit();
        auto packet = device::CanPacket8{uint64_t{0}};
        packet << motor_;
        builder.can_transmit(
            Spec::kCans.kCan1, {
                                   .can_id = motor_.send_id(),
                                   .can_data = packet.as_bytes(),
                               });
    }

private:
    // 计算到目标角度的优弧(>180°)误差，并生成发布用的滤波速度。
    void update_control() {
        double target = target_angle_.load();
        bool has_target = target_received_.load();

        if (sequence_wave_ && !sequence_angles_.empty()) {
            // 位置序列：依次在 sequence_angles 间走，每个角度停留 sequence_dwell 秒。
            const double elapsed =
                std::chrono::duration<double>(Clock::now() - sequence_start_).count();
            const double cycle_time = sequence_dwell_ * sequence_angles_.size();
            const double phase = std::fmod(elapsed, cycle_time);
            const auto index = static_cast<std::size_t>(phase / sequence_dwell_);
            target = sequence_angles_[std::min(index, sequence_angles_.size() - 1)];
            has_target = true;
        } else if (square_wave_) {
            // 位置方形波：目标角度在 square_high / square_low 间来回切，仍走优弧。
            const double elapsed =
                std::chrono::duration<double>(Clock::now() - square_wave_start_).count();
            const double phase = std::fmod(elapsed, 2.0 * square_period_);
            target = (phase < square_period_) ? square_high_ : square_low_;
            has_target = true;
        }

        *control_angle_ = target; // 暴露当前目标角度参考（方形波/topic）

        double error = 0.0;
        if (has_target) {
            // 仅在"目标真正变化"才重新选定优弧方向并解缠出终点；其余时间恒定持有。
            // 若不区分这次判定而每次都拿"当前多圈角度"重解缠，电机已停在终点附近时，
            // 每次重算都会在上一终点基础上再累加 ±2π，导致单向飞车。
            if (!std::isfinite(committed_target_)
                || std::abs(target - committed_target_raw_) > kTargetChangeEpsilon) {
                const double delta = target - motor_.angle();
                const double shortest = std::remainder(delta, 2.0 * std::numbers::pi);
                long_direction_ = (shortest >= 0.0) ? -1.0 : 1.0;
                committed_target_ = motor_.angle()
                                  + long_direction_
                                        * (2.0 * std::numbers::pi - std::abs(shortest));
                committed_target_raw_ = target;
            }
            // 误差限幅 ±2π，即使状态异常也不会进入单向飞车。
            error = std::clamp(
                committed_target_ - motor_.angle(), -2.0 * std::numbers::pi,
                2.0 * std::numbers::pi);
        }
        *angle_error_ = error;
        // 单圈规整角度(与单圈目标同刻度)，便于在 Foxglove 里直接和 /motor/control_angle 比对。
        *angle_single_turn_ = std::remainder(motor_.angle(), 2.0 * std::numbers::pi);
        *velocity_filtered_ = velocity_filter_.update(motor_.velocity());
        *enabled_ = has_target;
    }

    class MotorCommand : public rmcs_executor::Component {
    public:
        explicit MotorCommand(MotorPosition& motor)
            : motor_(motor) {}

        void update() override { motor_.command_update(); }

    private:
        MotorPosition& motor_;
    };

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can == Spec::kCans.kCan1)
            motor_.match_then_store_status(data.can_id, data.can_data);
    }

    std::shared_ptr<MotorCommand> command_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    device::DjiMotor motor_;

    std::atomic<double> target_angle_{0.0};
    std::atomic<bool> target_received_{false};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_angle_subscription_;

    // 优弧控制状态：目标真正变化时才锁定一条长弧方向并解缠出终点，之后不再翻转/累加。
    double committed_target_ = std::numeric_limits<double>::quiet_NaN(); // NaN 表示尚未锁定
    double committed_target_raw_ = std::numeric_limits<double>::quiet_NaN();
    double long_direction_ = 0.0; // +1 逆时针 / -1 顺时针
    static constexpr double kTargetChangeEpsilon = 1e-6;

    using Clock = std::chrono::steady_clock;

    filter::LowPassFilter<1> velocity_filter_{0.3};

    bool square_wave_ = false;
    double square_low_ = 0.0;
    double square_high_ = 1.5707963;
    double square_period_ = 1.0;
    Clock::time_point square_wave_start_{};

    bool sequence_wave_ = false;
    std::vector<double> sequence_angles_{};
    double sequence_dwell_ = 2.0;
    Clock::time_point sequence_start_{};

    OutputInterface<double> angle_error_;
    OutputInterface<double> angle_single_turn_;
    OutputInterface<double> control_angle_;
    OutputInterface<double> velocity_filtered_;
    OutputInterface<bool> enabled_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorPosition, rmcs_executor::Component)
