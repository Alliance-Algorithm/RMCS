#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <new>
#include <numbers>
#include <print>
#include <span>
#include <sstream>
#include <utility>

#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include "librmcs/board/c_board.hpp"

namespace rmcs_core::hardware {

class Gantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    Gantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        left_motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                static_cast<std::uint8_t>(get_parameter("left_id").as_int())}
                .enable_multi_turn_angle());
        right_motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                static_cast<std::uint8_t>(get_parameter("right_id").as_int())}
                .enable_multi_turn_angle()
                .set_reversed());

        // 电机输出轴转一圈横梁升高的米数
        lead_ = get_parameter("lead").as_double();
        height_per_rad_ = lead_ / (2.0 * std::numbers::pi);

        height_min_ = get_parameter("height_min").as_double();
        height_max_ = get_parameter("height_max").as_double();
        max_velocity_ = get_parameter("max_velocity").as_double();
        torque_limit_ = get_parameter("torque_limit").as_double();
        track_width_ = get_parameter("track_width").as_double();
        sync_error_limit_ = get_parameter("sync_error_limit").as_double();

        // 回零
        use_limit_switches_ = get_parameter("use_limit_switches").as_bool();
        left_limit_trigger_high_ = get_parameter("left_limit_trigger_high").as_bool();
        right_limit_trigger_high_ = get_parameter("right_limit_trigger_high").as_bool();
        homing_torque_ = get_parameter("homing_torque").as_double();
        homing_timeout_ = get_parameter("homing_timeout").as_double();

        // 未收到 GPIO 反馈前，限位状态置为“未触发”，避免误判回零完成
        left_limit_switch_.store(!left_limit_trigger_high_);
        right_limit_switch_.store(!right_limit_trigger_high_);

        // 重力/负载前馈：静态力矩偏置 + 慢积分负载观测（估计左右负载差）
        gravity_ff_left_ = get_parameter("gravity_feedforward_left").as_double();
        gravity_ff_right_ = get_parameter("gravity_feedforward_right").as_double();
        load_estimator_gain_ = get_parameter("load_estimator_gain").as_double();
        max_load_ff_ = get_parameter("max_load_feedforward").as_double();

        velocity_filter_.set_alpha(get_parameter("velocity_filter_alpha").as_double());

        // 外环：高度误差 -> 目标速度
        height_pid_.kp = get_parameter("height_kp").as_double();
        height_pid_.ki = get_parameter("height_ki").as_double();
        height_pid_.kd = get_parameter("height_kd").as_double();
        height_pid_.output_min = get_parameter("height_output_min").as_double();
        height_pid_.output_max = get_parameter("height_output_max").as_double();

        // 内环：速度误差 -> 公共力矩
        velocity_pid_.kp = get_parameter("velocity_kp").as_double();
        velocity_pid_.ki = get_parameter("velocity_ki").as_double();
        velocity_pid_.kd = get_parameter("velocity_kd").as_double();
        velocity_pid_.integral_min = get_parameter("velocity_integral_min").as_double();
        velocity_pid_.integral_max = get_parameter("velocity_integral_max").as_double();
        velocity_pid_.output_min = get_parameter("velocity_output_min").as_double();
        velocity_pid_.output_max = get_parameter("velocity_output_max").as_double();

        // 同步环：左右高度差 -> 差动力矩
        sync_pid_.kp = get_parameter("sync_kp").as_double();
        sync_pid_.ki = get_parameter("sync_ki").as_double();
        sync_pid_.kd = get_parameter("sync_kd").as_double();
        sync_pid_.output_min = get_parameter("sync_output_min").as_double();
        sync_pid_.output_max = get_parameter("sync_output_max").as_double();

        register_output("/gantry/height", height_, 0.0);
        register_output("/gantry/height_left", height_left_, 0.0);
        register_output("/gantry/height_right", height_right_, 0.0);
        register_output("/gantry/velocity", velocity_, 0.0);
        register_output("/gantry/sync_error", sync_error_, 0.0);
        register_output("/gantry/tilt_angle", tilt_angle_, 0.0);
        register_output("/gantry/target_height", target_height_output_, 0.0);
        register_output("/gantry/left/control_torque", left_control_torque_, 0.0);
        register_output("/gantry/right/control_torque", right_control_torque_, 0.0);
        register_output("/gantry/enabled", enabled_, false);
        register_output("/gantry/homed", homed_, false);
        register_output("/gantry/fault", fault_, false);

        target_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "/gantry/target_height_cmd", rclcpp::QoS{1},
            [this](std_msgs::msg::Float64::UniquePtr msg) {
                target_cmd_.store(msg->data);
                target_cmd_received_.store(true);
            });

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        home_service_ = create_service<std_srvs::srv::Trigger>(
            "/gantry/home", [this](
                                const std_srvs::srv::Trigger::Request::SharedPtr&,
                                const std_srvs::srv::Trigger::Response::SharedPtr& response) {
                home_requested_.store(true);
                response->success = true;
                response->message = "gantry homing started";
            });

        status_service_ = create_service<std_srvs::srv::Trigger>(
            "/rmcs/service/gantry_status",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr&,
                const std_srvs::srv::Trigger::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());

        board_->start_transmit().gpio_digital_read(
            Spec::kGpios.kPwm1, {
                                    .period_ms = 1,
                                    .asap = false,
                                    .rising_edge = false,
                                    .falling_edge = false,
                                    .capture_timestamp = false,
                                    .pull = librmcs::data::GpioPull::kUp,
                                });
        board_->start_transmit().gpio_digital_read(
            Spec::kGpios.kPwm2, {
                                    .period_ms = 1,
                                    .asap = false,
                                    .rising_edge = false,
                                    .falling_edge = false,
                                    .capture_timestamp = false,
                                    .pull = librmcs::data::GpioPull::kUp,
                                });

        last_update_ = Clock::now();
    }

    Gantry(const Gantry&) = delete;
    Gantry& operator=(const Gantry&) = delete;
    Gantry(Gantry&&) = delete;
    Gantry& operator=(Gantry&&) = delete;

    ~Gantry() override = default;

    void update() override {
        update_motors();
        dr16_.update_status();
        remote_control_->update();
        update_control();
    }

    void command_update() {
        auto builder = board_->start_transmit();
        device::CanPacket8 packet{
            left_motor_.generate_command(),
            right_motor_.generate_command(),
            device::CanPacket8::PaddingQuarter{},
            device::CanPacket8::PaddingQuarter{},
        };
        builder.can_transmit(
            Spec::kCans.kCan1, {
                                   .can_id = left_motor_.send_id(),
                                   .can_data = packet.as_bytes(),
                               });
    }

private:
    enum class State { kHoming, kReady, kFault };

    void update_motors() {
        left_motor_.update_status();
        right_motor_.update_status();
    }

    double angle_to_height(double angle) const { return angle * height_per_rad_; }

    void start_homing() {
        state_ = State::kHoming;
        left_homed_ = false;
        right_homed_ = false;
        homing_elapsed_ = 0.0;
        height_pid_.reset();
        velocity_pid_.reset();
        sync_pid_.reset();
        velocity_filter_.reset();
    }

    void finish_homing() {
        state_ = State::kReady;
        target_cmd_received_.store(false);
        target_height_ = 0.0;
        slew_target_ = 0.0;
        prev_slew_target_ = 0.0;
        height_pid_.reset();
        velocity_pid_.reset();
        sync_pid_.reset();
        velocity_filter_.reset();
        RCLCPP_INFO(get_logger(), "Gantry homing finished");
    }

    void enter_fault(const char* reason) {
        if (state_ == State::kFault)
            return;
        state_ = State::kFault;
        RCLCPP_ERROR(get_logger(), "Gantry fault: %s", reason);
        *left_control_torque_ = 0.0;
        *right_control_torque_ = 0.0;
    }

    void update_control() {
        if (home_requested_.exchange(false))
            start_homing();

        const auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last_update_).count();
        last_update_ = now;
        if (!(dt > 1e-5 && dt < 0.1))
            dt = 1.0 / 1000.0;

        const double h_left = angle_to_height(left_motor_.angle()) - left_zero_;
        const double h_right = angle_to_height(right_motor_.angle()) - right_zero_;
        const double v_left = angle_to_height(left_motor_.velocity());
        const double v_right = angle_to_height(right_motor_.velocity());
        const double h = 0.5 * (h_left + h_right);
        const double v = 0.5 * (v_left + v_right);
        const double sync_error = h_left - h_right;

        *height_left_ = h_left;
        *height_right_ = h_right;
        *height_ = h;
        *velocity_ = velocity_filter_.update(v);
        *sync_error_ = sync_error;
        *tilt_angle_ = std::atan2(sync_error, track_width_);
        *homed_ = (state_ == State::kReady);
        *fault_ = (state_ == State::kFault);

        switch (state_) {
        case State::kFault:
            *left_control_torque_ = 0.0;
            *right_control_torque_ = 0.0;
            *enabled_ = false;
            break;
        case State::kHoming:
            update_homing(dt, sync_error);
            *enabled_ = false;
            break;
        case State::kReady:
            update_ready(dt, h, v, v_left, v_right, sync_error);
            *enabled_ = (state_ == State::kReady);
            break;
        }
    }

    void update_homing(double dt, double sync_error) {
        if (!use_limit_switches_) {
            left_zero_ = angle_to_height(left_motor_.angle());
            right_zero_ = angle_to_height(right_motor_.angle());
            finish_homing();
            return;
        }

        homing_elapsed_ += dt;
        if (homing_elapsed_ > homing_timeout_) {
            enter_fault("homing timeout");
            return;
        }

        const bool left_triggered = left_limit_switch_.load() == left_limit_trigger_high_;
        const bool right_triggered = right_limit_switch_.load() == right_limit_trigger_high_;

        if (!left_homed_ && left_triggered) {
            left_zero_ = angle_to_height(left_motor_.angle());
            left_homed_ = true;
        }
        if (!right_homed_ && right_triggered) {
            right_zero_ = angle_to_height(right_motor_.angle());
            right_homed_ = true;
        }

        if (left_homed_ && right_homed_) {
            finish_homing();
            return;
        }

        const double tau_sync = sync_pid_.update(sync_error);
        double tau_left = left_homed_ ? 0.0 : -homing_torque_;
        double tau_right = right_homed_ ? 0.0 : -homing_torque_;
        tau_left -= tau_sync;
        tau_right += tau_sync;

        *left_control_torque_ = std::clamp(tau_left, -torque_limit_, torque_limit_);
        *right_control_torque_ = std::clamp(tau_right, -torque_limit_, torque_limit_);
    }

    void update_ready(
        double dt, double h, double v, double v_left, double v_right, double sync_error) {
        const bool armed = dr16_.valid() && dr16_.switch_right() == rmcs_msgs::Switch::MIDDLE;
        if (armed) {
            target_height_ += dr16_.joystick_left().y() * max_velocity_ * dt;
        } else if (target_cmd_received_.load()) {
            target_height_ = target_cmd_.load();
        }
        target_height_ = std::clamp(target_height_, height_min_, height_max_);

        // 目标速率限幅（梯形规划），保证平缓升降
        const double max_step = max_velocity_ * dt;
        slew_target_ += std::clamp(target_height_ - slew_target_, -max_step, max_step);
        const double target_rate = (slew_target_ - prev_slew_target_) / dt;
        prev_slew_target_ = slew_target_;

        if (std::abs(sync_error) > sync_error_limit_) {
            enter_fault("synchronization error exceeded limit");
            return;
        }

        // 静止时用速度残差积分估计左右侧保持力矩
        if (load_estimator_gain_ > 0.0 && std::abs(v) < 0.01) {
            load_left_ = std::clamp(
                load_left_ + load_estimator_gain_ * (-v_left) * dt, -max_load_ff_, max_load_ff_);
            load_right_ = std::clamp(
                load_right_ + load_estimator_gain_ * (-v_right) * dt, -max_load_ff_, max_load_ff_);
        }

        // 外环：高度 
        double v_cmd = height_pid_.update(slew_target_ - h);
        if (!std::isfinite(v_cmd))
            v_cmd = 0.0;
        v_cmd = std::clamp(v_cmd + target_rate, -max_velocity_, max_velocity_);

        // 内环：速度 
        double tau_common = velocity_pid_.update(v_cmd - v);
        if (!std::isfinite(tau_common))
            tau_common = 0.0;

        // 差动力矩消除左右错位
        double tau_sync = sync_pid_.update(sync_error);
        if (!std::isfinite(tau_sync))
            tau_sync = 0.0;

        const double tau_left = tau_common - tau_sync + gravity_ff_left_ + load_left_;
        const double tau_right = tau_common + tau_sync + gravity_ff_right_ + load_right_;

        *left_control_torque_ = std::clamp(tau_left, -torque_limit_, torque_limit_);
        *right_control_torque_ = std::clamp(tau_right, -torque_limit_, torque_limit_);
        *target_height_output_ = target_height_;
    }

    class GantryCommand : public rmcs_executor::Component {
    public:
        explicit GantryCommand(Gantry& gantry)
            : gantry_(gantry) {}

        void update() override { gantry_.command_update(); }

    private:
        Gantry& gantry_;
    };
    std::shared_ptr<GantryCommand> command_component_{
        create_partner_component<GantryCommand>(get_component_name() + "_command", *this),
    };

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can == Spec::kCans.kCan1) {
            left_motor_.match_then_store_status(data.can_id, data.can_data)
                || right_motor_.match_then_store_status(data.can_id, data.can_data);
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void gpio_digital_read_result_callback(
        const Spec::Gpio& gpio, const View::GpioDigital& data) override {
        if (gpio == Spec::kGpios.kPwm1)
            left_limit_switch_.store(data.high);
        else if (gpio == Spec::kGpios.kPwm2)
            right_limit_switch_.store(data.high);
    }

    void
        status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("    state: {}", state_name());
        text("    homed: {}", *homed_);
        text("    height: {:.4f} m", *height_);
        text("    sync_error: {:.4f} m", *sync_error_);
        text("    left_zero: {:.4f} m", left_zero_);
        text("    right_zero: {:.4f} m", right_zero_);

        response->message = feedback_message.str();
    }

    const char* state_name() const {
        switch (state_) {
        case State::kHoming: return "Homing";
        case State::kReady: return "Ready";
        case State::kFault: return "Fault";
        }
        return "Unknown";
    }

    using Clock = std::chrono::steady_clock;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> home_service_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    device::DjiMotor left_motor_{*this, *command_component_, "/gantry/left"};
    device::DjiMotor right_motor_{*this, *command_component_, "/gantry/right"};
    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    State state_ = State::kHoming;

    double lead_ = 0.005;
    double height_per_rad_ = 0.005 / (2.0 * std::numbers::pi);
    double height_min_ = 0.0;
    double height_max_ = 0.35;
    double max_velocity_ = 0.15;
    double torque_limit_ = 3.0;
    double track_width_ = 0.30;
    double sync_error_limit_ = 0.03;

    bool use_limit_switches_ = false;
    bool left_limit_trigger_high_ = false;
    bool right_limit_trigger_high_ = false;
    double homing_torque_ = 1.5;
    double homing_timeout_ = 6.0;
    double homing_elapsed_ = 0.0;
    bool left_homed_ = false;
    bool right_homed_ = false;
    std::atomic<bool> left_limit_switch_{false};
    std::atomic<bool> right_limit_switch_{false};

    double gravity_ff_left_ = 0.0;
    double gravity_ff_right_ = 0.0;
    double load_estimator_gain_ = 0.0;
    double max_load_ff_ = 2.0;
    double load_left_ = 0.0;
    double load_right_ = 0.0;

    double left_zero_ = 0.0;
    double right_zero_ = 0.0;

    double target_height_ = 0.0;
    double slew_target_ = 0.0;
    double prev_slew_target_ = 0.0;

    std::atomic<double> target_cmd_{0.0};
    std::atomic<bool> target_cmd_received_{false};
    std::atomic<bool> home_requested_{false};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription_;

    controller::pid::PidCalculator height_pid_;
    controller::pid::PidCalculator velocity_pid_;
    controller::pid::PidCalculator sync_pid_;

    filter::LowPassFilter<1> velocity_filter_{0.3};

    Clock::time_point last_update_{};

    OutputInterface<double> height_;
    OutputInterface<double> height_left_;
    OutputInterface<double> height_right_;
    OutputInterface<double> velocity_;
    OutputInterface<double> sync_error_;
    OutputInterface<double> tilt_angle_;
    OutputInterface<double> target_height_output_;
    OutputInterface<double> left_control_torque_;
    OutputInterface<double> right_control_torque_;
    OutputInterface<bool> enabled_;
    OutputInterface<bool> homed_;
    OutputInterface<bool> fault_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
