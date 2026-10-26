#include <algorithm>
#include <chrono>
#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::shooting {

class GripperKnobController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GripperKnobController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , close_speed_(get_parameter("close_speed").as_double())
        , open_speed_(get_parameter("open_speed").as_double())
        , position_kp_(get_parameter("position_kp").as_double())
        , max_lead_rad_(get_parameter("max_lead_rad").as_double())
        , torque_limit_(get_parameter("torque_limit").as_double())
        , clamp_torque_(get_parameter("clamp_torque").as_double())
        , clamp_time_s_(get_parameter("clamp_time_s").as_double())
        , stall_progress_rad_(get_parameter("stall_progress_rad").as_double())
        , stall_confirm_ticks_(get_parameter("stall_confirm_ticks").as_int())
        , stall_ignore_ticks_(get_parameter("stall_ignore_ticks").as_int())
        , over_speed_velocity_(get_parameter("over_speed_velocity").as_double())
        , edge_lockout_ticks_(get_parameter("edge_lockout_ticks").as_int())
        , knob_deadzone_(get_parameter("knob_deadzone").as_double())
        , angle_velocity_filter_alpha_(get_parameter("angle_velocity_filter_alpha").as_double()) {

        register_input("/shooting/gripper_motor/angle", angle_);
        register_input("/shooting/hardware_ready", hardware_ready_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/rotary_knob", knob_);

        register_output("/shooting/gripper_motor/control_torque", control_torque_, NAN);
        register_output("/shooting/gripper_motor/velocity_filtered", velocity_filtered_, NAN);
    }

    void update() override {
        const double velocity = update_velocity();
        *velocity_filtered_ = velocity;

        const auto ready = !(hardware_ready_.ready() && !*hardware_ready_);
        const auto armed = *switch_right_ == rmcs_msgs::Switch::DOWN;

        const auto previous = state_;
        state_ = update_state(velocity, ready, armed, read_knob());
        *control_torque_ = update_torque();

        log_transition(previous);
    }

private:
    enum class State { Disarmed, Hold, Closing, Clamping, Opening };

    enum class Knob { Unknown, Up, Middle, Down };

    static bool is_pushing(State state) {
        return state == State::Closing || state == State::Opening || state == State::Clamping;
    }

    static const char* to_string(State state) {
        switch (state) {
        case State::Disarmed: return "disarmed";
        case State::Hold: return "hold";
        case State::Closing: return "closing";
        case State::Clamping: return "clamping";
        case State::Opening: return "opening";
        default: return "unknown";
        }
    }

    void log_transition(State previous) {
        if (state_ == previous)
            return;

        if (previous == State::Closing && state_ == State::Clamping)
            RCLCPP_INFO(
                get_logger(), "gripper closing limit reached, clamping %.2f s",
                clamp_time_s_);
        else if (previous != State::Clamping && is_pushing(previous) && state_ == State::Hold)
            RCLCPP_INFO(
                get_logger(), "gripper %s done (%s), peak=%+.2f, travel=%+.2f rad",
                to_string(previous), over_speed_ ? "over-speed" : "stall", peak_velocity_,
                *angle_ - travel_start_angle_);
        else
            RCLCPP_INFO(get_logger(), "gripper %s", to_string(state_));
    }

    // 与上游 Dr16 / RemoteControl 的符号约定一致：负值 = 上拨，正值 = 下拨
    // 用模拟量判中位：旋钮静止位置往往有偏移，量化成开关会一直读成一个方向
    Knob read_knob() const {
        if (!knob_.ready())
            return Knob::Unknown;

        const double value = *knob_;
        if (value < -knob_deadzone_)
            return Knob::Up;
        if (value > knob_deadzone_)
            return Knob::Down;
        return Knob::Middle;
    }

    // 3508 自带的速度通道低速时噪声比真实值大一个量级，改用编码器角度差分
    double update_velocity() {
        const auto now = std::chrono::steady_clock::now();
        const double angle = *angle_;

        if (has_last_angle_) {
            const double dt = std::chrono::duration<double>(now - last_angle_time_).count();
            if (dt > 1e-4 && dt < 0.1) {
                dt_ = dt;

                double delta = angle - last_angle_;
                // 角度在 ±π 处跳变，直接差分会得到几百 rad/s 的假速度
                if (delta > M_PI)
                    delta -= 2.0 * M_PI;
                else if (delta < -M_PI)
                    delta += 2.0 * M_PI;

                estimated_velocity_ +=
                    angle_velocity_filter_alpha_ * (delta / dt - estimated_velocity_);
            }
        }

        last_angle_ = angle;
        last_angle_time_ = now;
        has_last_angle_ = true;

        return estimated_velocity_;
    }

    State update_state(double velocity, bool ready, bool armed, Knob knob) {
        if (!ready || !armed || knob == Knob::Unknown)
            return reset();

        if (state_ == State::Disarmed) {
            latch_ = knob;
            return prepare(State::Hold);
        }

        if (lockout_ticks_ > 0)
            --lockout_ticks_;

        if (knob == Knob::Middle)
            latch_ = Knob::Middle;
        else if (latch_ == Knob::Middle && lockout_ticks_ == 0) {
            latch_ = knob;
            if (knob == Knob::Up)
                return prepare(State::Closing);
            if (knob == Knob::Down)
                return prepare(State::Opening);
        }

        if (!is_pushing(state_))
            return State::Hold;

        if (std::abs(velocity) > over_speed_velocity_) {
            over_speed_ = true;
            return State::Hold;
        }

        // 已到合拢极限：保持压力一段时间，让接触面预紧
        // 用真实 dt 累计，所以就是实打实的秒数，不受控制周期抖动影响
        if (state_ == State::Clamping) {
            clamp_elapsed_s_ += dt_;
            return clamp_elapsed_s_ >= clamp_time_s_ ? State::Hold : State::Clamping;
        }

        peak_velocity_ = std::max(peak_velocity_, std::abs(velocity));

        // 堵转判据：看角度有没有真的前进，而不是看目标有没有被顶住
        // 高摩擦段机构会变慢（目标被 max_lead 顶住），但仍在前进，不能算堵转
        if (std::abs(*angle_ - last_progress_angle_) >= stall_progress_rad_) {
            last_progress_angle_ = *angle_;
            stall_ticks_ = 0;
            return state_;
        }

        if (ignore_ticks_ < stall_ignore_ticks_) {
            ++ignore_ticks_;
            return state_;
        }

        if (++stall_ticks_ < stall_confirm_ticks_)
            return state_;

        return state_ == State::Closing && clamp_time_s_ > 0.0 ? enter_clamping() : State::Hold;
    }

    // 斜坡跟踪：目标角按固定速率前进，力矩只负责跟上它
    // 目标领先量被 max_lead 限制，卡住时不会无限超前，脱困后也不会猛冲
    double update_torque() {
        double speed = 0.0;
        switch (state_) {
        case State::Closing:
        case State::Clamping: speed = close_speed_; break;
        case State::Opening: speed = open_speed_; break;
        default:
            target_angle_ = *angle_;
            return 0.0;
        }

        target_angle_ += speed * dt_;

        const double lead = target_angle_ - *angle_;
        if (std::abs(lead) > max_lead_rad_)
            target_angle_ = *angle_ + (lead > 0 ? max_lead_rad_ : -max_lead_rad_);

        const double limit = state_ == State::Clamping ? clamp_torque_ : torque_limit_;
        return std::clamp(position_kp_ * (target_angle_ - *angle_), -limit, limit);
    }

    State enter_clamping() {
        clamp_elapsed_s_ = 0.0;
        return State::Clamping;
    }

    State reset() {
        latch_ = Knob::Unknown;
        return prepare(State::Disarmed);
    }

    State prepare(State state) {
        ignore_ticks_ = 0;
        stall_ticks_ = 0;
        clamp_elapsed_s_ = 0.0;
        over_speed_ = false;
        peak_velocity_ = 0.0;
        travel_start_angle_ = *angle_;
        target_angle_ = *angle_;
        last_progress_angle_ = *angle_;
        lockout_ticks_ = is_pushing(state) ? edge_lockout_ticks_ : 0;
        return state;
    }

    const double close_speed_;
    const double open_speed_;
    const double position_kp_;
    const double max_lead_rad_;
    const double torque_limit_;
    const double clamp_torque_;
    const double clamp_time_s_;
    const double stall_progress_rad_;
    const int64_t stall_confirm_ticks_;
    const int64_t stall_ignore_ticks_;
    const double over_speed_velocity_;
    const int64_t edge_lockout_ticks_;
    const double knob_deadzone_;
    const double angle_velocity_filter_alpha_;

    State state_ = State::Disarmed;
    Knob latch_ = Knob::Unknown;
    int64_t ignore_ticks_ = 0;
    int64_t stall_ticks_ = 0;
    double clamp_elapsed_s_ = 0.0;
    int64_t lockout_ticks_ = 0;
    bool over_speed_ = false;
    double peak_velocity_ = 0.0;
    double travel_start_angle_ = 0.0;
    double target_angle_ = 0.0;
    double last_progress_angle_ = 0.0;

    double dt_ = 0.001;
    double estimated_velocity_ = 0.0;
    double last_angle_ = 0.0;
    bool has_last_angle_ = false;
    std::chrono::steady_clock::time_point last_angle_time_{};

    InputInterface<double> angle_;
    InputInterface<bool> hardware_ready_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> knob_;

    OutputInterface<double> control_torque_;
    OutputInterface<double> velocity_filtered_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::GripperKnobController, rmcs_executor::Component)
