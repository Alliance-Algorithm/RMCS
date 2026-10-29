#include <cmath>
#include <cstdint>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::shooting {

// 右拨杆 DOWN 时：旋钮上拨一下以 close_torque 开环合拢，下拨一下以 open_torque 开环张开
// 堵转后力矩归零，等待下一次拨动
class GripperKnobController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GripperKnobController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , close_torque_(get_parameter("close_torque").as_double())
        , open_torque_(get_parameter("open_torque").as_double())
        , stall_angle_(get_parameter("stall_angle").as_double())
        , stall_ticks_(get_parameter("stall_ticks").as_int())
        , knob_deadzone_(get_parameter("knob_deadzone").as_double()) {

        register_input("/shooting/gripper_motor/angle", angle_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/rotary_knob", knob_);

        register_output("/shooting/gripper_motor/control_torque", control_torque_, 0.0);
    }

    void update() override {
        if (*switch_right_ != rmcs_msgs::Switch::DOWN) {
            last_knob_ = Knob::Unknown;
            torque_ = 0.0;
        } else {
            // 只在旋钮离开中位的那一刻触发，刚进入夹爪模式时旋钮不在中位也不会误触发
            const auto knob = read_knob();
            if (last_knob_ == Knob::Middle && knob != Knob::Middle) {
                const bool closing = knob == Knob::Up;
                RCLCPP_INFO(get_logger(), "gripper %s", closing ? "closing" : "opening");
                start(closing ? close_torque_ : open_torque_);
            }
            last_knob_ = knob;

            if (torque_ != 0.0 && stalled()) {
                RCLCPP_INFO(get_logger(), "gripper stalled, torque released");
                torque_ = 0.0;
            }
        }

        *control_torque_ = torque_;
    }

private:
    enum class Knob { Unknown, Up, Middle, Down };

    // 与上游 Dr16 / RemoteControl 的符号约定一致：负值 = 上拨，正值 = 下拨
    // 用模拟量判中位：旋钮静止位置往往有偏移，量化成开关会一直读成一个方向
    Knob read_knob() const {
        if (*knob_ < -knob_deadzone_)
            return Knob::Up;
        if (*knob_ > knob_deadzone_)
            return Knob::Down;
        return Knob::Middle;
    }

    void start(double torque) {
        torque_ = torque;
        progress_angle_ = *angle_;
        stall_count_ = 0;
    }

    // 3508 速度通道低速时噪声很大，改看角度：
    // 连续 stall_ticks 个周期累计转不过 stall_angle 即为堵转
    bool stalled() {
        if (std::abs(*angle_ - progress_angle_) >= stall_angle_) {
            progress_angle_ = *angle_;
            stall_count_ = 0;
            return false;
        }
        return ++stall_count_ >= stall_ticks_;
    }

    const double close_torque_;
    const double open_torque_;
    const double stall_angle_;
    const int64_t stall_ticks_;
    const double knob_deadzone_;

    Knob last_knob_ = Knob::Unknown;
    double torque_ = 0.0;
    double progress_angle_ = 0.0;
    int64_t stall_count_ = 0;

    InputInterface<double> angle_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> knob_;

    OutputInterface<double> control_torque_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::GripperKnobController, rmcs_executor::Component)
