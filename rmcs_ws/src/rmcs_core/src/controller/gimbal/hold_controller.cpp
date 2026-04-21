#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class HoldController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HoldController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joystick_deadband_(get_parameter("joystick_deadband").as_double())
        , mouse_deadband_(get_parameter("mouse_deadband").as_double())
        , hold_yaw_kd_(get_parameter("hold_yaw_kd").as_double())
        , hold_pitch_kd_(get_parameter("hold_pitch_kd").as_double()) {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu_);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu_);
        register_output("/gimbal/hold/desired", hold_desired_, false);//false不强制绑定，在yaml文件里面注释这个component则自动脱离绑定
        register_output("/gimbal/yaw/hold_feedforward", yaw_hold_feedforward_, 0.0);
        register_output("/gimbal/pitch/hold_feedforward", pitch_hold_feedforward_, 0.0);
    }

    void update() override {
        using namespace rmcs_msgs; // NOLINT(google-build-using-namespace)
        const auto sl = *switch_left_;
        const auto sr = *switch_right_;

        const bool gimbal_disabled =
            sl == Switch::UNKNOWN || sr == Switch::UNKNOWN
            || (sl == Switch::DOWN && sr == Switch::DOWN);

        if (gimbal_disabled) {
            idle_counter_ = 0;
            *hold_desired_ = false;
        } else if (!is_idle()) {
            idle_counter_ = 0;
            *hold_desired_ = false;
        } else {
            idle_counter_ = std::min(idle_counter_ + 1, kEntryTicks);
            *hold_desired_ = (idle_counter_ >= kEntryTicks);
        }

        const bool feedforward_gate = *hold_desired_;

        if (feedforward_gate) {
            filtered_yaw_vel_ += kVelFilterAlpha * (*yaw_velocity_imu_ - filtered_yaw_vel_);
            filtered_pitch_vel_ += kVelFilterAlpha * (*pitch_velocity_imu_ - filtered_pitch_vel_);
            *yaw_hold_feedforward_ = -hold_yaw_kd_ * filtered_yaw_vel_;
            *pitch_hold_feedforward_ = -hold_pitch_kd_ * filtered_pitch_vel_;
        } else {
            filtered_yaw_vel_ = 0.0;
            filtered_pitch_vel_ = 0.0;
            *yaw_hold_feedforward_ = 0.0;
            *pitch_hold_feedforward_ = 0.0;
        }
    }

private:
    [[nodiscard]] bool is_idle() const {
        const auto joy_norm = joystick_left_->norm();
        const auto mouse_norm = mouse_velocity_->norm();
        return std::isfinite(joy_norm) && std::isfinite(mouse_norm)
            && joy_norm < joystick_deadband_ && mouse_norm < mouse_deadband_;
    }

    // 50 ticks @ 1 kHz = 50 ms debounce before engaging hold.
    static constexpr int kEntryTicks = 50;
    // LPF on IMU velocity for hold damping: f_c ≈ 50 Hz, removes gyro noise above 50 Hz.
    // α = dt / (τ + dt), τ = 1/(2π·50)
    static constexpr double kVelFilterAlpha = 0.12;

    const double joystick_deadband_;
    const double mouse_deadband_;
    const double hold_yaw_kd_;
    const double hold_pitch_kd_;
    int idle_counter_ = 0;
    double filtered_yaw_vel_ = 0.0;
    double filtered_pitch_vel_ = 0.0;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> yaw_velocity_imu_;
    InputInterface<double> pitch_velocity_imu_;

    OutputInterface<bool> hold_desired_;
    OutputInterface<double> yaw_hold_feedforward_;
    OutputInterface<double> pitch_hold_feedforward_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::HoldController, rmcs_executor::Component)
