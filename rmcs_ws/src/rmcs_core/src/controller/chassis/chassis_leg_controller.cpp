#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
namespace rmcs_core::controller::chassis {
class Chassis_and_LegController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Chassis_and_LegController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_output("/chassis_and_leg/enable_flag", is_chassis_and_leg_enable, true);
        register_output("/chassis/big_yaw/control_torque", torque_test);

        register_input("/steering/steering/lf/angle", steering_lf_angle);
        register_input("/steering/steering/lb/angle", steering_lb_angle);
        register_input("/steering/steering/rb/angle", steering_rb_angle);
        register_input("/steering/steering/rf/angle", steering_rf_angle);
        register_output(
            "/steering/steering/lf/target_angle_error", steering_lf_target_angle_error, NAN);
        register_output(
            "/steering/steering/lb/target_angle_error", steering_lb_target_angle_error, NAN);
        register_output(
            "/steering/steering/rb/target_angle_error", steering_rb_target_angle_error, NAN);
        register_output(
            "/steering/steering/rf/target_angle_error", steering_rf_target_angle_error, NAN);

        register_output("/steering/wheel/lf/target_vel", steering_wheel_lf_target_vel, NAN);
        register_output("/steering/wheel/lb/target_vel", steering_wheel_lb_target_vel, NAN);
        register_output("/steering/wheel/rb/target_vel", steering_wheel_rb_target_vel, NAN);
        register_output("/steering/wheel/rf/target_vel", steering_wheel_rf_target_vel, NAN);
    }
    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            *is_chassis_and_leg_enable = false;

        } else {
            *is_chassis_and_leg_enable = true;
            *torque_test               = 0;
            Eigen::Vector2d move_ = *joystick_left_;
            double spin_ = joystick_right_->x()/10;
            SteeringControl(move_,spin_);
        }
    }

private:
    void SteeringControl(const Eigen::Vector2d& move, double spin_speed) {

        Eigen::Vector2d lf_vel = Eigen::Vector2d{-spin_speed, spin_speed} + move;
        Eigen::Vector2d lb_vel = Eigen::Vector2d{-spin_speed, -spin_speed} + move;
        Eigen::Vector2d rb_vel = Eigen::Vector2d{spin_speed, -spin_speed} + move;
        Eigen::Vector2d rf_vel = Eigen::Vector2d{spin_speed, spin_speed} + move;

        Eigen::Vector2d lb_vel_angle = lb_vel;
        Eigen::Vector2d lf_vel_angle = lf_vel;
        Eigen::Vector2d rb_vel_angle = rb_vel;
        Eigen::Vector2d rf_vel_angle = rf_vel;

        if (lf_vel.x() == 0 && lf_vel.y() == 0)
            lf_vel_angle = lf_vel_last_;
        else
            lf_vel_last_ = lf_vel;
        if (lb_vel.x() == 0 && lb_vel.y() == 0)
            lb_vel_angle = lb_vel_last_;
        else
            lb_vel_last_ = lb_vel;
        if (rf_vel.x() == 0 && rf_vel.y() == 0)
            rf_vel_angle = rf_vel_last_;
        else
            rf_vel_last_ = rf_vel;
        if (rb_vel.x() == 0 && rb_vel.y() == 0)
            rb_vel_angle = rb_vel_last_;
        else
            rb_vel_last_ = rb_vel;

        double err[4] = {
            -atan2(lb_vel_angle.y(), lb_vel_angle.x()) - *steering_lb_angle,
            -atan2(lf_vel_angle.y(), lf_vel_angle.x()) - *steering_lf_angle,
            -atan2(rb_vel_angle.y(), rb_vel_angle.x()) - *steering_rb_angle,
            -atan2(rf_vel_angle.y(), rf_vel_angle.x()) - *steering_rf_angle};

        *steering_lf_target_angle_error = norm_error_angle(err[1]);
        *steering_lb_target_angle_error = norm_error_angle(err[0]);
        *steering_rb_target_angle_error = norm_error_angle(err[2]);
        *steering_rf_target_angle_error = norm_error_angle(err[3]);
        *steering_wheel_lf_target_vel =
            lf_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[1]);
        *steering_wheel_lb_target_vel =
            lb_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[0]);
        *steering_wheel_rb_target_vel =
            rb_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[2]);
        *steering_wheel_rf_target_vel =
            rf_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[3]);
    }
    void OmniWheelControl(const Eigen::Vector2d& move){
        
    }
    static inline double norm_error_angle(const double& angle) {
        double tmp = angle;
        while (tmp > 2 * M_PI)
            tmp -= 2 * M_PI;
        while (tmp <= 0)
            tmp += 2 * M_PI;
        if (tmp > 2 * M_PI - tmp)
            tmp = tmp - 2 * M_PI;
        if (tmp > M_PI / 2)
            tmp = tmp - M_PI;
        else if (tmp < -M_PI / 2)
            tmp = tmp + M_PI;
        return tmp;
    }
    static inline double check_error_angle(const double& angle) {
        double tmp = angle;
        while (tmp > 2 * M_PI)
            tmp -= 2 * M_PI;
        while (tmp <= 0)
            tmp += 2 * M_PI;
        tmp -= M_PI;

        return sin(abs(tmp) - M_PI_2);
    }

    static constexpr double speed_limit = 3;
    static constexpr double wheel_r     = 0.11;

    const Eigen::Vector2d lf_vel_{1, -1};
    const Eigen::Vector2d lb_vel_{1, 1};
    const Eigen::Vector2d rb_vel_{-1, -1};
    const Eigen::Vector2d rf_vel_{-1, 1};
    Eigen::Vector2d lf_vel_last_{1, -1};
    Eigen::Vector2d lb_vel_last_{1, 1};
    Eigen::Vector2d rb_vel_last_{-1, -1};
    Eigen::Vector2d rf_vel_last_{-1, 1};
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    OutputInterface<bool> is_chassis_and_leg_enable;
    OutputInterface<double> torque_test;
    // ————————————————————————steering————————————————————
    InputInterface<double> steering_lf_angle;
    InputInterface<double> steering_lb_angle;
    InputInterface<double> steering_rb_angle;
    InputInterface<double> steering_rf_angle;

    OutputInterface<double> steering_lf_target_angle_error;
    OutputInterface<double> steering_lb_target_angle_error;
    OutputInterface<double> steering_rb_target_angle_error;
    OutputInterface<double> steering_rf_target_angle_error;
    OutputInterface<double> steering_wheel_lf_target_vel;
    OutputInterface<double> steering_wheel_lb_target_vel;
    OutputInterface<double> steering_wheel_rb_target_vel;
    OutputInterface<double> steering_wheel_rf_target_vel;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::Chassis_and_LegController, rmcs_executor::Component)