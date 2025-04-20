#include "controller/pid/pid_calculator.hpp"
#include <algorithm>
#include <arm_mode.hpp>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
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
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(0.92, 0.0, 0.01) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_output("/chassis_and_leg/enable_flag", is_chassis_and_leg_enable, true);
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
        register_output("/leg/omni/l/target_vel", omni_l_target_vel, NAN);
        register_output("/leg/omni/r/target_vel", omni_r_target_vel, NAN);

        register_output("/leg/joint/lf/control_torque", leg1_t, NAN);
        register_output("/leg/joint/lb/control_torque", leg2_t, NAN);
        register_output("/leg/joint/rb/control_torque", leg3_t, NAN);
        register_output("/leg/joint/rf/control_torque", leg4_t, NAN);

        register_input("/leg/encoder/lf/angle", theta_lf);
        register_input("/leg/encoder/lb/angle", theta_lb);
        register_input("/leg/encoder/rb/angle", theta_rb);
        register_input("/leg/encoder/rf/angle", theta_rf);

        register_input("yaw_imu_velocity", yaw_imu_velocity);
        register_input("yaw_imu_angle", yaw_imu_angle);
        register_output(
            "/chassis/big_yaw/target_angle_error", chassis_big_yaw_target_angle_error, NAN);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);
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
            reset_motor();
            yaw_target_ = *yaw_imu_angle;

        } else {

            *is_chassis_and_leg_enable = true;
            ModeSelection();
            switch (chassis_mode) {
            case rmcs_msgs::ChassisMode::Four_Wheel_Normal_Move: {
                // todo:加入wasd
                Eigen::Vector2d move_ = *joystick_left_;
                double spin_          = joystick_right_->y() / 3;
                yaw_target_ += joystick_right_->y() * 0.001;
                SteeringControl(move_, spin_);
                OmniWheelControl(Eigen::Vector2d{NAN, NAN});
                ControlYaw(yaw_target_);
                break;
            }
            case rmcs_msgs::ChassisMode::Six_Wheel_Normal_Move: {
                // todo:加入wasd
                double chassis_theta = *chassis_big_yaw_angle;
                if(std::abs(chassis_theta) < 0.009) chassis_theta = 0.0f;
                double spin = std::clamp(
                    following_velocity_controller_.update(chassis_theta), -0.5, 0.5);
                    RCLCPP_INFO(this->get_logger(), "%f", spin);
                Eigen::Vector2d move_ = *joystick_left_;
                yaw_target_ += joystick_right_->y() * 0.002;
                yaw_target_ = normalizeAngle(yaw_target_);
                ControlYaw(yaw_target_);
                SteeringControl(move_, spin);
                OmniWheelControl(Eigen::Vector2d{NAN, NAN});
                //    RCLCPP_INFO(this->get_logger(),"%f",yaw_target_-*yaw_imu_angle);

                // RCLCPP_INFO(this->get_logger(),"%f %f
                // %f",following_velocity_controller_.update(*chassis_big_yaw_angle),*yaw_imu_angle,std::clamp(following_velocity_controller_.update(*chassis_big_yaw_angle),-0.3,0.3));
                break;
            }
            case rmcs_msgs::ChassisMode::SPIN: {
                double spin = 1.2;
                Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle);
                Eigen::Vector2d move_ = rotation * (*joystick_left_);
                yaw_target_ += joystick_right_->y() * 0.002;
                ControlYaw(yaw_target_);
                SteeringControl(move_, spin);
                OmniWheelControl(Eigen::Vector2d{NAN, NAN});
                break;
            }
            default: break;
            }
        }
    }

private:
    void ModeSelection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if (switch_left == Switch::MIDDLE && switch_right == Switch::MIDDLE) {
            chassis_mode = ChassisMode::Four_Wheel_Normal_Move;
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::DOWN) {
            chassis_mode = ChassisMode::Six_Wheel_Normal_Move;
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {
            chassis_mode = ChassisMode::SPIN;
        }
    }
    void ControlYaw(double target_angle) {
        *chassis_big_yaw_target_angle_error = normalizeAngle(target_angle - *yaw_imu_angle);
        // todo：：add free——yaw
    }
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
    void OmniWheelControl(const Eigen::Vector2d& move) {
        *omni_l_target_vel = move.x() * (speed_limit / wheel_r);
        *omni_r_target_vel = move.x() * (speed_limit / wheel_r);
    }
    void reset_motor() {
        *steering_lf_target_angle_error = NAN;
        *steering_lb_target_angle_error = NAN;
        *steering_rb_target_angle_error = NAN;
        *steering_rf_target_angle_error = NAN;
        *steering_lf_target_angle_error = NAN;
        *steering_lb_target_angle_error = NAN;
        *steering_rb_target_angle_error = NAN;
        *steering_rf_target_angle_error = NAN;
        *omni_l_target_vel              = NAN;
        *omni_r_target_vel              = NAN;
        *steering_wheel_lf_target_vel   = NAN;
        *steering_wheel_lb_target_vel   = NAN;
        *steering_wheel_rb_target_vel   = NAN;
        *steering_wheel_rf_target_vel   = NAN;
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
    static double normalizeAngle(double angle) {

        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    pid::PidCalculator following_velocity_controller_;
    static constexpr double speed_limit = 5; // m/s
    static constexpr double wheel_r     = 0.11;
    double yaw_target_;

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
    // ————————————————————————steering——————————————————————————
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
    // ————————————————————————leg————————————————————————————————
    OutputInterface<double> omni_l_target_vel;
    OutputInterface<double> omni_r_target_vel;
    InputInterface<double> theta_lf;
    InputInterface<double> theta_lb;
    InputInterface<double> theta_rb;
    InputInterface<double> theta_rf;

    InputInterface<double> yaw_imu_velocity;
    InputInterface<double> yaw_imu_angle;

    OutputInterface<double> leg1_t;
    OutputInterface<double> leg2_t;
    OutputInterface<double> leg3_t;
    OutputInterface<double> leg4_t;

    OutputInterface<double> chassis_big_yaw_target_angle_error;
    InputInterface<double> chassis_big_yaw_angle;
    rmcs_msgs::ChassisMode chassis_mode = rmcs_msgs::ChassisMode::Four_Wheel_Normal_Move;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::Chassis_and_LegController, rmcs_executor::Component)