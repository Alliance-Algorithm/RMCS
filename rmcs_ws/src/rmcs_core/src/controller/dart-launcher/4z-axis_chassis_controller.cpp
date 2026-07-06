#include "controller/pid/matrix_pid_calculator.hpp"

#include <algorithm>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace rmcs_core::controller::dart {

class FourZAxisChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FourZAxisChassisController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , angle_pid_controller_(0.0, 0.0, 0.0) {

        register_input("/dart/chassis/pose", chassis_pose_);
        register_input("/dart/chassis/target_pose", chassis_target_pose_);
        register_input("/dart/chassis/height_control_velocity", chassis_height_control_velocoty_);

        register_output("/dart/chassis/front_left/control_velocity", left_front_control_velocity_, nan_);
        register_output("/dart/chassis/front_right/control_velocity", right_front_control_velocity_, nan_);
        register_output("/dart/chassis/back_left/control_velocity", left_back_control_velocity_, nan_);
        register_output("/dart/chassis/back_right/control_velocity", right_back_control_velocity_, nan_);

        chassis_length_ = get_parameter_or("chassis_length", chassis_length_);
        chassis_width_ = get_parameter_or("chassis_width", chassis_width_);
        chassis_motor_max_velocity_ =
            std::abs(get_parameter_or("chassis_motor_max_velocity", chassis_motor_max_velocity_));

        angle_pid_controller_.kp = get_parameter_or("angle_kp", 0.05);
        angle_pid_controller_.ki = get_parameter_or("angle_ki", 0.0);
        angle_pid_controller_.kd = get_parameter_or("angle_kd", 0.1);

        const double integral_limit = std::abs(get_parameter_or("angle_pid_integral_limit", 0.1));
        angle_pid_controller_.integral_min.setConstant(-integral_limit);
        angle_pid_controller_.integral_max.setConstant(integral_limit);
        angle_pid_controller_.output_min.setConstant(-chassis_motor_max_velocity_);
        angle_pid_controller_.output_max.setConstant(chassis_motor_max_velocity_);
    }

    void before_updating() override {
        if (!chassis_pose_.ready()) {
            chassis_pose_.make_and_bind_directly(Eigen::Vector2d(nan_, nan_));
        }
        if (!chassis_target_pose_.ready()) {
            chassis_target_pose_.make_and_bind_directly(Eigen::Vector2d(nan_, nan_));
        }
        if (!chassis_height_control_velocoty_.ready()) {
            chassis_height_control_velocoty_.make_and_bind_directly(nan_);
        }
    }

    void update() override {
        const auto pose = *chassis_pose_;
        const auto target_pose = *chassis_target_pose_;
        const double height_velocity = *chassis_height_control_velocoty_;

        if (!std::isfinite(pose.x()) || !std::isfinite(pose.y()) || !std::isfinite(target_pose.x())
            || !std::isfinite(target_pose.y()) || !std::isfinite(height_velocity)) {
            disable_all_controls_();
            return;
        }

        const Eigen::Vector2d delta_pose = target_pose - pose;
        const Eigen::Vector4d pitch_control = 0.5 * chassis_length_ * delta_pose.x() * pitch_control_mat_;
        const Eigen::Vector4d roll_control = 0.5 * chassis_width_ * delta_pose.y() * roll_control_mat_;

        const Eigen::Vector4d control_angle_vector = pitch_control + roll_control;
        const Eigen::Vector4d angle_control_velocity = angle_pid_controller_.update(control_angle_vector);

        const Eigen::Vector4d control_velocity_vector = angle_control_velocity + height_velocity * height_control_mat_;

        *left_front_control_velocity_ =
            std::clamp(control_velocity_vector.x(), -chassis_motor_max_velocity_, chassis_motor_max_velocity_);
        *right_front_control_velocity_ =
            std::clamp(control_velocity_vector.y(), -chassis_motor_max_velocity_, chassis_motor_max_velocity_);
        *left_back_control_velocity_ =
            std::clamp(control_velocity_vector.z(), -chassis_motor_max_velocity_, chassis_motor_max_velocity_);
        *right_back_control_velocity_ =
            std::clamp(control_velocity_vector.w(), -chassis_motor_max_velocity_, chassis_motor_max_velocity_);
    }

private:
    void disable_all_controls_() {
        angle_pid_controller_.reset();
        *left_front_control_velocity_ = nan_;
        *right_front_control_velocity_ = nan_;
        *left_back_control_velocity_ = nan_;
        *right_back_control_velocity_ = nan_;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> chassis_pose_;
    InputInterface<Eigen::Vector2d> chassis_target_pose_;
    InputInterface<double> chassis_height_control_velocoty_;

    OutputInterface<double> left_front_control_velocity_;
    OutputInterface<double> right_front_control_velocity_;
    OutputInterface<double> left_back_control_velocity_;
    OutputInterface<double> right_back_control_velocity_;

    const Eigen::Vector4d pitch_control_mat_{1, 1, -1, -1};
    const Eigen::Vector4d roll_control_mat_{1, -1, 1, -1};
    const Eigen::Vector4d height_control_mat_{1, 1, 1, 1};

    pid::MatrixPidCalculator<4> angle_pid_controller_;

    double chassis_length_ = 1.0;
    double chassis_width_ = 0.6;
    double chassis_motor_max_velocity_ = 20.0;
};

} // namespace rmcs_core::controller::dart

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::FourZAxisChassisController, rmcs_executor::Component)
