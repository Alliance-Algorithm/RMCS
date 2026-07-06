#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class FourZAxisChassisStatus
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FourZAxisChassisStatus()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/dart/chassis/imu/pitch_angle", chassis_pitch_, false);
        register_input("/dart/chassis/imu/roll_angle", chassis_roll_, false);

        register_input("/dart/chassis/front_left/velocity", left_front_velocity_, false);
        register_input("/dart/chassis/front_right/velocity", right_front_velocity_, false);
        register_input("/dart/chassis/back_left/velocity", left_back_velocity_, false);
        register_input("/dart/chassis/back_right/velocity", right_back_velocity_, false);

        register_input("/dart/chassis/front_left/torque", left_front_torque_, false);
        register_input("/dart/chassis/front_right/torque", right_front_torque_, false);
        register_input("/dart/chassis/back_left/torque", left_back_torque_, false);
        register_input("/dart/chassis/back_right/torque", right_back_torque_, false);

        register_input("/dart/chassis/front_left/max_torque", left_front_max_torque_, false);
        register_input("/dart/chassis/front_right/max_torque", right_front_max_torque_, false);
        register_input("/dart/chassis/back_left/max_torque", left_back_max_torque_, false);
        register_input("/dart/chassis/back_right/max_torque", right_back_max_torque_, false);

        register_output("/dart/chassis/pose", chassis_pose_, 0.0, 0.0);
        register_output("/dart/chassis/front_left/stalled", left_front_stalled_, false);
        register_output("/dart/chassis/front_right/stalled", right_front_stalled_, false);
        register_output("/dart/chassis/back_left/stalled", left_back_stalled_, false);
        register_output("/dart/chassis/back_right/stalled", right_back_stalled_, false);

        stall_velocity_threshold_ = std::abs(get_parameter_or("stall_velocity_threshold", 0.5));
        stall_torque_ratio_threshold_ = std::abs(get_parameter_or("stall_torque_ratio_threshold", 0.6));
        stall_confirm_ticks_ = get_parameter_or("stall_confirm_ticks", 300);
        stall_release_ticks_ = get_parameter_or("stall_release_ticks", 50);
    }

    void update() override {
        chassis_pose_->x() = *chassis_pitch_;
        chassis_pose_->y() = *chassis_roll_;

        update_stall_state_(
            *left_front_velocity_, *left_front_torque_, *left_front_max_torque_, left_front_counter_,
            left_front_release_counter_, *left_front_stalled_);
        update_stall_state_(
            *right_front_velocity_, *right_front_torque_, *right_front_max_torque_, right_front_counter_,
            right_front_release_counter_, *right_front_stalled_);
        update_stall_state_(
            *left_back_velocity_, *left_back_torque_, *left_back_max_torque_, left_back_counter_,
            left_back_release_counter_, *left_back_stalled_);
        update_stall_state_(
            *right_back_velocity_, *right_back_torque_, *right_back_max_torque_, right_back_counter_,
            right_back_release_counter_, *right_back_stalled_);
    }

private:
    void update_stall_state_(
        double velocity, double torque, double max_torque, int& confirm_counter, int& release_counter,
        bool& stalled) const {
        if (!std::isfinite(velocity) || !std::isfinite(torque) || !std::isfinite(max_torque) || max_torque <= 0.0) {
            confirm_counter = 0;
            release_counter = 0;
            stalled = false;
            return;
        }

        const bool stall_condition = std::abs(velocity) <= stall_velocity_threshold_
                                  && std::abs(torque) >= stall_torque_ratio_threshold_ * max_torque;
        if (stall_condition) {
            confirm_counter = std::min(confirm_counter + 1, stall_confirm_ticks_);
            release_counter = 0;
            if (confirm_counter >= stall_confirm_ticks_)
                stalled = true;
            return;
        }

        confirm_counter = 0;
        if (!stalled) {
            release_counter = 0;
            return;
        }

        release_counter = std::min(release_counter + 1, stall_release_ticks_);
        if (release_counter >= stall_release_ticks_) {
            release_counter = 0;
            stalled = false;
        }
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> chassis_pitch_;
    InputInterface<double> chassis_roll_;

    InputInterface<double> left_front_velocity_;
    InputInterface<double> right_front_velocity_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;

    InputInterface<double> left_front_torque_;
    InputInterface<double> right_front_torque_;
    InputInterface<double> left_back_torque_;
    InputInterface<double> right_back_torque_;

    InputInterface<double> left_front_max_torque_;
    InputInterface<double> right_front_max_torque_;
    InputInterface<double> left_back_max_torque_;
    InputInterface<double> right_back_max_torque_;

    OutputInterface<Eigen::Vector2d> chassis_pose_;
    OutputInterface<bool> left_front_stalled_;
    OutputInterface<bool> right_front_stalled_;
    OutputInterface<bool> left_back_stalled_;
    OutputInterface<bool> right_back_stalled_;

    double stall_velocity_threshold_ = 0.0;
    double stall_torque_ratio_threshold_ = 0.0;
    int stall_confirm_ticks_ = 1;
    int stall_release_ticks_ = 1;

    mutable int left_front_counter_ = 0;
    mutable int right_front_counter_ = 0;
    mutable int left_back_counter_ = 0;
    mutable int right_back_counter_ = 0;

    mutable int left_front_release_counter_ = 0;
    mutable int right_front_release_counter_ = 0;
    mutable int left_back_release_counter_ = 0;
    mutable int right_back_release_counter_ = 0;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::FourZAxisChassisStatus, rmcs_executor::Component)
