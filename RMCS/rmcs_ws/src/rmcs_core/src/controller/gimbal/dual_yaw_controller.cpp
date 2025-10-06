
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

class DualYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
            pid.kp = get_parameter(name + "_kp").as_double();
            pid.ki = get_parameter(name + "_ki").as_double();
            pid.kd = get_parameter(name + "_kd").as_double();
            get_parameter(name + "_integral_min", pid.integral_min);
            get_parameter(name + "_integral_max", pid.integral_max);
            get_parameter(name + "_output_min", pid.output_min);
            get_parameter(name + "_output_max", pid.output_max);
        };
        set_pid_parameter(top_yaw_angle_pid_, "top_yaw_angle");
        set_pid_parameter(top_yaw_velocity_pid_, "top_yaw_velocity");
        set_pid_parameter(bottom_yaw_angle_pid_, "bottom_yaw_angle");
        set_pid_parameter(bottom_yaw_velocity_pid_, "bottom_yaw_velocity");

        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);

        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);

        register_input("/gimbal/yaw/control_angle_error", control_angle_error_);
        register_input("/gimbal/yaw/control_angle_shift", control_angle_shift_, false);

        register_output("/gimbal/top_yaw/control_torque", top_yaw_control_torque_, 0.0);
        register_output("/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque_, 0.0);

        register_output("/gimbal/top_yaw/control_angle", top_yaw_control_torque_, 0.0);
        register_output("/gimbal/bottom_yaw/control_angle_shift", bottom_yaw_control_torque_, 0.0);

        status_component_ =
            create_partner_component<DualYawStatus>(get_component_name() + "_status");
    }

    void before_updating() override {
        if (!control_angle_shift_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_shift\", set to NaN.");
            control_angle_shift_.bind_directly(nan_);
        }
    }

    void update() override {
        if (std::isnan(*control_angle_error_)) {
            *top_yaw_control_torque_ = nan_;
            *bottom_yaw_control_torque_ = nan_;
        } else {
            *top_yaw_control_torque_ = top_yaw_velocity_pid_.update(
                top_yaw_angle_pid_.update(*control_angle_error_) - *gimbal_yaw_velocity_imu_);

            *bottom_yaw_control_torque_ = bottom_yaw_velocity_pid_.update(
                bottom_yaw_angle_pid_.update(bottom_yaw_control_error())
                - bottom_yaw_velocity_imu());
        }

        if (std::isnan(*control_angle_shift_)) {
            *top_yaw_control_angle_ = nan_;
            *bottom_yaw_control_angle_shift_ = nan_;
        } else {
            *top_yaw_control_angle_ = 0.0;
            *bottom_yaw_control_angle_shift_ = *control_angle_shift_;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    double bottom_yaw_control_error() {
        double err = *top_yaw_angle_ + *control_angle_error_;
        if (err > std::numbers::pi)
            err -= 2 * std::numbers::pi;
        return err;
    }

    double bottom_yaw_velocity_imu() { return *chassis_yaw_velocity_imu_ + *bottom_yaw_velocity_; }

    InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
    InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;

    InputInterface<double> gimbal_yaw_velocity_imu_, chassis_yaw_velocity_imu_;

    InputInterface<double> control_angle_error_, control_angle_shift_;

    pid::PidCalculator top_yaw_angle_pid_, top_yaw_velocity_pid_;
    pid::PidCalculator bottom_yaw_angle_pid_, bottom_yaw_velocity_pid_;

    OutputInterface<double> top_yaw_control_torque_;
    OutputInterface<double> bottom_yaw_control_torque_;

    OutputInterface<double> top_yaw_control_angle_;
    OutputInterface<double> bottom_yaw_control_angle_shift_;

    class DualYawStatus : public rmcs_executor::Component {
    public:
        explicit DualYawStatus() {
            register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
            register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
            register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
            register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);

            register_output("/gimbal/yaw/angle", yaw_angle_, 0.0);
            register_output("/gimbal/yaw/velocity", yaw_velocity_, 0.0);
        }

        void update() override {
            double yaw_angle = *top_yaw_angle_ + *bottom_yaw_angle_;
            if (yaw_angle < 0)
                yaw_angle += 2 * std::numbers::pi;
            else if (yaw_angle > 2 * std::numbers::pi)
                yaw_angle -= 2 * std::numbers::pi;
            *yaw_angle_ = yaw_angle;

            *yaw_velocity_ = *top_yaw_velocity_ + *bottom_yaw_velocity_;
        }

    private:
        InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
        InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;

        OutputInterface<double> yaw_angle_, yaw_velocity_;
    };
    std::shared_ptr<DualYawStatus> status_component_;
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawController, rmcs_executor::Component)