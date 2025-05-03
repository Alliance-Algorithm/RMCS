#include <cmath>
#include <cstring>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using Formula = std::tuple<double, double, double>;

public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , steerings_angle_pid_caculator_(10.685, 0., 0.)
        , wheels_velocity_pid_calculator_(0.685, 0., 0.) {

        register_input("/chassis/left_front_wheel/max_torque", wheel_motor_max_control_torque_);

        register_input("/chassis/left_front_steering/angle", left_front_steering_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_steering_angle_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input("/chassis/control_velocity", control_velocity_, false);

        register_output(
            "/chassis/left_front_steering/control_angle_error",
            left_front_steering_control_angle_error_, nan_);
        register_output(
            "/chassis/left_back_steering/control_angle_error",
            left_back_steering_control_angle_error_, nan_);
        register_output(
            "/chassis/right_back_steering/control_angle_error",
            right_back_steering_control_angle_error_, nan_);
        register_output(
            "/chassis/right_front_steering/control_angle_error",
            right_front_steering_control_angle_error_, nan_);

        register_output(
            "/chassis/left_front_wheel/control_velocity", left_front_wheel_control_velocity_, nan_);
        register_output(
            "/chassis/left_back_wheel/control_velocity", left_back_wheel_control_velocity_, nan_);
        register_output(
            "/chassis/right_back_wheel/control_velocity", right_back_wheel_control_velocity_, nan_);
        register_output(
            "/chassis/right_front_wheel/control_velocity", right_front_wheel_control_velocity_,
            nan_);

        register_input("/chassis/control_power_limit", power_limit_, false);
        register_input("/chassis/steering/power", steering_power_);
        register_input("/chassis/power", chassis_power_);

        steering_power_publisher_ =
            create_publisher<std_msgs::msg::Float64>("/chassis/steering/power", 20);
        chassis_power_publisher_ = create_publisher<std_msgs::msg::Float64>("/chassis/power", 20);
        power_limit_publisher_ =
            create_publisher<std_msgs::msg::Float64>("chassis/power_limit", 20);

        const std::array<std::string, 4> publisher_prefixes = {
            "/chassis/lf_steering", "/chassis/lb_steering", "/chassis/rb_steering",
            "/chassis/rf_steering"};

        size_t index = 0;
        for (auto& publishers : steering_control_angle_error_publishers_) {
            steering_control_angle_error_msgs_[index] = std::make_unique<std_msgs::msg::Float64>();

            publishers = create_publisher<std_msgs::msg::Float64>(
                publisher_prefixes[index++] + "/control_angle_error", 20);
        }
    }

    void update() override {
        // Eigen::Vector3d wheel_velocities[4] = {
        //     Eigen::AngleAxisd(*left_front_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *left_front_wheel_velocity_ * wheel_radius_,
        //     Eigen::AngleAxisd(*left_back_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *left_back_wheel_velocity_ * wheel_radius_,
        //     Eigen::AngleAxisd(*right_back_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *right_back_wheel_velocity_ * wheel_radius_,
        //     Eigen::AngleAxisd(*right_front_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *right_front_wheel_velocity_ * wheel_radius_,
        // };

        auto [translational_control_velocity, angular_control_velocity] = update_control_velocity();
        control_vector_ =
            update_control_vector(translational_control_velocity, angular_control_velocity);

        update_control_torque_and_angle(control_vector_);

        std_msgs::msg::Float64 steering_power_msgs, chassis_power_msgs, power_limit_msgs;
        power_limit_msgs.data    = *power_limit_;
        chassis_power_msgs.data  = *chassis_power_;
        steering_power_msgs.data = *steering_power_;

        steering_power_publisher_->publish(steering_power_msgs);
        chassis_power_publisher_->publish(chassis_power_msgs);
        power_limit_publisher_->publish(power_limit_msgs);
    }

private:
    auto update_control_velocity() -> std::pair<Eigen::Vector2d, double> {
        std::pair<Eigen::Vector2d, double> result{};
        auto& [translational_control_velocity, angular_control_velocity] = result;

        auto control_velocity =
            Eigen::Vector2d{control_velocity_->vector.x(), control_velocity_->vector.y()};

        translational_control_velocity =
            Eigen::Vector2d{control_velocity.x(), control_velocity.y()};

        if (!std::isnan(control_velocity_->vector.z())) {
            angular_control_velocity = control_velocity_->vector.z();
        } else {
            angular_control_velocity = 0.;
        }

        return result;
    }

    void update_control_torque_and_angle(const std::array<Eigen::Vector2d, 4>& vector) {

        Eigen::Vector2d lf_wheel_vector = vector[0];
        Eigen::Vector2d lb_wheel_vector = vector[1];
        Eigen::Vector2d rb_wheel_vector = vector[2];
        Eigen::Vector2d rf_wheel_vector = vector[3];

        Eigen::Vector2d lf_steering_vector = lf_wheel_vector;
        Eigen::Vector2d lb_steering_vector = lb_wheel_vector;
        Eigen::Vector2d rb_steering_vector = rb_wheel_vector;
        Eigen::Vector2d rf_steering_vector = rf_wheel_vector;

        if (lf_wheel_vector.x() == 0 && lf_wheel_vector.y() == 0)
            lf_steering_vector = lf_vector_last_;
        else
            lf_vector_last_ = lf_wheel_vector;
        if (lb_wheel_vector.x() == 0 && lb_wheel_vector.y() == 0)
            lb_steering_vector = lb_vector_last_;
        else
            lb_vector_last_ = lb_wheel_vector;
        if (rb_wheel_vector.x() == 0 && rb_wheel_vector.y() == 0)
            rb_steering_vector = rb_vector_last_;
        else
            rb_vector_last_ = rb_wheel_vector;
        if (rf_wheel_vector.x() == 0 && rf_wheel_vector.y() == 0)
            rf_steering_vector = rf_vector_last_;
        else
            rf_vector_last_ = rf_wheel_vector;

        desire_attitude_[0] = revise_angle_error_and_vel_direction(
            atan2(lf_steering_vector.y(), lf_steering_vector.x()), *left_front_steering_angle_);
        desire_attitude_[1] = revise_angle_error_and_vel_direction(
            atan2(lb_steering_vector.y(), lb_steering_vector.x()), *left_back_steering_angle_);
        desire_attitude_[2] = revise_angle_error_and_vel_direction(
            atan2(rb_steering_vector.y(), rb_steering_vector.x()), *right_back_steering_angle_);
        desire_attitude_[3] = revise_angle_error_and_vel_direction(
            atan2(rf_steering_vector.y(), rf_steering_vector.x()), *right_front_steering_angle_);

        *left_front_steering_control_angle_error_  = std::get<0>(desire_attitude_[0]);
        *left_back_steering_control_angle_error_   = std::get<0>(desire_attitude_[1]);
        *right_back_steering_control_angle_error_  = std::get<0>(desire_attitude_[2]);
        *right_front_steering_control_angle_error_ = std::get<0>(desire_attitude_[3]);

        size_t index = 0;
        for (auto& msg : steering_control_angle_error_msgs_) {
            msg->data = std::get<0>(desire_attitude_[index]);
            steering_control_angle_error_publishers_[index++]->publish(std::move(msg));
            msg = std::make_unique<std_msgs::msg::Float64>();
        }

        *left_front_wheel_control_velocity_ =
            std::get<1>(desire_attitude_[0]) * lf_wheel_vector.norm() / wheel_radius_;
        *left_back_wheel_control_velocity_ =
            std::get<1>(desire_attitude_[1]) * lb_wheel_vector.norm() / wheel_radius_;
        *right_back_wheel_control_velocity_ =
            std::get<1>(desire_attitude_[2]) * rb_wheel_vector.norm() / wheel_radius_;
        *right_front_wheel_control_velocity_ =
            std::get<1>(desire_attitude_[3]) * rf_wheel_vector.norm() / wheel_radius_;
    }

    static std::array<Eigen::Vector2d, 4> update_control_vector(
        const Eigen::Vector2d& translational_control_vel, const double angular_control_vel) {

        return {
            Eigen::Vector2d{-angular_control_vel / sqrt2_,  angular_control_vel / sqrt2_}
                + translational_control_vel,
            Eigen::Vector2d{-angular_control_vel / sqrt2_, -angular_control_vel / sqrt2_}
                + translational_control_vel,
            Eigen::Vector2d{ angular_control_vel / sqrt2_, -angular_control_vel / sqrt2_}
                + translational_control_vel,
            Eigen::Vector2d{ angular_control_vel / sqrt2_,  angular_control_vel / sqrt2_}
                + translational_control_vel
        };
    }

    auto revise_angle_error_and_vel_direction(double target_angle, double measure_angle)
        -> std::tuple<double, double> {
        std::tuple<double, double> result{};
        auto& [error, forward] = result;

        if (measure_angle > pi_) {
            measure_angle -= 2 * pi_;
        }

        auto wheel_reverse = false;

        auto raw_error = target_angle - measure_angle;

        if (raw_error >= std::numbers::pi) {
            target_angle -= 2 * std::numbers::pi;
        } else if (raw_error <= -std::numbers::pi) {
            target_angle += 2 * std::numbers::pi;
        }

        error = target_angle - measure_angle;

        if (error > std::numbers::pi / 2) {
            wheel_reverse = true;
            error -= std::numbers::pi;
        } else if (error < -std::numbers::pi / 2) {
            wheel_reverse = true;
            error += std::numbers::pi;
        } else {
            wheel_reverse = false;
        }

        // error   = target_angle - measure_angle;
        forward = wheel_reverse ? -1 : 1;
        // RCLCPP_INFO(get_logger(), "err:%f,for:%f", error, forward);
        return result;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_point_ =
        std::chrono::system_clock::now();

    static constexpr double inf_   = std::numeric_limits<double>::infinity();
    static constexpr double nan_   = std::numeric_limits<double>::quiet_NaN();
    static constexpr double sqrt2_ = std::numbers::sqrt2;
    static constexpr double pi_    = std::numbers::pi;

    static constexpr double chassis_radius_ = 0.5617 / 2.;
    static constexpr double wheel_radius_   = 0.11;

    static constexpr double font_freq = 0.01, translation_accelerate_limit = 1.5;

    static constexpr double filter_tau_   = 1.0;
    static constexpr double update_freq_  = 0.001;
    static constexpr double filter_alpha_ = update_freq_ / (filter_tau_ + update_freq_);

    static constexpr double steering_angular_velocity_max = 0;

    std::array<Eigen::Vector2d, 4> control_vector_;

    pid::PidCalculator steerings_angle_pid_caculator_;
    pid::PidCalculator wheels_velocity_pid_calculator_;

    InputInterface<double> left_front_steering_angle_;
    InputInterface<double> left_back_steering_angle_;
    InputInterface<double> right_back_steering_angle_;
    InputInterface<double> right_front_steering_angle_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;

    InputInterface<double> wheel_motor_max_control_torque_;
    InputInterface<double> steering_power_;
    InputInterface<double> chassis_power_;
    InputInterface<double> power_limit_;

    OutputInterface<double> left_front_wheel_control_velocity_;
    OutputInterface<double> left_back_wheel_control_velocity_;
    OutputInterface<double> right_back_wheel_control_velocity_;
    OutputInterface<double> right_front_wheel_control_velocity_;

    OutputInterface<double> left_front_steering_control_angle_error_;
    OutputInterface<double> left_back_steering_control_angle_error_;
    OutputInterface<double> right_back_steering_control_angle_error_;
    OutputInterface<double> right_front_steering_control_angle_error_;

    std::array<std::tuple<double, double>, 4> desire_attitude_;

    Eigen::Vector2d lf_vector_last_{1, -1};
    Eigen::Vector2d lb_vector_last_{1, 1};
    Eigen::Vector2d rb_vector_last_{1, -1};
    Eigen::Vector2d rf_vector_last_{1, 1};

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_power_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr chassis_power_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr power_limit_publisher_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4>
        steering_control_angle_error_publishers_;
    std::array<std_msgs::msg::Float64::UniquePtr, 4> steering_control_angle_error_msgs_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)