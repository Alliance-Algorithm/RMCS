#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::smc {
class Smc_Observer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Smc_Observer()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        // gear_ratio_         = get_parameter("gear_ratio").as_double();
        velocity_limit_     = get_parameter("velocity_limit").as_double();
        acceleration_limit_ = get_parameter("acceleration_limit").as_double();

        register_input(get_parameter("control_error").as_string(), yaw_angle_error_);
        register_input(get_parameter("velocity").as_string(), yaw_velocity_);

        register_output(get_parameter("yaw_expect_velocity_").as_string(), yaw_expect_velocity_);
        register_output(get_parameter("yaw_expect_acceleration_").as_string(), yaw_expect_acceleration_);
    }

    void update() override {
        // update_gimbal_status();
        update_expect_value();
    }

private:
    // void update_gimbal_status() { *yaw_velocity_ = *yaw_motor_velocity_ * gear_ratio_; }

    void update_expect_value() {
        yaw_angle_error_window_.push_back(-*yaw_angle_error_);
        if (yaw_angle_error_window_.size() > max_window_size_) {
            yaw_angle_error_window_.pop_front();
        }

        double expect_velocity_error = least_square_calculator(yaw_angle_error_window_) / acquisition_interval;

        *yaw_expect_velocity_ = std::clamp(*yaw_velocity_ - expect_velocity_error, -velocity_limit_, velocity_limit_);

        yaw_expect_velocity_window_.push_back(*yaw_expect_velocity_);
        if (yaw_expect_velocity_window_.size() > max_window_size_) {
            yaw_expect_velocity_window_.pop_front();
        }
        double expect_acceleration = least_square_calculator(yaw_expect_velocity_window_) / acquisition_interval;

        *yaw_expect_acceleration_ = std::clamp(expect_acceleration, -acceleration_limit_, acceleration_limit_);
    }

    static double least_square_calculator(const std::deque<double>& value_window) {
        if (value_window.size() < 2) {
            return 0.0;
        }
        size_t n             = value_window.size();
        double sum_x         = 0.0;
        double sum_y         = 0.0;
        double sum_k_squared = 0.0;
        double sum_xy        = 0.0;

        for (size_t i = 0; i < n; ++i) {
            double x = static_cast<double>(i);
            double y = value_window[i];
            sum_x += x;
            sum_y += y;
            sum_k_squared += x * x;
            sum_xy += x * y;
        }

        double denominator = static_cast<double>(n) * sum_k_squared - sum_x * sum_x;
        if (std::abs(denominator) < 1e-6) {
            return 0.0;
        }

        double slope = (static_cast<double>(n) * sum_xy - sum_x * sum_y) / denominator;

        if (std::isnan(slope)) {
            slope = 0;
        }

        return slope;
    }

    rclcpp::Logger logger_;

    // double gear_ratio_;
    const size_t max_window_size_     = 10;
    const double acquisition_interval = 0.001; // s

    double velocity_limit_;
    double acceleration_limit_;

    InputInterface<double> yaw_angle_error_;
    InputInterface<double> yaw_velocity_;

    std::deque<double> yaw_angle_error_window_;
    std::deque<double> yaw_expect_velocity_window_;

    // OutputInterface<double> yaw_velocity_;
    OutputInterface<double> yaw_expect_velocity_;
    OutputInterface<double> yaw_expect_acceleration_;
};
} // namespace rmcs_core::controller::smc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::smc::Smc_Observer, rmcs_executor::Component)