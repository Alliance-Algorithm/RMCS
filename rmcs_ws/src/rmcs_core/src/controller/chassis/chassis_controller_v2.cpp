#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <numbers>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class ChassisControllerV2
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Clock = std::chrono::steady_clock;

public:
    ChassisControllerV2()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , control_mode_(parse_control_mode_(get_parameter_or<std::string>("control_mode", "fixed")))
        , function_type_(to_lower_(get_parameter_or<std::string>("function_type", "sine")))
        , fixed_target_angle_deg_(
              get_parameter_or("target_angle_deg", get_parameter_or("target_angle", 30.0)))
        , function_offset_deg_(get_parameter_or("function_offset_deg", fixed_target_angle_deg_))
        , function_amplitude_deg_(get_parameter_or("function_amplitude_deg", 5.0))
        , function_frequency_hz_(get_parameter_or("function_frequency_hz", 0.5))
        , function_phase_deg_(get_parameter_or("function_phase_deg", 0.0))
        , print_debug_(get_parameter_or("print_debug", false))
        , start_time_(Clock::now()) {

        register_output("/chassis/left_front_joint/target_angle", left_front_target_angle_, nan_);
        register_output("/chassis/left_back_joint/target_angle", left_back_target_angle_, nan_);
        register_output("/chassis/right_back_joint/target_angle", right_back_target_angle_, nan_);
        register_output("/chassis/right_front_joint/target_angle", right_front_target_angle_, nan_);

        if (control_mode_ == ControlMode::kFunction && function_type_ != "sine") {
            RCLCPP_WARN(
                get_logger(),
                "Unsupported function_type \"%s\", fallback to sine.",
                function_type_.c_str());
            function_type_ = "sine";
        }

        if (!std::isfinite(fixed_target_angle_deg_) || !std::isfinite(function_offset_deg_)
            || !std::isfinite(function_amplitude_deg_) || !std::isfinite(function_frequency_hz_)
            || !std::isfinite(function_phase_deg_)) {
            RCLCPP_WARN(get_logger(), "Found non-finite parameters, using safe defaults.");
            fixed_target_angle_deg_ = 30.0;
            function_offset_deg_ = fixed_target_angle_deg_;
            function_amplitude_deg_ = 5.0;
            function_frequency_hz_ = 0.5;
            function_phase_deg_ = 0.0;
        }

        const bool has_target_angle_deg = has_parameter("target_angle_deg");
        const bool has_target_angle_legacy = has_parameter("target_angle");
        if (has_target_angle_deg && has_target_angle_legacy) {
            RCLCPP_WARN(
                get_logger(),
                "Both \"target_angle_deg\" and legacy \"target_angle\" are set. "
                "Using \"target_angle_deg\".");
        } else if (has_target_angle_legacy) {
            RCLCPP_WARN(
                get_logger(),
                "Parameter \"target_angle\" is legacy. Please use \"target_angle_deg\".");
        }

        warn_angle_range_("target_angle_deg", fixed_target_angle_deg_, 180.0);
        warn_angle_range_("function_offset_deg", function_offset_deg_, 180.0);
        warn_angle_range_("function_amplitude_deg", function_amplitude_deg_, 180.0);
        warn_angle_range_("function_phase_deg", function_phase_deg_, 360.0);

        RCLCPP_INFO(
            get_logger(),
            "ChassisControllerV2 configured: mode=%s, target=%.3f deg (%.6f rad), "
            "function[offset=%.3f deg, amplitude=%.3f deg, freq=%.3f Hz, phase=%.3f deg]",
            control_mode_ == ControlMode::kFixed ? "fixed" : "function", fixed_target_angle_deg_,
            deg_to_rad_(fixed_target_angle_deg_), function_offset_deg_, function_amplitude_deg_,
            function_frequency_hz_, function_phase_deg_);
    }

    void update() override {
        const double elapsed_s =
            std::chrono::duration<double>(Clock::now() - start_time_).count();
        const double target_angle_deg = compute_target_angle_deg(elapsed_s);
        const double target_angle_rad = deg_to_rad_(target_angle_deg);

        *left_front_target_angle_ = target_angle_rad;
        *left_back_target_angle_ = target_angle_rad;
        *right_back_target_angle_ = target_angle_rad;
        *right_front_target_angle_ = target_angle_rad;

        if (print_debug_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "mode=%s target_angle_deg=%.3f target_angle_rad=%.6f elapsed=%.3f",
                control_mode_ == ControlMode::kFixed ? "fixed" : "function", target_angle_deg,
                target_angle_rad, elapsed_s);
        }
    }

private:
    enum class ControlMode {
        kFixed,
        kFunction,
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static std::string to_lower_(std::string value) {
        std::transform(
            value.begin(), value.end(), value.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value;
    }

    ControlMode parse_control_mode_(const std::string& value) {
        const std::string mode = to_lower_(value);
        if (mode == "fixed")
            return ControlMode::kFixed;
        if (mode == "function")
            return ControlMode::kFunction;

        RCLCPP_WARN(
            get_logger(), "Unknown control_mode \"%s\", fallback to fixed.", value.c_str());
        return ControlMode::kFixed;
    }

    static double deg_to_rad_(double degree) {
        return degree * std::numbers::pi / 180.0;
    }

    void warn_angle_range_(
        const char* parameter_name, double value_deg, double abs_limit_deg) const {
        if (std::abs(value_deg) > abs_limit_deg) {
            RCLCPP_WARN(
                get_logger(),
                "Parameter \"%s\" = %.3f deg exceeds recommended range [%.1f, %.1f] deg.",
                parameter_name, value_deg, -abs_limit_deg, abs_limit_deg);
        }
    }

    double compute_target_angle_deg(double elapsed_s) const {
        if (control_mode_ == ControlMode::kFixed)
            return fixed_target_angle_deg_;

        if (function_type_ == "sine")
            return function_offset_deg_
                 + (function_amplitude_deg_
                    * std::sin(
                        2.0 * std::numbers::pi * function_frequency_hz_ * elapsed_s
                        + deg_to_rad_(function_phase_deg_)));

        return fixed_target_angle_deg_;
    }

    OutputInterface<double> left_front_target_angle_;
    OutputInterface<double> left_back_target_angle_;
    OutputInterface<double> right_back_target_angle_;
    OutputInterface<double> right_front_target_angle_;

    ControlMode control_mode_;
    std::string function_type_;

    double fixed_target_angle_deg_;
    double function_offset_deg_;
    double function_amplitude_deg_;
    double function_frequency_hz_;
    double function_phase_deg_;

    bool print_debug_;

    Clock::time_point start_time_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisControllerV2, rmcs_executor::Component)
