#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

namespace {

using Clock = std::chrono::steady_clock;

std::size_t parse_joint_index(const std::string& joint_name) {
    if (joint_name == "left_front")
        return 0;
    if (joint_name == "left_back")
        return 1;
    if (joint_name == "right_back")
        return 2;
    if (joint_name == "right_front")
        return 3;

    throw std::runtime_error(
        "joint sweep parameter \"swept_joint\" must be one of: left_front, left_back, "
        "right_back, right_front");
}

double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

struct NormalizedAngleLimits {
    double lower_deg;
    double upper_deg;
    bool reordered;
};

NormalizedAngleLimits normalize_angle_limits(double lower_deg, double upper_deg) {
    return {std::min(lower_deg, upper_deg), std::max(lower_deg, upper_deg), lower_deg > upper_deg};
}

std::string format_angle_interval(double lower_deg, double upper_deg) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << "[" << lower_deg << ", " << upper_deg << "]";
    return stream.str();
}

std::string build_limit_value_error(double lower_deg, double upper_deg) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "joint sweep angle limits must be finite: joint_lower_limit_deg=" << lower_deg
           << ", joint_upper_limit_deg=" << upper_deg;
    return stream.str();
}

std::string build_zero_width_limit_error(
    double raw_lower_deg, double raw_upper_deg, const NormalizedAngleLimits& normalized_limits) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "joint sweep angle limits collapse to a zero-width interval: "
           << "joint_lower_limit_deg=" << raw_lower_deg
           << ", joint_upper_limit_deg=" << raw_upper_deg
           << ", effective_limits_deg="
           << format_angle_interval(normalized_limits.lower_deg, normalized_limits.upper_deg);
    return stream.str();
}

std::string build_amplitude_limit_error(
    double raw_lower_deg, double raw_upper_deg, const NormalizedAngleLimits& normalized_limits,
    double angle_amplitude_deg, double test_angle_deg) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3);

    const double sweep_lower_deg = test_angle_deg - angle_amplitude_deg;
    const double sweep_upper_deg = test_angle_deg + angle_amplitude_deg;

    stream << "joint sweep angle amplitude exceeds configured angle limits: "
           << "test_angle_deg=" << test_angle_deg
           << ", angle_amplitude_deg=" << angle_amplitude_deg
           << ", requested_sweep_deg=" << format_angle_interval(sweep_lower_deg, sweep_upper_deg)
           << ", joint_lower_limit_deg=" << raw_lower_deg
           << ", joint_upper_limit_deg=" << raw_upper_deg
           << ", effective_limits_deg="
           << format_angle_interval(normalized_limits.lower_deg, normalized_limits.upper_deg);
    return stream.str();
}

} // namespace

class DeformableJointSweepController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    static constexpr std::size_t kJointCount = 4;

    DeformableJointSweepController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , swept_joint_index_(0)
        , duration_seconds_(0.0)
        , settle_duration_seconds_(0.0)
        , angle_amplitude_rad_(0.0)
        , start_frequency_hz_(0.0)
        , end_frequency_hz_(0.0)
        , lower_angle_limit_rad_(0.0)
        , upper_angle_limit_rad_(0.0) {
        swept_joint_index_ = parse_joint_index(get_parameter("swept_joint").as_string());
        duration_seconds_ = get_parameter("duration").as_double();
        settle_duration_seconds_ = get_parameter_or("settle_duration", 1.0);

        const double angle_amplitude_deg = get_parameter("angle_amplitude_deg").as_double();
        angle_amplitude_rad_ = deg_to_rad(angle_amplitude_deg);

        start_frequency_hz_ = get_parameter("start_frequency_hz").as_double();
        end_frequency_hz_ = get_parameter("end_frequency_hz").as_double();

        const double raw_lower_angle_limit_deg = get_parameter("joint_lower_limit_deg").as_double();
        const double raw_upper_angle_limit_deg = get_parameter("joint_upper_limit_deg").as_double();
        if (!std::isfinite(raw_lower_angle_limit_deg) || !std::isfinite(raw_upper_angle_limit_deg))
            throw std::runtime_error{
                build_limit_value_error(raw_lower_angle_limit_deg, raw_upper_angle_limit_deg)};

        const NormalizedAngleLimits normalized_limits =
            normalize_angle_limits(raw_lower_angle_limit_deg, raw_upper_angle_limit_deg);
        lower_angle_limit_rad_ = deg_to_rad(normalized_limits.lower_deg);
        upper_angle_limit_rad_ = deg_to_rad(normalized_limits.upper_deg);

        if (duration_seconds_ <= 0.0)
            throw std::runtime_error{"joint sweep parameter \"duration\" must be positive"};
        if (settle_duration_seconds_ < 0.0)
            throw std::runtime_error{
                "joint sweep parameter \"settle_duration\" cannot be negative"};
        if (start_frequency_hz_ < 0.0 || end_frequency_hz_ < 0.0)
            throw std::runtime_error{"joint sweep frequency parameters must be non-negative"};
        if (!std::isfinite(angle_amplitude_rad_) || angle_amplitude_rad_ <= 0.0)
            throw std::runtime_error{
                "joint sweep parameter \"angle_amplitude_deg\" must be positive"};
        if (normalized_limits.lower_deg == normalized_limits.upper_deg)
            throw std::runtime_error{build_zero_width_limit_error(
                raw_lower_angle_limit_deg, raw_upper_angle_limit_deg, normalized_limits)};
        if (normalized_limits.reordered) {
            RCLCPP_WARN(
                get_logger(),
                "joint sweep angle limits were provided in reverse order; using normalized "
                "interval [%.3f, %.3f] deg from joint_lower_limit_deg=%.3f and "
                "joint_upper_limit_deg=%.3f",
                normalized_limits.lower_deg, normalized_limits.upper_deg,
                raw_lower_angle_limit_deg, raw_upper_angle_limit_deg);
        }

        if (has_parameter("test_angles_deg")) {
            for (double value : get_parameter("test_angles_deg").as_double_array()) {
                if (!std::isfinite(value))
                    throw std::runtime_error{
                        "joint sweep parameter \"test_angles_deg\" must be finite"};
                if (value - angle_amplitude_deg < normalized_limits.lower_deg
                    || value + angle_amplitude_deg > normalized_limits.upper_deg) {
                    throw std::runtime_error{build_amplitude_limit_error(
                        raw_lower_angle_limit_deg, raw_upper_angle_limit_deg, normalized_limits,
                        angle_amplitude_deg, value)};
                }
                test_angles_rad_.push_back(deg_to_rad(value));
            }
        }
        if (test_angles_rad_.empty())
            throw std::runtime_error{
                "joint sweep parameter \"test_angles_deg\" must contain at least one angle"};

        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output(
            "/chassis/left_front_joint/target_physical_angle", left_front_target_angle_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_angle", left_back_target_angle_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_angle", right_back_target_angle_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_angle", right_front_target_angle_, nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_velocity", left_front_target_velocity_,
            nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_velocity", left_back_target_velocity_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_velocity", right_back_target_velocity_,
            nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_velocity", right_front_target_velocity_,
            nan_);

        register_output("/chassis/joint_sweep/target_angle", active_target_angle_, nan_);
        register_output("/chassis/joint_sweep/excitation_offset", excitation_offset_, nan_);
        register_output("/chassis/joint_sweep/excitation_velocity", excitation_velocity_, nan_);
        register_output("/chassis/joint_sweep/frequency_hz", frequency_hz_, nan_);
        register_output("/chassis/joint_sweep/phase_rad", phase_rad_, nan_);
        register_output(
            "/chassis/joint_sweep/working_point_angle_deg", working_point_angle_deg_, nan_);
        register_output("/chassis/joint_sweep/segment_index", segment_index_, std::size_t{0});

        chassis_control_velocity_->vector.setZero();
    }

    void before_updating() override { reset_outputs_(); }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_left = *switch_left_;
        const auto switch_right = *switch_right_;
        const bool enabled = (switch_left != Switch::UNKNOWN && switch_right != Switch::UNKNOWN)
                          && !(switch_left == Switch::DOWN && switch_right == Switch::DOWN);
        if (!enabled) {
            stop_session_();
            reset_outputs_();
            return;
        }

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector.setZero();

        if (!session_active_ && !session_completed_)
            start_session_();

        if (session_completed_) {
            publish_targets_for_working_point_(last_working_point_index_, 0.0, 0.0, nan_, nan_);
            return;
        }

        const double elapsed_seconds =
            std::chrono::duration<double>(*timestamp_ - segment_start_time_).count();
        if (elapsed_seconds >= settle_duration_seconds_ + duration_seconds_) {
            advance_segment_();
            if (session_completed_) {
                publish_targets_for_working_point_(last_working_point_index_, 0.0, 0.0, nan_, nan_);
                return;
            }
        }

        const std::size_t current_index = segment_index_value_;
        const double settle_elapsed =
            std::chrono::duration<double>(*timestamp_ - segment_start_time_).count();
        if (settle_elapsed < settle_duration_seconds_) {
            publish_targets_for_working_point_(current_index, 0.0, 0.0, 0.0, nan_);
            return;
        }

        const double sweep_elapsed = settle_elapsed - settle_duration_seconds_;
        const double sweep_rate_hz_per_s =
            (end_frequency_hz_ - start_frequency_hz_) / duration_seconds_;
        const double current_frequency = start_frequency_hz_ + sweep_rate_hz_per_s * sweep_elapsed;
        const double phase = 2.0 * std::numbers::pi
                           * (start_frequency_hz_ * sweep_elapsed
                              + 0.5 * sweep_rate_hz_per_s * sweep_elapsed * sweep_elapsed);
        const double offset = angle_amplitude_rad_ * std::sin(phase);
        const double velocity =
            angle_amplitude_rad_ * std::cos(phase) * 2.0 * std::numbers::pi * current_frequency;

        publish_targets_for_working_point_(
            current_index, offset, velocity, current_frequency, phase);
    }

private:
    void start_session_() {
        session_active_ = true;
        session_completed_ = false;
        segment_index_value_ = 0;
        last_working_point_index_ = 0;
        segment_start_time_ = *timestamp_;
        *segment_index_ = segment_index_value_;
    }

    void stop_session_() {
        session_active_ = false;
        session_completed_ = false;
        segment_index_value_ = 0;
        last_working_point_index_ = 0;
    }

    void advance_segment_() {
        last_working_point_index_ = segment_index_value_;
        ++segment_index_value_;
        if (segment_index_value_ >= test_angles_rad_.size()) {
            session_active_ = false;
            session_completed_ = true;
            return;
        }

        segment_start_time_ = *timestamp_;
        *segment_index_ = segment_index_value_;
    }

    void publish_targets_for_working_point_(
        std::size_t working_point_index, double offset, double velocity, double frequency,
        double phase) {
        const double base_angle = test_angles_rad_[working_point_index];
        const double active_angle = base_angle + offset;

        const std::array<OutputInterface<double>*, kJointCount> angle_outputs{
            &left_front_target_angle_, &left_back_target_angle_, &right_back_target_angle_,
            &right_front_target_angle_};
        const std::array<OutputInterface<double>*, kJointCount> velocity_outputs{
            &left_front_target_velocity_, &left_back_target_velocity_, &right_back_target_velocity_,
            &right_front_target_velocity_};

        for (std::size_t i = 0; i < kJointCount; ++i) {
            *(*angle_outputs[i]) = (i == swept_joint_index_) ? active_angle : base_angle;
            *(*velocity_outputs[i]) = (i == swept_joint_index_) ? velocity : 0.0;
        }

        *active_target_angle_ = active_angle;
        *excitation_offset_ = offset;
        *excitation_velocity_ = velocity;
        *frequency_hz_ = frequency;
        *phase_rad_ = phase;
        *working_point_angle_deg_ =
            test_angles_rad_[working_point_index] * 180.0 / std::numbers::pi;
        *segment_index_ = working_point_index;
        last_working_point_index_ = working_point_index;
    }

    void reset_outputs_() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        *left_front_target_angle_ = nan_;
        *left_back_target_angle_ = nan_;
        *right_back_target_angle_ = nan_;
        *right_front_target_angle_ = nan_;
        *left_front_target_velocity_ = nan_;
        *left_back_target_velocity_ = nan_;
        *right_back_target_velocity_ = nan_;
        *right_front_target_velocity_ = nan_;

        *active_target_angle_ = nan_;
        *excitation_offset_ = nan_;
        *excitation_velocity_ = nan_;
        *frequency_hz_ = nan_;
        *phase_rad_ = nan_;
        *working_point_angle_deg_ = nan_;
        *segment_index_ = 0;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Clock::time_point> timestamp_;
    InputInterface<std::size_t> update_count_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    OutputInterface<double> left_front_target_angle_;
    OutputInterface<double> left_back_target_angle_;
    OutputInterface<double> right_back_target_angle_;
    OutputInterface<double> right_front_target_angle_;
    OutputInterface<double> left_front_target_velocity_;
    OutputInterface<double> left_back_target_velocity_;
    OutputInterface<double> right_back_target_velocity_;
    OutputInterface<double> right_front_target_velocity_;

    OutputInterface<double> active_target_angle_;
    OutputInterface<double> excitation_offset_;
    OutputInterface<double> excitation_velocity_;
    OutputInterface<double> frequency_hz_;
    OutputInterface<double> phase_rad_;
    OutputInterface<double> working_point_angle_deg_;
    OutputInterface<std::size_t> segment_index_;

    std::size_t swept_joint_index_;
    double duration_seconds_;
    double settle_duration_seconds_;
    double angle_amplitude_rad_;
    double start_frequency_hz_;
    double end_frequency_hz_;
    double lower_angle_limit_rad_;
    double upper_angle_limit_rad_;
    std::vector<double> test_angles_rad_;

    bool session_active_ = false;
    bool session_completed_ = false;
    std::size_t segment_index_value_ = 0;
    std::size_t last_working_point_index_ = 0;
    Clock::time_point segment_start_time_{};
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableJointSweepController, rmcs_executor::Component)
