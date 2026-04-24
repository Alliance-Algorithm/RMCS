#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/csv_writer.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::identification {

namespace {

using Clock = std::chrono::steady_clock;

constexpr auto kCenterInfoInterval = std::chrono::duration<double>(0.5);
constexpr double kRangeTolerance = 1e-9;

template <typename T>
T require_parameter(rclcpp::Node& node, const std::string& name) {
    if (!node.has_parameter(name))
        throw std::runtime_error("Missing required parameter: " + name);
    return node.get_parameter(name).get_value<T>();
}

void load_optional_parameter(rclcpp::Node& node, const std::string& name, double& value) {
    node.get_parameter(name, value);
}

void configure_pid_limits(
    rclcpp::Node& node, const std::string& prefix, pid::PidCalculator& calculator) {
    load_optional_parameter(node, prefix + "_integral_min", calculator.integral_min);
    load_optional_parameter(node, prefix + "_integral_max", calculator.integral_max);
    load_optional_parameter(node, prefix + "_integral_split_min", calculator.integral_split_min);
    load_optional_parameter(node, prefix + "_integral_split_max", calculator.integral_split_max);
    load_optional_parameter(node, prefix + "_output_min", calculator.output_min);
    load_optional_parameter(node, prefix + "_output_max", calculator.output_max);
}

std::string normalize_target(std::string target) {
    while (!target.empty() && target.back() == '/')
        target.pop_back();

    if (target.empty())
        throw std::runtime_error("Parameter 'target' cannot be empty");

    if (target.front() != '/')
        target.insert(target.begin(), '/');

    return target;
}

std::string interface_name(const std::string& target, std::string_view suffix) {
    return target + "/" + std::string{suffix};
}

std::string sanitize_file_component(std::string_view text) {
    std::string sanitized;
    sanitized.reserve(text.size());

    for (const char ch : text) {
        if (std::isalnum(static_cast<unsigned char>(ch)))
            sanitized.push_back(ch);
        else
            sanitized.push_back('_');
    }

    if (sanitized.empty())
        sanitized = "target";

    return sanitized;
}

double wrap_to_pi(double angle) {
    constexpr double kPi = std::numbers::pi_v<double>;
    angle = std::remainder(angle, 2.0 * kPi);
    if (angle <= -kPi)
        angle += 2.0 * kPi;
    return angle;
}

enum class RemoteMode {
    kMeasureRange,
    kCenterHold,
    kTestCommand,
    kIdle,
};

RemoteMode decode_remote_mode(rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
    using rmcs_msgs::Switch;
    if (switch_left == Switch::MIDDLE && switch_right == Switch::DOWN)
        return RemoteMode::kMeasureRange;
    if (switch_left == Switch::MIDDLE && switch_right == Switch::MIDDLE)
        return RemoteMode::kCenterHold;
    if (switch_left == Switch::MIDDLE && switch_right == Switch::UP)
        return RemoteMode::kTestCommand;
    return RemoteMode::kIdle;
}

} // namespace

class StaticTorqueTestController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    StaticTorqueTestController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , target_(normalize_target(require_parameter<std::string>(*this, "target")))
        , interval_angle_(require_parameter<double>(*this, "interval_angle"))
        , wait_time_s_(require_parameter<double>(*this, "wait_time"))
        , wait_duration_(std::chrono::duration<double>(wait_time_s_))
        , border_clip_(require_parameter<double>(*this, "border_clip"))
        , control_torque_name_(interface_name(target_, "control_torque"))
        , measured_torque_name_(interface_name(target_, "torque"))
        , measured_velocity_name_(interface_name(target_, "velocity"))
        , measured_angle_name_(interface_name(target_, "angle")) {
        if (!std::isfinite(interval_angle_) || interval_angle_ <= 0.0)
            throw std::runtime_error("interval_angle must be finite and positive");
        if (!std::isfinite(wait_time_s_) || wait_time_s_ <= 0.0)
            throw std::runtime_error("wait_time must be finite and positive");
        if (!std::isfinite(border_clip_) || border_clip_ < 0.0)
            throw std::runtime_error("border_clip must be finite and non-negative");

        position_pid_.kp = require_parameter<double>(*this, "position_kp");
        position_pid_.ki = require_parameter<double>(*this, "position_ki");
        position_pid_.kd = require_parameter<double>(*this, "position_kd");
        velocity_pid_.kp = require_parameter<double>(*this, "velocity_kp");
        velocity_pid_.ki = require_parameter<double>(*this, "velocity_ki");
        velocity_pid_.kd = require_parameter<double>(*this, "velocity_kd");

        configure_pid_limits(*this, "position", position_pid_);
        configure_pid_limits(*this, "velocity", velocity_pid_);

        register_input("/predefined/update_count", update_count_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input(measured_torque_name_, measured_torque_);
        register_input(measured_velocity_name_, measured_velocity_);
        register_input(measured_angle_name_, measured_angle_);

        register_output(control_torque_name_, control_torque_, nan_);
    }

    ~StaticTorqueTestController() override { stop_test(true); }

    void before_updating() override {
        position_pid_.reset();
        velocity_pid_.reset();
        stop_test(true);

        *control_torque_ = nan_;
        angle_tracking_initialized_ = false;
        range_initialized_ = false;
        remote_mode_initialized_ = false;
        next_center_info_time_ = Clock::time_point{};
    }

    void update() override {
        update_angle_tracking(*measured_angle_);

        const auto remote_mode = decode_remote_mode(*switch_left_, *switch_right_);
        const bool mode_changed = !remote_mode_initialized_ || remote_mode != last_remote_mode_;

        switch (remote_mode) {
        case RemoteMode::kMeasureRange: handle_measure_range(); break;
        case RemoteMode::kCenterHold: handle_center_hold(mode_changed); break;
        case RemoteMode::kTestCommand: handle_test_command(mode_changed); break;
        case RemoteMode::kIdle: handle_idle(); break;
        }

        last_remote_mode_ = remote_mode;
        remote_mode_initialized_ = true;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    void update_angle_tracking(double raw_angle) {
        current_wrapped_angle_ = wrap_to_pi(raw_angle);

        if (!angle_tracking_initialized_) {
            current_continuous_angle_ = current_wrapped_angle_;
            last_wrapped_angle_ = current_wrapped_angle_;
            angle_tracking_initialized_ = true;
            return;
        }

        current_continuous_angle_ += wrap_to_pi(current_wrapped_angle_ - last_wrapped_angle_);
        last_wrapped_angle_ = current_wrapped_angle_;
    }

    void handle_measure_range() {
        stop_test(true);
        position_pid_.reset();
        velocity_pid_.reset();
        update_measured_range();
        *control_torque_ = nan_;
    }

    void update_measured_range() {
        if (!range_initialized_) {
            range_min_ = current_continuous_angle_;
            range_max_ = current_continuous_angle_;
            range_initialized_ = true;
            return;
        }

        range_min_ = std::min(range_min_, current_continuous_angle_);
        range_max_ = std::max(range_max_, current_continuous_angle_);
    }

    void handle_center_hold(bool mode_changed) {
        stop_test(true);

        if (!range_initialized_) {
            *control_torque_ = nan_;
            return;
        }

        if (mode_changed) {
            position_pid_.reset();
            velocity_pid_.reset();
            next_center_info_time_ = *timestamp_;
        }

        active_setpoint_ = 0.5 * (range_min_ + range_max_);
        *control_torque_ = calculate_pid_output(active_setpoint_);

        maybe_log_center_hold_info();
    }

    void maybe_log_center_hold_info() {
        if (*timestamp_ < next_center_info_time_)
            return;

        const double control_error = active_setpoint_ - current_continuous_angle_;
        RCLCPP_INFO(
            get_logger(),
            "Center hold error=%.6f rad, setpoint=%.6f rad, angle=%.6f rad, range=[%.6f, %.6f] rad",
            control_error, wrap_to_pi(active_setpoint_), current_wrapped_angle_,
            wrap_to_pi(range_min_), wrap_to_pi(range_max_));

        do {
            next_center_info_time_ +=
                std::chrono::duration_cast<Clock::duration>(kCenterInfoInterval);
        } while (*timestamp_ >= next_center_info_time_);
    }

    void handle_test_command(bool mode_changed) {
        if (mode_changed) {
            if (last_remote_mode_ == RemoteMode::kCenterHold) {
                if (!start_test()) {
                    *control_torque_ = nan_;
                    return;
                }
            } else if (!test_setpoint_valid_) {
                *control_torque_ = nan_;
                return;
            } else {
                position_pid_.reset();
                velocity_pid_.reset();
            }
        }

        if (!test_setpoint_valid_) {
            *control_torque_ = nan_;
            return;
        }

        *control_torque_ = calculate_pid_output(active_setpoint_);

        if (!test_active_)
            return;

        if (*timestamp_ - dwell_start_time_ < wait_duration_)
            return;

        record_test_point();

        const double next_setpoint = active_setpoint_ + interval_angle_;
        if (next_setpoint > clipped_range_max_ + kRangeTolerance) {
            finish_test_sequence();
            return;
        }

        active_setpoint_ = next_setpoint;
        dwell_start_time_ = *timestamp_;
        position_pid_.reset();
        velocity_pid_.reset();
    }

    bool start_test() {
        stop_test(true);

        if (!range_initialized_) {
            RCLCPP_WARN(get_logger(), "Cannot start static torque test before range is measured.");
            return false;
        }

        clipped_range_min_ = range_min_ + border_clip_;
        clipped_range_max_ = range_max_ - border_clip_;
        if (clipped_range_min_ > clipped_range_max_ + kRangeTolerance) {
            RCLCPP_WARN(
                get_logger(),
                "border_clip=%.6f rad leaves no feasible range, measured span=%.6f rad",
                border_clip_, range_max_ - range_min_);
            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Measured mechanical limits continuous=[%.6f, %.6f] rad, wrapped=[%.6f, %.6f] rad, "
            "clipped=[%.6f, %.6f] rad",
            range_min_, range_max_, wrap_to_pi(range_min_), wrap_to_pi(range_max_),
            wrap_to_pi(clipped_range_min_), wrap_to_pi(clipped_range_max_));

        const auto path = build_csv_path();
        try {
            csv_writer_.open(path);
            csv_writer_.write_row(
                "update_count", "elapsed_s", control_torque_name_, measured_torque_name_,
                measured_velocity_name_, measured_angle_name_);
            csv_writer_.flush();
        } catch (const std::exception& exception) {
            const auto path_string = path.string();
            RCLCPP_ERROR(
                get_logger(), "Failed to start static torque log '%s': %s", path_string.c_str(),
                exception.what());
            return false;
        }

        current_csv_path_ = path;
        test_start_time_ = *timestamp_;
        dwell_start_time_ = *timestamp_;
        active_setpoint_ = clipped_range_min_;
        test_setpoint_valid_ = true;
        test_active_ = true;
        position_pid_.reset();
        velocity_pid_.reset();

        const auto path_string = current_csv_path_.string();
        RCLCPP_INFO(
            get_logger(), "Started static torque test at %.6f rad, log=%s",
            wrap_to_pi(active_setpoint_), path_string.c_str());
        return true;
    }

    void record_test_point() {
        const double elapsed_s =
            std::max(0.0, std::chrono::duration<double>(*timestamp_ - test_start_time_).count());

        csv_writer_.write_row(
            *update_count_, elapsed_s, *control_torque_, *measured_torque_, *measured_velocity_,
            current_wrapped_angle_);
        csv_writer_.flush();
    }

    void finish_test_sequence() {
        test_active_ = false;
        close_csv();

        RCLCPP_INFO(
            get_logger(), "Static torque test finished, final setpoint=%.6f rad",
            wrap_to_pi(active_setpoint_));
    }

    void handle_idle() {
        stop_test(true);
        position_pid_.reset();
        velocity_pid_.reset();
        *control_torque_ = nan_;
    }

    double calculate_pid_output(double setpoint) {
        const double position_error = setpoint - current_continuous_angle_;
        const double velocity_setpoint = position_pid_.update(position_error);
        return velocity_pid_.update(velocity_setpoint - *measured_velocity_);
    }

    void stop_test(bool clear_setpoint_hold) {
        close_csv();
        test_active_ = false;
        if (clear_setpoint_hold)
            test_setpoint_valid_ = false;
    }

    void close_csv() {
        if (!csv_writer_.is_open())
            return;

        csv_writer_.flush();
        csv_writer_.close();
        current_csv_path_.clear();
    }

    std::filesystem::path build_csv_path() const {
        const auto file_name = std::string{"static_torque_test_controller_"}
                             + sanitize_file_component(target_) + "_test_"
                             + std::to_string(*update_count_) + ".csv";
        return std::filesystem::path{"/tmp"} / file_name;
    }

    const std::string target_;
    const double interval_angle_;
    const double wait_time_s_;
    const std::chrono::duration<double> wait_duration_;
    const double border_clip_;

    const std::string control_torque_name_;
    const std::string measured_torque_name_;
    const std::string measured_velocity_name_;
    const std::string measured_angle_name_;

    InputInterface<std::size_t> update_count_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> measured_torque_;
    InputInterface<double> measured_velocity_;
    InputInterface<double> measured_angle_;

    OutputInterface<double> control_torque_;

    pid::PidCalculator position_pid_;
    pid::PidCalculator velocity_pid_;

    bool angle_tracking_initialized_ = false;
    double last_wrapped_angle_ = 0.0;
    double current_wrapped_angle_ = 0.0;
    double current_continuous_angle_ = 0.0;

    bool range_initialized_ = false;
    double range_min_ = 0.0;
    double range_max_ = 0.0;

    bool remote_mode_initialized_ = false;
    RemoteMode last_remote_mode_ = RemoteMode::kIdle;
    Clock::time_point next_center_info_time_{};

    double active_setpoint_ = 0.0;
    bool test_setpoint_valid_ = false;
    bool test_active_ = false;
    double clipped_range_min_ = 0.0;
    double clipped_range_max_ = 0.0;
    Clock::time_point test_start_time_{};
    Clock::time_point dwell_start_time_{};
    rmcs_utility::CsvWriter csv_writer_;
    std::filesystem::path current_csv_path_;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::StaticTorqueTestController, rmcs_executor::Component)
