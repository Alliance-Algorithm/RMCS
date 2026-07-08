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

constexpr auto kFlushInterval = std::chrono::duration<double>(0.1);

template <typename T>
T require_parameter(rclcpp::Node& node, const std::string& name) {
    if (!node.has_parameter(name))
        throw std::runtime_error("Missing required parameter: " + name);
    return node.get_parameter(name).get_value<T>();
}

template <typename T>
T parameter_or_declare(rclcpp::Node& node, const std::string& name, const T& default_value) {
    if (!node.has_parameter(name))
        node.declare_parameter<T>(name, default_value);
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

bool controller_enabled(rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
    using rmcs_msgs::Switch;
    if (switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
        return false;
    return !(switch_left == Switch::DOWN && switch_right == Switch::DOWN);
}

double chirp_output(
    double elapsed_s, double start_freq, double end_freq, double duration_s, double amplitude,
    bool logarithmic) {
    if (logarithmic) {
        if (std::abs(end_freq - start_freq) <= std::numeric_limits<double>::epsilon()) {
            return amplitude * std::sin(2.0 * std::numbers::pi_v<double> * start_freq * elapsed_s);
        }

        const double ratio = end_freq / start_freq;
        const double phase = 2.0 * std::numbers::pi_v<double> * start_freq * duration_s
                           / std::log(ratio) * (std::pow(ratio, elapsed_s / duration_s) - 1.0);
        return amplitude * std::sin(phase);
    }

    const double sweep_rate = (end_freq - start_freq) / duration_s;
    const double phase = 2.0 * std::numbers::pi_v<double>
                       * (start_freq * elapsed_s + 0.5 * sweep_rate * elapsed_s * elapsed_s);
    return amplitude * std::sin(phase);
}

} // namespace

class SweptFrequencyController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SweptFrequencyController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , target_(normalize_target(require_parameter<std::string>(*this, "target")))
        , sweep_enabled_(parameter_or_declare(*this, "sweep", false))
        , pid_enabled_(parameter_or_declare(*this, "pid", false))
        , logarithmic_(parameter_or_declare(*this, "logarithmic", false))
        , dc_offset_(parameter_or_declare(*this, "dc_offset", 0.0))
        , control_torque_name_(interface_name(target_, "control_torque"))
        , measured_torque_name_(interface_name(target_, "torque"))
        , measured_velocity_name_(interface_name(target_, "velocity"))
        , measured_angle_name_(interface_name(target_, "angle")) {
        if (sweep_enabled_) {
            start_freq_ = require_parameter<double>(*this, "start_freq");
            end_freq_ = require_parameter<double>(*this, "end_freq");
            sweep_duration_s_ = require_parameter<double>(*this, "duration");
            amplitude_ = require_parameter<double>(*this, "amplitude");

            if (!std::isfinite(start_freq_) || start_freq_ < 0.0)
                throw std::runtime_error("start_freq must be finite and non-negative");
            if (!std::isfinite(end_freq_) || end_freq_ < 0.0)
                throw std::runtime_error("end_freq must be finite and non-negative");
            if (!std::isfinite(sweep_duration_s_) || sweep_duration_s_ <= 0.0)
                throw std::runtime_error("duration must be finite and positive");
            if (!std::isfinite(amplitude_))
                throw std::runtime_error("amplitude must be finite");
            if (logarithmic_ && (start_freq_ <= 0.0 || end_freq_ <= 0.0))
                throw std::runtime_error(
                    "logarithmic sweep requires positive start_freq and end_freq");
        }

        if (pid_enabled_) {
            setpoint_ = require_parameter<double>(*this, "setpoint");
            position_pid_.kp = require_parameter<double>(*this, "position_kp");
            position_pid_.ki = require_parameter<double>(*this, "position_ki");
            position_pid_.kd = require_parameter<double>(*this, "position_kd");
            velocity_pid_.kp = require_parameter<double>(*this, "velocity_kp");
            velocity_pid_.ki = require_parameter<double>(*this, "velocity_ki");
            velocity_pid_.kd = require_parameter<double>(*this, "velocity_kd");

            if (!std::isfinite(setpoint_))
                throw std::runtime_error("setpoint must be finite");

            configure_pid_limits(*this, "position", position_pid_);
            configure_pid_limits(*this, "velocity", velocity_pid_);
        }

        register_input("/predefined/update_count", update_count_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input(measured_torque_name_, measured_torque_);
        register_input(measured_velocity_name_, measured_velocity_);
        register_input(measured_angle_name_, measured_angle_);

        register_output(control_torque_name_, control_torque_, nan_);
    }

    ~SweptFrequencyController() override { finish_sweep(); }

    void before_updating() override {
        reset_pid_state();
        finish_sweep();

        *control_torque_ = nan_;
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    }

    void update() override {
        const auto current_switch_left = *switch_left_;
        const auto current_switch_right = *switch_right_;
        const bool is_enabled = controller_enabled(current_switch_left, current_switch_right);

        if (!is_enabled) {
            reset_pid_state();
            finish_sweep();
            *control_torque_ = nan_;
            store_switch_state(current_switch_left, current_switch_right);
            return;
        }

        if (should_start_sweep(current_switch_left, current_switch_right))
            start_sweep();

        const double pid_output = pid_enabled_ ? calculate_pid_output() : 0.0;
        const double dc_offset_output = dc_offset_;

        bool sweep_finished = false;
        const double sweep_output = sweep_active_ ? calculate_sweep_output(sweep_finished) : 0.0;

        const double control_output = pid_output + dc_offset_output + sweep_output;
        *control_torque_ = control_output;

        if (sweep_active_) {
            log_sample(pid_output, sweep_output, control_output);
            if (sweep_finished)
                finish_sweep();
        }

        store_switch_state(current_switch_left, current_switch_right);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    bool should_start_sweep(
        rmcs_msgs::Switch current_switch_left, rmcs_msgs::Switch current_switch_right) const {
        using rmcs_msgs::Switch;
        return sweep_enabled_ && !sweep_active_ && last_switch_left_ == Switch::MIDDLE
            && last_switch_right_ == Switch::MIDDLE && current_switch_left == Switch::MIDDLE
            && current_switch_right == Switch::UP;
    }

    void store_switch_state(
        rmcs_msgs::Switch current_switch_left, rmcs_msgs::Switch current_switch_right) {
        last_switch_left_ = current_switch_left;
        last_switch_right_ = current_switch_right;
    }

    void reset_pid_state() {
        position_pid_.reset();
        velocity_pid_.reset();
    }

    double calculate_pid_output() {
        const double position_error = wrap_to_pi(setpoint_ - *measured_angle_);
        const double velocity_setpoint = position_pid_.update(position_error);
        return velocity_pid_.update(velocity_setpoint - *measured_velocity_);
    }

    void start_sweep() {
        finish_sweep();

        const auto path = build_csv_path();
        try {
            csv_writer_.open(path);
            csv_writer_.write_row(
                "update_count", "elapsed_s", "pid_output", "sweep_output", control_torque_name_,
                measured_torque_name_, measured_velocity_name_, measured_angle_name_);
        } catch (const std::exception& exception) {
            const auto path_string = path.string();
            RCLCPP_ERROR(
                get_logger(), "Failed to start sweep log '%s': %s", path_string.c_str(),
                exception.what());
            return;
        }

        sweep_active_ = true;
        sweep_start_time_ = *timestamp_;
        next_flush_time_ =
            sweep_start_time_ + std::chrono::duration_cast<Clock::duration>(kFlushInterval);
        current_csv_path_ = path;

        const auto path_string = current_csv_path_.string();
        RCLCPP_INFO(get_logger(), "Started sweep logging to %s", path_string.c_str());
    }

    void finish_sweep() {
        if (!sweep_active_ && !csv_writer_.is_open())
            return;

        csv_writer_.flush();
        csv_writer_.close();

        if (!current_csv_path_.empty()) {
            const auto path_string = current_csv_path_.string();
            RCLCPP_INFO(get_logger(), "Finished sweep logging to %s", path_string.c_str());
        }

        current_csv_path_.clear();
        sweep_active_ = false;
    }

    double calculate_sweep_output(bool& sweep_finished) const {
        const double elapsed_s =
            std::max(0.0, std::chrono::duration<double>(*timestamp_ - sweep_start_time_).count());
        const double clamped_elapsed_s = std::clamp(elapsed_s, 0.0, sweep_duration_s_);
        sweep_finished = elapsed_s >= sweep_duration_s_;
        return chirp_output(
            clamped_elapsed_s, start_freq_, end_freq_, sweep_duration_s_, amplitude_, logarithmic_);
    }

    void log_sample(double pid_output, double sweep_output, double control_output) {
        const double elapsed_s =
            std::max(0.0, std::chrono::duration<double>(*timestamp_ - sweep_start_time_).count());

        csv_writer_.write_row(
            *update_count_, elapsed_s, pid_output, sweep_output, control_output, *measured_torque_,
            *measured_velocity_, *measured_angle_);

        if (*timestamp_ >= next_flush_time_) {
            csv_writer_.flush();
            while (*timestamp_ >= next_flush_time_) {
                next_flush_time_ += std::chrono::duration_cast<Clock::duration>(kFlushInterval);
            }
        }
    }

    std::filesystem::path build_csv_path() const {
        const auto file_name = std::string{"swept_frequency_controller_"}
                             + sanitize_file_component(target_) + "_sweep_"
                             + std::to_string(*update_count_) + ".csv";
        return std::filesystem::path{"/tmp"} / file_name;
    }

    const std::string target_;
    const bool sweep_enabled_;
    const bool pid_enabled_;
    const bool logarithmic_;
    const double dc_offset_;

    const std::string control_torque_name_;
    const std::string measured_torque_name_;
    const std::string measured_velocity_name_;
    const std::string measured_angle_name_;

    double start_freq_ = 0.0;
    double end_freq_ = 0.0;
    double sweep_duration_s_ = 0.0;
    double amplitude_ = 0.0;

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
    double setpoint_ = 0.0;

    bool sweep_active_ = false;
    Clock::time_point sweep_start_time_{};
    Clock::time_point next_flush_time_{};
    rmcs_utility::CsvWriter csv_writer_;
    std::filesystem::path current_csv_path_;

    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::SweptFrequencyController, rmcs_executor::Component)
