#include <algorithm>
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
constexpr auto kSettleDwell = std::chrono::duration<double>(0.2);
constexpr int kUnfoldState = 0;

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

double wrap_to_pi(double angle) {
    constexpr double kPi = std::numbers::pi_v<double>;
    angle = std::remainder(angle, 2.0 * kPi);
    if (angle <= -kPi)
        angle += 2.0 * kPi;
    return angle;
}

double chirp_phase(
    double elapsed_s, double start_freq, double end_freq, double duration_s, bool logarithmic) {
    constexpr double kPi = std::numbers::pi_v<double>;
    if (logarithmic) {
        if (std::abs(end_freq - start_freq) <= std::numeric_limits<double>::epsilon())
            return 2.0 * kPi * start_freq * elapsed_s;

        const double ratio = end_freq / start_freq;
        return 2.0 * kPi * start_freq * duration_s / std::log(ratio)
             * (std::pow(ratio, elapsed_s / duration_s) - 1.0);
    }

    const double sweep_rate = (end_freq - start_freq) / duration_s;
    return 2.0 * kPi * (start_freq * elapsed_s + 0.5 * sweep_rate * elapsed_s * elapsed_s);
}

double chirp_frequency(
    double elapsed_s, double start_freq, double end_freq, double duration_s, bool logarithmic) {
    if (logarithmic) {
        if (std::abs(end_freq - start_freq) <= std::numeric_limits<double>::epsilon())
            return start_freq;

        const double ratio = end_freq / start_freq;
        return start_freq * std::pow(ratio, elapsed_s / duration_s);
    }

    return start_freq + (end_freq - start_freq) * elapsed_s / duration_s;
}

bool controller_enabled(rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
    using rmcs_msgs::Switch;
    if (switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
        return false;
    return !(switch_left == Switch::DOWN && switch_right == Switch::DOWN);
}

} // namespace

class TopYawSweepController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TopYawSweepController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , enable_(parameter_or_declare(*this, "enable", true))
        , angle_min_(require_parameter<double>(*this, "angle_min"))
        , angle_max_(require_parameter<double>(*this, "angle_max"))
        , start_freq_(require_parameter<double>(*this, "start_freq"))
        , end_freq_(require_parameter<double>(*this, "end_freq"))
        , duration_s_(require_parameter<double>(*this, "duration"))
        , logarithmic_(parameter_or_declare(*this, "logarithmic", true))
        , settle_time_s_(parameter_or_declare(*this, "settle_time", 2.0))
        , angle_tolerance_(parameter_or_declare(*this, "angle_tolerance", 0.02))
        , approach_timeout_s_(parameter_or_declare(*this, "approach_timeout", 10.0))
        , abort_margin_(parameter_or_declare(*this, "abort_margin", 0.03))
        , abort_velocity_(parameter_or_declare(*this, "abort_velocity", 0.0))
        , log_directory_(
              parameter_or_declare(*this, "log_directory", std::string{"/workspaces/RMCS"})) {
        validate_parameters();
        center_ = 0.5 * (angle_min_ + angle_max_);
        amplitude_ = 0.5 * (angle_max_ - angle_min_);
        configure_pid();

        register_input("/predefined/update_count", update_count_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);

        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/top_yaw/torque", top_yaw_torque_);
        register_input("/gimbal/top_yaw/control_torque", gimbal_top_yaw_torque_);
        register_input("/gimbal/roll/angle", roll_angle_);
        register_input("/gimbal/roll/velocity", roll_velocity_);
        register_input("/gimbal/roll/torque", roll_torque_);
        register_input("/gimbal/roll/control_torque", roll_control_torque_);
        register_input("/gimbal/fold/state", fold_state_);

        register_output(test_torque_name_, test_torque_, nan_);
    }

    ~TopYawSweepController() override { finish_logging(); }

    void before_updating() override {
        angle_pid_.reset();
        velocity_pid_.reset();
        finish_logging();

        state_ = SweepState::Idle;
        *test_torque_ = nan_;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    }

    void update() override {
        const auto now = *timestamp_;
        const auto current_left = *switch_left_;
        const auto current_right = *switch_right_;
        const bool right_up_edge = last_switch_right_ == rmcs_msgs::Switch::MIDDLE
                                && current_right == rmcs_msgs::Switch::UP;
        last_switch_right_ = current_right;

        const bool fold_unfolded = *fold_state_ == kUnfoldState;
        const bool enabled = controller_enabled(current_left, current_right);

        if (state_ == SweepState::Idle) {
            *test_torque_ = nan_;
            if (enable_ && enabled && fold_unfolded && right_up_edge)
                start_approach(now);
            return;
        }

        if (!enabled || !fold_unfolded || current_right != rmcs_msgs::Switch::UP) {
            stop_sweep("trigger released");
            return;
        }

        switch (state_) {
        case SweepState::Approach: update_approach(now); break;
        case SweepState::Sweep: update_sweep(now); break;
        case SweepState::Hold: update_hold(); break;
        case SweepState::Idle: break;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr const char* test_torque_name_ = "/gimbal/top_yaw/control_torque_test";

    enum class SweepState {
        Idle,
        Approach,
        Sweep,
        Hold,
    };

    void validate_parameters() const {
        if (!std::isfinite(angle_min_) || !std::isfinite(angle_max_))
            throw std::runtime_error("angle_min/angle_max must be finite");
        if (angle_min_ >= angle_max_)
            throw std::runtime_error("angle_min must be less than angle_max");
        if (angle_max_ - angle_min_ > std::numbers::pi_v<double>)
            throw std::runtime_error("sweep range must not exceed pi (minor arc required)");

        if (!std::isfinite(start_freq_) || start_freq_ < 0.0)
            throw std::runtime_error("start_freq must be finite and non-negative");
        if (!std::isfinite(end_freq_) || end_freq_ < 0.0)
            throw std::runtime_error("end_freq must be finite and non-negative");
        if (logarithmic_ && (start_freq_ <= 0.0 || end_freq_ <= 0.0))
            throw std::runtime_error("logarithmic sweep requires positive start_freq and end_freq");
        if (!std::isfinite(duration_s_) || duration_s_ <= 0.0)
            throw std::runtime_error("duration must be finite and positive");

        if (!std::isfinite(settle_time_s_) || settle_time_s_ <= 0.0)
            throw std::runtime_error("settle_time must be finite and positive");
        if (!std::isfinite(angle_tolerance_) || angle_tolerance_ <= 0.0)
            throw std::runtime_error("angle_tolerance must be finite and positive");
        if (!std::isfinite(approach_timeout_s_) || approach_timeout_s_ <= 0.0)
            throw std::runtime_error("approach_timeout must be finite and positive");
        if (!std::isfinite(abort_margin_) || abort_margin_ < 0.0)
            throw std::runtime_error("abort_margin must be finite and non-negative");
        if (!std::isfinite(abort_velocity_) || abort_velocity_ < 0.0)
            throw std::runtime_error("abort_velocity must be finite and non-negative");
    }

    void configure_pid() {
        angle_pid_.kp = require_parameter<double>(*this, "angle_kp");
        angle_pid_.ki = require_parameter<double>(*this, "angle_ki");
        angle_pid_.kd = require_parameter<double>(*this, "angle_kd");
        velocity_pid_.kp = require_parameter<double>(*this, "velocity_kp");
        velocity_pid_.ki = require_parameter<double>(*this, "velocity_ki");
        velocity_pid_.kd = require_parameter<double>(*this, "velocity_kd");

        angle_pid_.output_min = -1.0;
        angle_pid_.output_max = 1.0;
        velocity_pid_.output_min = -0.5;
        velocity_pid_.output_max = 0.5;

        load_optional_parameter(*this, "angle_integral_min", angle_pid_.integral_min);
        load_optional_parameter(*this, "angle_integral_max", angle_pid_.integral_max);
        load_optional_parameter(*this, "angle_integral_split_min", angle_pid_.integral_split_min);
        load_optional_parameter(*this, "angle_integral_split_max", angle_pid_.integral_split_max);
        load_optional_parameter(*this, "angle_output_min", angle_pid_.output_min);
        load_optional_parameter(*this, "angle_output_max", angle_pid_.output_max);

        load_optional_parameter(*this, "velocity_integral_min", velocity_pid_.integral_min);
        load_optional_parameter(*this, "velocity_integral_max", velocity_pid_.integral_max);
        load_optional_parameter(
            *this, "velocity_integral_split_min", velocity_pid_.integral_split_min);
        load_optional_parameter(
            *this, "velocity_integral_split_max", velocity_pid_.integral_split_max);
        load_optional_parameter(*this, "velocity_output_min", velocity_pid_.output_min);
        load_optional_parameter(*this, "velocity_output_max", velocity_pid_.output_max);
    }

    void start_approach(Clock::time_point now) {
        angle_pid_.reset();
        velocity_pid_.reset();

        approach_start_time_ = now;
        approach_start_angle_ = *top_yaw_angle_;
        settle_dwell_start_ = Clock::time_point{};
        active_ref_ = approach_start_angle_;
        state_ = SweepState::Approach;

        RCLCPP_INFO(
            get_logger(), "Top yaw sweep approach started: current=%.4f rad, center=%.4f rad",
            wrap_to_pi(approach_start_angle_), center_);
    }

    void update_approach(Clock::time_point now) {
        const double elapsed_s =
            std::max(0.0, std::chrono::duration<double>(now - approach_start_time_).count());

        double ref = center_;
        if (elapsed_s < settle_time_s_) {
            const double progress = elapsed_s / settle_time_s_;
            const double smooth = progress * progress * (3.0 - 2.0 * progress);
            ref = wrap_to_pi(
                approach_start_angle_ + wrap_to_pi(center_ - approach_start_angle_) * smooth);
        }
        active_ref_ = ref;
        apply_control(ref);

        const bool settled = std::abs(wrap_to_pi(ref - *top_yaw_angle_)) <= angle_tolerance_;
        if (elapsed_s >= settle_time_s_ && settled) {
            if (settle_dwell_start_ == Clock::time_point{})
                settle_dwell_start_ = now;
            if (now - settle_dwell_start_ >= kSettleDwell) {
                start_sweep(now);
                return;
            }
        } else {
            settle_dwell_start_ = Clock::time_point{};
        }

        if (elapsed_s >= approach_timeout_s_) {
            RCLCPP_WARN(
                get_logger(), "Top yaw sweep approach timeout, error=%.4f rad",
                wrap_to_pi(center_ - *top_yaw_angle_));
            enter_hold();
        }
    }

    void start_sweep(Clock::time_point now) {
        angle_pid_.reset();
        velocity_pid_.reset();

        sweep_start_time_ = now;
        next_flush_time_ = now + std::chrono::duration_cast<Clock::duration>(kFlushInterval);
        state_ = SweepState::Sweep;

        open_log();

        RCLCPP_INFO(
            get_logger(), "Top yaw sweep started: %.3f~%.3f rad, %.3f->%.3f Hz, %.1f s%s",
            angle_min_, angle_max_, start_freq_, end_freq_, duration_s_,
            logarithmic_ ? " (log)" : "");
    }

    void update_sweep(Clock::time_point now) {
        const double elapsed_s =
            std::max(0.0, std::chrono::duration<double>(now - sweep_start_time_).count());
        const double clamped_s = std::clamp(elapsed_s, 0.0, duration_s_);

        const double phase =
            chirp_phase(clamped_s, start_freq_, end_freq_, duration_s_, logarithmic_);
        const double ref = center_ + amplitude_ * std::sin(phase);
        active_ref_ = ref;
        apply_control(ref);

        const double offset = std::abs(wrap_to_pi(*top_yaw_angle_ - center_));
        const bool out_of_range = offset > amplitude_ + abort_margin_;
        const bool overspeed =
            abort_velocity_ > 0.0 && std::abs(*top_yaw_velocity_) > abort_velocity_;
        if (out_of_range || overspeed) {
            RCLCPP_WARN(
                get_logger(), "Top yaw sweep abort: offset=%.4f rad, velocity=%.4f rad/s", offset,
                *top_yaw_velocity_);
            finish_logging();
            enter_hold();
            return;
        }

        log_sample(
            elapsed_s,
            chirp_frequency(clamped_s, start_freq_, end_freq_, duration_s_, logarithmic_));

        if (elapsed_s >= duration_s_) {
            RCLCPP_INFO(get_logger(), "Top yaw sweep finished");
            finish_logging();
            enter_hold();
        }
    }

    void update_hold() { apply_control(center_); }

    void enter_hold() { state_ = SweepState::Hold; }

    void stop_sweep(const char* reason) {
        finish_logging();
        angle_pid_.reset();
        velocity_pid_.reset();
        state_ = SweepState::Idle;
        *test_torque_ = nan_;
        RCLCPP_INFO(get_logger(), "Top yaw sweep stopped (%s)", reason);
    }

    void apply_control(double ref) {
        if (!std::isfinite(*top_yaw_angle_) || !std::isfinite(*top_yaw_velocity_)) {
            *test_torque_ = nan_;
            return;
        }

        const double error = wrap_to_pi(ref - *top_yaw_angle_);
        const double velocity_reference = angle_pid_.update(error);
        const double torque = velocity_pid_.update(velocity_reference - *top_yaw_velocity_);
        *test_torque_ = std::isfinite(torque) ? torque : nan_;
    }

    void open_log() {
        const auto path = build_csv_path();
        try {
            csv_writer_.open(path);
            csv_writer_.write_row(
                "update_count", "elapsed_s", "freq_hz", "ref_angle", "top_yaw_angle",
                "top_yaw_velocity", "top_yaw_torque", "test_torque", "gimbal_torque", "roll_angle",
                "roll_velocity", "roll_torque", "roll_control_torque");
            csv_writer_.flush();
        } catch (const std::exception& exception) {
            RCLCPP_ERROR(get_logger(), "Failed to start sweep log: %s", exception.what());
            current_csv_path_.clear();
            return;
        }

        current_csv_path_ = path;
        const auto path_string = current_csv_path_.string();
        RCLCPP_INFO(get_logger(), "Logging top yaw sweep to %s", path_string.c_str());
    }

    void finish_logging() {
        if (!csv_writer_.is_open() && current_csv_path_.empty())
            return;

        csv_writer_.flush();
        csv_writer_.close();

        if (!current_csv_path_.empty()) {
            const auto path_string = current_csv_path_.string();
            RCLCPP_INFO(get_logger(), "Top yaw sweep log saved to %s", path_string.c_str());
        }
        current_csv_path_.clear();
    }

    void log_sample(double elapsed_s, double frequency_hz) {
        if (!csv_writer_.is_open())
            return;

        csv_writer_.write_row(
            *update_count_, elapsed_s, frequency_hz, active_ref_, *top_yaw_angle_,
            *top_yaw_velocity_, *top_yaw_torque_, *test_torque_, *gimbal_top_yaw_torque_,
            *roll_angle_, *roll_velocity_, *roll_torque_, *roll_control_torque_);

        if (*timestamp_ >= next_flush_time_) {
            csv_writer_.flush();
            while (*timestamp_ >= next_flush_time_) {
                next_flush_time_ += std::chrono::duration_cast<Clock::duration>(kFlushInterval);
            }
        }
    }

    std::filesystem::path build_csv_path() const {
        const auto file_name =
            std::string{"top_yaw_sweep_"} + std::to_string(*update_count_) + ".csv";
        return std::filesystem::path{log_directory_} / file_name;
    }

    const bool enable_;
    const double angle_min_;
    const double angle_max_;
    const double start_freq_;
    const double end_freq_;
    const double duration_s_;
    const bool logarithmic_;
    const double settle_time_s_;
    const double angle_tolerance_;
    const double approach_timeout_s_;
    const double abort_margin_;
    const double abort_velocity_;
    const std::string log_directory_;

    double center_ = 0.0;
    double amplitude_ = 0.0;

    pid::PidCalculator angle_pid_;
    pid::PidCalculator velocity_pid_;

    InputInterface<std::size_t> update_count_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> top_yaw_angle_;
    InputInterface<double> top_yaw_velocity_;
    InputInterface<double> top_yaw_torque_;
    InputInterface<double> gimbal_top_yaw_torque_;
    InputInterface<double> roll_angle_;
    InputInterface<double> roll_velocity_;
    InputInterface<double> roll_torque_;
    InputInterface<double> roll_control_torque_;
    InputInterface<int> fold_state_;

    OutputInterface<double> test_torque_;

    SweepState state_ = SweepState::Idle;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    Clock::time_point approach_start_time_{};
    Clock::time_point settle_dwell_start_{};
    Clock::time_point sweep_start_time_{};
    Clock::time_point next_flush_time_{};

    double approach_start_angle_ = 0.0;
    double active_ref_ = 0.0;

    rmcs_utility::CsvWriter csv_writer_;
    std::filesystem::path current_csv_path_;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::TopYawSweepController, rmcs_executor::Component)
