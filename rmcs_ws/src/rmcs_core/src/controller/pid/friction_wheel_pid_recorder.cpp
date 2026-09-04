#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::pid {

class FrictionWheelPidRecorder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FrictionWheelPidRecorder()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        const auto wheels =
            declare_parameter<std::vector<std::string>>("wheels", std::vector<std::string>{});
        if (wheels.empty())
            throw std::runtime_error("Parameter \"wheels\" must not be empty");

        min_setpoint_abs_ = declare_parameter("min_setpoint_abs", 10.0);

        const auto flush_every_n_samples = declare_parameter<int64_t>("flush_every_n_samples", 100);
        if (flush_every_n_samples <= 0)
            throw std::runtime_error("Parameter \"flush_every_n_samples\" must be positive");
        flush_every_n_samples_ = static_cast<size_t>(flush_every_n_samples);

        const auto output_dir = std::filesystem::path{
            declare_parameter<std::string>("output_dir", "/tmp/friction_pid_logs")};
        const auto output_name = declare_parameter<std::string>("output_name", "");

        wheel_count_ = wheels.size();
        wheel_ids_ = std::make_unique<std::string[]>(wheel_count_);
        setpoints_ = std::make_unique<InputInterface<double>[]>(wheel_count_);
        measurements_ = std::make_unique<InputInterface<double>[]>(wheel_count_);
        control_torques_ = std::make_unique<InputInterface<double>[]>(wheel_count_);
        run_ids_ = std::make_unique<uint64_t[]>(wheel_count_);
        sample_indices_ = std::make_unique<uint64_t[]>(wheel_count_);
        previously_enabled_ = std::make_unique<bool[]>(wheel_count_);

        for (size_t i = 0; i < wheel_count_; ++i) {
            wheel_ids_[i] = wheel_id_from_topic(wheels[i]);
            register_input(wheels[i] + "/control_velocity", setpoints_[i]);
            register_input(wheels[i] + "/velocity", measurements_[i]);
            register_input(wheels[i] + "/control_torque", control_torques_[i]);

            run_ids_[i] = 0;
            sample_indices_[i] = 0;
            previously_enabled_[i] = false;
        }

        std::filesystem::create_directories(output_dir);
        log_file_path_ = output_dir / resolve_output_name(output_name);
        log_stream_.open(log_file_path_);
        if (!log_stream_.is_open())
            throw std::runtime_error(
                "Failed to open recorder output file: " + log_file_path_.string());

        log_stream_.setf(std::ios::fixed);
        log_stream_.precision(9);
        log_stream_
            << "timestamp_us,wheel_id,run_id,sample_idx,setpoint_velocity,measured_velocity,"
               "control_torque,enabled\n";

        RCLCPP_INFO(get_logger(), "FrictionWheelPidRecorder writing to %s", log_file_path_.c_str());
    }

    ~FrictionWheelPidRecorder() override {
        if (log_stream_.is_open()) {
            log_stream_.flush();
            log_stream_.close();
        }
    }

    void update() override {
        const auto timestamp_us = now_timestamp_us();

        for (size_t i = 0; i < wheel_count_; ++i) {
            const bool enabled = should_record(i);
            if (enabled && !previously_enabled_[i]) {
                ++run_ids_[i];
                sample_indices_[i] = 0;
            }

            if (enabled) {
                log_stream_ << timestamp_us << ',' << wheel_ids_[i] << ',' << run_ids_[i] << ','
                            << sample_indices_[i]++ << ',' << *setpoints_[i] << ','
                            << *measurements_[i] << ',' << *control_torques_[i] << ",1\n";
                if (++unflushed_samples_ >= flush_every_n_samples_) {
                    log_stream_.flush();
                    unflushed_samples_ = 0;
                }
            }

            previously_enabled_[i] = enabled;
        }
    }

private:
    static std::string wheel_id_from_topic(std::string_view topic) {
        auto pos = topic.find_last_of('/');
        if (pos == std::string_view::npos || pos + 1 >= topic.size())
            return std::string(topic);
        return std::string(topic.substr(pos + 1));
    }

    static uint64_t now_timestamp_us() {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    }

    static std::string resolve_output_name(const std::string& configured_name) {
        if (!configured_name.empty())
            return configured_name;

        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        return "friction_wheel_pid_" + std::to_string(ms) + ".csv";
    }

    bool should_record(size_t index) const {
        if (!setpoints_[index].ready() || !measurements_[index].ready()
            || !control_torques_[index].ready())
            return false;

        const auto setpoint = *setpoints_[index];
        const auto measurement = *measurements_[index];
        const auto control_torque = *control_torques_[index];

        return std::isfinite(setpoint) && std::isfinite(measurement)
            && std::isfinite(control_torque) && std::abs(setpoint) >= min_setpoint_abs_;
    }

    size_t wheel_count_ = 0;
    double min_setpoint_abs_ = 10.0;
    size_t flush_every_n_samples_ = 100;
    size_t unflushed_samples_ = 0;

    std::filesystem::path log_file_path_;
    std::ofstream log_stream_;

    std::unique_ptr<std::string[]> wheel_ids_;
    std::unique_ptr<InputInterface<double>[]> setpoints_;
    std::unique_ptr<InputInterface<double>[]> measurements_;
    std::unique_ptr<InputInterface<double>[]> control_torques_;
    std::unique_ptr<uint64_t[]> run_ids_;
    std::unique_ptr<uint64_t[]> sample_indices_;
    std::unique_ptr<bool[]> previously_enabled_;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::pid::FrictionWheelPidRecorder, rmcs_executor::Component)
