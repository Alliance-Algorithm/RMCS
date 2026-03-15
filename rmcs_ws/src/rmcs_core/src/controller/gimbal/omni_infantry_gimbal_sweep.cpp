#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

namespace {

using Clock = std::chrono::steady_clock;

std::pair<double, double> yaw_pitch_from_direction(const Eigen::Vector3d& direction) {
    const double xy_norm = std::hypot(direction.x(), direction.y());
    return {std::atan2(direction.y(), direction.x()), std::atan2(-direction.z(), xy_norm)};
}

struct AxisCommand {
    double torque = 0.0;
    double frequency = 0.0;
};

struct AxisSweepConfig {
    double bias = 0.0;
    double amplitude = 0.0;
    double start_frequency = 0.0;
    double end_frequency = 0.0;
};

AxisCommand sample_axis_command(
    double elapsed_seconds, double duration_seconds, const AxisSweepConfig& config) {
    AxisCommand command{.torque = config.bias};
    if (std::abs(config.amplitude) <= 1e-9)
        return command;

    const double sweep_rate = (config.end_frequency - config.start_frequency) / duration_seconds;
    command.frequency = config.start_frequency + sweep_rate * elapsed_seconds;

    const double phase = 2.0 * std::numbers::pi
                       * (config.start_frequency * elapsed_seconds
                          + 0.5 * sweep_rate * elapsed_seconds * elapsed_seconds);
    command.torque += config.amplitude * std::sin(phase);
    return command;
}

} // namespace

class OmniInfantryGimbalSweep
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OmniInfantryGimbalSweep()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , duration_seconds_(get_parameter("duration").as_double())
        , yaw_torque_bias_(get_parameter("yaw_torque_bias").as_double())
        , pitch_torque_bias_(get_parameter("pitch_torque_bias").as_double())
        , yaw_torque_amplitude_(get_parameter("yaw_torque_amplitude").as_double())
        , pitch_torque_amplitude_(get_parameter("pitch_torque_amplitude").as_double())
        , yaw_start_frequency_(get_parameter("yaw_start_frequency").as_double())
        , yaw_end_frequency_(get_parameter("yaw_end_frequency").as_double())
        , pitch_start_frequency_(get_parameter("pitch_start_frequency").as_double())
        , pitch_end_frequency_(get_parameter("pitch_end_frequency").as_double())
        , csv_path_(get_parameter("output_csv_path").as_string())
        , csv_decimation_(get_parameter("csv_decimation").as_int()) {
        if (duration_seconds_ <= 0.0)
            throw std::runtime_error{"gimbal sweep parameter \"duration\" must be positive"};
        if (yaw_start_frequency_ < 0.0 || yaw_end_frequency_ < 0.0 || pitch_start_frequency_ < 0.0
            || pitch_end_frequency_ < 0.0) {
            throw std::runtime_error{
                "gimbal sweep frequency parameters must be non-negative"};
        }
        if (csv_path_.empty())
            throw std::runtime_error{"gimbal sweep parameter \"output_csv_path\" cannot be empty"};
        if (csv_decimation_ <= 0)
            throw std::runtime_error{"gimbal sweep parameter \"csv_decimation\" must be positive"};

        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/tf", tf_);
        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu_);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu_);

        register_output("/gimbal/yaw/control_torque", yaw_control_torque_, nan_);
        register_output("/gimbal/pitch/control_torque", pitch_control_torque_, nan_);
        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, 0.0);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, 0.0);
    }

    ~OmniInfantryGimbalSweep() override { stop_session(); }

    void before_updating() override { reset_all_controls(); }

    void update() override {
        using namespace rmcs_msgs;

        const auto switch_left = *switch_left_;
        const auto switch_right = *switch_right_;
        const bool enabled = (switch_left != Switch::UNKNOWN && switch_right != Switch::UNKNOWN)
                          && !(switch_left == Switch::DOWN && switch_right == Switch::DOWN);
        if (!enabled) {
            stop_session();
            reset_all_controls();
            return;
        }

        if (!session_active_)
            start_session();

        const auto [cycle_index, elapsed_seconds] = current_elapsed();
        const AxisCommand yaw_command = sample_axis_command(
            elapsed_seconds, duration_seconds_,
            AxisSweepConfig{
                .bias = yaw_torque_bias_,
                .amplitude = yaw_torque_amplitude_,
                .start_frequency = yaw_start_frequency_,
                .end_frequency = yaw_end_frequency_,
            });
        const AxisCommand pitch_command = sample_axis_command(
            elapsed_seconds, duration_seconds_,
            AxisSweepConfig{
                .bias = pitch_torque_bias_,
                .amplitude = pitch_torque_amplitude_,
                .start_frequency = pitch_start_frequency_,
                .end_frequency = pitch_end_frequency_,
            });

        *yaw_control_torque_ = yaw_command.torque;
        *pitch_control_torque_ = pitch_command.torque;
        *yaw_angle_error_ = 0.0;
        *pitch_angle_error_ = 0.0;

        maybe_log_csv(cycle_index, elapsed_seconds, yaw_command, pitch_command);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    void start_session() {
        session_start_time_ = *timestamp_;
        open_csv();
        session_active_ = true;
        RCLCPP_INFO(get_logger(), "Started gimbal torque sweep, csv output: %s", csv_path_.c_str());
    }

    void stop_session() {
        if (!session_active_)
            return;

        session_active_ = false;
        close_csv();
        RCLCPP_INFO(get_logger(), "Stopped gimbal torque sweep");
    }

    std::pair<std::size_t, double> current_elapsed() const {
        double total_elapsed =
            std::chrono::duration<double>(*timestamp_ - session_start_time_).count();
        if (total_elapsed < 0.0)
            total_elapsed = 0.0;

        const std::size_t cycle_index =
            static_cast<std::size_t>(std::floor(total_elapsed / duration_seconds_));
        return {cycle_index, total_elapsed - cycle_index * duration_seconds_};
    }

    std::pair<double, double> current_world_yaw_pitch() const {
        auto direction =
            fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        Eigen::Vector3d vector = *direction;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return yaw_pitch_from_direction(vector);
    }

    void reset_all_controls() {
        *yaw_control_torque_ = nan_;
        *pitch_control_torque_ = nan_;
        *yaw_angle_error_ = 0.0;
        *pitch_angle_error_ = 0.0;
    }

    void open_csv() {
        close_csv();

        const std::filesystem::path path{csv_path_};
        if (path.has_parent_path())
            std::filesystem::create_directories(path.parent_path());

        csv_stream_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_stream_.is_open())
            throw std::runtime_error{"failed to open gimbal sweep csv path: " + csv_path_};

        csv_stream_ << std::fixed << std::setprecision(9);
        csv_stream_ << "update_count,elapsed_s,cycle_index,cmd_yaw_torque,cmd_pitch_torque,"
                       "actual_world_yaw,actual_world_pitch,actual_yaw_velocity_imu,"
                       "actual_pitch_velocity_imu,yaw_control_angle_error,"
                       "pitch_control_angle_error,yaw_frequency_hz,pitch_frequency_hz\n";
    }

    void close_csv() {
        if (!csv_stream_.is_open())
            return;

        csv_stream_.flush();
        csv_stream_.close();
    }

    void maybe_log_csv(
        std::size_t cycle_index, double elapsed_seconds, const AxisCommand& yaw_command,
        const AxisCommand& pitch_command) {
        if (!csv_stream_.is_open())
            return;
        if (*update_count_ % static_cast<std::size_t>(csv_decimation_) != 0)
            return;

        const auto [actual_world_yaw, actual_world_pitch] = current_world_yaw_pitch();
        csv_stream_ << *update_count_ << ',' << elapsed_seconds << ',' << cycle_index << ','
                    << yaw_command.torque << ',' << pitch_command.torque << ','
                    << actual_world_yaw << ',' << actual_world_pitch << ',' << *yaw_velocity_imu_
                    << ',' << *pitch_velocity_imu_ << ',' << *yaw_angle_error_ << ','
                    << *pitch_angle_error_ << ',' << yaw_command.frequency << ','
                    << pitch_command.frequency << '\n';

        if (*update_count_ % 100 == 0)
            csv_stream_.flush();
    }

    double duration_seconds_;
    double yaw_torque_bias_;
    double pitch_torque_bias_;
    double yaw_torque_amplitude_;
    double pitch_torque_amplitude_;
    double yaw_start_frequency_;
    double yaw_end_frequency_;
    double pitch_start_frequency_;
    double pitch_end_frequency_;
    std::string csv_path_;
    int64_t csv_decimation_;

    bool session_active_ = false;
    Clock::time_point session_start_time_{};
    std::ofstream csv_stream_;

    InputInterface<Clock::time_point> timestamp_;
    InputInterface<std::size_t> update_count_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Tf> tf_;
    InputInterface<double> yaw_velocity_imu_;
    InputInterface<double> pitch_velocity_imu_;

    OutputInterface<double> yaw_control_torque_;
    OutputInterface<double> pitch_control_torque_;
    OutputInterface<double> yaw_angle_error_;
    OutputInterface<double> pitch_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::OmniInfantryGimbalSweep, rmcs_executor::Component)
