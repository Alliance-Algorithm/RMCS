#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

namespace {

using Clock = std::chrono::steady_clock;

} // namespace

class DeformableJointSweepRecorder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableJointSweepRecorder()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_name_(get_parameter("joint_name").as_string())
        , output_directory_(get_parameter("output_directory").as_string())
        , csv_decimation_(get_parameter("csv_decimation").as_int()) {
        if (joint_name_.empty())
            throw std::runtime_error{
                "joint sweep recorder parameter \"joint_name\" cannot be empty"};
        if (output_directory_.empty()) {
            throw std::runtime_error{
                "joint sweep recorder parameter \"output_directory\" cannot be empty"};
        }
        if (csv_decimation_ <= 0) {
            throw std::runtime_error{
                "joint sweep recorder parameter \"csv_decimation\" must be positive"};
        }

        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);
        register_input(get_parameter("target_angle").as_string(), target_angle_);
        register_input(get_parameter("excitation_offset").as_string(), excitation_offset_);
        register_input(get_parameter("excitation_velocity").as_string(), excitation_velocity_);
        register_input(get_parameter("frequency_hz").as_string(), frequency_hz_);
        register_input(get_parameter("phase_rad").as_string(), phase_rad_);
        register_input(
            get_parameter("working_point_angle_deg").as_string(), working_point_angle_deg_);
        register_input(get_parameter("segment_index").as_string(), segment_index_);
        register_input(get_parameter("measurement_angle").as_string(), measurement_angle_);
        register_input(get_parameter("measurement_velocity").as_string(), measurement_velocity_);
        register_input(get_parameter("measured_torque").as_string(), measured_torque_);
        register_input(get_parameter("control_torque").as_string(), control_torque_);
        if (has_parameter("eso_z3")) {
            register_input(get_parameter("eso_z3").as_string(), eso_z3_);
            has_eso_z3_ = true;
        }
        if (has_parameter("eso_z2")) {
            register_input(get_parameter("eso_z2").as_string(), eso_z2_);
            has_eso_z2_ = true;
        }
    }

    ~DeformableJointSweepRecorder() override { close_csv_(); }

    void update() override {
        if (!std::isfinite(*target_angle_) || !std::isfinite(*working_point_angle_deg_)
            || !std::isfinite(*frequency_hz_)) {
            close_csv_();
            session_open_ = false;
            return;
        }

        if (!session_open_) {
            run_token_ = *update_count_;
            session_open_ = true;
            last_segment_index_ = std::numeric_limits<std::size_t>::max();
        }

        if (*segment_index_ != last_segment_index_) {
            open_csv_(*segment_index_, *working_point_angle_deg_);
            segment_start_time_ = *timestamp_;
            last_segment_index_ = *segment_index_;
        }

        if (!csv_stream_.is_open())
            return;
        if (*update_count_ % static_cast<std::size_t>(csv_decimation_) != 0)
            return;

        const double elapsed_seconds =
            std::chrono::duration<double>(*timestamp_ - segment_start_time_).count();
        csv_stream_ << *update_count_ << ',' << elapsed_seconds << ',' << *segment_index_ << ','
                    << joint_name_ << ',' << *working_point_angle_deg_ << ',' << *target_angle_
                    << ',' << *excitation_offset_ << ',' << *excitation_velocity_ << ','
                    << *measurement_angle_ << ',' << *measurement_velocity_ << ','
                    << *control_torque_ << ',' << *measured_torque_ << ',' << *frequency_hz_ << ','
                    << *phase_rad_ << ','
                    << (has_eso_z3_ && std::isfinite(*eso_z3_) ? *eso_z3_ : nan_) << ','
                    << (has_eso_z2_ && std::isfinite(*eso_z2_) ? *eso_z2_ : nan_) << '\n';

        if (*update_count_ % 100 == 0)
            csv_stream_.flush();
    }

private:
    void open_csv_(std::size_t segment_index, double working_point_angle_deg) {
        close_csv_();

        const std::filesystem::path base_path{output_directory_};
        std::filesystem::create_directories(base_path);

        const std::filesystem::path path =
            base_path
            / (joint_name_ + "_run" + std::to_string(run_token_) + "_segment"
               + std::to_string(segment_index) + "_angle_"
               + std::to_string(static_cast<int>(std::round(working_point_angle_deg))) + ".csv");

        csv_stream_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_stream_.is_open()) {
            throw std::runtime_error{"failed to open joint sweep csv path: " + path.string()};
        }

        csv_stream_ << std::fixed << std::setprecision(9);
        csv_stream_ << "update_count,elapsed_s,segment_index,joint_name,working_point_angle_deg,"
                       "target_angle,excitation_offset,excitation_velocity,actual_angle,"
                       "actual_velocity,control_torque,measured_torque,frequency_hz,phase_rad,"
                       "eso_z3,eso_z2\n";
    }

    void close_csv_() {
        if (!csv_stream_.is_open())
            return;

        csv_stream_.flush();
        csv_stream_.close();
    }

    InputInterface<Clock::time_point> timestamp_;
    InputInterface<std::size_t> update_count_;
    InputInterface<double> target_angle_;
    InputInterface<double> excitation_offset_;
    InputInterface<double> excitation_velocity_;
    InputInterface<double> frequency_hz_;
    InputInterface<double> phase_rad_;
    InputInterface<double> working_point_angle_deg_;
    InputInterface<std::size_t> segment_index_;
    InputInterface<double> measurement_angle_;
    InputInterface<double> measurement_velocity_;
    InputInterface<double> measured_torque_;
    InputInterface<double> control_torque_;
    InputInterface<double> eso_z3_;
    InputInterface<double> eso_z2_;

    std::string joint_name_;
    std::string output_directory_;
    int64_t csv_decimation_;

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    bool session_open_ = false;
    bool has_eso_z3_ = false;
    bool has_eso_z2_ = false;
    std::size_t run_token_ = 0;
    std::size_t last_segment_index_ = std::numeric_limits<std::size_t>::max();
    Clock::time_point segment_start_time_{};
    std::ofstream csv_stream_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableJointSweepRecorder, rmcs_executor::Component)
