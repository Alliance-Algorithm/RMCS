#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/target_snapshot.hpp>

namespace rmcs_core::controller::gimbal {

namespace {

template <typename T>
void declare_param(rclcpp::Node& node, const std::string& name, const T& value) {
    if (!node.has_parameter(name))
        node.declare_parameter<T>(name, value);
}

double wrap_pi(double angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi);
}

std::pair<double, double> yaw_pitch_from_direction(const Eigen::Vector3d& dir) {
    double xy = std::hypot(dir.x(), dir.y());
    return {std::atan2(dir.y(), dir.x()), std::atan2(-dir.z(), xy)};
}

} // namespace

class AutoAimResponseRecorder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    using Clock = std::chrono::steady_clock;

    AutoAimResponseRecorder()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        declare_param(*this, "output_csv_path", std::string{"/tmp/flight_auto_aim.csv"});
        declare_param(*this, "csv_decimation", int64_t{10});

        register_input("/tf", tf_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu_, false);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu_, false);

        register_input("/gimbal/auto_aim/target_snapshot", target_snapshot_, false);
        register_input("/gimbal/auto_aim/control_direction", control_direction_, false);
        register_input("/gimbal/auto_aim/fire_control", fire_control_, false);
        register_input("/gimbal/auto_aim/laser_distance", laser_distance_, false);
        register_input("/gimbal/auto_aim/plan_yaw", plan_yaw_, false);
        register_input("/gimbal/auto_aim/plan_pitch", plan_pitch_, false);
        register_input("/gimbal/auto_aim/plan_yaw_velocity", plan_yaw_velocity_, false);
        register_input("/gimbal/auto_aim/plan_pitch_velocity", plan_pitch_velocity_, false);

        register_input("/gimbal/yaw/control_angle_error", yaw_control_angle_error_, false);
        register_input("/gimbal/pitch/control_angle_error", pitch_control_angle_error_, false);
        register_input("/gimbal/yaw/feedforward", yaw_ff_, false);
        register_input("/gimbal/pitch/feedforward", pitch_ff_, false);
    }

    ~AutoAimResponseRecorder() override { close_csv(); }

    void before_updating() override {
        if (!yaw_velocity_imu_.ready())
            yaw_velocity_imu_.make_and_bind_directly(0.0);
        if (!pitch_velocity_imu_.ready())
            pitch_velocity_imu_.make_and_bind_directly(0.0);
        if (!target_snapshot_.ready())
            target_snapshot_.make_and_bind_directly(rmcs_msgs::TargetSnapshot{});
        if (!control_direction_.ready())
            control_direction_.make_and_bind_directly(Eigen::Vector3d::Zero());
        if (!fire_control_.ready())
            fire_control_.bind_directly(false);
        if (!laser_distance_.ready())
            laser_distance_.bind_directly(0.0);
        if (!plan_yaw_.ready())
            plan_yaw_.make_and_bind_directly(kNan);
        if (!plan_pitch_.ready())
            plan_pitch_.make_and_bind_directly(kNan);
        if (!plan_yaw_velocity_.ready())
            plan_yaw_velocity_.make_and_bind_directly(kNan);
        if (!plan_pitch_velocity_.ready())
            plan_pitch_velocity_.make_and_bind_directly(kNan);
        if (!yaw_control_angle_error_.ready())
            yaw_control_angle_error_.make_and_bind_directly(kNan);
        if (!pitch_control_angle_error_.ready())
            pitch_control_angle_error_.make_and_bind_directly(kNan);
        if (!yaw_ff_.ready())
            yaw_ff_.make_and_bind_directly(0.0);
        if (!pitch_ff_.ready())
            pitch_ff_.make_and_bind_directly(0.0);

        csv_path_ = get_parameter("output_csv_path").as_string();
        csv_decimation_ = get_parameter("csv_decimation").as_int();

        start_time_initialized_ = false;
        log_count_ = 0;
        open_csv();
    }

    void update() override {
        if (!start_time_initialized_) {
            start_time_ = *timestamp_;
            start_time_initialized_ = true;
        }

        if (!is_auto_aim_active())
            return;
        if (*update_count_ % static_cast<std::size_t>(csv_decimation_) != 0)
            return;

        const double elapsed =
            std::max(0.0, std::chrono::duration<double>(*timestamp_ - start_time_).count());
        const auto snapshot = *target_snapshot_;
        const auto [actual_yaw, actual_pitch] = current_world_yaw_pitch();
        const bool planner_active = control_direction_->squaredNorm() > 1e-18;
        const double target_age =
            std::chrono::duration<double>(*timestamp_ - snapshot.timestamp).count();

        const double yaw_error = planner_active ? wrap_pi(*plan_yaw_ - actual_yaw) : kNan;
        const double pitch_error = planner_active ? (*plan_pitch_ - actual_pitch) : kNan;

        csv_ << *update_count_ << ',' << elapsed << ','
             << (snapshot.valid ? 1 : 0) << ',' << (snapshot.converged ? 1 : 0) << ','
             << static_cast<int>(snapshot.armor_count) << ','
             << static_cast<int>(snapshot.armor_type) << ','
             << static_cast<int>(snapshot.armor_name) << ',' << target_age << ','
             << snapshot.state[0] << ',' << snapshot.state[1] << ','
             << snapshot.state[2] << ',' << snapshot.state[3] << ','
             << snapshot.state[4] << ',' << snapshot.state[5] << ','
             << (planner_active ? 1 : 0) << ','
             << *plan_yaw_ << ',' << *plan_pitch_ << ','
             << *plan_yaw_velocity_ << ',' << *plan_pitch_velocity_ << ','
             << actual_yaw << ',' << actual_pitch << ','
             << *yaw_velocity_imu_ << ',' << *pitch_velocity_imu_ << ','
             << yaw_error << ',' << pitch_error << ','
             << *yaw_control_angle_error_ << ',' << *pitch_control_angle_error_ << ','
             << *yaw_ff_ << ',' << *pitch_ff_ << ','
             << (*fire_control_ ? 1 : 0) << ',' << *laser_distance_ << '\n';

        if (++log_count_ % 100 == 0)
            csv_.flush();
    }

private:
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    bool is_auto_aim_active() const {
        using rmcs_msgs::Switch;
        auto sl = *switch_left_, sr = *switch_right_;
        if (sl == Switch::UNKNOWN || sr == Switch::UNKNOWN)
            return false;
        if (sl == Switch::DOWN && sr == Switch::DOWN)
            return false;
        return sr == Switch::UP;
    }

    std::pair<double, double> current_world_yaw_pitch() const {
        auto dir = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        Eigen::Vector3d v = *dir;
        if (v.norm() > 1e-9)
            v.normalize();
        else
            v = Eigen::Vector3d::UnitX();
        return yaw_pitch_from_direction(v);
    }

    void open_csv() {
        close_csv();
        const std::filesystem::path path{csv_path_};
        if (path.has_parent_path())
            std::filesystem::create_directories(path.parent_path());

        csv_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_.is_open())
            throw std::runtime_error{"failed to open: " + csv_path_};

        csv_ << std::fixed << std::setprecision(6);
        csv_ << "tick,elapsed_s,"
                "target_valid,target_converged,armor_count,armor_type,armor_name,target_age_s,"
                "state_x,state_vx,state_y,state_vy,state_z,state_vz,"
                "planner_active,"
                "plan_yaw,plan_pitch,plan_yaw_vel,plan_pitch_vel,"
                "actual_yaw,actual_pitch,imu_yaw_vel,imu_pitch_vel,"
                "yaw_error,pitch_error,"
                "yaw_ctrl_err,pitch_ctrl_err,yaw_ff,pitch_ff,"
                "fire_control,laser_dist\n";
    }

    void close_csv() {
        if (!csv_.is_open())
            return;
        csv_.flush();
        csv_.close();
    }

    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<std::size_t> update_count_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<double> yaw_velocity_imu_;
    InputInterface<double> pitch_velocity_imu_;

    InputInterface<rmcs_msgs::TargetSnapshot> target_snapshot_;
    InputInterface<Eigen::Vector3d> control_direction_;
    InputInterface<bool> fire_control_;
    InputInterface<double> laser_distance_;
    InputInterface<double> plan_yaw_;
    InputInterface<double> plan_pitch_;
    InputInterface<double> plan_yaw_velocity_;
    InputInterface<double> plan_pitch_velocity_;

    InputInterface<double> yaw_control_angle_error_;
    InputInterface<double> pitch_control_angle_error_;
    InputInterface<double> yaw_ff_;
    InputInterface<double> pitch_ff_;

    Clock::time_point start_time_{};
    bool start_time_initialized_ = false;
    std::string csv_path_;
    int64_t csv_decimation_ = 10;
    std::size_t log_count_ = 0;
    std::ofstream csv_;
};

} // namespace rmcs_core::controller::gimbal

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::AutoAimResponseRecorder, rmcs_executor::Component)
