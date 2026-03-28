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
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/target_snapshot.hpp>

namespace rmcs_core::controller::gimbal {

namespace {

template <typename T>
void declare_parameter_if_missing(rclcpp::Node& node, const std::string& name, const T& value) {
    if (!node.has_parameter(name))
        node.declare_parameter<T>(name, value);
}

double limit_rad(double angle) {
    constexpr double kPi = std::numbers::pi_v<double>;
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle <= -kPi)
        angle += 2.0 * kPi;
    return angle;
}

std::pair<double, double> yaw_pitch_from_direction(const Eigen::Vector3d& direction) {
    const double xy_norm = std::hypot(direction.x(), direction.y());
    return {std::atan2(direction.y(), direction.x()), std::atan2(-direction.z(), xy_norm)};
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
        declare_parameter_if_missing(
            *this, "output_csv_path", std::string{"/tmp/auto_aim_response.csv"});
        declare_parameter_if_missing(*this, "csv_decimation", int64_t{1});
        declare_parameter_if_missing(*this, "require_auto_aim_request", true);

        register_input("/tf", tf_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_, false);
        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu_, false);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu_, false);
        register_input("/referee/shooter/initial_speed", bullet_speed_, false);

        register_input("/gimbal/auto_aim/target_snapshot", target_snapshot_, false);
        register_input("/gimbal/auto_aim/control_direction", control_direction_, false);
        register_input("/gimbal/auto_aim/fire_control", fire_control_, false);
        register_input("/gimbal/auto_aim/laser_distance", laser_distance_, false);
        register_input("/gimbal/auto_aim/plan_target_yaw", plan_target_yaw_, false);
        register_input("/gimbal/auto_aim/plan_target_pitch", plan_target_pitch_, false);
        register_input("/gimbal/auto_aim/plan_yaw", plan_yaw_, false);
        register_input("/gimbal/auto_aim/plan_pitch", plan_pitch_, false);
        register_input("/gimbal/auto_aim/plan_yaw_velocity", plan_yaw_velocity_, false);
        register_input("/gimbal/auto_aim/plan_pitch_velocity", plan_pitch_velocity_, false);
        register_input("/gimbal/auto_aim/plan_yaw_acceleration", plan_yaw_acceleration_, false);
        register_input(
            "/gimbal/auto_aim/plan_pitch_acceleration", plan_pitch_acceleration_, false);
        register_input("/gimbal/auto_aim/yaw_velocity_ff", yaw_velocity_ff_, false);
        register_input("/gimbal/auto_aim/pitch_velocity_ff", pitch_velocity_ff_, false);
        register_input("/gimbal/auto_aim/yaw_acceleration_ff", yaw_acceleration_ff_, false);
        register_input("/gimbal/auto_aim/pitch_acceleration_ff", pitch_acceleration_ff_, false);
        register_input("/gimbal/auto_aim/yaw_velocity_ref", yaw_velocity_ref_, false);
        register_input("/gimbal/auto_aim/pitch_velocity_ref", pitch_velocity_ref_, false);
        register_input("/gimbal/auto_aim/yaw_velocity_feedback", yaw_velocity_feedback_, false);
        register_input("/gimbal/auto_aim/pitch_velocity_feedback", pitch_velocity_feedback_, false);
        register_input("/gimbal/auto_aim/chassis_yaw_velocity_imu", chassis_yaw_velocity_imu_, false);
        register_input("/gimbal/auto_aim/yaw_chassis_spin_torque_ff", yaw_chassis_spin_torque_ff_, false);

        register_input("/gimbal/yaw/control_angle_error", yaw_control_angle_error_, false);
        register_input("/gimbal/pitch/control_angle_error", pitch_control_angle_error_, false);
        register_input("/gimbal/yaw/control_torque", yaw_control_torque_, false);
        register_input("/gimbal/pitch/control_torque", pitch_control_torque_, false);
    }

    ~AutoAimResponseRecorder() override { close_csv(); }

    void before_updating() override {
        if (!mouse_.ready())
            mouse_.make_and_bind_directly(rmcs_msgs::Mouse{});
        if (!yaw_velocity_imu_.ready())
            yaw_velocity_imu_.make_and_bind_directly(0.0);
        if (!pitch_velocity_imu_.ready())
            pitch_velocity_imu_.make_and_bind_directly(0.0);
        if (!bullet_speed_.ready())
            bullet_speed_.bind_directly(std::numeric_limits<float>::quiet_NaN());
        if (!target_snapshot_.ready())
            target_snapshot_.make_and_bind_directly(rmcs_msgs::TargetSnapshot{});
        if (!control_direction_.ready())
            control_direction_.make_and_bind_directly(Eigen::Vector3d::Zero());
        if (!fire_control_.ready())
            fire_control_.bind_directly(false);
        if (!laser_distance_.ready())
            laser_distance_.bind_directly(0.0);
        if (!plan_target_yaw_.ready())
            plan_target_yaw_.make_and_bind_directly(nan_);
        if (!plan_target_pitch_.ready())
            plan_target_pitch_.make_and_bind_directly(nan_);
        if (!plan_yaw_.ready())
            plan_yaw_.make_and_bind_directly(nan_);
        if (!plan_pitch_.ready())
            plan_pitch_.make_and_bind_directly(nan_);
        if (!plan_yaw_velocity_.ready())
            plan_yaw_velocity_.make_and_bind_directly(nan_);
        if (!plan_pitch_velocity_.ready())
            plan_pitch_velocity_.make_and_bind_directly(nan_);
        if (!plan_yaw_acceleration_.ready())
            plan_yaw_acceleration_.make_and_bind_directly(nan_);
        if (!plan_pitch_acceleration_.ready())
            plan_pitch_acceleration_.make_and_bind_directly(nan_);
        if (!yaw_velocity_ff_.ready())
            yaw_velocity_ff_.make_and_bind_directly(nan_);
        if (!pitch_velocity_ff_.ready())
            pitch_velocity_ff_.make_and_bind_directly(nan_);
        if (!yaw_acceleration_ff_.ready())
            yaw_acceleration_ff_.make_and_bind_directly(nan_);
        if (!pitch_acceleration_ff_.ready())
            pitch_acceleration_ff_.make_and_bind_directly(nan_);
        if (!yaw_velocity_ref_.ready())
            yaw_velocity_ref_.make_and_bind_directly(nan_);
        if (!pitch_velocity_ref_.ready())
            pitch_velocity_ref_.make_and_bind_directly(nan_);
        if (!yaw_velocity_feedback_.ready())
            yaw_velocity_feedback_.make_and_bind_directly(nan_);
        if (!pitch_velocity_feedback_.ready())
            pitch_velocity_feedback_.make_and_bind_directly(nan_);
        if (!chassis_yaw_velocity_imu_.ready())
            chassis_yaw_velocity_imu_.make_and_bind_directly(nan_);
        if (!yaw_chassis_spin_torque_ff_.ready())
            yaw_chassis_spin_torque_ff_.make_and_bind_directly(nan_);
        if (!yaw_control_angle_error_.ready())
            yaw_control_angle_error_.make_and_bind_directly(nan_);
        if (!pitch_control_angle_error_.ready())
            pitch_control_angle_error_.make_and_bind_directly(nan_);
        if (!yaw_control_torque_.ready())
            yaw_control_torque_.make_and_bind_directly(nan_);
        if (!pitch_control_torque_.ready())
            pitch_control_torque_.make_and_bind_directly(nan_);

        csv_path_ = get_parameter("output_csv_path").as_string();
        csv_decimation_ = get_parameter("csv_decimation").as_int();
        require_auto_aim_request_ = get_parameter("require_auto_aim_request").as_bool();
        if (csv_path_.empty())
            throw std::runtime_error{"output_csv_path cannot be empty"};
        if (csv_decimation_ <= 0)
            throw std::runtime_error{"csv_decimation must be positive"};

        start_time_initialized_ = false;
        log_count_ = 0;
        open_csv();
    }

    void update() override {
        if (!start_time_initialized_) {
            start_time_ = *timestamp_;
            start_time_initialized_ = true;
        }

        if (!should_log_this_cycle())
            return;
        if (*update_count_ % static_cast<std::size_t>(csv_decimation_) != 0)
            return;

        const double elapsed_seconds =
            std::max(0.0, std::chrono::duration<double>(*timestamp_ - start_time_).count());
        const auto snapshot = *target_snapshot_;
        const auto [actual_yaw, actual_pitch] = current_world_yaw_pitch();
        const bool planner_active = control_direction_->squaredNorm() > 1e-18;
        const double target_age =
            std::chrono::duration<double>(*timestamp_ - snapshot.timestamp).count();

        const double yaw_error =
            planner_active ? limit_rad(*plan_yaw_ - actual_yaw) : nan_;
        const double pitch_error =
            planner_active ? (*plan_pitch_ - actual_pitch) : nan_;

        csv_stream_ << *update_count_ << ',' << elapsed_seconds << ','
                    << static_cast<double>(*bullet_speed_) << ','
                    << (snapshot.valid ? 1 : 0) << ',' << (snapshot.converged ? 1 : 0) << ','
                    << static_cast<int>(snapshot.armor_count) << ','
                    << static_cast<int>(snapshot.armor_type) << ','
                    << static_cast<int>(snapshot.armor_name) << ',' << target_age << ','
                    << snapshot.state[0] << ',' << snapshot.state[1] << ',' << snapshot.state[2]
                    << ',' << snapshot.state[3] << ',' << snapshot.state[4] << ','
                    << snapshot.state[5] << ',' << snapshot.state[6] << ','
                    << snapshot.state[7] << ',' << snapshot.state[8] << ','
                    << snapshot.state[9] << ',' << snapshot.state[10] << ','
                    << (planner_active ? 1 : 0) << ',' << *plan_target_yaw_ << ','
                    << *plan_target_pitch_ << ',' << *plan_yaw_ << ',' << *plan_pitch_ << ','
                    << *plan_yaw_velocity_ << ',' << *plan_pitch_velocity_ << ','
                    << *plan_yaw_acceleration_ << ',' << *plan_pitch_acceleration_ << ','
                    << *yaw_velocity_ff_ << ',' << *pitch_velocity_ff_ << ','
                    << *yaw_acceleration_ff_ << ',' << *pitch_acceleration_ff_ << ','
                    << *yaw_velocity_ref_ << ',' << *pitch_velocity_ref_ << ','
                    << *yaw_velocity_feedback_ << ',' << *pitch_velocity_feedback_ << ','
                    << *chassis_yaw_velocity_imu_ << ',' << *yaw_chassis_spin_torque_ff_ << ','
                    << actual_yaw << ',' << actual_pitch << ',' << *yaw_velocity_imu_ << ','
                    << *pitch_velocity_imu_ << ',' << yaw_error << ',' << pitch_error << ','
                    << (*fire_control_ ? 1 : 0) << ',' << *laser_distance_ << ','
                    << *yaw_control_angle_error_ << ',' << *pitch_control_angle_error_ << ','
                    << *yaw_control_torque_ << ',' << *pitch_control_torque_ << '\n';

        if (++log_count_ % 100 == 0)
            csv_stream_.flush();
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    bool should_log_this_cycle() const {
        if (!require_auto_aim_request_)
            return true;

        using rmcs_msgs::Switch;
        if (*switch_left_ == Switch::UNKNOWN || *switch_right_ == Switch::UNKNOWN)
            return false;
        if (*switch_left_ == Switch::DOWN)
            return false;
        return mouse_->right || *switch_right_ == Switch::UP;
    }

    std::pair<double, double> current_world_yaw_pitch() const {
        auto direction = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        Eigen::Vector3d vector = *direction;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return yaw_pitch_from_direction(vector);
    }

    void open_csv() {
        close_csv();

        const std::filesystem::path path{csv_path_};
        if (path.has_parent_path())
            std::filesystem::create_directories(path.parent_path());

        csv_stream_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_stream_.is_open())
            throw std::runtime_error{"failed to open csv path: " + csv_path_};

        csv_stream_ << std::fixed << std::setprecision(9);
        csv_stream_ << "update_count,elapsed_s,bullet_speed,"
                       "target_valid,target_converged,target_armor_count,target_armor_type_id,"
                       "target_armor_name_id,target_age_s,"
                       "target_center_x,target_velocity_x,target_center_y,target_velocity_y,"
                       "target_center_z,target_velocity_z,target_angle,target_angular_velocity,"
                       "target_base_radius,target_radius_delta,target_height_delta,"
                       "planner_active,plan_target_yaw,plan_target_pitch,"
                       "plan_yaw,plan_pitch,plan_yaw_velocity,plan_pitch_velocity,"
                       "plan_yaw_acceleration,plan_pitch_acceleration,"
                       "yaw_velocity_ff,pitch_velocity_ff,"
                       "yaw_acceleration_ff,pitch_acceleration_ff,"
                       "yaw_velocity_ref,pitch_velocity_ref,"
                       "yaw_velocity_feedback,pitch_velocity_feedback,"
                       "chassis_yaw_velocity_imu,yaw_chassis_spin_torque_ff,"
                       "actual_yaw,actual_pitch,actual_yaw_velocity,actual_pitch_velocity,"
                       "yaw_error,pitch_error,fire_control,laser_distance,"
                       "yaw_control_angle_error,pitch_control_angle_error,"
                       "yaw_control_torque,pitch_control_torque\n";
    }

    void close_csv() {
        if (!csv_stream_.is_open())
            return;
        csv_stream_.flush();
        csv_stream_.close();
    }

    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<std::size_t> update_count_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<double> yaw_velocity_imu_;
    InputInterface<double> pitch_velocity_imu_;
    InputInterface<float> bullet_speed_;

    InputInterface<rmcs_msgs::TargetSnapshot> target_snapshot_;
    InputInterface<Eigen::Vector3d> control_direction_;
    InputInterface<bool> fire_control_;
    InputInterface<double> laser_distance_;
    InputInterface<double> plan_target_yaw_;
    InputInterface<double> plan_target_pitch_;
    InputInterface<double> plan_yaw_;
    InputInterface<double> plan_pitch_;
    InputInterface<double> plan_yaw_velocity_;
    InputInterface<double> plan_pitch_velocity_;
    InputInterface<double> plan_yaw_acceleration_;
    InputInterface<double> plan_pitch_acceleration_;
    InputInterface<double> yaw_velocity_ff_;
    InputInterface<double> pitch_velocity_ff_;
    InputInterface<double> yaw_acceleration_ff_;
    InputInterface<double> pitch_acceleration_ff_;
    InputInterface<double> yaw_velocity_ref_;
    InputInterface<double> pitch_velocity_ref_;
    InputInterface<double> yaw_velocity_feedback_;
    InputInterface<double> pitch_velocity_feedback_;
    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<double> yaw_chassis_spin_torque_ff_;
    InputInterface<double> yaw_control_angle_error_;
    InputInterface<double> pitch_control_angle_error_;
    InputInterface<double> yaw_control_torque_;
    InputInterface<double> pitch_control_torque_;

    Clock::time_point start_time_{};
    bool start_time_initialized_ = false;
    bool require_auto_aim_request_ = true;
    std::string csv_path_;
    int64_t csv_decimation_ = 1;
    std::size_t log_count_ = 0;
    std::ofstream csv_stream_;
};

} // namespace rmcs_core::controller::gimbal

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::AutoAimResponseRecorder, rmcs_executor::Component)
