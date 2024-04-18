#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

#include "forwarder/status/dji_motor_status_forwarder.hpp"
#include "forwarder/status/dr16_publisher.hpp"
#include "forwarder/status/imu.hpp"
#include "forwarder/status/package_receiver.hpp"

namespace rmcs_core::forwarder {

class StatusForwarder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    StatusForwarder()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        std::string path;
        if (get_parameter("path", path))
            RCLCPP_INFO(get_logger(), "Path: %s", path.c_str());
        else
            throw std::runtime_error{"Unable to get parameter 'path'"};

        const std::string stty_command = "stty -F " + path + " raw";
        if (std::system(stty_command.c_str()) != 0)
            throw std::runtime_error{"Unable to call '" + stty_command + "'"};

        register_output("/serial", serial_, path, 9600, serial::Timeout::simpleTimeout(0));

        package_receiver_.subscribe(
            0x11, [this](auto&& pkg) { can1_receive_callback(std::forward<decltype(pkg)>(pkg)); });
        package_receiver_.subscribe(
            0x12, [this](auto&& pkg) { can2_receive_callback(std::forward<decltype(pkg)>(pkg)); });
        package_receiver_.subscribe(
            0x23, [this](auto&& pkg) { dbus_receive_callback(std::forward<decltype(pkg)>(pkg)); });
        package_receiver_.subscribe(
            0x31, [this](auto&& pkg) { imu_receive_callback(std::forward<decltype(pkg)>(pkg)); });
        package_receiver_.subscribe(0x32, [this](auto&& pkg) {
            quaternion_receive_callback(std::forward<decltype(pkg)>(pkg));
        });

        for (auto& motor : chassis_wheel_motors_)
            motor.set_motor_m3508().set_reverse(true).set_reduction_ratio(1 / 14.0);

        gimbal_yaw_motor_.set_motor_gm6020().set_offset(0.771);
        gimbal_pitch_motor_.set_motor_gm6020().set_offset(-0.300);

        gimbal_left_friction_.set_motor_m3508().set_reverse(false);
        gimbal_right_friction_.set_motor_m3508().set_reverse(true);
        gimbal_bullet_deliver_.set_motor_m2006().set_reduction_ratio(1 / 36.0);

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_output("/tf", tf_);

        using namespace rmcs_description;

        tf_->set_transform<PitchLink, ImuLink>(
            Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()});

        constexpr double gimbal_center_height = 0.32059;
        constexpr double wheel_distance_x = 0.15897, wheel_distance_y = 0.15897;
        tf_->set_transform<BaseLink, GimbalCenterLink>(
            Eigen::Translation3d{0, 0, gimbal_center_height});
        tf_->set_transform<BaseLink, LeftFrontWheelLink>(
            Eigen::Translation3d{wheel_distance_x / 2, wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, LeftBackWheelLink>(
            Eigen::Translation3d{-wheel_distance_x / 2, wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightBackWheelLink>(
            Eigen::Translation3d{-wheel_distance_x / 2, -wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightFrontWheelLink>(
            Eigen::Translation3d{wheel_distance_x / 2, -wheel_distance_y / 2, 0});
    }
    ~StatusForwarder() = default;

    void update() override { package_receiver_.update(*serial_, logger_); }

private:
    void can1_receive_callback(std::unique_ptr<Package> package) {
        auto& static_part = package->static_part();

        if (package->dynamic_part_size() < sizeof(can_id_t)) {
            RCLCPP_ERROR(
                get_logger(), "CAN1 package does not contain can id: [0x%02X 0x%02X] (size = %d)",
                static_part.type, static_part.index, static_part.data_size);
            return;
        }

        auto can_id = package->dynamic_part<can_id_t>();
        using namespace rmcs_description;
        if (can_id == 0x201) {
            auto& motor = chassis_wheel_motors_[0];
            motor.update_status(std::move(package), logger_);
            tf_->set_state<BaseLink, LeftFrontWheelLink>(motor.get_angle());
        } else if (can_id == 0x202) {
            auto& motor = chassis_wheel_motors_[1];
            motor.update_status(std::move(package), logger_);
            tf_->set_state<BaseLink, RightFrontWheelLink>(motor.get_angle());
        } else if (can_id == 0x203) {
            auto& motor = chassis_wheel_motors_[2];
            motor.update_status(std::move(package), logger_);
            tf_->set_state<BaseLink, RightBackWheelLink>(motor.get_angle());
        } else if (can_id == 0x204) {
            auto& motor = chassis_wheel_motors_[3];
            motor.update_status(std::move(package), logger_);
            tf_->set_state<BaseLink, LeftBackWheelLink>(motor.get_angle());
        } else if (can_id == 0x205) {
            gimbal_yaw_motor_.update_status(std::move(package), logger_);
            tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.get_angle());
        } else if (can_id == 0x206) {
            gimbal_pitch_motor_.update_status(std::move(package), logger_);
            tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.get_angle());
        }
    }

    void can2_receive_callback(std::unique_ptr<Package> package) {
        auto& static_part = package->static_part();

        if (package->dynamic_part_size() < sizeof(can_id_t)) {
            RCLCPP_ERROR(
                logger_, "CAN2 package does not contain can id: [0x%02X 0x%02X] (size = %d)",
                static_part.type, static_part.index, static_part.data_size);
            return;
        }

        auto can_id = package->dynamic_part<can_id_t>();
        if (can_id == 0x202) {
            gimbal_bullet_deliver_.update_status(std::move(package), logger_);
        } else if (can_id == 0x203) {
            gimbal_left_friction_.update_status(std::move(package), logger_);
        } else if (can_id == 0x204) {
            gimbal_right_friction_.update_status(std::move(package), logger_);
        }
    }

    void dbus_receive_callback(std::unique_ptr<Package> package) {
        dr16_publisher_.update_status(std::move(package), logger_);
    }

    void imu_receive_callback(std::unique_ptr<Package> package) {
        auto& data      = package->dynamic_part<ImuData>();
        auto solve_acc  = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };
        double gx = solve_gyro(data.gyro_x), gy = solve_gyro(data.gyro_y),
               gz = solve_gyro(data.gyro_z);
        double ax = solve_acc(data.acc_x), ay = solve_acc(data.acc_y), az = solve_acc(data.acc_z);

        *gimbal_yaw_velocity_imu_   = gz;
        *gimbal_pitch_velocity_imu_ = gx;

        auto gimbal_imu_pose = imu_.update(gx, gy, gz, ax, ay, az);
        tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());
    }

    void quaternion_receive_callback(std::unique_ptr<Package> package) {
        auto& static_part = package->static_part();
        if (static_part.data_size != sizeof(QuaternionData)) {
            RCLCPP_ERROR(
                get_logger(),
                "Package size does not match (quaternion): [0x%02X 0x%02X] (size = %d)",
                static_part.type, static_part.index, static_part.data_size);
            return;
        }
        auto& dynamic_part = package->dynamic_part<QuaternionData>();

        // Some hacks for wiping the butt of those embedded engineers.
        Eigen::AngleAxisd angle_axis{
            Eigen::Quaterniond{dynamic_part.w, dynamic_part.x, dynamic_part.y, dynamic_part.z}
        };
        angle_axis.axis().x() = -angle_axis.axis().x();
        angle_axis.axis().y() = -angle_axis.axis().y();

        auto gimbal_imu_pose = Eigen::Quaterniond{angle_axis};
        tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());
    }

    rclcpp::Logger logger_;

    OutputInterface<serial::Serial> serial_;
    PackageReceiver package_receiver_;

    DjiMotorStatusForwarder chassis_wheel_motors_[4] = {
        {this,  "/chassis/left_front_wheel"},
        {this, "/chassis/right_front_wheel"},
        {this,  "/chassis/right_back_wheel"},
        {this,   "/chassis/left_back_wheel"}
    };

    DjiMotorStatusForwarder gimbal_yaw_motor_   = {this, "/gimbal/yaw"};
    DjiMotorStatusForwarder gimbal_pitch_motor_ = {this, "/gimbal/pitch"};

    DjiMotorStatusForwarder gimbal_left_friction_  = {this, "/gimbal/left_friction"};
    DjiMotorStatusForwarder gimbal_right_friction_ = {this, "/gimbal/right_friction"};
    DjiMotorStatusForwarder gimbal_bullet_deliver_ = {this, "/gimbal/bullet_deliver"};

    Dr16Publisher dr16_publisher_{this};

    Imu imu_;
    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;
    OutputInterface<rmcs_description::Tf> tf_;
};

} // namespace rmcs_core::forwarder

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::forwarder::StatusForwarder, rmcs_executor::Component)