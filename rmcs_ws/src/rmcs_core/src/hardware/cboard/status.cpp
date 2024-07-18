#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int32.hpp>

#include "hardware/cboard/dji_motor_status.hpp"
#include "hardware/cboard/dr16_status.hpp"
#include "hardware/cboard/imu_status.hpp"
#include "hardware/cboard/package_receiver.hpp"
#include "hardware/cboard/supercap_status.hpp"

namespace rmcs_core::hardware::cboard {

class Status
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Status()
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
        package_receiver_.subscribe(0x32, [](auto&&) {});

        for (auto& motor : chassis_wheel_motors_)
            motor.configure(DjiMotorConfig{DjiMotorType::M3508}
                                .reverse()
                                .set_reduction_ratio(13.)
                                .enable_multi_turn_angle());

        gimbal_yaw_motor_.configure(DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));
        gimbal_pitch_motor_.configure(DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(get_parameter("pitch_motor_zero_point").as_int())));

        gimbal_left_friction_.configure(
            DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));
        gimbal_right_friction_.configure(
            DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));
        gimbal_bullet_feeder_.configure(
            DjiMotorConfig{DjiMotorType::M2006}.enable_multi_turn_angle());

        imu_gx_bias_ = get_parameter("imu_gx_bias").as_double();
        imu_gy_bias_ = get_parameter("imu_gy_bias").as_double();
        imu_gz_bias_ = get_parameter("imu_gz_bias").as_double();

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

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
    }
    ~Status() = default;

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
            motor.update(std::move(package), logger_);
            tf_->set_state<BaseLink, LeftFrontWheelLink>(motor.get_angle());
        } else if (can_id == 0x202) {
            auto& motor = chassis_wheel_motors_[1];
            motor.update(std::move(package), logger_);
            tf_->set_state<BaseLink, RightFrontWheelLink>(motor.get_angle());
        } else if (can_id == 0x203) {
            auto& motor = chassis_wheel_motors_[2];
            motor.update(std::move(package), logger_);
            tf_->set_state<BaseLink, RightBackWheelLink>(motor.get_angle());
        } else if (can_id == 0x204) {
            auto& motor = chassis_wheel_motors_[3];
            motor.update(std::move(package), logger_);
            tf_->set_state<BaseLink, LeftBackWheelLink>(motor.get_angle());
        } else if (can_id == 0x205) {
            gimbal_yaw_motor_.update(std::move(package), logger_);
            tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.get_angle());
        } else if (can_id == 0x206) {
            gimbal_pitch_motor_.update(std::move(package), logger_);
            tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.get_angle());
        } else if (can_id == 0x300) {
            supercap_.update(std::move(package), logger_);
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
        using namespace rmcs_description;
        if (can_id == 0x202) {
            gimbal_bullet_feeder_.update(std::move(package), logger_);
        } else if (can_id == 0x203) {
            gimbal_left_friction_.update(std::move(package), logger_);
        } else if (can_id == 0x204) {
            gimbal_right_friction_.update(std::move(package), logger_);
        } else if (can_id == 0x206) {
            gimbal_pitch_motor_.update(std::move(package), logger_);
            tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.get_angle());
        }
    }

    void dbus_receive_callback(std::unique_ptr<Package> package) {
        dr16_.update_status(std::move(package), logger_);
    }

    void imu_receive_callback(std::unique_ptr<Package> package) {
        auto& data      = package->dynamic_part<ImuData>();
        auto solve_acc  = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };
        double gx = solve_gyro(data.gyro_x), gy = solve_gyro(data.gyro_y),
               gz = solve_gyro(data.gyro_z);
        gx -= imu_gx_bias_;
        gy -= imu_gy_bias_;
        gz -= imu_gz_bias_;

        double ax = solve_acc(data.acc_x), ay = solve_acc(data.acc_y), az = solve_acc(data.acc_z);

        *gimbal_yaw_velocity_imu_   = gz;
        *gimbal_pitch_velocity_imu_ = gx;

        auto gimbal_imu_pose = imu_.update(gx, gy, gz, ax, ay, az);
        tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New yaw offset: %d",
            gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New pitch offset: %d",
            gimbal_pitch_motor_.calibrate_zero_point());
    }

    rclcpp::Logger logger_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    OutputInterface<serial::Serial> serial_;
    PackageReceiver package_receiver_;

    DjiMotorStatus chassis_wheel_motors_[4]{
        {this,  "/chassis/left_front_wheel"},
        {this, "/chassis/right_front_wheel"},
        {this,  "/chassis/right_back_wheel"},
        {this,   "/chassis/left_back_wheel"}
    };
    SupercapStatus supercap_{this};

    DjiMotorStatus gimbal_yaw_motor_{this, "/gimbal/yaw"};
    DjiMotorStatus gimbal_pitch_motor_{this, "/gimbal/pitch"};

    DjiMotorStatus gimbal_left_friction_{this, "/gimbal/left_friction"};
    DjiMotorStatus gimbal_right_friction_{this, "/gimbal/right_friction"};
    DjiMotorStatus gimbal_bullet_feeder_{this, "/gimbal/bullet_feeder"};

    Dr16Status dr16_{this};

    ImuStatus imu_;
    double imu_gx_bias_, imu_gy_bias_, imu_gz_bias_;

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;
    OutputInterface<rmcs_description::Tf> tf_;
};

} // namespace rmcs_core::hardware::cboard

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::cboard::Status, rmcs_executor::Component)