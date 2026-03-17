#include <cstdint>
#include <memory>
#include <mutex>
#include <tuple>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/agent/c_board.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

class Flight
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    Flight()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::CBoard{}
        , local_position_interface_{*this, px4_ros2::PoseFrame::LocalNED, px4_ros2::VelocityFrame::LocalNED}
        , logger_(get_logger())
        , flight_command_(
              create_partner_component<FlightCommand>(get_component_name() + "_command", *this))
        , gimbal_yaw_motor_(*this, *flight_command_, "/gimbal/yaw")
        , gimbal_pitch_motor_(*this, *flight_command_, "/gimbal/pitch")
        , gimbal_left_friction_(*this, *flight_command_, "/gimbal/left_friction")
        , gimbal_right_friction_(*this, *flight_command_, "/gimbal/right_friction")
        , gimbal_bullet_feeder_(*this, *flight_command_, "/gimbal/bullet_feeder")
        , dr16_(*this)
        , bmi088_(500.0, 0.3, 0.005) {

        if (!local_position_interface_.doRegister()) {
            throw std::runtime_error("Failed to register LocalPositionMeasurementInterface");
        }
        RCLCPP_INFO(logger_, "LocalPositionMeasurementInterface registered successfully.");

        auto odin_odom_qos = rclcpp::QoS{1};
        odin_odom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        odin_odom_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        odin_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odin1/odometry", odin_odom_qos,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
                odin_odometry_subscription_callback(*msg);
            });
        RCLCPP_INFO(logger_, "Subscribed to Odin driver odometry on /odin1/odometry.");

        gimbal_yaw_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMHF7015}
                .set_reversed()
                .set_encoder_zero_point(
                    static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));
        gimbal_pitch_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                static_cast<int>(get_parameter("pitch_motor_zero_point").as_int())));
        gimbal_left_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reversed()
                .set_reduction_ratio(1.));
        gimbal_right_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));
        gimbal_bullet_feeder_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006}
                .set_reversed()
                .enable_multi_turn_angle());

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_output("/tf", tf_);

        bmi088_.set_coordinate_mapping(
            [](double x, double y, double z) { return std::make_tuple(-x, z, y); });

        using namespace rmcs_description;

        constexpr double rotor_distance_x = 0.83637;
        constexpr double rotor_distance_y = 0.83637;

        tf_->set_transform<BaseLink, LeftFrontWheelLink>(
            Eigen::Translation3d{rotor_distance_x / 2, rotor_distance_y / 2, 0});
        tf_->set_transform<BaseLink, LeftBackWheelLink>(
            Eigen::Translation3d{rotor_distance_x / 2, -rotor_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightBackWheelLink>(
            Eigen::Translation3d{-rotor_distance_x / 2, rotor_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightFrontWheelLink>(
            Eigen::Translation3d{-rotor_distance_x / 2, -rotor_distance_y / 2, 0});

        constexpr double gimbal_center_x = 0;
        constexpr double gimbal_center_y = 0;
        constexpr double gimbal_center_z = -0.20552;
        tf_->set_transform<BaseLink, GimbalCenterLink>(
            Eigen::Translation3d{gimbal_center_x, gimbal_center_y, gimbal_center_z});

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.0557, 0, 0.053});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
        register_output("/referee/serial", referee_serial_);
        referee_serial_->read = [this](std::byte* buffer, size_t size) {
            return referee_ring_buffer_receive_.pop_front_n(
                [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            start_transmit().uart2_transmit({
                std::span{buffer, size}
            });
            return size;
        };
    }

    ~Flight() override = default;

    void update() override {
        update_motors();
        update_imu();
        dr16_.update_status();
        update_local_position();
    }

    void command_update() {
        auto yaw_cmd = gimbal_yaw_motor_.generate_command();
        auto pitch_cmd = gimbal_pitch_motor_.generate_command();

        device::CanPacket8 dji_cmds{
            gimbal_bullet_feeder_.generate_command(), device::CanPacket8::PaddingQuarter{},
            gimbal_right_friction_.generate_command(), gimbal_left_friction_.generate_command()};

        start_transmit()
            .can2_transmit({.can_id = 0x141, .can_data = yaw_cmd.as_bytes()})
            .can2_transmit({.can_id = 0x142, .can_data = pitch_cmd.as_bytes()})
            .can1_transmit({.can_id = 0x200, .can_data = dji_cmds.as_bytes()});
    }

private:
    void update_motors() {
        using namespace rmcs_description;
        gimbal_yaw_motor_.update_status();
        tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.angle());

        gimbal_pitch_motor_.update_status();
        tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.angle());

        gimbal_bullet_feeder_.update_status();
        gimbal_left_friction_.update_status();
        gimbal_right_friction_.update_status();
    }

    void update_imu() {
        bmi088_.update_status();
        Eigen::Quaterniond gimbal_imu_pose{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());

        *gimbal_yaw_velocity_imu_ = bmi088_.gz();
        *gimbal_pitch_velocity_imu_ = bmi088_.gy();
    }

    void update_local_position() {
        OdomSnapshot snap;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!odom_ready_)
                return;
            snap = latest_odom_;
            odom_ready_ = false;
        }

        const auto& odin_to_px4_rotation = odin1_base_link_to_px4_rotation();
        const auto& odin_to_px4_offset = odin1_base_link_to_px4_translation();

        Eigen::Quaternionf odom_to_odin_q{snap.q_w, snap.q_x, snap.q_y, snap.q_z};
        odom_to_odin_q.normalize();

        Eigen::Isometry3f odom_to_odin = Eigen::Isometry3f::Identity();
        odom_to_odin.linear() = odom_to_odin_q.toRotationMatrix();
        odom_to_odin.translation() = Eigen::Vector3f{snap.pos_x, snap.pos_y, snap.pos_z};

        Eigen::Isometry3f odin_to_px4 = Eigen::Isometry3f::Identity();
        odin_to_px4.linear() = odin_to_px4_rotation;
        odin_to_px4.translation() = odin_to_px4_offset;

        const Eigen::Isometry3f odom_to_px4 = odom_to_odin * odin_to_px4;

        // nav_msgs/Odometry twists are reported in child_frame_id, which is odin1_base_link here.
        const Eigen::Vector3f odin_linear_velocity{snap.vel_x, snap.vel_y, snap.vel_z};
        const Eigen::Vector3f odin_angular_velocity{snap.ang_x, snap.ang_y, snap.ang_z};
        const Eigen::Vector3f px4_velocity_odom =
            odom_to_odin.linear()
            * (odin_linear_velocity + odin_angular_velocity.cross(odin_to_px4_offset));

        Eigen::Matrix3f odin_velocity_covariance = Eigen::Matrix3f::Zero();
        odin_velocity_covariance.diagonal() =
            Eigen::Vector3f{snap.vel_cov_xx, snap.vel_cov_yy, snap.vel_cov_zz};
        const Eigen::Matrix3f px4_velocity_covariance =
            odom_to_odin.linear() * odin_velocity_covariance * odom_to_odin.linear().transpose();

        px4_ros2::LocalPositionMeasurement meas{};
        meas.timestamp_sample =
            snap.timestamp_sample.nanoseconds() > 0 ? snap.timestamp_sample : get_clock()->now();

        meas.position_xy = odom_to_px4.translation().head<2>();
        meas.position_xy_variance = Eigen::Vector2f{
            snap.cov_xx > 0.f ? snap.cov_xx : 0.1f, snap.cov_yy > 0.f ? snap.cov_yy : 0.1f};

        meas.position_z = odom_to_px4.translation().z();
        meas.position_z_variance = snap.cov_zz > 0.f ? snap.cov_zz : 0.1f;

        meas.velocity_xy = px4_velocity_odom.head<2>();
        meas.velocity_xy_variance = Eigen::Vector2f{
            px4_velocity_covariance(0, 0) > 0.f ? px4_velocity_covariance(0, 0) : 0.1f,
            px4_velocity_covariance(1, 1) > 0.f ? px4_velocity_covariance(1, 1) : 0.1f};

        meas.velocity_z = px4_velocity_odom.z();
        meas.velocity_z_variance =
            px4_velocity_covariance(2, 2) > 0.f ? px4_velocity_covariance(2, 2) : 0.1f;

        meas.attitude_quaternion = Eigen::Quaternionf{odom_to_px4.linear()};
        meas.attitude_variance = Eigen::Vector3f{0.05f, 0.05f, 0.05f};

        try {
            local_position_interface_.update(meas);
            RCLCPP_INFO(
                logger_, "Published local position measurement with timestamp %f",
                meas.timestamp_sample.seconds());
        } catch (const px4_ros2::NavigationInterfaceInvalidArgument& e) {
            RCLCPP_ERROR_THROTTLE(
                logger_, *get_clock(), 1000, "Navigation update error: %s", e.what());
        }
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        // RCLCPP_INFO(
        //     logger_, "[gimbal calibration] New yaw offset: %ld",
        //     gimbal_yaw_motor_.calibrate_zero_point());
        // RCLCPP_INFO(
        //     logger_, "[gimbal calibration] New pitch offset: %ld",
        //     gimbal_pitch_motor_.calibrate_zero_point());
    }

    // ---- Shared odometry snapshot (filled by ROS subscription, read by update()) ----
    struct OdomSnapshot {
        rclcpp::Time timestamp_sample{};
        float pos_x{}, pos_y{}, pos_z{};
        float vel_x{}, vel_y{}, vel_z{};
        float ang_x{}, ang_y{}, ang_z{};
        float q_x{}, q_y{}, q_z{}, q_w{1.f};
        float cov_xx{}, cov_yy{}, cov_zz{};
        float vel_cov_xx{}, vel_cov_yy{}, vel_cov_zz{};
    };

    std::mutex odom_mutex_;
    OdomSnapshot latest_odom_;
    bool odom_ready_{false};

    void odin_odometry_subscription_callback(const nav_msgs::msg::Odometry& msg) {
        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 1000, "Received odometry message with timestamp %u.%u",
            msg.header.stamp.sec, msg.header.stamp.nanosec);

        OdomSnapshot snap{};
        snap.timestamp_sample = rclcpp::Time{msg.header.stamp};
        snap.pos_x = static_cast<float>(msg.pose.pose.position.x);
        snap.pos_y = static_cast<float>(msg.pose.pose.position.y);
        snap.pos_z = static_cast<float>(msg.pose.pose.position.z);
        snap.vel_x = static_cast<float>(msg.twist.twist.linear.x);
        snap.vel_y = static_cast<float>(msg.twist.twist.linear.y);
        snap.vel_z = static_cast<float>(msg.twist.twist.linear.z);
        snap.ang_x = static_cast<float>(msg.twist.twist.angular.x);
        snap.ang_y = static_cast<float>(msg.twist.twist.angular.y);
        snap.ang_z = static_cast<float>(msg.twist.twist.angular.z);
        snap.q_x = static_cast<float>(msg.pose.pose.orientation.x);
        snap.q_y = static_cast<float>(msg.pose.pose.orientation.y);
        snap.q_z = static_cast<float>(msg.pose.pose.orientation.z);
        snap.q_w = static_cast<float>(msg.pose.pose.orientation.w);
        snap.cov_xx = static_cast<float>(msg.pose.covariance[0]);
        snap.cov_yy = static_cast<float>(msg.pose.covariance[7]);
        snap.cov_zz = static_cast<float>(msg.pose.covariance[14]);
        snap.vel_cov_xx = static_cast<float>(msg.twist.covariance[0]);
        snap.vel_cov_yy = static_cast<float>(msg.twist.covariance[7]);
        snap.vel_cov_zz = static_cast<float>(msg.twist.covariance[14]);

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            latest_odom_ = snap;
            odom_ready_ = true;
        }
    }

    static const Eigen::Matrix3f& odin1_base_link_to_px4_rotation() {
        static const Eigen::Matrix3f rotation = [] {
            Eigen::Matrix3f matrix;
            // Odin axes are x->down, y->left, z->front. PX4 expects body FRD.
            matrix << 0.f, 0.f, 1.f, 0.f, -1.f, 0.f, 1.f, 0.f, 0.f;
            return matrix;
        }();
        return rotation;
    }

    static const Eigen::Vector3f& odin1_base_link_to_px4_translation() {
        static const Eigen::Vector3f translation = [] {
            // Odin is mounted 135 mm below, 312 mm right, and 296 mm behind the PX4 controller.
            // This is the odin1_base_link -> PX4 origin offset expressed in Odin axes.
            // return Eigen::Vector3f{-0.135f, 0.312f, 0.296f};
            return Eigen::Vector3f{-0.0f, 0.0f, 0.0f};
        }();
        return translation;
    }

protected:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (data.can_id == 0x201) {
            gimbal_bullet_feeder_.store_status(data.can_data);
        } else if (data.can_id == 0x204) {
            gimbal_left_friction_.store_status(data.can_data);
        } else if (data.can_id == 0x203) {
            gimbal_right_friction_.store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_remote_transmission || data.is_extended_can_id || data.can_data.size() < 8)
            [[unlikely]]
            return;
        if (data.can_id == 0x142) {
            gimbal_pitch_motor_.store_status(data.can_data);
        } else if (data.can_id == 0x141) {
            gimbal_yaw_motor_.store_status(data.can_data);
        }
    }

    void uart2_receive_callback(const librmcs::data::UartDataView& data) override {
        const std::byte* ptr = data.uart_data.data();
        referee_ring_buffer_receive_.emplace_back_n(
            [&ptr](std::byte* storage) noexcept { new (storage) std::byte{*ptr++}; },
            data.uart_data.size());
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data) override {
        bmi088_.store_accelerometer_status(data.x, data.y, data.z);
    }

    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        bmi088_.store_gyroscope_status(data.x, data.y, data.z);
    }

private:
    // ---- Members ----
    px4_ros2::LocalPositionMeasurementInterface local_position_interface_;
    rclcpp::Logger logger_;

    class FlightCommand : public rmcs_executor::Component {
    public:
        explicit FlightCommand(Flight& flight)
            : flight_(flight) {}
        void update() override { flight_.command_update(); }
        Flight& flight_;
    };

    std::shared_ptr<FlightCommand> flight_command_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odin_odometry_subscription_;
    device::LkMotor gimbal_yaw_motor_;
    device::LkMotor gimbal_pitch_motor_;

    device::DjiMotor gimbal_left_friction_;
    device::DjiMotor gimbal_right_friction_;
    device::DjiMotor gimbal_bullet_feeder_;

    device::Dr16 dr16_;
    device::Bmi088 bmi088_;

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;

    OutputInterface<rmcs_description::Tf> tf_;

    rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Flight, rmcs_executor::Component)
