#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <tuple>

#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include <librmcs/agent/c_board.hpp>

// Odin LiDAR C API (no ROS layer)
#include <lidar_api.h>
#include <lidar_api_type.h>

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
        , local_position_interface_{
              *this, px4_ros2::PoseFrame::LocalNED, px4_ros2::VelocityFrame::LocalNED}
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

        g_instance_ = this;

        if (!local_position_interface_.doRegister()) {
            throw std::runtime_error("Failed to register LocalPositionMeasurementInterface");
        }
        RCLCPP_INFO(logger_, "LocalPositionMeasurementInterface registered successfully.");

        if (lidar_system_init(&Flight::device_event_callback)) {
            throw std::runtime_error("lidar_system_init failed");
        }
        RCLCPP_INFO(logger_, "Odin lidar system initialized.");

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
            start_transmit().uart1_transmit({std::span{buffer, size}});
            return size;
        };
    }

    ~Flight() override {
        if (odin_device_) {
            lidar_stop_stream(odin_device_, LIDAR_MODE_SLAM);
            lidar_unregister_stream_callback(odin_device_);
            lidar_close_device(odin_device_);
            lidar_destory_device(odin_device_);
        }
        lidar_system_deinit();
    }

    void update() override {
        update_motors();
        update_imu();
        dr16_.update_status();
        update_local_position();
        // RCLCPP_INFO(
        //     logger_, "[gimbal calibration] New pitch offset: %ld",
        //     gimbal_pitch_motor_.calibrate_zero_point());
    }

    void command_update() {
        auto yaw_cmd = gimbal_yaw_motor_.generate_torque_command();
        auto pitch_cmd = gimbal_pitch_motor_.generate_command();

        // RCLCPP_INFO(
        //     logger_, "[gimbal calibration] New yaw offset: %ld",
        //     gimbal_yaw_motor_.calibrate_zero_point());
        // RCLCPP_INFO(
        //     logger_, "[gimbal calibration] New pitch offset: %ld",
        //     gimbal_pitch_motor_.calibrate_zero_point());

        device::CanPacket8 dji_cmds{
            gimbal_bullet_feeder_.generate_command(),
            device::CanPacket8::PaddingQuarter{},
            gimbal_right_friction_.generate_command(),
            gimbal_left_friction_.generate_command()};

        start_transmit()
            .can1_transmit({.can_id = 0x141, .can_data = yaw_cmd.as_bytes()})
            .can2_transmit({.can_id = 0x141, .can_data = pitch_cmd.as_bytes()})
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

        px4_ros2::LocalPositionMeasurement meas{};
        meas.timestamp_sample = get_clock()->now();

        meas.position_xy = Eigen::Vector2f{snap.pos_x, snap.pos_y};
        meas.position_xy_variance = Eigen::Vector2f{
            snap.cov_xx > 0.f ? snap.cov_xx : 0.1f,
            snap.cov_yy > 0.f ? snap.cov_yy : 0.1f};

        meas.position_z = snap.pos_z;
        meas.position_z_variance = snap.cov_zz > 0.f ? snap.cov_zz : 0.1f;

        meas.velocity_xy = Eigen::Vector2f{snap.vel_x, snap.vel_y};
        meas.velocity_xy_variance = Eigen::Vector2f{0.1f, 0.1f};

        // Quaternion from odin is (x, y, z, w); Eigen ctor is (w, x, y, z)
        meas.attitude_quaternion =
            Eigen::Quaternionf{snap.q_w, snap.q_x, snap.q_y, snap.q_z};
        meas.attitude_variance = Eigen::Vector3f{0.05f, 0.05f, 0.05f};

        try {
            local_position_interface_.update(meas);
        } catch (const px4_ros2::NavigationInterfaceInvalidArgument& e) {
            RCLCPP_ERROR_THROTTLE(
                logger_, *get_clock(), 1000, "Navigation update error: %s", e.what());
        }
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New yaw offset: %ld",
            gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New pitch offset: %ld",
            gimbal_pitch_motor_.calibrate_zero_point());
    }

    // ---- Shared odometry snapshot (filled by SDK callback, read by update()) ----
    struct OdomSnapshot {
        float pos_x{}, pos_y{}, pos_z{};
        float vel_x{}, vel_y{};
        float q_x{}, q_y{}, q_z{}, q_w{1.f};
        float cov_xx{}, cov_yy{}, cov_zz{};
    };

    std::mutex odom_mutex_;
    OdomSnapshot latest_odom_;
    bool odom_ready_{false};

    // ---- Device handle (set once when device connects) ----
    device_handle odin_device_{nullptr};

    // ---- Static callbacks (SDK uses plain C function pointers) ----

    // Called by SDK when device connects or disconnects.
    static void device_event_callback(const lidar_device_info_t* device_info, bool attach) {
        if (!attach || !g_instance_)
            return;
        g_instance_->on_device_attach(device_info);
    }

    // Called by SDK for every incoming data frame.
    static void data_callback(const lidar_data_t* data, void* /*user_data*/) {
        if (!g_instance_ || data->type != LIDAR_DT_SLAM_ODOMETRY)
            return;
        g_instance_->on_odom_frame(&data->stream);
    }

    // ---- Instance-level handlers ----

    void on_device_attach(const lidar_device_info_t* device_info) {
        RCLCPP_INFO(logger_, "Odin device attached, setting up SLAM odometry stream.");

        lidar_device_info_t info = *device_info; // mutable copy required by API
        device_handle dev = nullptr;
        if (lidar_create_device(&info, &dev)) {
            RCLCPP_ERROR(logger_, "lidar_create_device failed");
            return;
        }

        if (device_info->initial_state == LIDAR_DEVICE_NOT_INITIALIZED) {
            if (lidar_open_device(dev)) {
                RCLCPP_ERROR(logger_, "lidar_open_device failed");
                lidar_destory_device(dev);
                return;
            }
            if (lidar_set_mode(dev, LIDAR_MODE_SLAM)) {
                RCLCPP_ERROR(logger_, "lidar_set_mode(SLAM) failed");
                lidar_close_device(dev);
                lidar_destory_device(dev);
                return;
            }
        }

        lidar_data_callback_info_t cb_info{};
        cb_info.data_callback = &Flight::data_callback;
        cb_info.user_data = nullptr;
        if (lidar_register_stream_callback(dev, cb_info)) {
            RCLCPP_ERROR(logger_, "lidar_register_stream_callback failed");
            lidar_close_device(dev);
            lidar_destory_device(dev);
            return;
        }

        uint32_t dtof_subframe_odr = 0;
        if (lidar_start_stream(dev, LIDAR_MODE_SLAM, dtof_subframe_odr)) {
            RCLCPP_ERROR(logger_, "lidar_start_stream failed");
            lidar_close_device(dev);
            lidar_destory_device(dev);
            return;
        }

        lidar_activate_stream_type(dev, LIDAR_DT_SLAM_ODOMETRY);

        odin_device_ = dev;
        RCLCPP_INFO(logger_, "Odin SLAM odometry stream active.");
    }

    void on_odom_frame(const capture_Image_List_t* stream) {
        const uint32_t data_len = stream->imageList[0].length;
        const void* addr = stream->imageList[0].pAddr;

        OdomSnapshot snap{};

        if (data_len == sizeof(ros_odom_convert_complete_t)) {
            // Full odometry: position + velocity + covariance
            const auto* d = static_cast<const ros_odom_convert_complete_t*>(addr);

            // pos encoded as fixed-point ×1e6
            snap.pos_x = static_cast<float>(d->pos[0]) / 1e6f;
            snap.pos_y = static_cast<float>(d->pos[1]) / 1e6f;
            snap.pos_z = static_cast<float>(d->pos[2]) / 1e6f;

            // orientation quaternion (x, y, z, w) encoded ×1e6
            snap.q_x = static_cast<float>(d->orient[0]) / 1e6f;
            snap.q_y = static_cast<float>(d->orient[1]) / 1e6f;
            snap.q_z = static_cast<float>(d->orient[2]) / 1e6f;
            snap.q_w = static_cast<float>(d->orient[3]) / 1e6f;

            // linear velocity encoded ×1e6
            snap.vel_x = static_cast<float>(d->linear_velocity[0]) / 1e6f;
            snap.vel_y = static_cast<float>(d->linear_velocity[1]) / 1e6f;

            // pose covariance diagonal (pose_cov[0]=xx, pose_cov[4]=yy, pose_cov[8]=zz)
            snap.cov_xx = static_cast<float>(d->pose_cov[0]);
            snap.cov_yy = static_cast<float>(d->pose_cov[4]);
            snap.cov_zz = static_cast<float>(d->pose_cov[8]);

        } else if (data_len == sizeof(ros2_odom_convert_t)) {
            // Compact odometry: position + orientation only
            const auto* d = static_cast<const ros2_odom_convert_t*>(addr);

            snap.pos_x = static_cast<float>(d->pos[0]) / 1e6f;
            snap.pos_y = static_cast<float>(d->pos[1]) / 1e6f;
            snap.pos_z = static_cast<float>(d->pos[2]) / 1e6f;

            snap.q_x = static_cast<float>(d->orient[0]) / 1e6f;
            snap.q_y = static_cast<float>(d->orient[1]) / 1e6f;
            snap.q_z = static_cast<float>(d->orient[2]) / 1e6f;
            snap.q_w = static_cast<float>(d->orient[3]) / 1e6f;
        } else {
            return; // Unknown payload; skip
        }

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            latest_odom_ = snap;
            odom_ready_ = true;
        }
    }

protected:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]] return;

        if (data.can_id == 0x141) {
            gimbal_yaw_motor_.store_status(data.can_data);
        } else if (data.can_id == 0x201) {
            gimbal_bullet_feeder_.store_status(data.can_data);
        } else if (data.can_id == 0x204) {
            gimbal_left_friction_.store_status(data.can_data);
        } else if (data.can_id == 0x203) {
            gimbal_right_friction_.store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_remote_transmission || data.is_extended_can_id || data.can_data.size() < 8)
            [[unlikely]] return;
        if (data.can_id == 0x141) {
            gimbal_pitch_motor_.store_status(data.can_data);
        }
    }

    void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
        const std::byte* ptr = data.uart_data.data();
        referee_ring_buffer_receive_.emplace_back_n(
            [&ptr](std::byte* storage) noexcept { new (storage) std::byte{*ptr++}; },
            data.uart_data.size());
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void accelerometer_receive_callback(
        const librmcs::data::AccelerometerDataView& data) override {
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

    // Global pointer used by static C callbacks to reach the active instance.
    // Only one Flight component is expected per process.
    static Flight* g_instance_;
};

// Static member definition
Flight* Flight::g_instance_ = nullptr;

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Flight, rmcs_executor::Component)
