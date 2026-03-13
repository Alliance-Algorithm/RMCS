#include <atomic>
#include <mutex>
#include <thread>

#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

// Odin LiDAR C API (no ROS layer)
#include <lidar_api.h>
#include <lidar_api_type.h>

namespace rmcs_core::hardware {

class Flight
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Flight()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , local_position_interface_{*this, px4_ros2::PoseFrame::LocalNED, px4_ros2::VelocityFrame::LocalNED}
        , logger_(get_logger()) {
        g_instance_ = this;

        if (!local_position_interface_.doRegister()) {
            throw std::runtime_error("Failed to register LocalPositionMeasurementInterface");
        }
        RCLCPP_INFO(logger_, "LocalPositionMeasurementInterface registered successfully.");

        if (lidar_system_init(&Flight::device_event_callback)) {
            throw std::runtime_error("lidar_system_init failed");
        }
        RCLCPP_INFO(logger_, "Odin lidar system initialized.");
    }

    ~Flight() {
        if (odin_device_) {
            lidar_stop_stream(odin_device_, LIDAR_MODE_SLAM);
            lidar_unregister_stream_callback(odin_device_);
            lidar_close_device(odin_device_);
            lidar_destory_device(odin_device_);
        }
        lidar_system_deinit();
    }

    void update() override {
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
            RCLCPP_ERROR_THROTTLE(logger_, *get_clock(), 1000, "Navigation update error: %s", e.what());
        }
    }

private:
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

    // ---- Members ----
    px4_ros2::LocalPositionMeasurementInterface local_position_interface_;
    rclcpp::Logger logger_;

    // Global pointer used by static C callbacks to reach the active instance.
    // Only one Flight component is expected per process.
    static Flight* g_instance_;
};

// Static member definition
Flight* Flight::g_instance_ = nullptr;

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Flight, rmcs_executor::Component)
