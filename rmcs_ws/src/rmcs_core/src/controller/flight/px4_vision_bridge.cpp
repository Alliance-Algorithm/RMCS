#include "nav_msgs/msg/odometry.hpp"
#include <mavlink/v2.0/common/mavlink.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>

namespace rmcs_core::hardware{

class Px4VisionBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Px4VisionBridge()
        :Node{
            get_component_name(),
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        register_input("/px4/serial", px4_serial_);

        source_topic_  = get_parameter("source_topic").as_string();
        system_id_     = get_parameter("system_id").as_int();
        component_id_  = get_parameter("component_id").as_int();
        max_send_rate_hz_ = get_parameter("max_send_rate_hz").as_double();

        const auto m =get_parameter("mount_rpy").as_double_array();

        q_mount_ =quaternion_from_rpy_zyx(m.at(0),m.at(1),m.at(2));

        min_send_interval_ =rclcpp::Duration::from_seconds(1.0/max_send_rate_hz_);
        last_send_time_ =now();
        last_heartbeat_send_ =now();

        odom_subscription_ =create_subscription<nav_msgs::msg::Odometry>(
            source_topic_,
            rclcpp::SensorDataQoS{},
            [this](nav_msgs::msg::Odometry::UniquePtr msg){
                odometry_callback(std::move(msg));
            }
        );

        odin_autostart_        =get_parameter("odin_autostart").as_bool();
        odin_watchdog_timeout_ =get_parameter("odin_watchdog_timeout").as_double();
        odin_restart_cooldown_ =get_parameter("odin_restart_cooldown").as_double();

        last_odom_arrival_ns_.store(steady_now_ns(),std::memory_order_relaxed);
        if(odin_autostart_)
            odin_manager_thread_ =std::jthread{[this](std::stop_token st){odin_manager_loop(st);}};
    }

    Px4VisionBridge(const Px4VisionBridge&) = delete;
    Px4VisionBridge& operator=(const Px4VisionBridge&) = delete;

    void update() override {
        send_heartbeat_if_due();   //1hz  HEARTBEAT
        send_vision_if_pending();  //消费回调缓存的最新位姿,变换并发送
    }

private:

    struct OdomSample {
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    };

    void odometry_callback(const nav_msgs::msg::Odometry::UniquePtr msg){
        last_odom_arrival_ns_.store(steady_now_ns(),std::memory_order_relaxed);

        const auto& p =msg->pose.pose.position;
        const auto& o =msg->pose.pose.orientation;

        const std::scoped_lock lock{odom_mutex_};
        pending_odom_.stamp       =rclcpp::Time{msg->header.stamp};
        pending_odom_.position    =Eigen::Vector3d{p.x,p.y,p.z};
        pending_odom_.orientation =Eigen::Quaterniond{o.w,o.x,o.y,o.z};
        odom_pending_ =true;
    }

    void send_vision_if_pending(){
        if((now() -last_send_time_)<min_send_interval_)
            return;

        OdomSample sample;
        {
            const std::scoped_lock lock{odom_mutex_};
            if(!odom_pending_)
                return;
            odom_pending_ =false;
            sample =pending_odom_;
        }
        last_send_time_ =now();

        const auto x =static_cast<float>(sample.position.y());
        const auto y =static_cast<float>(sample.position.x());
        const auto z =static_cast<float>(-sample.position.z());

        const Eigen::Quaterniond q_enu_flu =sample.orientation *q_mount_;

        Eigen::Quaterniond q_ned_frd =kNedEnuQ *(q_enu_flu*kAircraftBaselinkQ);

        q_ned_frd.normalize();
        double roll,pitch,yaw;
        quaternion_to_rpy_zyx(q_ned_frd,roll,pitch,yaw);

        const auto usec = static_cast<uint64_t>(sample.stamp.nanoseconds())/1000ull;

        float covariance[21];
        std::fill(std::begin(covariance),std::end(covariance),NAN);

        mavlink_message_t msg_mavlink;
        mavlink_msg_vision_position_estimate_pack(
            system_id_,
            component_id_,
            &msg_mavlink,
            usec,
            x,y,z,
            static_cast<float>(roll),
            static_cast<float>(pitch),
            static_cast<float>(yaw),
            covariance,
            /*reset_counter=*/0
        );

        send_message(msg_mavlink);
    }

    void send_heartbeat_if_due(){
        if((now()-last_heartbeat_send_).seconds()<1.0)
            return;
        last_heartbeat_send_ =now();
        mavlink_message_t msg_mavlink;
        mavlink_msg_heartbeat_pack(
            system_id_,
            component_id_,
            &msg_mavlink,
            MAV_TYPE_ONBOARD_CONTROLLER,
            MAV_AUTOPILOT_INVALID,
            /*base_mode=*/0,
            /*custom_mode=*/0,
            MAV_STATE_ACTIVE
        );
        send_message(msg_mavlink);
    }

    void send_message(const mavlink_message_t& msg){
        if(!px4_serial_.active())[[unlikely]]
            return;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len =mavlink_msg_to_send_buffer(buffer,&msg);
        px4_serial_->write(reinterpret_cast<const std::byte*>(buffer), len);
    }

    static int64_t steady_now_ns(){
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    // 独立线程看门狗：断流超时且过了冷却期就拉起 tmux-launch.sh；
    void odin_manager_loop(std::stop_token st){
        const std::string cmd =
            "\""+ament_index_cpp::get_package_prefix("odin_ros_driver")
            +"/lib/odin_ros_driver/tmux-launch.sh\"";

        const auto timeout_ns  =static_cast<int64_t>(odin_watchdog_timeout_*1e9);
        const auto cooldown_ns =static_cast<int64_t>(odin_restart_cooldown_*1e9);
        int64_t last_launch_ns =steady_now_ns()-cooldown_ns;  //首次断流即可拉起
        bool online =true;  //启动宽限：先假定在线，超时才告警

        // 锁仅为 wait_for 语义存在 request_stop 会直接唤醒等待
        std::mutex sleep_mutex;
        std::condition_variable_any sleep_cv;
        std::unique_lock lock{sleep_mutex};
        while(!sleep_cv.wait_for(
            lock,st,std::chrono::seconds{1},[&]{return st.stop_requested();})){

            const auto now_ns =steady_now_ns();
            const bool fresh =
                (now_ns-last_odom_arrival_ns_.load(std::memory_order_relaxed))<timeout_ns;

            if(fresh!=online){
                online =fresh;
                if(online)
                    RCLCPP_INFO(logger_,"Odin1 online, odometry flowing");
                else
                    RCLCPP_WARN(
                        logger_,
                        "Odin1 offline (no odometry for %.0fs), relaunching every %.0fs until online",
                        odin_watchdog_timeout_,odin_restart_cooldown_);
            }

            if(!online &&(now_ns-last_launch_ns)>cooldown_ns){
                last_launch_ns =now_ns;
                // （脚本内部的清杀/启动轮询）存在阻塞，单独在本线程做
                if(std::system(cmd.c_str())!=0)
                    RCLCPP_ERROR(logger_,"Odin1 launch script failed");
            }
        }
    }

    static Eigen::Quaterniond quaternion_from_rpy_zyx(double roll, double pitch, double yaw){

        return Eigen::Quaterniond{
            
            Eigen::AngleAxisd{yaw,   Eigen::Vector3d::UnitZ()} *
            Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()} *
            Eigen::AngleAxisd{roll,  Eigen::Vector3d::UnitX()}
        };
    }    

    static void quaternion_to_rpy_zyx(const Eigen::Quaterniond& q, double& roll,double& pitch, double& yaw){
        const double w =q.w(),x =q.x(),y =q.y(),z =q.z();
        const double m20 =2.0*(x*z - w*y);          // -sin(pitch)
        const double m21 =2.0*(y*z + w*x);
        const double m22 =1.0 - 2.0*(x*x + y*y);
        const double sin_pitch =std::clamp(-m20, -1.0, 1.0);
        const double cos_pitch =std::sqrt(m21*m21 + m22*m22);

        if(cos_pitch > 1e-9){
            pitch = std::atan2(sin_pitch, cos_pitch);
            roll  = std::atan2(m21, m22);
            yaw   = std::atan2(2.0*(x*y + w*z), 1.0 - 2.0*(y*y + z*z));
            return;
        }
        // 万向节锁 (pitch=±π/2)：roll/yaw 只剩组合自由度可观测，
        // 约定 roll=0、组合角全部归 yaw，保证三元组重建仍是原旋转
        const double m11 =1.0 - 2.0*(x*x + z*z);
        const double m12 =2.0*(y*z - w*x);
        roll  = 0.0;
        pitch = (sin_pitch > 0.0) ? M_PI_2 : -M_PI_2;
        yaw   = (sin_pitch > 0.0) ? std::atan2(m12, m11) : std::atan2(-m12, m11);
    }
    //ENU->NED
    static inline const Eigen::Quaterniond& kNedEnuQ{
        0.0,0.70710678118655,0.70710678118655,0.0
    };
    //FLU->FRD
    static inline const Eigen::Quaterniond& kAircraftBaselinkQ{
        0.0,1.0,0.0,0.0
    };

    rclcpp::Logger logger_;
    InputInterface<rmcs_msgs::SerialInterface> px4_serial_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    // 回调线程(rclcpp spin)与 update() 线程(executor)间的最新位姿交接
    std::mutex odom_mutex_;
    OdomSample pending_odom_;
    bool odom_pending_ =false;

    std::string source_topic_;
    uint8_t system_id_;          
    uint8_t component_id_;       
    double  max_send_rate_hz_;
    Eigen::Quaterniond  q_mount_;

    rclcpp::Duration min_send_interval_ =rclcpp::Duration::from_seconds(0.02);
    rclcpp::Time last_send_time_        =rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_heartbeat_send_{0,0,RCL_ROS_TIME};

    // Odin1 拉起与看门狗
    bool   odin_autostart_;
    double odin_watchdog_timeout_;
    double odin_restart_cooldown_;
    std::atomic<int64_t> last_odom_arrival_ns_{0};
    // 必须最后声明：其隐式析构 request_stop+join，须早于线程所用成员的销毁
    std::jthread odin_manager_thread_;
};

}// namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Px4VisionBridge, rmcs_executor::Component)