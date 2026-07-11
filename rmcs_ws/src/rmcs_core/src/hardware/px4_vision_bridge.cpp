#include "nav_msgs/msg/odometry.hpp"
#include <mavlink/v2.0/common/mavlink.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

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
        link_timeout_sec_ = get_parameter("link_timeout_sec").as_double();
    
        const auto m =get_parameter("mount_rpy").as_double_array();

        q_mount_ =quaternion_from_rpy_zyx(m.at(0),m.at(1),m.at(2));

        min_send_interval_ =rclcpp::Duration::from_seconds(1.0/max_send_rate_hz_);
        last_send_time_ =now();
        last_heartbeat_send_ =now();
        last_heartbeat_recv_ =now();

        odom_subscription_ =create_subscription<nav_msgs::msg::Odometry>(
            source_topic_,
            rclcpp::SensorDataQoS{},
            [this](nav_msgs::msg::Odometry::UniquePtr msg){
                odometry_callback(std::move(msg));
            }
        );
    }

    Px4VisionBridge(const Px4VisionBridge&) = delete;
    Px4VisionBridge& operator=(const Px4VisionBridge&) = delete;

    void update() override {
        send_heartbeat_if_due();   //1hz  HEARTBEAT
        send_vision_if_pending();  //消费回调缓存的最新位姿,变换并发送
        drain_downlink();          //消费HEARTBEAT/STATUSTEXT,维护链路超时
    }

private:

    struct OdomSample {
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    };

    void odometry_callback(const nav_msgs::msg::Odometry::UniquePtr msg){
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

    void drain_downlink(){
        if(!px4_serial_.active())[[unlikely]]
            return;
        std::byte buffer[256];
        const size_t n =px4_serial_->read(buffer,sizeof(buffer));

        for(size_t i=0;i<n;i++){
            mavlink_message_t msg_mavlink;
            mavlink_status_t  status;
            if(mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(buffer[i]), &msg_mavlink, &status)){
                RCLCPP_INFO_THROTTLE(
                    logger_, *get_clock(), 1000, // 最多 1s 一条
                    "[px4 rx] parsed msgid=%u", static_cast<unsigned>(msg_mavlink.msgid));
                switch (msg_mavlink.msgid){
                    case MAVLINK_MSG_ID_HEARTBEAT: last_heartbeat_recv_ =now();
                        break;
                    case MAVLINK_MSG_ID_STATUSTEXT:{
                        mavlink_statustext_t st;
                        mavlink_msg_statustext_decode(&msg_mavlink, &st);
                        RCLCPP_WARN(logger_, "[px4]%.*s",static_cast<int>(sizeof(st.text)),st.text);
                        break;
                    }
                    default:
                        break;
                }
            }
        }

        
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

    static Eigen::Quaterniond quaternion_from_rpy_zyx(double roll, double pitch, double yaw){

        return Eigen::Quaterniond{
            
            Eigen::AngleAxisd{yaw,   Eigen::Vector3d::UnitZ()} *
            Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()} *
            Eigen::AngleAxisd{roll,  Eigen::Vector3d::UnitX()}
        };
    }    

    static void quaternion_to_rpy_zyx(const Eigen::Quaterniond& q, double& roll,double& pitch, double& yaw){
        const double w =q.w(),x =q.x(),y =q.y(),z =q.z();
        double sin_pitch =2.0*(w*y - z*x);
        sin_pitch = std::clamp(sin_pitch, -1.0, 1.0);
        pitch = std::asin(sin_pitch);
        roll  = std::atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));
        yaw   = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
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
    double  link_timeout_sec_;   
    Eigen::Quaterniond  q_mount_;

    rclcpp::Duration min_send_interval_ =rclcpp::Duration::from_seconds(0.02);
    rclcpp::Time last_send_time_        =rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_heartbeat_send_{0,0,RCL_ROS_TIME};
    rclcpp::Time last_heartbeat_recv_{0,0,RCL_ROS_TIME};
};

}// namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Px4VisionBridge, rmcs_executor::Component)