#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

/*
angle controls & launch parameter settings
键位：
双下：全部停止
双中：初始状态
    此时{
        右拨杆下拨再回中：切换上膛和退膛
        处于上膛状态时右拨杆打到上：发射
    }
左拨杆上：设置模式
    此时{
        右拨杆在中：调整角度，左右摇杆分别控制yaw和pitch以防误触
        右拨杆在下：调整拉力，在yaml中设置力闭环模式或者手动控制模式
    }
*/

namespace rmcs_core::controller::dart {
class DartSettings
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartSettings()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {}

    void update() override {};

private:
    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    OutputInterface<double> yaw_control_velocity_; // 只需要速度环，摇杆直接映射成速度
    OutputInterface<double> pitch_angle_setpoint_; // 双环pid，外环角度内环速度，摇杆映射成目标角度

    OutputInterface<double> force_setpoint_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettings, rmcs_executor::Component)