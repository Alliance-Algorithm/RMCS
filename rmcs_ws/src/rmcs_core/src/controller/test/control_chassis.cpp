#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"


namespace rmcs_core::controller::test{

class TestChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TestChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        
        , following_velocity_controller_(7.0, 0.0, 0.0) {
        following_velocity_controller_.output_max = angular_velocity_max;
        following_velocity_controller_.output_min = -angular_velocity_max;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);



//        register_output("/chassis/angle", chassis_angle_, nan);
//        register_output("/chassis/control_angle", chassis_control_angle_, nan);


        register_output("/chassis/control_velocity", chassis_control_velocity_);
    }


        void before_updating() override {
        if (!joystick_left_.ready()) {

            RCLCPP_WARN(get_logger(), "Failed to fetch \"/joystick_left\". Set to 0.0.");
        }
        if (!joystick_right_.ready()) {
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/joystick_right\". Set to 0.0.");
        }
    }
    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;



        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                // 安全设置（锁定）
                break;
            }
            // if(!mode_.active()){

            //     *mode_=rmcs_msgs::ChassisMode::AUTO;
            // }


            auto mode = *mode_;
            if (switch_left != Switch::DOWN) {//左拨杆不为down才允许切换模式
                if (last_switch_right_ == Switch::UP && switch_right == Switch::MIDDLE) {
                    if (mode == rmcs_msgs::ChassisMode::AUTO) {
                        mode = rmcs_msgs::ChassisMode::SPIN;
                    } 
                } else if (last_switch_right_==Switch::MIDDLE && switch_right== Switch::DOWN) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        spinning_forward_=!spinning_forward_;
                    }
                } else if (last_switch_right_==Switch::DOWN && switch_right== Switch::MIDDLE) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        spinning_forward_=!spinning_forward_;
                    }
                } else if (last_switch_right_==Switch::MIDDLE && switch_right== Switch::UP) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        mode = rmcs_msgs::ChassisMode::AUTO;
                    } 
                }
                *mode_ = mode;
                // mode模式切换，采用上升下降沿切换判断方法
            }
            RCLCPP_INFO(this->get_logger(), "ChassisMode = %s", to_string(*mode_).c_str());
            update_velocity_control();
            
        } while (false);

        last_switch_right_= switch_right;
        last_switch_left_ = switch_left;
        // 更新遥控器存储
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        *chassis_control_velocity_ = {nan, nan, nan};
    }

    void update_velocity_control() {
//        RCLCPP_INFO(logger_, "start to update translational_velocity");
        auto translational_velocity = update_translational_velocity_control();
//        RCLCPP_INFO(logger_, "start to update angular_velocity");
        auto angular_velocity       = update_angular_velocity_control();

        
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;  //可以实现同时转动加上全向移动
    }

    Eigen::Vector2d update_translational_velocity_control() {
        
        Eigen::Vector2d translational_velocity =(*joystick_left_);
//直接默认方向作为正方向
        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max;

        return translational_velocity;
        // 制作出了由遥控器给的目标速度（给出一个有方向和大小的向量=translational_velocity）
    }

    double update_angular_velocity_control() {
        double angular_velocity      = 0.0;
//        double chassis_control_angle = nan;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;
        case rmcs_msgs::ChassisMode::SPIN: {
            angular_velocity =
                0.6 * (spinning_forward_ ? angular_velocity_max : -angular_velocity_max);
        } break;
        case rmcs_msgs::ChassisMode::LAUNCH_RAMP:{

            (*mode_)=rmcs_msgs::ChassisMode::AUTO;
        } break;
        case rmcs_msgs::ChassisMode::STEP_DOWN:{

            (*mode_)=rmcs_msgs::ChassisMode::AUTO;
        } break;

        }
//        *chassis_angle_         = 2 * std::numbers::pi - *gimbal_yaw_angle_;
//        *chassis_control_angle_ = chassis_control_angle;

        return angular_velocity;
    }


    std::string to_string(rmcs_msgs::ChassisMode mode) {
    switch (mode) {
        case rmcs_msgs::ChassisMode::AUTO:        return "AUTO";
        case rmcs_msgs::ChassisMode::SPIN:        return "SPIN";
        case rmcs_msgs::ChassisMode::STEP_DOWN:   return "STEP_DOWN";
        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: return "LAUNCH_RAMP";
        default:                                  return "UNKNOWN";
    }
}

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;
    // Maximum control velocities
    static constexpr double translational_velocity_max = 10.0;
    static constexpr double angular_velocity_max       = 16.0;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;


    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;


//    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
//    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    bool spinning_forward_ = true;
    pid::PidCalculator following_velocity_controller_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    };

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::TestChassisController, rmcs_executor::Component)





