#include "controller/pid/matrix_pid_calculator.hpp"
#include <algorithm>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_launch_stage.hpp>
#include <rmcs_msgs/switch.hpp>
#include <sys/types.h>

/*
launch controls
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
class DartLaunchController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , drive_belt_pid_calculator_(
              get_parameter("b_kp").as_double(), get_parameter("b_ki").as_double(),
              get_parameter("b_kd").as_double()) {

        dirve_belt_working_velocity_ = get_parameter("belt_velocity").as_double();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        max_control_torque_ = get_parameter("max_control_torque").as_double();

        launch_trigger_angle_ = get_parameter("trigger_free_angle").as_int();
        launch_lock_angle_ = get_parameter("trigger_lock_angle").as_int();

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/dart/drive_belt/left/velocity", left_drive_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_drive_belt_velocity_);

        register_output("/dart/drive_belt/left/control_torque", left_drive_belt_control_torque_, 0);
        register_output(
            "/dart/drive_belt/right/control_torque", right_drive_belt_control_torque_, 0);

        register_output("/dart/trigger_servo/control_angle", trigger_control_angle);
    }

    void update() override {
        using namespace rmcs_msgs;

        if ((*switch_left_ == Switch::DOWN || *switch_left_ == Switch::UNKNOWN)
            && (*switch_right_ == Switch::DOWN || *switch_right_ == Switch::UNKNOWN)) {
            *launch_stage_ = DartLaunchStages::DISABLE;
            reset_all_controls();

        } else if (*switch_left_ == Switch::MIDDLE) {

            if (last_launch_stage_ == DartLaunchStages::DISABLE) {
                *launch_stage_ = DartLaunchStages::RESETTING;
            }

            if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::DOWN) {
                if (last_launch_stage_ == DartLaunchStages::INIT) {
                    *launch_stage_ = DartLaunchStages::LOADING;
                } else if (last_launch_stage_ == DartLaunchStages::READY) {
                    *launch_stage_ = DartLaunchStages::CANCEL;
                }
            } else if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::UP) {
                if (last_launch_stage_ == DartLaunchStages::READY) {
                    *launch_stage_ = DartLaunchStages::INIT;
                    trigger_lock_flag_ = false;
                } else {
                    RCLCPP_INFO(logger_, "Dart has't been loaded !");
                }
            }

            if (blocking_detection()) {
                if (last_launch_stage_ == DartLaunchStages::LOADING) {
                    *launch_stage_ = DartLaunchStages::RESETTING;
                    trigger_lock_flag_ = true;
                    dirve_belt_block_count_ = 0;

                } else if (last_launch_stage_ == DartLaunchStages::CANCEL) {
                    *launch_stage_ = DartLaunchStages::RESETTING;
                    trigger_lock_flag_ = false;
                    dirve_belt_block_count_ = 0;

                } else if (last_launch_stage_ == DartLaunchStages::RESETTING) {
                    *launch_stage_ =
                        trigger_lock_flag_ ? DartLaunchStages::READY : DartLaunchStages::INIT;
                    dirve_belt_block_count_ = 0;
                }
            }
            double control_velocity = 0;

            if (*launch_stage_ == rmcs_msgs::DartLaunchStages::RESETTING) {
                control_velocity = -dirve_belt_working_velocity_;
            } else if (
                *launch_stage_ == rmcs_msgs::DartLaunchStages::LOADING
                || *launch_stage_ == rmcs_msgs::DartLaunchStages::CANCEL) {
                control_velocity = dirve_belt_working_velocity_;
            } else {
                control_velocity = 0;
            }
            drive_belt_sync_control(control_velocity);
            RCLCPP_INFO(logger_, "%lf", control_velocity);
        }

        *trigger_control_angle = trigger_lock_flag_ ? launch_lock_angle_ : launch_trigger_angle_;

        last_switch_left_ = *switch_left_;
        last_switch_right_ = *switch_right_;
        last_launch_stage_ = *launch_stage_;
        // RCLCPP_INFO(logger_, "%lf | %lf", *right_drive_belt_velocity_,
        // *left_drive_belt_velocity_);
        // RCLCPP_INFO(
        //     logger_, "%d | %lf | %lf", static_cast<int>(*launch_stage_),
        //     *left_drive_belt_control_torque_, *right_drive_belt_control_torque_);
    }

private:
    void reset_all_controls() {
        *launch_stage_ = rmcs_msgs::DartLaunchStages::DISABLE;
        *left_drive_belt_control_torque_ = 0;
        *right_drive_belt_control_torque_ = 0;
    }

    void drive_belt_sync_control(double set_velocity) {
        if (set_velocity == 0) {
            *left_drive_belt_control_torque_ = 0;
            *right_drive_belt_control_torque_ = 0;
            return;
        }

        Eigen::Vector2d setpoint_error{
            set_velocity - *left_drive_belt_velocity_, set_velocity - *right_drive_belt_velocity_};

        Eigen::Vector2d relative_velocity{
            *left_drive_belt_velocity_ - *right_drive_belt_velocity_,
            *right_drive_belt_velocity_ - *left_drive_belt_velocity_};

        Eigen::Vector2d control_torques = setpoint_error - sync_coefficient_ * relative_velocity;

        *left_drive_belt_control_torque_ =
            std::clamp(control_torques[0], -max_control_torque_, max_control_torque_);
        *right_drive_belt_control_torque_ =
            std::clamp(control_torques[1], -max_control_torque_, max_control_torque_);
    }

    bool blocking_detection() {
        if ((abs(*left_drive_belt_velocity_) < 0.5 && abs(*left_drive_belt_control_torque_) > 0.5)
            || (abs(*right_drive_belt_velocity_) < 0.5
                && abs(*right_drive_belt_control_torque_) > 0.5)) {
            dirve_belt_block_count_++;
        }

        return dirve_belt_block_count_ > 1000 ? true : false;
    }

    int dirve_belt_block_count_ = 0;
    double dirve_belt_working_velocity_;

    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    rmcs_msgs::Switch last_switch_right_;
    rmcs_msgs::Switch last_switch_left_;

    OutputInterface<double> left_drive_belt_control_torque_;
    OutputInterface<double> right_drive_belt_control_torque_;
    InputInterface<double> left_drive_belt_velocity_;
    InputInterface<double> right_drive_belt_velocity_;

    double max_control_torque_;

    OutputInterface<rmcs_msgs::DartLaunchStages> launch_stage_;
    rmcs_msgs::DartLaunchStages last_launch_stage_ = rmcs_msgs::DartLaunchStages::DISABLE;

    uint16_t launch_lock_angle_;
    uint16_t launch_trigger_angle_;
    bool trigger_lock_flag_ = false;
    OutputInterface<uint16_t> trigger_control_angle;

    pid::MatrixPidCalculator<2> drive_belt_pid_calculator_;
    double sync_coefficient_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartLaunchController, rmcs_executor::Component)