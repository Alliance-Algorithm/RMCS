#include "controller/pid/matrix_pid_calculator.hpp"
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_launch_status.hpp>
#include <rmcs_msgs/switch.hpp>

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
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , drive_belt_pid_calculator_(
              get_parameter("b_kp").as_double(), get_parameter("b_ki").as_double(), get_parameter("b_kd").as_double()) {

        dirve_belt_working_velocity_ = get_parameter("belt_velocity").as_double();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/dart/drive_belt/left/velocity", left_drive_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_drive_belt_velocity_);

        register_output("/dart/drive_belt/left/control_torque", left_drive_belt_control_torque_, 0);
        register_output("/dart/drive_belt/right/control_torque", right_drive_belt_control_torque_, 0);
    }

    void update() override {
        using namespace rmcs_msgs;
        if ((*switch_left_ == Switch::DOWN || *switch_left_ == Switch::UNKNOWN)
            && (*switch_right_ == Switch::DOWN || *switch_right_ == Switch::UNKNOWN)) {
            reset_all_controls();

        } else if (*switch_left_ == Switch::MIDDLE) {
            if (dart_launch_status_ == rmcs_msgs::DartLaunchStatus::DISABLE) {
                dart_launch_status_ = rmcs_msgs::DartLaunchStatus::INIT;
                return;
            }
            if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::DOWN) {
                dirve_belt_block_count_ = 0;
                drive_belt_control_direction_ = -1;
                if (dart_launch_status_ == DartLaunchStatus::INIT) {
                    dart_launch_status_ = DartLaunchStatus::LOADING;
                }
            }

            if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::UP) {
                if (dart_launch_status_ == DartLaunchStatus::LAUNCH_READY) {
                    dart_launch_status_ = rmcs_msgs::DartLaunchStatus::INIT;
                    trigger_enable_ = true;
                } else {
                    RCLCPP_INFO(logger_, "Dart Hasn't Been Loaded!");
                }
            }

            launch_load_control();
            drive_belt_sync_control();
        }

        last_switch_left_ = *switch_left_;
        last_switch_right_ = *switch_right_;

        {
            if (log_count_++ > 500) {
                RCLCPP_INFO(logger_, "dart status: %d", static_cast<int>(dart_launch_status_));
                log_count_ = 0;
            }
        } // DEBUG
    }
    double log_count_;

private:
    void reset_all_controls() {
        dart_launch_status_ = rmcs_msgs::DartLaunchStatus::DISABLE;
        *left_drive_belt_control_torque_ = 0;
        *right_drive_belt_control_torque_ = 0;
        drive_belt_control_direction_ = 0;
    }

    void launch_load_control() {
        using namespace rmcs_msgs;
        dirve_belt_block_count_++;
        if (dirve_belt_block_count_ >= 500) {
            if (drive_belt_control_direction_ == -1) {
                drive_belt_control_direction_ = 1;
                if (dart_launch_status_ == DartLaunchStatus::LAUNCH_READY) {
                    trigger_enable_ = true;
                    dart_launch_status_ = rmcs_msgs::DartLaunchStatus::INIT;
                }
            } else if (drive_belt_control_direction_ == 1) {
                drive_belt_control_direction_ = 0;
                if (dart_launch_status_ == DartLaunchStatus::LOADING) {
                    dart_launch_status_ = DartLaunchStatus::LAUNCH_READY;
                }
            }
        }
    }

    void drive_belt_sync_control() {
        auto setpoint = drive_belt_control_direction_ * dirve_belt_working_velocity_;
        Eigen::Vector2d setpoint_error{setpoint - *left_drive_belt_velocity_, setpoint - *right_drive_belt_velocity_};

        Eigen::Vector2d relative_velocity{
            *left_drive_belt_velocity_ - *right_drive_belt_velocity_,
            *right_drive_belt_velocity_ - *left_drive_belt_velocity_};

        Eigen::Vector2d control_torques = setpoint_error - sync_coefficient_ * relative_velocity;

        *left_drive_belt_control_torque_ = control_torques[0];
        *right_drive_belt_control_torque_ = control_torques[1];
    }

    void trigger_control() {}

    int dirve_belt_block_count_ = 0;
    double drive_belt_control_direction_ = 0;
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

    rmcs_msgs::DartLaunchStatus dart_launch_status_;
    bool trigger_enable_ = false;

    pid::MatrixPidCalculator<2> drive_belt_pid_calculator_;
    double sync_coefficient_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartLaunchController, rmcs_executor::Component)