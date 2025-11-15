#include "controller/pid/matrix_pid_calculator.hpp"
#include "rmcs_msgs/switch.hpp"
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

/*
    所有的电机运动方向均以令底盘升高为正方向
    暂时使用手动控制，遥控器键位不够用了，屏蔽自瞄和小陀螺模式
*/

namespace rmcs_core::controller::chassis {
class ChassisClimberController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisClimberController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , front_velocity_pid_calculator_(
              get_parameter("front_kp").as_double(), get_parameter("front_ki").as_double(),
              get_parameter("front_kd").as_double())
        , back_velocity_pid_calculator_(
              get_parameter("back_kp").as_double(), get_parameter("back_ki").as_double(),
              get_parameter("back_kd").as_double()) {

        track_velocity_max_ = get_parameter("front_climber_velocity").as_double();
        climber_back_control_velocity_abs_ = get_parameter("back_climber_velocity").as_double();

        register_output(
            "/chassis/climber/left_front_motor/control_torque", climber_front_left_control_torque_, nan_);
        register_output(
            "/chassis/climber/right_front_motor/control_torque", climber_front_right_control_torque_, nan_);
        register_output(
            "/chassis/climber/left_back_motor/control_torque", climber_back_left_control_torque_, nan_);
        register_output(
            "/chassis/climber/right_back_motor/control_torque", climber_back_right_control_torque_, nan_);

        register_input("/chassis/climber/left_front_motor/velocity", climber_front_left_velocity_);
        register_input("/chassis/climber/right_front_motor/velocity", climber_front_right_velocity_);
        register_input("/chassis/climber/left_back_motor/velocity", climber_back_left_velocity_);
        register_input("/chassis/climber/right_back_motor/velocity", climber_back_right_velocity_);

        register_input("/chassis/climber/left_back_motor/torque", climber_back_left_torque_);
        register_input("/chassis/climber/right_back_motor/torque", climber_back_right_torque_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/joystick/right", joystick_left_);
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else if (switch_left != Switch::DOWN) {

            if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::UP) {
                front_climber_enable_ = !front_climber_enable_;

            } else if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                back_climber_dir_ = -1 * back_climber_dir_;
                back_climber_block_count_ = 0;
            }

            double track_control_velocity =
                front_climber_enable_ ? joystick_left_->x() * track_velocity_max_ : nan_;
            double back_climber_control_velocioty;

            if (abs(*climber_back_left_torque_) > 0.1 && abs(*climber_back_right_torque_) > 0.1
                && abs(*climber_back_left_velocity_) < 0.1 && abs(*climber_back_right_velocity_) < 0.1) {
                back_climber_block_count_++;
            }

            if (back_climber_block_count_ >= 500) {
                back_climber_control_velocioty = 0;
            } else {
                back_climber_control_velocioty = climber_back_control_velocity_abs_ * back_climber_dir_;
            }

            front_climber_sync_control(track_control_velocity);
            back_climber_sync_control(back_climber_control_velocioty);
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;

        // RCLCPP_INFO(
        //     logger_, "control torque: %lf | %lf | velocity: %lf | %lf ",
        //     *climber_front_left_control_torque_, *climber_front_right_control_torque_,
        //     *climber_front_left_velocity_, *climber_front_right_velocity_);
        // RCLCPP_INFO(logger_, "torque: %lf | %lf", *climber_back_left_torque_, *climber_back_right_torque_);
    }

private:
    void reset_all_controls() {
        *climber_front_left_control_torque_ = 0;
        *climber_front_right_control_torque_ = 0;
        *climber_back_left_control_torque_ = 0;
        *climber_back_right_control_torque_ = 0;
        front_climber_enable_ = false;
    }

    void front_climber_sync_control(double setpoint) {
        Eigen::Vector2d setpoint_error{
            setpoint - *climber_front_left_velocity_, setpoint - *climber_front_right_velocity_};

        Eigen::Vector2d relative_velocity{
            *climber_front_left_velocity_ - *climber_front_right_velocity_,
            *climber_front_right_velocity_ - *climber_front_left_velocity_};

        Eigen::Vector2d control_error = setpoint_error - sync_coefficient_ * relative_velocity;

        auto control_torques = front_velocity_pid_calculator_.update(control_error);

        *climber_front_left_control_torque_ = control_torques[0];
        *climber_front_right_control_torque_ = control_torques[1];
    }

    void back_climber_sync_control(double setpoint) {
        Eigen::Vector2d setpoint_error{
            setpoint - *climber_back_left_velocity_, setpoint - *climber_back_right_velocity_};

        Eigen::Vector2d relative_velocity{
            *climber_back_left_velocity_ - *climber_back_right_velocity_,
            *climber_back_right_velocity_ - *climber_back_left_velocity_};

        Eigen::Vector2d control_torques = setpoint_error - sync_coefficient_ * relative_velocity;

        *climber_back_left_control_torque_ = control_torques[0];
        *climber_back_right_control_torque_ = control_torques[1];
    }

    int back_climber_block_count_;

    rclcpp::Logger logger_;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    double sync_coefficient_;

    bool front_climber_enable_ = false;
    double back_climber_dir_ = 1;

    double track_velocity_max_;
    double climber_back_control_velocity_abs_;

    OutputInterface<double> climber_front_left_control_torque_;
    OutputInterface<double> climber_front_right_control_torque_;
    OutputInterface<double> climber_back_left_control_torque_;
    OutputInterface<double> climber_back_right_control_torque_;

    InputInterface<double> climber_front_left_velocity_;
    InputInterface<double> climber_front_right_velocity_;
    InputInterface<double> climber_back_left_velocity_;
    InputInterface<double> climber_back_right_velocity_;

    InputInterface<double> climber_back_left_torque_;
    InputInterface<double> climber_back_right_torque_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> joystick_left_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

    pid::MatrixPidCalculator<2> front_velocity_pid_calculator_, back_velocity_pid_calculator_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisClimberController, rmcs_executor::Component)