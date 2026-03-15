#include "controller/arm/trajectory.hpp"
#include "controller/leg/hsm/up_stairs_interface.hpp"
#include "rmcs_msgs/arm_mode.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/leg_mode.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/up_stairs_mode.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

namespace rmcs_core::controller::leg {

class LegController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LegController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , forward_x_position_in_FourWheel_(
              get_parameter("forward_x_position_in_FourWheel").as_double())
        , forward_x_position_in_SixWheel_(
              get_parameter("forward_x_position_in_SixWheel").as_double())
        , backward_x_position_in_SixWheel_(
              get_parameter("backward_x_position_in_SixWheel").as_double())
        , initial_parameter_(get_parameter("initial_parameter").as_double_array())
        , press_parameter_(get_parameter("press_parameter").as_double_array())
        , lift_parameter_(get_parameter("lift_parameter").as_double_array())
        , lift_and_initial_parameter_(get_parameter("lift_and_initial_parameter").as_double_array())
        , initial_again_parameter_(get_parameter("initial_again_parameter").as_double_array())
        , press_again_parameter_(get_parameter("press_again_parameter").as_double_array())
        , lift_again_parameter_(get_parameter("lift_again_parameter").as_double_array())
        , up_stairs{*this,this->get_logger()} {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_); //
        register_input("/remote/mouse", mouse_);                   //
        register_input("/remote/keyboard", keyboard_);

        register_input("/chassis/control_velocity", chassis_velocity_);
        register_output("/leg/omni/l/target_vel", omni_l_target_vel, NAN);
        register_output("/leg/omni/r/target_vel", omni_r_target_vel, NAN);

        register_input("/leg/encoder/lf/angle", theta_lf);
        register_input("/leg/encoder/lb/angle", theta_lb);
        register_input("/leg/encoder/rb/angle", theta_rb);
        register_input("/leg/encoder/rf/angle", theta_rf);

        register_output("/leg/joint/lf/target_theta", leg_lf_target_theta, NAN);
        register_output("/leg/joint/rf/target_theta", leg_rf_target_theta, NAN);
        register_output("/leg/joint/lb/target_theta", leg_lb_target_theta, NAN);
        register_output("/leg/joint/rb/target_theta", leg_rb_target_theta, NAN);

        register_input("/arm/mode", arm_mode);

        register_input("/arm/joint_1/theta", joint1_theta);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);

        //  register_input("/tof/distance",tof_distance_);

        std::array<double, 2> four_wheel_angle = leg_inverse_kinematic(
            forward_x_position_in_FourWheel_, wheel_distance - forward_x_position_in_FourWheel_,
            false, false);
        four_wheel_trajectory
            .set_end_point(
                std::vector<double>{
                    four_wheel_angle[0], four_wheel_angle[1], four_wheel_angle[1],
                    four_wheel_angle[0]})
            .set_total_step(1000);
        std::array<double, 2> six_wheel_angle = leg_inverse_kinematic(
            forward_x_position_in_SixWheel_, backward_x_position_in_SixWheel_, false, false);
        six_wheel_trajectory
            .set_end_point(
                std::vector<double>{
                    six_wheel_angle[0], six_wheel_angle[1], six_wheel_angle[1], six_wheel_angle[0]})
            .set_total_step(500);
        down_stairs_trajectory
            .set_end_point(std::vector<double>{1.151109, 1.694066, 1.694066, 1.151109})
            .set_total_step(800);
        up_stairs.load(
            initial_parameter_,
            press_parameter_,
            lift_parameter_,
            lift_and_initial_parameter_,
            initial_again_parameter_,
            press_again_parameter_,
            lift_again_parameter_);
    }

    void update() override {
        // RCLCPP_INFO(this->get_logger(), " %x",*arm_mode);
        auto switch_right               = *switch_right_;
        auto switch_left                = *switch_left_;
        auto mouse                      = *mouse_;
        auto keyboard                   = *keyboard_;
        static bool initial_check_done_ = false;
        using namespace rmcs_msgs;
        if (!initial_check_done_) {
            reset_motor();
            leg_mode = rmcs_msgs::LegMode::None;
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                reset_motor();
                initial_check_done_ = true;
                leg_mode            = rmcs_msgs::LegMode::None;
            }
        } else {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_motor();
                leg_mode = rmcs_msgs::LegMode::None;
            } else {
                mode_selection();
                omniwheel_control();
                leg_control();
                last_arm_mode = *arm_mode;
                last_leg_mode = leg_mode;
            }
        }
    }

private:
    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;

        if (switch_left == Switch::MIDDLE && switch_right == Switch::MIDDLE) {
            leg_mode = rmcs_msgs::LegMode::Four_Wheel;
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::DOWN) {

            leg_mode = rmcs_msgs::LegMode::Six_Wheel;

        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {

            leg_mode = rmcs_msgs::LegMode::Four_Wheel;

        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            if (keyboard.v) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    leg_mode = rmcs_msgs::LegMode::Six_Wheel;
                }
                if (keyboard.shift && !keyboard.ctrl) {
                    leg_mode = rmcs_msgs::LegMode::Four_Wheel;
                }
            }
            if (keyboard.s) {
                leg_mode = rmcs_msgs::LegMode::Four_Wheel;
            }
            if (last_arm_mode != *arm_mode) {

                switch (*arm_mode) {
                case rmcs_msgs::ArmMode::Custome:{
                    leg_mode = rmcs_msgs::LegMode::Four_Wheel;
                    break;
                }
                case rmcs_msgs::ArmMode::Auto_Up_One_Stairs: {
                    leg_mode = rmcs_msgs::LegMode::Up_One_Stairs;
                    up_stairs.stop();
                    up_stairs.Set_One_Stairs();
                    up_stairs.start();
                    break;
                };
                case rmcs_msgs::ArmMode::Auto_Up_Two_Stairs: {
                    leg_mode = rmcs_msgs::LegMode::Up_Two_Stairs;
                    up_stairs.stop();
                    up_stairs.Set_Two_Stairs_Lift();
                    up_stairs.start();
                    break;
                };
                case rmcs_msgs::ArmMode::Auto_Down_Stairs: {
                    leg_mode = rmcs_msgs::LegMode::Down_Stairs;
                    break;
                }
                default: {
                    leg_mode = rmcs_msgs::LegMode::Six_Wheel;
                    break;
                }
                }
            }
        }
    }

    void leg_control() {

        if (last_leg_mode != leg_mode) {

            if (leg_mode == rmcs_msgs::LegMode::Four_Wheel) {
                four_wheel_trajectory.reset();
                four_wheel_trajectory.set_start_point(
                    std::vector<double>{*theta_lf, *theta_lb, *theta_rb, *theta_rf});
                if (last_leg_mode == rmcs_msgs::LegMode::Up_Two_Stairs
                    || last_leg_mode == rmcs_msgs::LegMode::Down_Stairs
                    || last_leg_mode == rmcs_msgs::LegMode::Up_One_Stairs) {
                    four_wheel_trajectory.set_total_step(1500);
                }
            } else if (leg_mode == rmcs_msgs::LegMode::Six_Wheel) {
                six_wheel_trajectory.reset();
                six_wheel_trajectory.set_start_point(
                    std::vector<double>{*theta_lf, *theta_lb, *theta_rb, *theta_rf});
                if (last_leg_mode == rmcs_msgs::LegMode::Up_One_Stairs
                    || last_leg_mode == rmcs_msgs::LegMode::Down_Stairs
                    || last_leg_mode == rmcs_msgs::LegMode::Up_Two_Stairs) {
                    six_wheel_trajectory.set_total_step(1500);
                }

            } else if (leg_mode == rmcs_msgs::LegMode::Down_Stairs) {
                down_stairs_trajectory.reset();
                down_stairs_trajectory.set_start_point(
                    std::vector<double>{*theta_lf, *theta_lb, *theta_rb, *theta_rf});
            }
        }
        switch (leg_mode) {
        case rmcs_msgs::LegMode::Four_Wheel: execute_joint_trajectory(four_wheel_trajectory); break;
        case rmcs_msgs::LegMode::Six_Wheel: {
            auto joints = six_wheel_trajectory.trajectory();
            if (six_wheel_trajectory.get_complete()) {
                joints[0] = NAN;
                joints[3] = NAN;
            }
            leg_joint_controller(joints[0], joints[1], joints[2], joints[3]);
            break;
        }
        case rmcs_msgs::LegMode::Down_Stairs:
            execute_joint_trajectory(down_stairs_trajectory);
            break;
        case rmcs_msgs::LegMode::Up_One_Stairs: // to execute_up_stairs()
        case rmcs_msgs::LegMode::Up_Two_Stairs: execute_up_stairs(); break;
        case rmcs_msgs::LegMode::None:
            leg_joint_controller(*theta_lf, *theta_lb, *theta_rb, *theta_rf);
            break;
        }
    }

    void execute_joint_trajectory(
        rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>&
            trajectory) {
        const auto joints = trajectory.trajectory();
        leg_joint_controller(joints[0], joints[1], joints[2], joints[3]);
    }
    void execute_up_stairs() {
        if (keyboard_->ctrl) {
            (void)up_stairs.resume();
        }

        up_stairs.tick();
        const auto& joints = up_stairs.get_result();
        leg_joint_controller(joints[0], joints[1], joints[2], joints[3]);
    }

    void omniwheel_control() {
        Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
        if (leg_mode != rmcs_msgs::LegMode::Four_Wheel) {
            Eigen::Vector2d velocity_xy((*chassis_velocity_)->x(), (*chassis_velocity_)->y());
            //  Eigen::Vector2d rotated_velocity = rotation * velocity_xy;
            Eigen::Vector2d rotated_velocity = velocity_xy;

            *omni_l_target_vel = rotated_velocity.x() / wheel_r;
            *omni_r_target_vel = rotated_velocity.x() / wheel_r;
        } else {
            *omni_l_target_vel = NAN;
            *omni_r_target_vel = NAN;
        }
    }
    void leg_joint_controller(double lf, double lb, double rb, double rf) {

        *leg_lf_target_theta = lf;
        *leg_rf_target_theta = rf;
        *leg_lb_target_theta = lb;
        *leg_rb_target_theta = rb;
    };
    void reset_motor() {
        *omni_l_target_vel   = NAN;
        *omni_r_target_vel   = NAN;
        *leg_lf_target_theta = NAN;
        *leg_lb_target_theta = NAN;
        *leg_rb_target_theta = NAN;
        *leg_rf_target_theta = NAN;
    }
    static std::array<double, 2> leg_inverse_kinematic(
        double f_x, double b_x, bool is_front_ecd_obtuse, bool is_back_ecd_obtuse) {
        constexpr double link1 = 240.0f, link2 = 120.0f, link3 = 160.0f;
        constexpr double link_angle = 5 * std::numbers::pi / 6.0;
        double theta_f, theta_b;
        theta_b        = !is_back_ecd_obtuse ? asin(b_x / link1) : (M_PI - asin(b_x / link1));
        double x_link2 = link2 * sin(link_angle - theta_b);
        theta_f        = !is_front_ecd_obtuse ? asin((f_x - x_link2) / link3)
                                              : (M_PI - asin((f_x - x_link2) / link3));

        return {theta_f, theta_b};
    }
    const double forward_x_position_in_FourWheel_;
    const double forward_x_position_in_SixWheel_;
    const double backward_x_position_in_SixWheel_;
    const std::vector<double> initial_parameter_;
    const std::vector<double> press_parameter_;
    const std::vector<double> lift_parameter_;
    const std::vector<double> lift_and_initial_parameter_;
    const std::vector<double> initial_again_parameter_;
    const std::vector<double> press_again_parameter_;
    const std::vector<double> lift_again_parameter_;

    InputInterface<rmcs_msgs::ArmMode> arm_mode;
    InputInterface<rmcs_msgs::ChassisMode> chassis_mode;
    rmcs_msgs::ArmMode last_arm_mode = rmcs_msgs::ArmMode::None;
    rmcs_msgs::LegMode leg_mode{rmcs_msgs::LegMode::None};
    rmcs_msgs::LegMode last_leg_mode{rmcs_msgs::LegMode::None};

    static constexpr double wheel_distance = 458.0f;
    static constexpr double wheel_r        = 0.11;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_velocity_;
    InputInterface<double> tof_distance_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    InputInterface<double> theta_lf;
    InputInterface<double> theta_lb;
    InputInterface<double> theta_rb;
    InputInterface<double> theta_rf;

    OutputInterface<double> omni_l_target_vel;
    OutputInterface<double> omni_r_target_vel;
    OutputInterface<double> leg_lf_target_theta;
    OutputInterface<double> leg_rf_target_theta;
    OutputInterface<double> leg_lb_target_theta;
    OutputInterface<double> leg_rb_target_theta;

    InputInterface<double> chassis_big_yaw_angle;
    InputInterface<double> joint1_theta;

    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        four_wheel_trajectory{4};

    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        six_wheel_trajectory{4};

    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        down_stairs_trajectory{4};

    hsm::up_stairs::Auto_Leg_Up_Stairs up_stairs;
};
} // namespace rmcs_core::controller::leg
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::leg::LegController, rmcs_executor::Component)
