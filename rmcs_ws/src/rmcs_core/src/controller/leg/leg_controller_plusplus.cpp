#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/trajectory.hpp"
#include "hardware/hsm/HSM_up_stairs.hpp"
#include "rmcs_msgs/arm_mode.hpp"
#include <array>
#include <cmath>
#include <cstdint>
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
        , initial_end_point_(get_parameter("initial_end_point").as_double_array())
        , press_end_point_(get_parameter("press_end_point").as_double_array())
        , lift_end_point_(get_parameter("lift_end_point").as_double_array())
        , press_and_lift_end_point_(get_parameter("press_and_lift_end_point").as_double_array())
        , press_again_end_point_(get_parameter("press_again_end_point").as_double_array())
        , k_(get_parameter("k").as_double_array())
        , b_(get_parameter("b").as_double_array()) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_); //
        register_input("/remote/mouse", mouse_);                   //
        register_input("/remote/keyboard", keyboard_);

        // register_input("/move_speed_limit", speed_limit_);
        register_input("/chassis/control_velocity", chassis_velocity_);
        register_output("/leg/enable_flag", is_leg_enable, false);
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

        register_output("/leg/lf_result", lf_result_, 0);
        register_output("/leg/lb_result", lb_result_, 0);
        register_output("/leg/rf_result", rf_result_, 0);
        register_output("/leg/rb_result", rb_result_, 0);

        register_input("/arm/mode", arm_mode);

        register_input("/arm/Joint1/theta", joint1_theta);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);

        //  register_input("/tof/distance",tof_distance_);

        std::array<double, 2> four_wheel_angle = leg_inverse_kinematic(
            forward_x_position_in_FourWheel_, wheel_distance - forward_x_position_in_FourWheel_,
            false, false);
        four_wheel_trajectory
            .set_end_point(
                {four_wheel_angle[0], four_wheel_angle[1], four_wheel_angle[1], four_wheel_angle[0],
                 0, 0})
            .set_total_step(2000.0);
        std::array<double, 2> six_wheel_angle = leg_inverse_kinematic(
            forward_x_position_in_SixWheel_, backward_x_position_in_SixWheel_, false, false);
        six_wheel_trajectory
            .set_end_point(
                {six_wheel_angle[0], six_wheel_angle[1], six_wheel_angle[1], six_wheel_angle[0], 0,
                 0})
            .set_total_step(500.0);
        up_stairs
            .bind_real_theta_and_speed(theta_lf, theta_lb, theta_rb, theta_rf, chassis_velocity_)
            .bind_k_and_b_parameter(k_, b_)
            .init_and_trajectory_set(
                initial_end_point_, press_end_point_, lift_end_point_, press_and_lift_end_point_,press_again_end_point_);

        // up_stairs_initial
        //     .set_end_point(
        //         {initial_end_point_[0], initial_end_point_[1], initial_end_point_[2],
        //          initial_end_point_[3], 0, 0})
        //     .set_total_step(1500);
        // up_stairs_leg_press.set_end_point({0.406814, 0.395241, 0.395241, 0.4066814, 0, 0});
        // up_stairs_leg_lift.set_end_point(
        //     {lift_end_point_[0], lift_end_point_[1], lift_end_point_[2], lift_end_point_[3], 0,
        //     0});
        // up_stairs_leg_press_and_lift.set_end_point(
        //     {press_and_lift_end_point_[0], press_and_lift_end_point_[1],
        //      press_and_lift_end_point_[2], press_and_lift_end_point_[3], 0, 0});

        // RCLCPP_INFO(this->get_logger(), " lf%f   lb%f",six_wheel_angle[0], six_wheel_angle[1]);
    }

    void update() override {
        RCLCPP_INFO(this->get_logger(), " rf%f   rb%f",*theta_rf, *theta_rb);
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if (!initial_check_done_) {
            *is_leg_enable = false;
            reset_motor();
            leg_mode = rmcs_msgs::LegMode::None;
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {

                initial_check_done_ = true;
            }
        } else {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                *is_leg_enable = false;
                reset_motor();
                leg_mode = rmcs_msgs::LegMode::None;
            } else {
                *is_leg_enable = true;
                Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
                // Eigen::Vector2d move_ = rotation * (*joystick_left_);
                // Eigen::Vector2d move_ = *joystick_left_;
                mode_selection();
                if (*arm_mode == rmcs_msgs::ArmMode::Auto_Spin) {
                    leg_mode = rmcs_msgs::LegMode::Four_Wheel;
                }
                omniwheel_control();
                leg_control();
                // RCLCPP_INFO(
                //   this->get_logger(), "%f %f %f %f", result[0], result[1], result[2], result[3]);

                last_arm_mode = *arm_mode;
                last_leg_mode = leg_mode;
                //                last_upstairs_mode = up_stairs_mode;
                {
                    *lf_result_ = (result)[0];
                    *lb_result_ = (result)[1];
                    *rf_result_ = (result)[2];
                    *rb_result_ = (result)[3];
                }
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
           // RCLCPP_INFO(this->get_logger(),"switch -> four wheel");
            leg_mode = rmcs_msgs::LegMode::Four_Wheel;

        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::DOWN) {
//RCLCPP_INFO(this->get_logger(),"switch -> six wheel");
            leg_mode = rmcs_msgs::LegMode::Six_Wheel;

        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {

            leg_mode = rmcs_msgs::LegMode::Four_Wheel;

        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            if (keyboard.v) {
                if (!keyboard.shift && !keyboard.ctrl) {

                   // RCLCPP_INFO(this->get_logger(),"V -> six wheel");
                    leg_mode = rmcs_msgs::LegMode::Six_Wheel;
                }
                if (keyboard.shift && !keyboard.ctrl) {
                  //  RCLCPP_INFO(this->get_logger(),"V -> four wheel");
                    leg_mode = rmcs_msgs::LegMode::Four_Wheel;
                }
            }
            if (keyboard.b) {
                leg_mode = rmcs_msgs::LegMode::Up_Stairs;
                // up_stairs_initial.reset();
                // up_stairs_initial.set_start_point(
                //     {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});
                // if (!keyboard.shift && !keyboard.ctrl) {
                //     up_stairs_mode = rmcs_msgs::UpStairsMode::Step_By_Two;
                // }
                // if (keyboard.shift && !keyboard.ctrl) {
                //     up_stairs_mode = rmcs_msgs::UpStairsMode::Step_By_Two;
                // }
                up_stairs.stop();
                up_stairs.start(UpStairsState::Initial);
            }
            if (keyboard.d) {
                leg_mode = rmcs_msgs::LegMode::Four_Wheel;
               // RCLCPP_INFO(this->get_logger(),"d -> four wheel");
            }

            if (last_arm_mode != *arm_mode) {

                if (*arm_mode == rmcs_msgs::ArmMode::Customer) {
                    leg_mode = rmcs_msgs::LegMode::Four_Wheel;
                } else {
                    leg_mode = rmcs_msgs::LegMode::Six_Wheel;
                }
            }

        } else {
            leg_mode = rmcs_msgs::LegMode::None;
        }
    }

    void reset_motor() {
        *omni_l_target_vel   = NAN;
        *omni_r_target_vel   = NAN;
        *leg_lf_target_theta = NAN;
        *leg_lb_target_theta = NAN;
        *leg_rb_target_theta = NAN;
        *leg_rf_target_theta = NAN;
    }

    void leg_control() {

        auto reset_leg_trajectory = [&] {
            if (last_leg_mode != leg_mode) {

                if (leg_mode == rmcs_msgs::LegMode::Four_Wheel) {
                    four_wheel_trajectory.reset();
                    // RCLCPP_INFO(this->get_logger(), "change and reset");
                    four_wheel_trajectory.set_start_point(
                        {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});
                    if (last_leg_mode == rmcs_msgs::LegMode::Up_Stairs) {
                        four_wheel_trajectory.set_total_step(1500);
                    }

                } else if (leg_mode == rmcs_msgs::LegMode::Six_Wheel) {
                    six_wheel_trajectory.reset();
                    six_wheel_trajectory.set_start_point(
                        {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});
                    if (last_leg_mode == rmcs_msgs::LegMode::Up_Stairs) {
                        six_wheel_trajectory.set_total_step(1500);
                    }

                } else if (leg_mode == rmcs_msgs::LegMode::Up_Stairs) {

                }
            }
        };
        reset_leg_trajectory();
        switch (leg_mode) {
        case rmcs_msgs::LegMode::Four_Wheel: four_wheel_controller(); break;
        case rmcs_msgs::LegMode::Six_Wheel: six_wheel_controller(); break;
        case rmcs_msgs::LegMode::Up_Stairs: up_stairs_controller(); break;
        case rmcs_msgs::LegMode::None: {
            (result)[0] = *theta_lf;
            (result)[1] = *theta_lb;
            (result)[2] = *theta_rb;
            (result)[3] = *theta_rf;
            break;
        }
        }
        leg_joint_controller((result)[0], (result)[1], (result)[2], (result)[3]);
    }

    void leg_joint_controller(double lf, double lb, double rb, double rf) {

        *leg_lf_target_theta = lf;
        *leg_rf_target_theta = rf;
        *leg_lb_target_theta = lb;
        *leg_rb_target_theta = rb;
    };
    void four_wheel_controller() { result = four_wheel_trajectory.trajectory(); }

    void six_wheel_controller() {
        if (!six_wheel_trajectory.get_complete()) {
            result = six_wheel_trajectory.trajectory();
        } else {
            result = six_wheel_trajectory.trajectory();
            if (*theta_lf < std::numbers::pi / 2.0 || *theta_rf < std::numbers::pi / 2.0) {
                (result)[0] = NAN;
                (result)[3] = NAN;
            }
        }
    }

    void up_stairs_controller() {

        if (keyboard_->shift) {
            up_stairs.processEvent(
                rmcs_core::hardware::hsm::up_stairs_hsm::events::go_to_TwoProcess);
        }

        if (keyboard_->q) {
            up_stairs.processEvent(rmcs_core::hardware::hsm::up_stairs_hsm::events::go_to_Lift);
        }


        if(keyboard_->ctrl){
            up_stairs.processEvent(rmcs_core::hardware::hsm::up_stairs_hsm::events::go_to_Press_Again);
        }




        up_stairs.processEvent(rmcs_core::hardware::hsm::up_stairs_hsm::events::tick);

        result = up_stairs.get_result();
            //RCLCPP_INFO(this->get_logger(), "当前期望角度 %f ",result[0]);
            
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

    void omniwheel_control() {
        if (leg_mode != rmcs_msgs::LegMode::Four_Wheel) {
            *omni_l_target_vel = (*chassis_velocity_)->x() / wheel_r;
            *omni_r_target_vel = (*chassis_velocity_)->x() / wheel_r;
        } else {
            *omni_l_target_vel = NAN;
            *omni_r_target_vel = NAN;
        }
    }

    static double normalizeAngle(double angle) {

        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }



    std::array<double, 6> result;
    const double forward_x_position_in_FourWheel_;
    const double forward_x_position_in_SixWheel_;
    const double backward_x_position_in_SixWheel_;
    const std::vector<double> initial_end_point_;
    const std::vector<double> press_end_point_;
    const std::vector<double> lift_end_point_;
    const std::vector<double> press_and_lift_end_point_;
    const std::vector<double> press_again_end_point_;
    const std::vector<double> k_;
    const std::vector<double> b_;

    InputInterface<rmcs_msgs::ArmMode> arm_mode;
    InputInterface<rmcs_msgs::ChassisMode> chassis_mode;
    rmcs_msgs::ArmMode last_arm_mode;

    static constexpr double wheel_r = 0.11;
    // static constexpr double v_reference = 1.5;
    // InputInterface<double> speed_limit_; // m/s
    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_velocity_;
    InputInterface<double> tof_distance_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> is_leg_enable;

    OutputInterface<double> omni_l_target_vel;
    OutputInterface<double> omni_r_target_vel;
    InputInterface<double> theta_lf;
    InputInterface<double> theta_lb;
    InputInterface<double> theta_rb;
    InputInterface<double> theta_rf;

    OutputInterface<double> leg_lf_target_theta;
    OutputInterface<double> leg_rf_target_theta;
    OutputInterface<double> leg_lb_target_theta;
    OutputInterface<double> leg_rb_target_theta;

    OutputInterface<double> lf_result_;
    OutputInterface<double> lb_result_;
    OutputInterface<double> rf_result_;
    OutputInterface<double> rb_result_;

    //    bool up_stairs_is_leg_press = false;
    //    bool up_stairs_is_leg_lift  = false;

    rmcs_msgs::LegMode leg_mode      = rmcs_msgs::LegMode::Six_Wheel;
    rmcs_msgs::LegMode last_leg_mode = rmcs_msgs::LegMode::None;
    //    rmcs_msgs::UpStairsMode last_upstairs_mode = rmcs_msgs::UpStairsMode::Step_By_Two;
    //    rmcs_msgs::UpStairsMode up_stairs_mode     = rmcs_msgs::UpStairsMode::Step_By_Two;
    InputInterface<double> chassis_big_yaw_angle;
    InputInterface<double> joint1_theta;

    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> four_wheel_trajectory;
    static constexpr double wheel_distance = 458.0f;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> six_wheel_trajectory;
    //  hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_initial;
    // hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_press;
    // hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_lift;
    //  hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
    //    up_stairs_leg_press_and_lift;
    bool initial_check_done_ = false;
    hardware::hsm::up_stairs_hsm::Auto_Leg_Up_Stairs up_stairs;
};

} // namespace rmcs_core::controller::leg
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::leg::LegController, rmcs_executor::Component)