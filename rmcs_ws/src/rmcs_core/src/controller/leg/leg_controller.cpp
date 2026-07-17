#include "controller/arm/trajectory.hpp"
#include "controller/leg/hsm/up_stairs_interface.hpp"
#include "controller/leg/leg_inverse_kinematic.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "rmcs_msgs/arm_mode.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <map>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/leg_mode.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/up_stairs_mode.hpp>
#include <rmcs_utility/normalize_angle.hpp>
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
              rclcpp::NodeOptions{
    }
                  .automatically_declare_parameters_from_overrides(true))
        , leg_back_command_(create_partner_component<LegBackCommand>("leg_back_command", *this))
        , forward_x_position_in_FourWheel_(
              get_parameter("forward_x_position_in_FourWheel").as_double())
        , forward_x_position_in_SixWheel_(
              get_parameter("forward_x_position_in_SixWheel").as_double())
        , backward_x_position_in_SixWheel_(
              get_parameter("backward_x_position_in_SixWheel").as_double())
        , lf_angle_pid_controller_(400.0, 0.0, 6.5)
        , rf_angle_pid_controller_(500.0, 0.0, 6.5)
        , lf_velocity_pid_controller_(0.8, 0.0, 0.001)
        , rf_velocity_pid_controller_(1.0, 0.0, 0.001)
        , up_stairs{
              {*this, "up_one_stairs", {"initial", "press", "wait", "lift"}},
              {*this,
               "up_two_stairs",
               {"initial", "press", "lift", "initial_again", "press_again", "lift_again"}}} {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/chassis/control_velocity", chassis_velocity_);
        register_output("/leg/omni/l/target_vel", omni_l_target_vel, NAN);
        register_output("/leg/omni/r/target_vel", omni_r_target_vel, NAN);

        register_input("/leg/encoder/lf/angle", theta_lf);
        register_input("/leg/encoder/lb/angle", theta_lb);
        register_input("/leg/encoder/rb/angle", theta_rb);
        register_input("/leg/encoder/rf/angle", theta_rf);
        register_input("/leg/joint/lf/velocity", leg_lf_joint_velocity_, NAN);
        register_output("/leg/joint/lf/control_torque", leg_lf_joint_control_torque_);
        register_input("/leg/joint/rf/velocity", leg_rf_joint_velocity_, NAN);

        register_output("/leg/joint/rf/control_torque", leg_rf_joint_control_torque_);
        register_output("/leg/joint/lb/target_theta", leg_lb_target_theta, NAN);
        register_output("/leg/joint/rb/target_theta", leg_rb_target_theta, NAN);

        register_input("/arm/mode", arm_mode);

        register_input("/arm/joint_1/theta", joint1_theta);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);

        register_input("/leg/tof/distance", tof_distance_);

        register_input("/leg/omni/l/velocity", omni_l_velocity, NAN);
        register_input("/leg/omni/l/torque", omni_l_torque, NAN);

        register_input("pitch_imu_velocity", pitch_imu_velocity_, NAN);
        register_input("pitch_imu_angle", pitch_imu_angle_, NAN);
        register_input("roll_imu_velocity", roll_imu_velocity_, NAN);
        register_input("roll_imu_angle", roll_imu_angle_, NAN);
        register_output("/leg/pitch_imu_angle_offsetted", pitch_imu_angle_offsetted_, NAN);

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
            .set_end_point(std::vector<double>{1.109164, 1.55792, 1.55792, 1.109164})
            .set_total_step(800);

        up_stairs[0].set_layer_connections("initial", [this]() {
            if (keyboard_->ctrl) {
                return true;
            };
            if ((*tof_distance_) <= this->get_parameter("tof_b").as_double()) {
                return true;
            }
            return false;
        });

        up_stairs[1].set_layer_connections("initial", [this]() {
            if (keyboard_->ctrl) {
                return true;
            };
            if ((*tof_distance_) <= this->get_parameter("tof_b").as_double()) {
                return true;
            };

            return false;
        });
    }

    void update() override {
        auto switch_right               = *switch_right_;
        auto switch_left                = *switch_left_;
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
                update_pitch_imu_angle_offset();

                last_arm_mode = *arm_mode;
                last_leg_mode = leg_mode;
            }
        }
    }

private:
    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
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
            if (last_arm_mode != *arm_mode) {
                switch (*arm_mode) {
                case rmcs_msgs::ArmMode::Custome:
                case rmcs_msgs::ArmMode::Auto_Spin: {
                    leg_mode = rmcs_msgs::LegMode::Four_Wheel;
                    break;
                }
                case rmcs_msgs::ArmMode::Auto_Up_One_Stairs: {
                    leg_mode = rmcs_msgs::LegMode::Up_One_Stairs;
                    up_stairs[0].start();
                    break;
                };
                case rmcs_msgs::ArmMode::Auto_Up_Two_Stairs: {
                    leg_mode = rmcs_msgs::LegMode::Up_Two_Stairs;
                    up_stairs[1].start();
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
                down_stairs_front_legs_released_ = false;
                first_into_downstairs            = true;
                down_stairs_trajectory.reset();
                down_stairs_trajectory.set_start_point(
                    std::vector<double>{*theta_lf, *theta_lb, *theta_rb, *theta_rf});
            }
        }

        switch (leg_mode) {
        case rmcs_msgs::LegMode::Four_Wheel: {
            const auto joints = four_wheel_trajectory.trajectory();
            set_leg_joint_theta(joints[0], joints[1], joints[2], joints[3]);
            break;
        }
        case rmcs_msgs::LegMode::Six_Wheel: {
            auto joints = six_wheel_trajectory.trajectory();
            if (six_wheel_trajectory.get_complete()) {
                joints[0] = NAN;
                joints[3] = NAN;
            }
            set_leg_joint_theta(joints[0], joints[1], joints[2], joints[3]);
            break;
        }
        case rmcs_msgs::LegMode::Down_Stairs: execute_down_stairs(); break;
        case rmcs_msgs::LegMode::Up_One_Stairs: {
            up_stairs[0].tick();
            const auto& joints = up_stairs[0].get_result();
            set_leg_joint_theta(joints[0], joints[1], joints[2], joints[3]);
            break;
        }
        case rmcs_msgs::LegMode::Up_Two_Stairs: {
            up_stairs[1].tick();
            const auto& joints = up_stairs[1].get_result();
            set_leg_joint_theta(joints[0], joints[1], joints[2], joints[3]);
            break;
        }
        case rmcs_msgs::LegMode::None:
            set_leg_joint_theta(*theta_lf, *theta_lb, *theta_rb, *theta_rf);
            break;
        }
    }

    void execute_down_stairs() {
        if (first_into_downstairs && !down_stairs_trajectory.get_complete()) {
            const auto joints = down_stairs_trajectory.trajectory();
            set_leg_joint_theta(joints[0], joints[1], joints[2], joints[3]);
            if (down_stairs_trajectory.get_complete())
                first_into_downstairs = false;
        } else {
            const double pitch_offset = *pitch_imu_angle_offsetted_;
            if (down_stairs_front_legs_released_ && std::isfinite(pitch_offset)
                && pitch_offset < -0.1) {
                down_stairs_trajectory.reset();
                down_stairs_trajectory.set_start_point(
                    std::vector<double>{*theta_lf, *theta_lb, *theta_rb, *theta_rf});
            }
            auto joints = down_stairs_trajectory.trajectory();

            down_stairs_front_legs_released_ = false;
            if (std::isfinite(pitch_offset) && pitch_offset < 0.35 && pitch_offset > -0.3) {
                joints[0]                        = NAN;
                joints[3]                        = NAN;
                down_stairs_front_legs_released_ = true;
            }
            set_leg_joint_theta(joints[0], joints[1], joints[2], joints[3]);
        }
    }
    void omniwheel_control() {
        Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);

        if (leg_mode == rmcs_msgs::LegMode::Four_Wheel
            || up_stairs[0].get_current_layer_id() == "press"
            || up_stairs[1].get_current_layer_id() == "press"
            || up_stairs[1].get_current_layer_id() == "press_again") {

            *omni_l_target_vel = NAN;
            *omni_r_target_vel = NAN;
        } else {
            Eigen::Vector2d velocity_xy((*chassis_velocity_)->x(), (*chassis_velocity_)->y());
            Eigen::Vector2d rotated_velocity = velocity_xy;

            *omni_l_target_vel = rotated_velocity.x() / wheel_r;
            *omni_r_target_vel = rotated_velocity.x() / wheel_r;
        }
    }
    void set_leg_joint_theta(double lf, double lb, double rb, double rf) {
        *leg_lb_target_theta = lb;
        *leg_rb_target_theta = rb;

        if (leg_mode == rmcs_msgs::LegMode::Down_Stairs) {
            lf_angle_pid_controller_.kp    = 260.0;
            lf_angle_pid_controller_.ki    = 0.0;
            lf_angle_pid_controller_.kd    = 3.0;
            rf_angle_pid_controller_.kp    = 260.0;
            rf_angle_pid_controller_.ki    = 0.0;
            rf_angle_pid_controller_.kd    = 3.0;
            lf_velocity_pid_controller_.kp = 0.6;
            lf_velocity_pid_controller_.ki = 0.0;
            lf_velocity_pid_controller_.kd = 0.0;
            rf_velocity_pid_controller_.kp = 0.6;
            rf_velocity_pid_controller_.ki = 0.0;
            rf_velocity_pid_controller_.kd = 0.0;
        } else {
            lf_angle_pid_controller_.kp    = 400.0;
            lf_angle_pid_controller_.ki    = 0.0;
            lf_angle_pid_controller_.kd    = 6.5;
            rf_angle_pid_controller_.kp    = 500.0;
            rf_angle_pid_controller_.ki    = 0.0;
            rf_angle_pid_controller_.kd    = 6.5;
            lf_velocity_pid_controller_.kp = 0.8;
            lf_velocity_pid_controller_.ki = 0.0;
            lf_velocity_pid_controller_.kd = 0.001;
            rf_velocity_pid_controller_.kp = 1.0;
            rf_velocity_pid_controller_.ki = 0.0;
            rf_velocity_pid_controller_.kd = 0.001;
        }
        lf_velocity_pid_controller_.output_max = 27.0;
        rf_velocity_pid_controller_.output_max = 27.0;

        const double lf_error = rmcs_utility::normalize_angle(lf - *theta_lf);
        const double rf_error = rmcs_utility::normalize_angle(rf - *theta_rf);
        const double lf_control_velocity = lf_angle_pid_controller_.update(lf_error);
        const double rf_control_velocity = rf_angle_pid_controller_.update(rf_error);

        *leg_lf_joint_control_torque_ =
            lf_velocity_pid_controller_.update(lf_control_velocity - *leg_lf_joint_velocity_);
        *leg_rf_joint_control_torque_ =
            rf_velocity_pid_controller_.update(rf_control_velocity - *leg_rf_joint_velocity_);
    }
    void reset_motor() {
        *omni_l_target_vel            = NAN;
        *omni_r_target_vel            = NAN;
        *leg_lb_target_theta          = NAN;
        *leg_rb_target_theta          = NAN;
        *leg_lf_joint_control_torque_ = NAN;
        *leg_rf_joint_control_torque_ = NAN;
        lf_angle_pid_controller_.reset();
        rf_angle_pid_controller_.reset();
        lf_velocity_pid_controller_.reset();
        rf_velocity_pid_controller_.reset();
    }
    void update_pitch_imu_angle_offset() {
        const double raw_pitch = *pitch_imu_angle_;
        if (leg_mode != rmcs_msgs::LegMode::Four_Wheel) {
            if (std::isfinite(pitch_imu_angle_offset_value_) && std::isfinite(raw_pitch)) {
                *pitch_imu_angle_offsetted_ = raw_pitch - pitch_imu_angle_offset_value_;
            } else {
                *pitch_imu_angle_offsetted_ = raw_pitch;
            }
            return;
        }
        if (std::isfinite(raw_pitch) && std::abs(raw_pitch) > 1e-6) {
            pitch_imu_angle_offset_value_ = raw_pitch;
            *pitch_imu_angle_offsetted_   = raw_pitch - pitch_imu_angle_offset_value_;
        }
    }
    class LegBackCommand : public rmcs_executor::Component {
    public:
        explicit LegBackCommand(LegController& leg_controller)
            : leg_controller_(leg_controller) {
            register_output("/leg_back/up_stairs_step", up_stairs_layer, std::string{"none"});
        }
        void update() override {
            if (leg_controller_.leg_mode == rmcs_msgs::LegMode::Up_One_Stairs) {
                const auto current = leg_controller_.up_stairs[0].get_current_layer_id();
                *up_stairs_layer   = current.value_or("none");
            }
            if (leg_controller_.leg_mode == rmcs_msgs::LegMode::Up_Two_Stairs) {
                const auto current = leg_controller_.up_stairs[1].get_current_layer_id();
                *up_stairs_layer   = current.value_or("none");
            }
        }

    private:
        LegController& leg_controller_;
        OutputInterface<std::string> up_stairs_layer;
    };
    std::shared_ptr<LegBackCommand> leg_back_command_;
    const double forward_x_position_in_FourWheel_;
    const double forward_x_position_in_SixWheel_;
    const double backward_x_position_in_SixWheel_;

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

    OutputInterface<double> leg_lb_target_theta;
    OutputInterface<double> leg_rb_target_theta;

    InputInterface<double> leg_lf_joint_velocity_;
    InputInterface<double> leg_rf_joint_velocity_;
    OutputInterface<double> leg_lf_joint_control_torque_;
    OutputInterface<double> leg_rf_joint_control_torque_;

    InputInterface<double> chassis_big_yaw_angle;
    InputInterface<double> joint1_theta;

    InputInterface<double> omni_l_velocity;
    InputInterface<double> omni_l_torque;

    InputInterface<double> pitch_imu_velocity_;
    InputInterface<double> pitch_imu_angle_;
    InputInterface<double> roll_imu_velocity_;
    InputInterface<double> roll_imu_angle_;

    pid::PidCalculator lf_angle_pid_controller_;
    pid::PidCalculator rf_angle_pid_controller_;
    pid::PidCalculator lf_velocity_pid_controller_;
    pid::PidCalculator rf_velocity_pid_controller_;
    OutputInterface<double> pitch_imu_angle_offsetted_;
    double pitch_imu_angle_offset_value_{NAN};

    bool first_into_downstairs{false};
    bool down_stairs_front_legs_released_{false};

    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        four_wheel_trajectory{4};

    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        six_wheel_trajectory{4};

    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        down_stairs_trajectory{4};

    hsm::up_stairs::Auto_Leg_Up_Stairs up_stairs[2];
};
} // namespace rmcs_core::controller::leg
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::leg::LegController, rmcs_executor::Component)
