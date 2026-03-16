#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/wheel_leg_mode.hpp>
#include <rmcs_msgs/wheel_leg_state.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

#include "controller/lqr/lqr_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "desire_state_solver.hpp"
#include "filter/kalman_filter.hpp"
#include "filter/low_pass_filter.hpp"
#include "rmcs_executor/component.hpp"

#include "vmc_solver.hpp"

namespace rmcs_core::controller::chassis {
class WheelLegController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , body_mess_(13.3)
        , leg_mess_(0.629)
        , wheel_mess_(0.459)
        , wheel_radius_(0.06)
        , wheel_distance_(0.2192)
        , centroid_position_coefficient_(0.2945)
        , left_leg_vmc_solver_(0.21, 0.25, 0.0)
        , right_leg_vmc_solver_(0.21, 0.25, 0.0)
        , velocity_kalman_filter_(A_, H_, Q_, R_)
        , chassis_gyro_velocity_filter_(5.0, 1000.0)
        , desire_state_solver_(0.15)
        , roll_angle_pid_calculator_(1.0, 0.0, 1.0)
        , leg_length_pid_calculator_(90.0, 0.0, 0.0)
        , balanceless_left_wheel_pid_calculator_(3.0, 0.0, 0.0)
        , balanceless_right_wheel_pid_calculator_(3.0, 0.0, 0.0)
        , rescue_velocity_pid_calculator_(3.0, 0.0, 0.0) {

        register_input("/chassis/left_front_hip/max_torque", hip_motor_max_control_torque_);
        register_input("/chassis/left_wheel/max_torque", wheel_motor_max_control_torque_);

        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/control_mode", mode_);

        register_input("/chassis/control_angle", chassis_control_angle_);

        register_input("/chassis/status/balanceless", is_balanceless_);
        register_input("/chassis/status/rescue_tip_over", is_rescue_tip_over_);

        register_input("/chassis/status/leg_extended", is_leg_extended_);
        register_input("/chassis/status/jump", is_jump_active_);
        register_input("/chassis/status/climb", is_climb_active_);

        register_input("/chassis/left_front_hip/angle", left_front_hip_angle_);
        register_input("/chassis/left_back_hip/angle", left_back_hip_angle_);
        register_input("/chassis/right_front_hip/angle", right_front_hip_angle_);
        register_input("/chassis/right_back_hip/angle", right_back_hip_angle_);

        register_input("/chassis/left_front_hip/velocity", left_front_hip_velocity_);
        register_input("/chassis/left_back_hip/velocity", left_back_hip_velocity_);
        register_input("/chassis/right_front_hip/velocity", right_front_hip_velocity_);
        register_input("/chassis/right_back_hip/velocity", right_back_hip_velocity_);

        register_input("/chassis/left_front_hip/torque", left_front_hip_torque_);
        register_input("/chassis/left_back_hip/torque", left_back_hip_torque_);
        register_input("/chassis/right_front_hip/torque", right_front_hip_torque_);
        register_input("/chassis/right_back_hip/torque", right_back_hip_torque_);

        register_input("/chassis/left_wheel/velocity", left_wheel_velocity_);
        register_input("/chassis/right_wheel/velocity", right_wheel_velocity_);

        register_input("/chassis/x_axis/acceleration", chassis_x_axis_acceleration_imu_);
        register_input("/chassis/z_axis/acceleration", chassis_z_axis_acceleration_imu_);

        register_input("/chassis/yaw/velocity", chassis_yaw_velocity_imu_);
        register_input("/chassis/pitch/velocity", chassis_pitch_velocity_imu_);
        register_input("/chassis/roll/velocity", chassis_roll_velocity_imu_);

        register_input("/chassis/yaw/angle", chassis_yaw_angle_imu_);
        register_input("/chassis/pitch/angle", chassis_pitch_angle_imu_);
        register_input("/chassis/roll/angle", chassis_roll_angle_imu_);

        register_output("/chassis/left_front_hip/control_torque", left_front_hip_control_torque_);
        register_output("/chassis/left_back_hip/control_torque", left_back_hip_control_torque_);
        register_output("/chassis/right_front_hip/control_torque", right_front_hip_control_torque_);
        register_output("/chassis/right_back_hip/control_torque", right_back_hip_control_torque_);

        register_output("/chassis/left_front_hip/control_angle", left_front_hip_control_angle_);
        register_output("/chassis/left_back_hip/control_angle", left_back_hip_control_angle_);
        register_output("/chassis/right_front_hip/control_angle", right_front_hip_control_angle_);
        register_output("/chassis/right_back_hip/control_angle", right_back_hip_control_angle_);

        register_output("/chassis/left_wheel/control_torque", left_wheel_control_torque_);
        register_output("/chassis/right_wheel/control_torque", right_wheel_control_torque_);

        velocity_kalman_filter_.set_process_noise_transition(W_);
    }

    ~WheelLegController() override { reset_all_controls(); }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(), "Max control torque of hip motor: %.f, wheel motor: %.f",
            *hip_motor_max_control_torque_, *wheel_motor_max_control_torque_);
    }

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0])) {
            reset_all_controls();
            return;
        }

        // Observer
        auto hip_angles = calculate_hips_angles();
        auto wheel_velocities = calculate_wheel_velocities();

        auto leg_posture = calculate_leg_posture(hip_angles);
        auto distance = calculate_translational_distance(leg_posture, wheel_velocities);

        // State: [s ds phi d_phi theta_l d_theta_l theta_r d_theta_r theta_b d_theta_b]
        auto measure_state = calculate_measure_state(distance, leg_posture);

        // Controller
        auto chassis_control_velocity = calculate_chassis_control_velocity();
        auto leg_forces = calculate_leg_force(leg_posture, measure_state);

        auto desire_state = calculate_desire_state(chassis_control_velocity, measure_state);
        auto control_torques = calculate_control_torques(desire_state, measure_state, leg_posture);

        // if (*is_balanceless_) {
        // update_balanceless_control_torques(chassis_control_velocity, wheel_velocities);
        //     return;
        // }

        // if (!stand_active_) {
        //     update_hip_stand_control_torques(leg_posture);
        //     return;
        // }

        // if (*is_rescue_tip_over_) {
        //     update_hip_rescue_tip_over_control_torques();
        // }

        update_hip_and_wheel_torques(leg_forces, control_torques);

        // *left_front_hip_control_torque_ = 0.0;
        // *left_back_hip_control_torque_ = 0.0;
        *right_front_hip_control_torque_ = 0.0;
        *right_back_hip_control_torque_ = 0.0;

        *left_wheel_control_torque_ = 0.0;
        *right_wheel_control_torque_ = 0.0;
    }

private:
    struct LegPosture {
        Eigen::Vector2d leg_length;
        Eigen::Vector2d tilt_angle;

        Eigen::Vector2d diff_leg_length;
        Eigen::Vector2d diff_tilt_angle;

        Eigen::Vector2d second_order_diff_leg_length;
        Eigen::Vector2d second_order_diff_tilt_angle;
    };

    void reset_all_controls() {
        left_leg_vmc_solver_.reset();
        right_leg_vmc_solver_.reset();

        desire_state_solver_.reset();
        velocity_kalman_filter_.reset();
        chassis_gyro_velocity_filter_.reset();

        *left_front_hip_control_torque_ = nan_;
        *left_back_hip_control_torque_ = nan_;
        *right_front_hip_control_torque_ = nan_;
        *right_back_hip_control_torque_ = nan_;

        *left_wheel_control_torque_ = 0.0;
        *right_wheel_control_torque_ = 0.0;

        *left_front_hip_control_angle_ = nan_;
        *left_back_hip_control_angle_ = nan_;
        *right_front_hip_control_angle_ = nan_;
        *right_back_hip_control_angle_ = nan_;
    }

    Eigen::Vector2d calculate_wheel_velocities() {
        return {
            *left_wheel_velocity_ * wheel_radius_, //
            *right_wheel_velocity_ * wheel_radius_};
    }

    Eigen::Vector4d calculate_hips_angles() {
        return {
            *left_front_hip_angle_,                //
            *left_back_hip_angle_,                 //
            *right_back_hip_angle_,                //
            *right_front_hip_angle_,               //
        };
    }

    LegPosture calculate_leg_posture(const Eigen::Vector4d& hip_angles) {
        const auto& [lf, lb, rb, rf] = hip_angles;
        LegPosture result;

        const auto& [left_leg_length, left_tilt_angle] = left_leg_vmc_solver_.update(lf, lb);
        const auto& [right_leg_length, right_tilt_angle] = right_leg_vmc_solver_.update(rf, rb);

        result.leg_length = Eigen::Vector2d{left_leg_length, right_leg_length};
        result.tilt_angle = Eigen::Vector2d{left_tilt_angle, right_tilt_angle};

        result.tilt_angle.array() += *chassis_pitch_angle_imu_;

        result.diff_leg_length = (result.leg_length - last_leg_length_) / dt_;
        last_leg_length_ = result.leg_length;
        result.diff_tilt_angle = (result.tilt_angle - last_tilt_angle_) / dt_;
        last_tilt_angle_ = result.tilt_angle;

        result.second_order_diff_leg_length = (result.diff_leg_length - last_dot_leg_length_) / dt_;
        last_dot_leg_length_ = result.diff_leg_length;
        result.second_order_diff_tilt_angle = (result.diff_tilt_angle - last_dot_tilt_angle_) / dt_;
        last_dot_tilt_angle_ = result.diff_tilt_angle;

        return result;
    }

    Eigen::Vector2d calculate_support_force(LegPosture leg_posture) {
        Eigen::Vector2d result;
        auto& [left_support_force, right_support_force] = result;

        const auto& [left_leg_force, left_leg_torque] = left_leg_vmc_solver_.update_virtual_torque(
            *left_front_hip_torque_, *left_back_hip_torque_);
        const auto& [right_leg_force, right_leg_torque] =
            right_leg_vmc_solver_.update_virtual_torque(
                *right_front_hip_torque_, *right_back_hip_torque_);

        auto left_leg_to_wheel_force =
            left_leg_force * std::cos(leg_posture.tilt_angle.x())
            + left_leg_torque * std::sin(leg_posture.tilt_angle.x()) / leg_posture.leg_length.x();
        auto right_leg_to_wheel_force =
            right_leg_force * std::cos(leg_posture.tilt_angle.y())
            + right_leg_torque * std::sin(leg_posture.tilt_angle.y()) / leg_posture.leg_length.y();

        // todo: test
        auto left_wheel_vertical_accel =
            *chassis_z_axis_acceleration_imu_
            - leg_posture.second_order_diff_leg_length.x() * std::cos(leg_posture.tilt_angle.x())
            + 2 * leg_posture.diff_leg_length.x() * leg_posture.diff_tilt_angle.x()
                  * std::sin(leg_posture.tilt_angle.x())
            + leg_posture.leg_length.x() * leg_posture.second_order_diff_tilt_angle.x()
                  * std::sin(leg_posture.tilt_angle.x())
            + leg_posture.leg_length.x() * leg_posture.diff_tilt_angle.x()
                  * leg_posture.diff_tilt_angle.x() * std::cos(leg_posture.tilt_angle.x());

        auto right_wheel_vertical_accel =
            *chassis_z_axis_acceleration_imu_
            - leg_posture.second_order_diff_leg_length.y() * std::cos(leg_posture.tilt_angle.y())
            + 2 * leg_posture.diff_leg_length.y() * leg_posture.diff_tilt_angle.y()
                  * std::sin(leg_posture.tilt_angle.y())
            + leg_posture.leg_length.y() * leg_posture.second_order_diff_tilt_angle.y()
                  * std::sin(leg_posture.tilt_angle.y())
            + leg_posture.leg_length.y() * leg_posture.diff_tilt_angle.y()
                  * leg_posture.diff_tilt_angle.y() * std::cos(leg_posture.tilt_angle.y());

        left_support_force =
            left_leg_to_wheel_force + wheel_mess_ * (g_ + left_wheel_vertical_accel);
        right_support_force =
            right_leg_to_wheel_force + wheel_mess_ * (g_ + right_wheel_vertical_accel);

        return result;
    }

    Eigen::Vector2d
        calculate_translational_distance(LegPosture leg_posture, Eigen::Vector2d wheel_velocities) {
        Eigen::Vector2d result;
        auto& [distance, velocity] = result;

        auto wheel_velocity = (wheel_velocities.x() + wheel_velocities.y()) / 2.0;

        auto left_leg_velocity =
            leg_posture.leg_length.x() * std::cos(leg_posture.tilt_angle.x())
                * leg_posture.diff_tilt_angle.x()
            + leg_posture.diff_leg_length.x() * std::sin(leg_posture.tilt_angle.x());

        auto right_leg_velocity =
            leg_posture.leg_length.y() * std::cos(leg_posture.tilt_angle.y())
                * leg_posture.diff_tilt_angle.y()
            + leg_posture.diff_leg_length.y() * std::sin(leg_posture.tilt_angle.y());

        // auto calculate_velocity = wheel_velocity + (left_leg_velocity + right_leg_velocity)
        // / 2.0;
        auto calculate_velocity = wheel_velocity;

        // Velocity ​​estimation is referenced from the article:
        // https://zhuanlan.zhihu.com/p/689921165
        // auto estimate_velocity = velocity_kalman_filter_.update(
        //     Eigen::Vector2d{calculate_velocity, *chassis_x_axis_acceleration_imu_});

        velocity = calculate_velocity;

        auto is_parked = std::abs(velocity) < 1e-2;
        // When the vehicle stops the position control is activated.
        if (!last_is_parked_ && is_parked) {
            distance = last_distance_ + velocity * dt_;
        } else {
            distance = 0.0;
        }
        last_is_parked_ = is_parked;

        distance = last_distance_ + velocity * dt_;
        last_distance_ = distance;

        return Eigen::Vector2d{distance, velocity};
    }

    rmcs_msgs::WheelLegState
        calculate_measure_state(Eigen::Vector2d distance, LegPosture leg_posture) {
        rmcs_msgs::WheelLegState measure_state;

        measure_state.distance = distance.x();
        measure_state.velocity = distance.y();

        measure_state.yaw_angle = *chassis_yaw_angle_imu_;
        measure_state.yaw_velocity = *chassis_yaw_velocity_imu_;

        measure_state.left_tilt_angle = leg_posture.tilt_angle.x();
        measure_state.left_tilt_velocity = leg_posture.diff_tilt_angle.x();

        measure_state.right_tilt_angle = leg_posture.tilt_angle.y();
        measure_state.right_tilt_velocity = leg_posture.diff_tilt_angle.y();

        measure_state.body_pitch_angle = *chassis_pitch_angle_imu_;
        measure_state.body_pitch_velocity = *chassis_pitch_velocity_imu_;

        measure_state.distance = 0.0;
        measure_state.velocity = 0.0;

        // measure_state.yaw_angle = 0.0;
        // measure_state.yaw_velocity = 0.0;

        // measure_state.left_tilt_angle = 0.0;
        // measure_state.left_tilt_velocity = 0.0;

        // measure_state.right_tilt_angle = 0.0;
        // measure_state.right_tilt_velocity = 0.0;

        // measure_state.body_pitch_angle = 0.0;
        // measure_state.body_pitch_velocity = 0.0;

        return measure_state;
    }

    rmcs_msgs::WheelLegState detect_chassis_levitate(
        Eigen::Vector2d support_forces, rmcs_msgs::WheelLegState measure_state) {
        auto& [left_support_force, right_support_force] = support_forces;

        double levitate_force = 20.0;
        double normal_force = 50.0;

        auto average_support_force = (left_support_force + right_support_force) / 2.0;

        auto mode = *mode_;
        auto mode_active = mode != rmcs_msgs::WheelLegMode::BALANCELESS;

        if (mode_active) {
            if (average_support_force < levitate_force) {
                levitate_active_ = true;
            } else if (average_support_force > normal_force) {
                levitate_active_ = false;
            }

            if (left_support_force < levitate_force) {
                left_levitate_active_ = true;
            } else if (left_support_force > normal_force) {
                left_levitate_active_ = false;
            }

            if (right_support_force < levitate_force) {
                right_levitate_active_ = true;
            } else if (right_support_force > normal_force) {
                right_levitate_active_ = false;
            }
        } else {
            levitate_active_ = false;
            left_levitate_active_ = false;
            right_levitate_active_ = false;
        }

        if (levitate_active_) {
            RCLCPP_INFO(
                get_logger(), "Chassis is levitating! Average support force: %f",
                average_support_force);
            measure_state.distance = 0.0;
        }
        return measure_state;
    }

    // When the vehicle is about to fall, reset the leg length.
    void detect_chassis_fall() {
        // Detect pitch and roll angle
        if (fabs(*chassis_roll_angle_imu_ - desire_roll_angle_) > about_to_fall_roll_angle_) {
            about_to_fall_ = true;
        }

        if (levitate_active_) {
            about_to_fall_ = false;
        }

        // It will inactive when rotating, climbing, or launching ramp.
        if (*is_climb_active_ || *mode_ == rmcs_msgs::WheelLegMode::LAUNCH_RAMP
            || *mode_ == rmcs_msgs::WheelLegMode::SPIN) {
            about_to_fall_ = false;
        }
    }

    Eigen::Vector3d calculate_chassis_control_velocity() {
        Eigen::Vector3d chassis_control_velocity;
        chassis_control_velocity = chassis_control_velocity_->vector;
        return chassis_control_velocity;
    }

    rmcs_msgs::WheelLegState calculate_desire_state(
        Eigen::Vector3d control_velocity, rmcs_msgs::WheelLegState measure_state) {
        rmcs_msgs::WheelLegState desire_state{};

        desire_state_solver_.update_measure_state(measure_state);
        desire_state = desire_state_solver_.update(std::move(control_velocity));

        desire_leg_length_ =
            desire_state_solver_.update_desire_leg_length(*is_leg_extended_, 0.13, 0.36);
        desire_roll_angle_ = desire_state_solver_.update_desire_roll_angle();

        return desire_state;
    }

    Eigen::Vector2d
        calculate_leg_force(LegPosture leg_posture, rmcs_msgs::WheelLegState measure_state) {
        Eigen::Vector2d result;

        auto leg_length = (leg_posture.leg_length.x() + leg_posture.leg_length.y()) / 2.0;

        auto roll_control_force =
            roll_angle_pid_calculator_.update(desire_roll_angle_ - *chassis_roll_angle_imu_);
        auto leg_length_control_force =
            leg_length_pid_calculator_.update(desire_leg_length_ - leg_length);

        auto calculate_compensation_feedforward_force = [this](double coefficient) {
            return (body_mess_ / 2.0 + centroid_position_coefficient_ * leg_mess_) * coefficient;
        };

        auto gravity_feedforward_control_force = calculate_compensation_feedforward_force(g_);
        auto inertial_feedforward_control_force = calculate_compensation_feedforward_force(
            leg_length / (2 * wheel_distance_) * measure_state.yaw_velocity
            * measure_state.velocity);

        //                       F_ψ
        // F_bl_l =  1 1 1 -1 *  F_l
        // F_bl_r   -1 1 1  1    F_g
        //                       F_i
        result = Eigen::Vector2d{
            roll_control_force + leg_length_control_force + gravity_feedforward_control_force
                - inertial_feedforward_control_force, // F_bl_l
            -roll_control_force + leg_length_control_force + gravity_feedforward_control_force
                + inertial_feedforward_control_force  // F_bl_r
        };

        return result;
    }

    Eigen::Vector4d calculate_control_torques(
        rmcs_msgs::WheelLegState desire_state, rmcs_msgs::WheelLegState measure_state,
        LegPosture leg_posture) {
        Eigen::Vector4d result{};

        Eigen::Vector<double, 10> error_state = desire_state.vector() - measure_state.vector();
        auto gain = lqr_calculator_.update(leg_posture.leg_length.x(), leg_posture.leg_length.y());

        if (levitate_active_) {
            // When levitating, only keep legs vertical,and only remain legs' outputs.
            // set zero
            error_state(0) = 0.0; // distance
            error_state(1) = 0.0; // velocity
            error_state(2) = 0.0; // yaw angle
            error_state(3) = 0.0; // yaw velocity

            error_state(8) = 0.0; // body pitch angle
            error_state(9) = 0.0; // body pitch velocity
        }

        result = -1.0 * gain * error_state;

        auto output_0 = -gain(0, 0) * error_state(0);
        auto output_1 = -gain(0, 1) * error_state(1);
        auto output_2 = -gain(0, 2) * error_state(2);
        auto output_3 = -gain(0, 3) * error_state(3);
        auto output_4 = -gain(0, 4) * error_state(4);
        auto output_5 = -gain(0, 5) * error_state(5);
        auto output_6 = -gain(0, 6) * error_state(6);
        auto output_7 = -gain(0, 7) * error_state(7);
        auto output_8 = -gain(0, 8) * error_state(8);
        auto output_9 = -gain(0, 9) * error_state(9);

        RCLCPP_INFO(
            get_logger(), "output: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f], %f, %f", output_0,
            output_1, output_2, output_3, output_4, output_5, output_6, output_7, output_8,
            output_9, result.x(), result.y());

        // RCLCPP_INFO(
        //     get_logger(), "measure: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
        //     measure_state.distance, measure_state.velocity, measure_state.yaw_angle,
        //     measure_state.yaw_velocity, measure_state.left_tilt_angle,
        //     measure_state.left_tilt_velocity, measure_state.right_tilt_angle,
        //     measure_state.right_tilt_velocity, measure_state.body_pitch_angle,
        //     measure_state.body_pitch_velocity);

        return result;
    }

    void update_balanceless_control_torques(
        Eigen::Vector3d chassis_control_velocity, Eigen::Vector2d wheel_velocities) {
        auto& [chassis_control_velocity_x, chassis_control_velocity_y, chassis_control_velocity_z] =
            chassis_control_velocity;
        auto& [left_wheel_velocity, right_wheel_velocity] = wheel_velocities;

        double left_wheel_control_torque = balanceless_left_wheel_pid_calculator_.update(
            (chassis_control_velocity_x - chassis_control_velocity_y) - left_wheel_velocity);
        double right_wheel_control_torque = balanceless_right_wheel_pid_calculator_.update(
            (chassis_control_velocity_x + chassis_control_velocity_y) - right_wheel_velocity);

        *left_wheel_control_torque_ = clamp_wheel_control_torque(left_wheel_control_torque);
        *right_wheel_control_torque_ = clamp_wheel_control_torque(right_wheel_control_torque);
    }

    void update_hip_zero_point_control_angles() {
        *left_front_hip_control_angle_ = 0.0;
        *left_back_hip_control_angle_ = 0.0;
        *right_front_hip_control_angle_ = 0.0;
        *right_back_hip_control_angle_ = 0.0;

        reset_control_torques();
    }

    void update_hip_rescue_tip_over_control_torques() {
        double control_velocity = pi_;

        // stand_active_ = false;
        auto left_front_hip_control_torque =
            rescue_velocity_pid_calculator_.update(control_velocity - *left_front_hip_velocity_);
        auto left_back_hip_control_torque =
            rescue_velocity_pid_calculator_.update(control_velocity - *left_back_hip_velocity_);
        auto right_front_hip_control_torque =
            rescue_velocity_pid_calculator_.update(control_velocity - *right_front_hip_velocity_);
        auto right_back_hip_control_torque =
            rescue_velocity_pid_calculator_.update(control_velocity - *right_back_hip_velocity_);

        *left_front_hip_control_torque_ = left_front_hip_control_torque;
        *left_back_hip_control_torque_ = left_back_hip_control_torque;
        *right_front_hip_control_torque_ = right_front_hip_control_torque;
        *right_back_hip_control_torque_ = right_back_hip_control_torque;

        reset_control_angles();
    }

    void update_hip_stand_control_torques(LegPosture leg_posture) {
        auto leg_length = (leg_posture.leg_length.x() + leg_posture.leg_length.y()) / 2.0;

        auto leg_length_control_force =
            leg_length_pid_calculator_.update(min_leg_length_ - leg_length);

        if (leg_length < min_leg_length_ + 0.01) {
            stand_active_ = true;
        }

        RCLCPP_INFO(
            get_logger(), "control force: %f, leg length: %f,stand active: %d",
            leg_length_control_force, leg_length, stand_active_);

        *left_front_hip_control_torque_ = -leg_length_control_force;
        *left_back_hip_control_torque_ = leg_length_control_force;
        *right_front_hip_control_torque_ = -leg_length_control_force;
        *right_back_hip_control_torque_ = leg_length_control_force;
    }

    void update_hip_and_wheel_torques(Eigen::Vector2d leg_forces, Eigen::Vector4d control_torques) {
        auto& [left_leg_force, right_leg_force] = leg_forces;
        auto& [left_wheel_control_torque, right_wheel_control_torque, left_leg_control_torque, right_leg_control_torque] =
            control_torques;

        auto left_hip_control_torque =
            left_leg_vmc_solver_.update_joint_torque(left_leg_force, left_leg_control_torque);
        auto right_hip_control_torque =
            right_leg_vmc_solver_.update_joint_torque(right_leg_force, right_leg_control_torque);

        // RCLCPP_INFO(
        //     get_logger(), "lf: %f, lb: %f, rf: %f, rb: %f,control_torque: %f",
        //     left_hip_control_torque.x(), left_hip_control_torque.y(),
        //     right_hip_control_torque.x(), right_hip_control_torque.y(), torque);

        if (levitate_active_) {
            *left_wheel_control_torque_ = 0.0;
            *right_wheel_control_torque_ = 0.0;
        }

        *left_wheel_control_torque_ = clamp_wheel_control_torque(left_wheel_control_torque);
        *right_wheel_control_torque_ = clamp_wheel_control_torque(right_wheel_control_torque);

        *left_front_hip_control_torque_ = clamp_hip_control_torque(-left_hip_control_torque.x());
        *left_back_hip_control_torque_ = clamp_hip_control_torque(left_hip_control_torque.y());
        *right_front_hip_control_torque_ = clamp_hip_control_torque(-right_hip_control_torque.x());
        *right_back_hip_control_torque_ = clamp_hip_control_torque(right_hip_control_torque.y());

        // RCLCPP_INFO(
        //     get_logger(), "lf: %f, lb: %f, rb: %f, rf: %f", *left_front_hip_control_torque_,
        //     *left_back_hip_control_torque_, *right_back_hip_control_torque_,
        //     *right_front_hip_control_torque_);

        reset_control_angles();
    }

    static double clamp_wheel_control_torque(const double& torque) {
        return std::clamp(torque, -2.4, 2.4);
    }

    static double clamp_hip_control_torque(const double& torque) {
        return std::clamp(torque, -10., 10.);
    }

    void reset_control_torques() {
        *left_front_hip_control_torque_ = nan_;
        *left_back_hip_control_torque_ = nan_;
        *right_front_hip_control_torque_ = nan_;
        *right_back_hip_control_torque_ = nan_;
    }

    void reset_control_angles() {
        *left_front_hip_control_angle_ = nan_;
        *left_back_hip_control_angle_ = nan_;
        *right_front_hip_control_angle_ = nan_;
        *right_back_hip_control_angle_ = nan_;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double pi_ = std::numbers::pi;

    static constexpr double dt_ = 1e-3;
    static constexpr double g_ = 9.80665;

    static constexpr double max_leg_length_ = 0.36;
    static constexpr double min_leg_length_ = 0.13;

    static constexpr double about_to_fall_roll_angle_ = 0.6;

    static constexpr double sigma_q_ = 1.0;
    static constexpr double sigma_v_ = 0.75;
    static constexpr double sigma_a_ = 1.0;

    const Eigen::Matrix2d A_ = (Eigen::Matrix2d() << 1, dt_, 1, 1).finished();
    const Eigen::Matrix2d H_ = Eigen::Vector2d::Identity().asDiagonal();
    const Eigen::Matrix2d W_ = Eigen::Vector2d{0.5 * dt_ * dt_, dt_}.asDiagonal();
    const Eigen::Matrix2d Q_ =
        Eigen::Vector2d{sigma_q_ * sigma_q_, sigma_q_* sigma_q_}.asDiagonal();
    const Eigen::Matrix2d R_ =
        Eigen::Vector2d{sigma_v_ * sigma_v_, sigma_a_* sigma_a_}.asDiagonal();

    const double body_mess_;
    const double leg_mess_;
    const double wheel_mess_;
    const double wheel_radius_;
    const double wheel_distance_;
    const double centroid_position_coefficient_;

    InputInterface<double> hip_motor_max_control_torque_;
    InputInterface<double> wheel_motor_max_control_torque_;

    InputInterface<double> left_front_hip_angle_;
    InputInterface<double> left_back_hip_angle_;
    InputInterface<double> right_front_hip_angle_;
    InputInterface<double> right_back_hip_angle_;

    InputInterface<double> left_front_hip_velocity_;
    InputInterface<double> left_back_hip_velocity_;
    InputInterface<double> right_front_hip_velocity_;
    InputInterface<double> right_back_hip_velocity_;

    InputInterface<double> left_front_hip_torque_;
    InputInterface<double> left_back_hip_torque_;
    InputInterface<double> right_front_hip_torque_;
    InputInterface<double> right_back_hip_torque_;

    InputInterface<double> left_wheel_velocity_;
    InputInterface<double> right_wheel_velocity_;

    InputInterface<double> chassis_x_axis_acceleration_imu_;
    InputInterface<double> chassis_z_axis_acceleration_imu_;

    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<double> chassis_pitch_velocity_imu_;
    InputInterface<double> chassis_roll_velocity_imu_;

    InputInterface<double> chassis_yaw_angle_imu_;
    InputInterface<double> chassis_pitch_angle_imu_;
    InputInterface<double> chassis_roll_angle_imu_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<rmcs_msgs::WheelLegMode> mode_;

    InputInterface<double> chassis_control_angle_;

    InputInterface<bool> is_balanceless_;
    InputInterface<bool> is_rescue_tip_over_;

    InputInterface<bool> is_leg_extended_;
    InputInterface<bool> is_jump_active_;
    InputInterface<bool> is_climb_active_;

    OutputInterface<double> left_front_hip_control_torque_;
    OutputInterface<double> left_back_hip_control_torque_;
    OutputInterface<double> right_front_hip_control_torque_;
    OutputInterface<double> right_back_hip_control_torque_;

    OutputInterface<double> left_front_hip_control_angle_;
    OutputInterface<double> left_back_hip_control_angle_;
    OutputInterface<double> right_front_hip_control_angle_;
    OutputInterface<double> right_back_hip_control_angle_;

    OutputInterface<double> left_wheel_control_torque_;
    OutputInterface<double> right_wheel_control_torque_;

    double desire_leg_length_, desire_roll_angle_;
    bool levitate_active_{false}, left_levitate_active_{false}, right_levitate_active_{false};

    bool about_to_fall_{false}, rescue_tip_over_{false};

    bool stand_active_{false};
    Eigen::Vector2d last_leg_length_, last_tilt_angle_;
    Eigen::Vector2d last_dot_leg_length_, last_dot_tilt_angle_;
    double last_distance_;
    bool last_is_parked_{false};

    Eigen::Vector3d filtered_gyro_velocity_;

    VmcSolver left_leg_vmc_solver_, right_leg_vmc_solver_;
    filter::KalmanFilter<2, 2> velocity_kalman_filter_;
    filter::LowPassFilter<3> chassis_gyro_velocity_filter_;

    DesireStateSolver desire_state_solver_;

    pid::PidCalculator roll_angle_pid_calculator_, leg_length_pid_calculator_;
    pid::PidCalculator balanceless_left_wheel_pid_calculator_,
        balanceless_right_wheel_pid_calculator_;
    pid::PidCalculator rescue_velocity_pid_calculator_;
    lqr::LqrCalculator lqr_calculator_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegController, rmcs_executor::Component)