#include <algorithm>
#include <cmath>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
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
        , body_mess_(14.19)
        , leg_mess_(0.629)
        , wheel_mess_(0.459)
        , wheel_radius_(0.06)
        , wheel_distance_(0.2135)
        , centroid_position_coefficient_(0.2945)
        , left_leg_vmc_solver_(0.21, 0.25, 0.0)
        , right_leg_vmc_solver_(0.21, 0.25, 0.0)
        , velocity_kalman_filter_(A_, H_, Q_, R_)
        , chassis_angle_filter_(5.0, 1000.0)
        , desire_state_solver_(0.15)
        , roll_angle_pid_calculator_(1.0, 0.0, 1.0)
        , leg_length_pid_calculator_(350.0, 0.0, 0.0) {

        register_input("/chassis/left_front_hip/max_torque", hip_motor_max_control_torque_);
        register_input("/chassis/left_wheel/max_torque", wheel_motor_max_control_torque_);

        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/control_mode", mode_);
        // register_input("/chassis/control_power_limit", power_limit_);

        register_input("/chassis/left_front_hip/angle", left_front_hip_angle_);
        register_input("/chassis/left_back_hip/angle", left_back_hip_angle_);
        register_input("/chassis/right_front_hip/angle", right_front_hip_angle_);
        register_input("/chassis/right_back_hip/angle", right_back_hip_angle_);

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

        register_output("/chassis/velocity", velocity_);

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

        auto filtered_chassis_angle = chassis_angle_filter_.update(
            Eigen::Vector3d{
                *chassis_roll_angle_imu_, *chassis_pitch_angle_imu_, *chassis_yaw_angle_imu_});

        // State: [s ds phi d_phi theta_l d_theta_l theta_r d_theta_r theta_b d_theta_b]
        auto measure_state = calculate_measure_state(distance, leg_posture, filtered_chassis_angle);

        // Controller
        auto chassis_control_velocity = calculate_chassis_control_velocity();
        auto desire_state = calculate_desire_state(chassis_control_velocity);

        auto leg_forces = calculate_leg_force(leg_posture, measure_state);
        auto control_torques = calculate_control_torques(desire_state, measure_state, leg_posture);

        update_hip_and_wheel_torques(leg_forces, control_torques);

        *left_front_hip_control_angle_ = 0.0;
        *left_back_hip_control_angle_ = 0.0;
        *right_front_hip_control_angle_ = 0.0;
        *right_back_hip_control_angle_ = 0.0;

        // RCLCPP_INFO(
        //     get_logger(), "lf: %f, lb: %f, lw: %f, rf: %f, rb: %f, rw:%f",
        //     *left_front_hip_control_torque_, *left_back_hip_control_torque_,
        //     *left_wheel_control_torque_, *right_front_hip_control_torque_,
        //     *right_back_hip_control_torque_, *right_wheel_control_torque_);

        *left_front_hip_control_torque_ = 0.0;
        *left_back_hip_control_torque_ = -0.0;
        *right_front_hip_control_torque_ = -0.0;
        *right_back_hip_control_torque_ = 0.0;

        // RCLCPP_INFO(
        //     get_logger(), "pitch: %f, lw torque: %f, rw torque: %f", *chassis_pitch_angle_imu_,
        //     *left_wheel_control_torque_, *right_wheel_control_torque_);

        // *left_wheel_control_torque_ = 0.0;
        // *right_wheel_control_torque_ = 0.0;
    }

private:
    struct LegPosture {
        Eigen::Vector2d leg_length;
        Eigen::Vector2d tilt_angle;

        Eigen::Vector2d dot_leg_length;
        Eigen::Vector2d dot_tilt_angle;

        Eigen::Vector2d second_order_diff_leg_length;
        Eigen::Vector2d second_order_diff_tilt_angle;
    };

    void reset_all_controls() {
        left_leg_vmc_solver_.reset();
        right_leg_vmc_solver_.reset();

        desire_state_solver_.reset();
        velocity_kalman_filter_.reset();
        chassis_angle_filter_.reset();

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

        result.dot_leg_length = (result.leg_length - last_leg_length_) / dt_;
        last_leg_length_ = result.leg_length;
        result.dot_tilt_angle = (result.tilt_angle - last_tilt_angle_) / dt_;
        last_tilt_angle_ = result.tilt_angle;

        result.second_order_diff_leg_length = (result.dot_leg_length - last_dot_leg_length_) / dt_;
        last_dot_leg_length_ = result.dot_leg_length;
        result.second_order_diff_tilt_angle = (result.dot_tilt_angle - last_dot_tilt_angle_) / dt_;
        last_dot_tilt_angle_ = result.dot_tilt_angle;

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

        //@todo: test
        auto left_wheel_vertical_accel =
            *chassis_z_axis_acceleration_imu_
            - leg_posture.second_order_diff_leg_length.x() * std::cos(leg_posture.tilt_angle.x())
            + 2 * leg_posture.dot_leg_length.x() * leg_posture.dot_tilt_angle.x()
                  * std::sin(leg_posture.tilt_angle.x())
            + leg_posture.leg_length.x() * leg_posture.second_order_diff_tilt_angle.x()
                  * std::sin(leg_posture.tilt_angle.x())
            + leg_posture.leg_length.x() * leg_posture.dot_tilt_angle.x()
                  * leg_posture.dot_tilt_angle.x() * std::cos(leg_posture.tilt_angle.x());

        auto right_wheel_vertical_accel =
            *chassis_z_axis_acceleration_imu_
            - leg_posture.second_order_diff_leg_length.y() * std::cos(leg_posture.tilt_angle.y())
            + 2 * leg_posture.dot_leg_length.y() * leg_posture.dot_tilt_angle.y()
                  * std::sin(leg_posture.tilt_angle.y())
            + leg_posture.leg_length.y() * leg_posture.second_order_diff_tilt_angle.y()
                  * std::sin(leg_posture.tilt_angle.y())
            + leg_posture.leg_length.y() * leg_posture.dot_tilt_angle.y()
                  * leg_posture.dot_tilt_angle.y() * std::cos(leg_posture.tilt_angle.y());

        left_support_force =
            left_leg_to_wheel_force + wheel_mess_ * (g_ + left_wheel_vertical_accel);
        right_support_force =
            right_leg_to_wheel_force + wheel_mess_ * (g_ + right_wheel_vertical_accel);

        return result;
    }

    void detect_chassis_leave_the_ground() {}

    Eigen::Vector2d
        calculate_translational_distance(LegPosture leg_posture, Eigen::Vector2d wheel_velocities) {
        Eigen::Vector2d result;
        auto& [distance, velocity] = result;

        auto wheel_velocity = (wheel_velocities.x() + wheel_velocities.y()) / 2.0;
        leg_posture.tilt_angle.array() += *chassis_pitch_angle_imu_;

        auto left_leg_velocity =
            leg_posture.leg_length.x() * std::cos(leg_posture.tilt_angle.x())
                * leg_posture.dot_tilt_angle.x()
            + leg_posture.dot_leg_length.x() * std::sin(leg_posture.tilt_angle.x());

        auto right_leg_velocity =
            leg_posture.leg_length.y() * std::cos(leg_posture.tilt_angle.y())
                * leg_posture.dot_tilt_angle.y()
            + leg_posture.dot_leg_length.y() * std::sin(leg_posture.tilt_angle.y());

        auto calculate_velocity = wheel_velocity + (left_leg_velocity + right_leg_velocity) / 2.0;

        // Velocity ​​estimation is referenced from the article:
        // https://zhuanlan.zhihu.com/p/689921165
        auto estimate_velocity = velocity_kalman_filter_.update(
            Eigen::Vector2d{calculate_velocity, *chassis_x_axis_acceleration_imu_});

        velocity = calculate_velocity;
        distance = last_distance_ + velocity * dt_;
        last_distance_ = distance;

        return Eigen::Vector2d{distance, velocity};
    }

    rmcs_msgs::WheelLegState calculate_measure_state(
        Eigen::Vector2d distance, LegPosture leg_posture, Eigen::Vector3d filtered_chassis_angle) {
        rmcs_msgs::WheelLegState measure_state;
        measure_state.distance = distance.x();
        measure_state.velocity = distance.y();

        measure_state.yaw_angle = *chassis_yaw_angle_imu_;
        measure_state.yaw_velocity = *chassis_yaw_velocity_imu_;

        measure_state.left_tilt_angle = leg_posture.tilt_angle.x();
        measure_state.left_tilt_velocity = leg_posture.dot_tilt_angle.x();

        measure_state.right_tilt_angle = leg_posture.tilt_angle.y();
        measure_state.right_tilt_velocity = leg_posture.dot_tilt_angle.y();

        measure_state.body_pitch_angle = *chassis_pitch_angle_imu_;
        measure_state.body_pitch_velocity = *chassis_pitch_velocity_imu_;

        measure_state.yaw_angle = 0.0;
        measure_state.yaw_velocity = 0.0;

        measure_state.left_tilt_angle = 0.0;
        measure_state.left_tilt_velocity = 0.0;

        measure_state.right_tilt_angle = 0.0;
        measure_state.right_tilt_velocity = 0.0;

        return measure_state;
    }

    Eigen::Vector3d calculate_chassis_control_velocity() {
        Eigen::Vector3d chassis_control_velocity;
        chassis_control_velocity = chassis_control_velocity_->vector;
        return chassis_control_velocity;
    }

    rmcs_msgs::WheelLegState calculate_desire_state(Eigen::Vector3d control_velocity) {
        rmcs_msgs::WheelLegState desire_state{};

        // x-axis translational velocity, z-axis vertical velocity, z-axis angular velocity
        auto& [vx, vz, wz] = control_velocity;

        desire_state = desire_state_solver_.update(vx, vz, wz);
        desire_roll_angle_ = desire_state_solver_.update_desire_roll_angle();
        desire_leg_length_ = desire_state_solver_.update_desire_leg_length(0.13, 0.36);

        return desire_state;
    }

    Eigen::Vector2d
        calculate_leg_force(LegPosture leg_posture, rmcs_msgs::WheelLegState measure_state) {
        Eigen::Vector2d result;

        auto roll_control_force =
            roll_angle_pid_calculator_.update(desire_roll_angle_ - *chassis_roll_angle_imu_);
        auto leg_length_control_force = leg_length_pid_calculator_.update(
            desire_leg_length_ - (leg_posture.leg_length.x() + leg_posture.leg_length.y()) / 2.0);

        auto calculate_compensation_feedforward_force = [this](double coefficient) {
            return (body_mess_ / 2.0 + centroid_position_coefficient_ * leg_mess_) * coefficient;
        };

        auto gravity_feedforward_control_force = calculate_compensation_feedforward_force(g_);
        auto inertial_feedforward_control_force = calculate_compensation_feedforward_force(
            (leg_posture.leg_length.x() + leg_posture.leg_length.y()) / 2.0 / (2 * wheel_distance_)
            * measure_state.yaw_velocity * measure_state.velocity);

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

        auto error_state = desire_state.vector() - measure_state.vector();
        // RCLCPP_INFO(
        //     get_logger(),
        //     "desire_state: "
        //     "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
        //     desire_state.vector()(0), desire_state.vector()(1), desire_state.vector()(2),
        //     desire_state.vector()(3), desire_state.vector()(4), desire_state.vector()(5),
        //     desire_state.vector()(6), desire_state.vector()(7), desire_state.vector()(8),
        //     desire_state.vector()(9));

        // RCLCPP_INFO(
        //     get_logger(),
        //     "measure_state: "
        //     "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
        //     measure_state.vector()(0), measure_state.vector()(1), measure_state.vector()(2),
        //     measure_state.vector()(3), measure_state.vector()(4), measure_state.vector()(5),
        //     measure_state.vector()(6), measure_state.vector()(7), measure_state.vector()(8),
        //     measure_state.vector()(9));

        // RCLCPP_INFO(
        //     get_logger(),
        //     "error_state: "
        //     "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
        //     error_state(0), error_state(1), error_state(2), error_state(3), error_state(4),
        //     error_state(5), error_state(6), error_state(7), error_state(8), error_state(9));

        auto gain = lqr_calculator_.update(leg_posture.leg_length.x(), leg_posture.leg_length.y());

        auto err_0 = error_state(0);
        auto err_1 = error_state(1);
        auto err_8 = error_state(8);

        RCLCPP_INFO(get_logger(), "gain debug: %f, %f, %f", err_0, err_1, err_8);

        result = -gain * error_state;

        std::ostringstream oss;
        oss << "\nLQR gain matrix (4x10):\n" << gain;

        // RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

        return result;
    }

    void update_hip_and_wheel_torques(Eigen::Vector2d leg_forces, Eigen::Vector4d control_torques) {
        auto& [left_leg_force, right_leg_force] = leg_forces;
        auto& [left_wheel_control_torque, right_wheel_control_torque, left_leg_control_torque, right_leg_control_torque] =
            control_torques;

        auto left_hip_control_torque =
            left_leg_vmc_solver_.update_joint_torque(left_leg_force, left_leg_control_torque);
        auto right_hip_control_torque =
            right_leg_vmc_solver_.update_joint_torque(right_leg_force, right_leg_control_torque);

        *left_wheel_control_torque_ = clamp_wheel_control_torque(left_wheel_control_torque);
        *right_wheel_control_torque_ = clamp_wheel_control_torque(right_wheel_control_torque);

        *left_front_hip_control_torque_ = clamp_hip_control_torque(left_hip_control_torque.x());
        *left_back_hip_control_torque_ = clamp_hip_control_torque(-left_hip_control_torque.y());
        *right_front_hip_control_torque_ = clamp_hip_control_torque(right_hip_control_torque.x());
        *right_back_hip_control_torque_ = clamp_hip_control_torque(-right_hip_control_torque.y());
    }

    static double clamp_wheel_control_torque(const double& torque) {
        return std::clamp(torque, -5., 5.);
    }

    static double clamp_hip_control_torque(const double& torque) {
        return std::clamp(torque, -5., 5.);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double pi_ = std::numbers::pi;

    static constexpr double dt_ = 1e-3;
    static constexpr double g_ = 9.80665;

    static constexpr double front_upper_limit_angle_ = 7 * pi_ / 6;
    static constexpr double front_lower_limit_angle_ = 2 * pi_ / 3;

    static constexpr double back_upper_limit_angle_ = -pi_ / 6;
    static constexpr double back_lower_limit_angle_ = pi_ / 3;

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
    InputInterface<double> power_limit_;
    InputInterface<rmcs_msgs::ChassisMode> mode_;

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

    OutputInterface<double> distance_;
    OutputInterface<double> velocity_;

    double desire_leg_length_, desire_roll_angle_;

    Eigen::Vector2d last_leg_length_, last_tilt_angle_;
    Eigen::Vector2d last_dot_leg_length_, last_dot_tilt_angle_;
    double last_distance_;

    VmcSolver left_leg_vmc_solver_, right_leg_vmc_solver_;
    filter::KalmanFilter<2, 2> velocity_kalman_filter_;
    filter::LowPassFilter<3> chassis_angle_filter_;

    DesireStateSolver desire_state_solver_;

    pid::PidCalculator roll_angle_pid_calculator_, leg_length_pid_calculator_;
    lqr::LqrCalculator lqr_calculator_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegController, rmcs_executor::Component)