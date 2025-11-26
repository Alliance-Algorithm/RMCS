#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/wheel_leg_state.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

#include "filter/kalman_filter.hpp"
#include "vmc_solver.hpp"

namespace rmcs_core::controller::chassis {
class Observer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Observer()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , body_mess_(15.0)
        , leg_mess_(0.86)
        , wheel_radius_(0.06)
        , wheel_distance_(0.2135)
        , centroid_position_coefficient_()
        , left_leg_vmc_solver_(0.21, 0.25, 0.0)
        , right_leg_vmc_solver_(0.21, 0.25, 0.0)
        , velocity_kalman_filter_(A_, H_, Q_, R_, W_) {

        register_input("/chassis/left_front_hip/angle", left_front_hip_angle_);
        register_input("/chassis/left_back_hip/angle", left_back_hip_angle_);
        register_input("/chassis/right_front_hip/angle", right_front_hip_angle_);
        register_input("/chassis/right_back_hip/angle", right_back_hip_angle_);

        register_input("/chassis/left_wheel/velocity", left_wheel_velocity_);
        register_input("/chassis/right_wheel/velocity", right_wheel_velocity_);

        register_input("/chassis/x/acceleration", chassis_x_acceleration_imu_);

        register_input("/chassis/yaw/velocity", chassis_yaw_velocity_imu_);
        register_input("/chassis/pitch/velocity", chassis_pitch_velocity_imu_);
        register_input("/chassis/roll/velocity", chassis_roll_velocity_imu_);

        register_input("/chassis/yaw/angle", chassis_yaw_angle_imu_);
        register_input("/chassis/pitch/angle", chassis_pitch_angle_imu_);
        register_input("/chassis/roll/angle", chassis_roll_angle_imu_);

        register_output("/chassis/left_leg_length", left_leg_length_);
        register_output("chassis/right_leg_length", right_leg_length_);
        register_output("/chassis/measure_state", measure_state_);
    }

    void update() override {
        // state vector: s ds phi dphi theta_l d_theta_l theta_r d_theta_r theta_b d_theta_b
        auto wheel_velocities = calculate_wheel_velocities();
        auto hip_angles = calculate_hips_angles();

        auto leg_posture = calculate_leg_posture(hip_angles);
        auto distance = calculate_translational_distance(leg_posture, wheel_velocities);

        auto support_force = calculate_support_force(leg_posture);

        *measure_state_ = calculate_measure_state(distance, leg_posture);
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
            *right_front_hip_angle_};
    }

    LegPosture calculate_leg_posture(const Eigen::Vector4d& hip_angles) {
        // const auto& [lf, lb, rb, rf] = hip_angles;
        LegPosture result;

        const auto &lf = hip_angles.x(), &lb = hip_angles.y(), &rb = hip_angles.z(),
                   &rf = hip_angles.w();

        const auto& [left_leg_length, left_tilt_angle] = left_leg_vmc_solver_.update(lf, lb);
        const auto& [right_leg_length, right_tilt_angle] = right_leg_vmc_solver_.update(rf, rb);

        result.leg_length = Eigen::Vector2d{left_leg_length, right_leg_length};
        result.tilt_angle = Eigen::Vector2d{left_tilt_angle, right_tilt_angle};

        *left_leg_length_ = left_leg_length;
        *right_leg_length_ = right_leg_length;

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

    Eigen::Vector2d
        calculate_translational_distance(LegPosture leg_posture, Eigen::Vector2d wheel_velocities) {
        std::tuple<double, double> result{};
        auto& [distance, velocity] = result;

        auto wheel_velocity = (wheel_velocities.x() + wheel_velocities.y()) / 2.0;
        leg_posture.tilt_angle.array() += *chassis_pitch_angle_imu_;

        auto left_leg_velocity =
            leg_posture.leg_length.x() * std::cos(leg_posture.tilt_angle.x())
                * leg_posture.dot_tilt_angle.x()
            + leg_posture.dot_leg_length.x() * std::sin(leg_posture.tilt_angle.x());

        auto right_leg_velocity = leg_posture.leg_length.y() * std::cos(leg_posture.tilt_angle.y())
                                    * leg_posture.dot_tilt_angle.y()
                                + leg_posture.dot_leg_length.y()
                                + std::sin(leg_posture.tilt_angle.y());

        auto calculate_velocity = wheel_velocity + (left_leg_velocity + right_leg_velocity) / 2.0;

        auto estimate_velocity = velocity_kalman_filter_.update(
            Eigen::Vector2d{calculate_velocity, *chassis_x_acceleration_imu_});

        velocity = estimate_velocity.x();
        distance += velocity * dt_;

        return Eigen::Vector2d{};
    }

    Eigen::Vector2d calculate_support_force(LegPosture leg_posture) {}

    rmcs_msgs::WheelLegState
        calculate_measure_state(Eigen::Vector2d distance, LegPosture leg_posture) {
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

        return measure_state;
    }

    static constexpr double dt_ = 1e-3;
    static constexpr double g_ = 9.80665;

    static constexpr double sigma_q_ = 1.0;

    static constexpr double sigma_v_ = 0.75;
    static constexpr double sigma_a_ = 1.0;

    const Eigen::Matrix2d A_ = (Eigen::Matrix2d() << 1, dt_, 1, 1).finished();
    const Eigen::Matrix2d H_ = (Eigen::Matrix2d() << 1.0, 0.0, 0.0, 1.0).finished();
    const Eigen::Matrix2d W_ = (Eigen::Matrix2d() << 0.5 * dt_ * dt_, 0.0, 0.0, dt_).finished();
    const Eigen::Matrix2d Q_ =
        (Eigen::Matrix2d() << sigma_q_ * sigma_q_, 0.0, 0.0, sigma_q_* sigma_q_).finished();
    const Eigen::Matrix2d R_ =
        (Eigen::Matrix2d() << sigma_v_ * sigma_v_, 0.0, 0.0, sigma_a_* sigma_a_).finished();

    const double body_mess_;
    const double leg_mess_;
    const double wheel_radius_;
    const double wheel_distance_;
    const double centroid_position_coefficient_;

    InputInterface<double> left_front_hip_angle_;
    InputInterface<double> left_back_hip_angle_;
    InputInterface<double> right_front_hip_angle_;
    InputInterface<double> right_back_hip_angle_;

    InputInterface<double> left_wheel_velocity_;
    InputInterface<double> right_wheel_velocity_;

    InputInterface<double> chassis_x_acceleration_imu_;

    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<double> chassis_pitch_velocity_imu_;
    InputInterface<double> chassis_roll_velocity_imu_;

    InputInterface<double> chassis_yaw_angle_imu_;
    InputInterface<double> chassis_pitch_angle_imu_;
    InputInterface<double> chassis_roll_angle_imu_;

    OutputInterface<double> left_leg_length_;
    OutputInterface<double> right_leg_length_;

    OutputInterface<rmcs_msgs::WheelLegState> measure_state_;

    Eigen::Vector2d last_leg_length_, last_tilt_angle_, last_dot_leg_length_, last_dot_tilt_angle_;
    double last_left_leg_length_, last_right_leg_length_;

    VmcSolver left_leg_vmc_solver_, right_leg_vmc_solver_;
    filter::KalmanFilter<2, 2> velocity_kalman_filter_;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::Observer, rmcs_executor::Component)