
#include <OsqpEigen/Constants.hpp>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <OsqpEigen/OsqpEigen.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "controller/mpc/mpc_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/control_mode.hpp>

namespace rmcs_core::controller::mpc {

class MPCController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MPCController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , wheel_velocity_controller_{pid::PidCalculator(1., 0., 0.), pid::PidCalculator(1., 0., 0.), pid::PidCalculator(1., 0., 0.), pid::PidCalculator(1., 0., 0.)}
        , steering_velocity_controller_{pid::PidCalculator(0.8, 0., 0.), pid::PidCalculator(0.8, 0., 0.), pid::PidCalculator(0.8, 0., 0.), pid::PidCalculator(0.8, 0., 0.)}
        , velocity_mpc() {
        beta << M_PI / 4., M_PI * 3. / 4., M_PI * 5. / 4., M_PI * 7. / 4.;
        register_input("/chassis/left_front_steering/angle", left_front_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_angle_);

        register_input("/chassis/left_front_steering/velocity", left_front_omega_);
        register_input("/chassis/left_back_steering/velocity", left_back_omega_);
        register_input("/chassis/right_back_steering/velocity", right_back_omega_);
        register_input("/chassis/right_front_steering/velocity", right_front_omega_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        // register_input("trajectory/velocity", manual_move_vector_);
        // register_input("trajectory/position", manual_move_vector_);
        register_input("/reference_vector", reference_vector);
        register_input("/control_mode", control_mode_);

        register_output(
            "/chassis/left_front_steering/control_torque", left_front_control_omega_, nan);
        register_output(
            "/chassis/left_back_steering/control_torque", left_back_control_omega_, nan);
        register_output(
            "/chassis/right_back_steering/control_torque", right_back_control_omega_, nan);
        register_output(
            "/chassis/right_front_steering/control_torque", right_front_control_omega_, nan);

        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_control_velocity_, nan);
        register_output(
            "/chassis/left_back_wheel/control_torque", left_back_control_velocity_, nan);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_control_velocity_, nan);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_control_velocity_, nan);
    }
    void update() override {

        if (*control_mode_ == rmcs_msgs::ControlMode::MANUAL) {

            Eigen::Vector<double, 4> force;
            Eigen::Vector<double, 4> velocity;
            Eigen::Vector<double, 4> theta;
            Eigen::Vector<double, 4> sin_theta;
            Eigen::Vector<double, 4> cos_theta;
            Eigen::Vector<double, 4> theta_k;
            Eigen::Vector<double, 4> alpha;
            Eigen::Vector<double, 4> sin_alpha;
            Eigen::Vector<double, 4> cos_alpha;
            Eigen::Vector<double, 4> alpha_k;
            Eigen::Vector<double, 4> sin_theta_k;
            Eigen::Vector<double, 4> sin_alpha_k;
            force << out(0), out(2), out(4), out(6);
            force *= 1;
            velocity << *left_front_velocity_, *left_back_velocity_, *right_back_velocity_,
                *right_front_velocity_;
            theta << out(1), out(3), out(5), out(7);
            sin_theta << sin(theta(0)), sin(theta(1)), sin(theta(2)), sin(theta(3));
            cos_theta << cos(theta(0)), cos(theta(1)), cos(theta(2)), cos(theta(3));
            alpha << *left_front_angle_, *left_back_angle_, *right_back_angle_, *right_front_angle_;
            cos_alpha << cos(alpha(0)), cos(alpha(1)), cos(alpha(2)), cos(alpha(3));
            sin_alpha << sin(alpha(0)), sin(alpha(1)), sin(alpha(2)), sin(alpha(3));
            theta_k = beta - theta;
            alpha_k = beta - alpha;
            sin_theta_k << sin(theta_k(0)), sin(theta_k(1)), sin(theta_k(2)), sin(theta_k(3));
            sin_alpha_k << sin(alpha_k(0)), sin(alpha_k(1)), sin(alpha_k(2)), sin(alpha_k(3));

            Eigen::Matrix<double, 3, 3> a;
            Eigen::Matrix<double, 3, 8> b;
            Eigen::DiagonalMatrix<double, 8 * mpcWindwos> R;
            Eigen::SparseMatrix<double> linearMatrix;
            Eigen::VectorXd upperBound;
            Eigen::VectorXd lowerBound;
            Eigen::Matrix<double, 3, 1> x0;
            a << Eigen::MatrixXd::Identity(3, 3);
            b << cos_theta(0), -sin_theta(0) * force(0), cos_theta(1), -sin_theta(1) * force(1),
                cos_theta(2), -sin_theta(2) * force(2), cos_theta(3), -sin_theta(3) * force(3),
                sin_theta(0), cos_theta(0) * force(0), sin_theta(1), cos_theta(1) * force(1),
                sin_theta(2), cos_theta(2) * force(2), sin_theta(3), cos_theta(3) * force(3),
                sin(theta_k(0)), cos(theta_k(0)) * force(0), sin(theta_k(1)),
                cos(theta_k(1)) * force(1), sin(theta_k(2)), cos(theta_k(2)) * force(2),
                sin(theta_k(3)), cos(theta_k(3)) * force(3);
            b = b * t;
            for (int i = 0; i < 8 * mpcWindwos; i += 2) {
                R.diagonal()(i)     = 0.1;
                R.diagonal()(i + 1) = 1;
            }
            linearMatrix.resize(8 * mpcWindwos, 8 * mpcWindwos + 1);
            upperBound.resize(8 * mpcWindwos);
            lowerBound.resize(8 * mpcWindwos);
            for (int i = 0; i < mpcWindwos; i++) {
                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k <= i; k++) {
                        linearMatrix.insert(i * 8 + j * 2 + 1, (i - k) * 8 + j * 2 + 1) = 1;
                        linearMatrix.insert(i * 8 + j * 2, (i - k) * 8 + j * 2)         = 1;
                    }
                    upperBound(i * 8 + j * 2 + 1) = 0.1 - velocity_mpc.u_hat_pre(j * 2 + 1);
                    lowerBound(i * 8 + j * 2 + 1) = -0.1 - velocity_mpc.u_hat_pre(j * 2 + 1);
                    upperBound(i * 8 + j * 2)     = 3. - velocity_mpc.u_pre(j * 2);
                    lowerBound(i * 8 + j * 2)     = -3. - velocity_mpc.u_pre(j * 2);
                }
            }
            x0 << cos_alpha.transpose() * velocity / 80, sin_alpha.transpose() * velocity / 80,
                sin_alpha_k.transpose() * velocity / 0.15f / 80;
            velocity_mpc.Update(
                a, b, 1, R, linearMatrix, lowerBound, upperBound, x0, *reference_vector, out);

            *left_front_control_velocity_  = out(0);
            *left_back_control_velocity_   = out(2);
            *right_back_control_velocity_  = out(4);
            *right_front_control_velocity_ = out(6);
            *left_front_control_omega_ =
                steering_velocity_controller_[0].update(cast_rad((out(1) - *left_front_angle_)));
            *left_back_control_omega_ =
                steering_velocity_controller_[1].update(cast_rad((out(3) - *left_back_angle_)));
            *right_back_control_omega_ =
                steering_velocity_controller_[2].update(cast_rad((out(5) - *right_back_angle_)));
            *right_front_control_omega_ =
                steering_velocity_controller_[3].update(cast_rad((out(7) - *right_front_angle_)));
            // RCLCPP_INFO(
            //     get_logger(), "%f,%f,%f,%f", *left_back_angle_, atan(tan((out)(1))), out(2),
            //     atan(tan(out(3))));
        } else {
            for (auto& i : wheel_velocity_controller_)
                i.reset();
            *left_front_control_velocity_  = nan;
            *left_back_control_velocity_   = nan;
            *right_back_control_velocity_  = nan;
            *right_front_control_velocity_ = nan;
            *left_front_control_omega_     = nan;
            *left_back_control_omega_      = nan;
            *right_back_control_omega_     = nan;
            *right_front_control_omega_    = nan;
            velocity_mpc.reset();
        }
    }

private:
    static double cast_rad(double rad) {
        while (rad > M_PI_2) {
            rad -= M_PI;
        }
        while (rad < -M_PI_2) {
            rad += M_PI;
        }
        return rad;
    }
    const Eigen::MatrixXd C         = Eigen::MatrixXd::Identity(14, 3);
    Eigen::Matrix<double, 8, 1> out = Eigen::MatrixXd::Zero(8, 1);

    double t                              = 0.01;
    static const constexpr int mpcWindwos = 3;

    static constexpr const double max_velocity = 28.937262372;

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    Eigen::Vector<double, 4> beta;

    InputInterface<Eigen::Matrix<double, 3, 1>> reference_vector;
    // InputInterface<Eigen::Vector2d> trajectory_velocity_;
    // InputInterface<Eigen::Vector2d> trajectory_position_;
    InputInterface<rmcs_msgs::ControlMode> control_mode_;

    InputInterface<double> left_front_angle_;
    InputInterface<double> left_back_angle_;
    InputInterface<double> right_back_angle_;
    InputInterface<double> right_front_angle_;

    InputInterface<double> left_front_omega_;
    InputInterface<double> left_back_omega_;
    InputInterface<double> right_back_omega_;
    InputInterface<double> right_front_omega_;

    InputInterface<double> left_front_velocity_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;
    InputInterface<double> right_front_velocity_;

    OutputInterface<double> left_front_control_omega_;
    OutputInterface<double> left_back_control_omega_;
    OutputInterface<double> right_back_control_omega_;
    OutputInterface<double> right_front_control_omega_;

    OutputInterface<double> left_front_control_velocity_;
    OutputInterface<double> left_back_control_velocity_;
    OutputInterface<double> right_back_control_velocity_;
    OutputInterface<double> right_front_control_velocity_;

    pid::PidCalculator wheel_velocity_controller_[4];
    pid::PidCalculator steering_velocity_controller_[4];

    MPCCalculator<3, 8, 3> velocity_mpc;
};
} // namespace rmcs_core::controller::mpc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::mpc::MPCController, rmcs_executor::Component)