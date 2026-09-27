#include <cmath>
#include <limits>

#include <eigen3/Eigen/Geometry>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

// The board axes match the real chassis. Only the policy uses the training
// frame: x_RL = y_body, y_RL = -x_body, z_RL = z_body.
class WheelLegRlImu : public rmcs_executor::Component {
public:
    WheelLegRlImu() {
        register_input("/wheel_leg/imu/quaternion", orientation_);
        register_input("/wheel_leg/imu/angular_velocity", angular_velocity_);
        register_output(
            "/wheel_leg/rl/imu/angular_velocity", rl_angular_velocity_,
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
        register_output(
            "/wheel_leg/rl/imu/projected_gravity", rl_projected_gravity_,
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
    }

    void update() override {
        if (!orientation_.ready() || !angular_velocity_.ready()
            || !orientation_->coeffs().allFinite() || !angular_velocity_->allFinite()
            || !std::isfinite(orientation_->squaredNorm())
            || orientation_->squaredNorm() < 1e-12) {
            *rl_angular_velocity_ =
                Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            *rl_projected_gravity_ = *rl_angular_velocity_;
            return;
        }

        *rl_angular_velocity_ = body_to_rl(*angular_velocity_);
        // Bmi088Ekf publishes q_WB (body -> world). Training expects the
        // unit world-down vector expressed in its local base/IMU frame.
        const Eigen::Vector3d gravity_body =
            orientation_->normalized().conjugate() * Eigen::Vector3d{0.0, 0.0, -1.0};
        *rl_projected_gravity_ = body_to_rl(gravity_body);
    }

private:
    static Eigen::Vector3d body_to_rl(const Eigen::Vector3d& value) {
        return {value.y(), -value.x(), value.z()};
    }

    InputInterface<Eigen::Quaterniond> orientation_;
    InputInterface<Eigen::Vector3d> angular_velocity_;
    OutputInterface<Eigen::Vector3d> rl_angular_velocity_;
    OutputInterface<Eigen::Vector3d> rl_projected_gravity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegRlImu, rmcs_executor::Component)
