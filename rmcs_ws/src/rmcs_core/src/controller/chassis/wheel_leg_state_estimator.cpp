#include "wheel_leg/joint_coordinates.hpp"
#include "wheel_leg/leg_kinematics.hpp"
#include "wheel_leg/model/mechanics.hpp"
#include "wheel_leg/policy/selection.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <numbers>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <stdexcept>
#include <string>
namespace rmcs_core::controller::chassis {
class WheelLegStateEstimator final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegStateEstimator()
        : Node{get_component_name(), node::options()}
        , model_(
              wheel_leg::mechanics::kThighLength, wheel_leg::mechanics::kShankLength,
              wheel_leg::LegLinkage::knee_limit_min(), wheel_leg::LegLinkage::knee_limit_max(),
              linkage_) {
        for (std::size_t i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kMotorNames[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
        }
        register_input("/wheel_leg/imu/quaternion", orientation_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/feedback_available", available_);
        register_input("/chassis/reset_count", reset_);
        register_output("/wheel_leg/state/position", q_, JointVector{});
        register_output("/wheel_leg/state/velocity", dq_, JointVector{});
        register_output("/wheel_leg/state/valid", valid_, false);
        register_output("/wheel_leg/state/gravity", gravity_, Eigen::Vector3d::Zero());
        register_output("/wheel_leg/state/attitude", attitude_, Eigen::Vector3d::Zero());
        register_output("/wheel_leg/state/passive_knee", knee_, std::array<double, 2>{});
        register_output("/wheel_leg/state/passive_knee_rate", knee_rate_, std::array<double, 2>{});
        register_output("/wheel_leg/state/shank", shank_, std::array<double, 2>{});
        register_output("/wheel_leg/state/shank_rate", shank_rate_, std::array<double, 2>{});
        register_output("/wheel_leg/state/leg_length", length_, std::array<double, 2>{});
        register_output("/wheel_leg/state/forward_velocity", forward_velocity_, 0.);
        register_output("/wheel_leg/state/distance", distance_output_, 0.);
        register_output("/wheel_leg/state/com_offset", com_offset_, 0.);
        register_output("/wheel_leg/state/base_height", height_, 0.);
        register_output("/wheel_leg/state/model_offsets", model_offsets_, kMotorOffsets);
        register_output(
            "/wheel_leg/state/calibration_sha256", calibration_sha256_,
            std::string{full_vehicle_calibration110::kCalibrationSha256});
        register_output(
            "/wheel_leg/state/calibration_model_spec_sha256", calibration_model_spec_sha256_,
            std::string{full_vehicle_calibration110::kModelSpecSha256});
        const auto policy = get_parameter_or<std::string>("policy", "traditional");
        bounds_ =
            policy == "traditional" ? kMechanicalBounds : policy_parameters(policy).winding_bounds;
        dt_ = get_parameter_or("dt", .001);
        velocity_sign_ = get_parameter_or("velocity_sign", -1.);
        if (!std::isfinite(dt_) || dt_ <= 0. || (velocity_sign_ != 1. && velocity_sign_ != -1.))
            throw std::invalid_argument("Invalid state estimator timing or velocity sign");
    }
    void update() override {
        if (*reset_ != last_reset_) {
            initialized_ = false;
            distance_ = 0.;
            last_reset_ = *reset_;
        }
        *valid_ = false;
        JointVector api{}, speed{};
        for (int i = 0; i < 6; ++i) {
            api[i] = *angle_[i];
            speed[i] = *velocity_[i];
            if (!std::isfinite(api[i]) || !std::isfinite(speed[i])) {
                initialized_ = false;
                return;
            }
        }
        if (!*available_ || !gyro_->allFinite() || !orientation_->coeffs().allFinite()
            || orientation_->norm() == 0.) {
            initialized_ = false;
            return;
        }
        const auto orientation = orientation_->normalized();
        auto q = model_position(api);
        LegVector measured{}, previous{};
        std::copy_n(q.begin(), 4, measured.begin());
        std::copy_n(q_->begin(), 4, previous.begin());
        const auto continuous =
            continuous_legs(measured, initialized_ ? previous : measured, bounds_);
        std::copy(continuous.begin(), continuous.end(), q.begin());
        *q_ = q;
        *dq_ = model_velocity(speed);
        *gravity_ = orientation.conjugate() * -Eigen::Vector3d::UnitZ();
        const double yaw = std::atan2(
            2. * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
            1. - 2. * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
        unwrapped_yaw_ = initialized_
                           ? unwrapped_yaw_ + std::remainder(yaw - last_yaw_, 2. * std::numbers::pi)
                           : yaw;
        last_yaw_ = yaw;
        *attitude_ = Eigen::Vector3d{
            std::atan2(gravity_->x(), -gravity_->z()), std::atan2(gravity_->y(), -gravity_->z()),
            unwrapped_yaw_};
        std::array<wheel_leg::LegShape, 2> shape;
        for (int side = 0; side < 2; ++side) {
            shape[side] = model_.forward(
                api[2 * side], api[2 * side + 1], wheel_leg::CanonicalCalibration::side(side));
            const auto& s = shape[side];
            (*knee_rate_)[side] = initialized_ ? (s.beta - (*knee_)[side]) / dt_ : 0.;
            (*shank_rate_)[side] = initialized_ ? (s.shank - (*shank_)[side]) / dt_ : 0.;
            (*knee_)[side] = s.beta;
            (*shank_)[side] = s.shank;
            (*length_)[side] = s.leg_length;
        }
        const double beta = .5 * (shape[0].beta + shape[1].beta),
                     phi = .5 * (shape[0].shank + shape[1].shank);
        double kx, kz, ax, az;
        model_.fk(beta, phi, kx, kz, ax, az);
        const double pitch = attitude_->x();
        *height_ = std::max(
            wheel_leg::mechanics::kWheelRadius + shape[0].axle_x * gravity_->x()
                + shape[0].axle_z * gravity_->z(),
            wheel_leg::mechanics::kWheelRadius + shape[1].axle_x * gravity_->x()
                + shape[1].axle_z * gravity_->z());
        const auto rot = [](double a, double x, double z) {
            return std::array{std::cos(a) * x - std::sin(a) * z, std::sin(a) * x + std::cos(a) * z};
        };
        const auto thigh =
            rot(wheel_leg::mechanics::kThighComAngle, kx / wheel_leg::mechanics::kThighLength,
                kz / wheel_leg::mechanics::kThighLength);
        const auto shank = rot(
            wheel_leg::mechanics::kShankComAngle, (ax - kx) / wheel_leg::mechanics::kShankLength,
            (az - kz) / wheel_leg::mechanics::kShankLength);
        const double body_x = -ax + std::cos(pitch) * wheel_leg::mechanics::kBodyComForward
                            + std::sin(pitch) * wheel_leg::mechanics::kBodyComUp;
        const double thigh_x = -ax + wheel_leg::mechanics::kThighComOffset * thigh[0],
                     shank_x = kx - ax + wheel_leg::mechanics::kShankComOffset * shank[0];
        *com_offset_ =
            (wheel_leg::mechanics::kBodyMass * body_x + wheel_leg::mechanics::kThighMass * thigh_x
             + wheel_leg::mechanics::kShankMass * shank_x)
            / (wheel_leg::mechanics::kBodyMass + wheel_leg::mechanics::kThighMass
               + wheel_leg::mechanics::kShankMass + wheel_leg::mechanics::kWheelMass);
        *forward_velocity_ =
            velocity_sign_ * wheel_leg::mechanics::kWheelRadius * .5 * (speed[4] + speed[5]);
        distance_ += *forward_velocity_ * dt_;
        *distance_output_ = distance_;
        initialized_ = true;
        *valid_ = true;
    }

private:
    std::array<InputInterface<double>, 6> angle_, velocity_;
    InputInterface<Eigen::Quaterniond> orientation_;
    InputInterface<Eigen::Vector3d> gyro_;
    InputInterface<bool> available_;
    InputInterface<std::size_t> reset_;
    OutputInterface<JointVector> q_, dq_;
    OutputInterface<bool> valid_;
    OutputInterface<Eigen::Vector3d> gravity_, attitude_;
    OutputInterface<std::array<double, 2>> knee_, knee_rate_, shank_, shank_rate_, length_;
    OutputInterface<double> forward_velocity_, distance_output_, com_offset_, height_;
    OutputInterface<LegVector> model_offsets_;
    OutputInterface<std::string> calibration_sha256_, calibration_model_spec_sha256_;
    wheel_leg::LegLinkage linkage_;
    wheel_leg::WheelLegLegModel model_;
    double dt_ = 0.001, velocity_sign_ = -1., distance_ = 0., last_yaw_ = 0., unwrapped_yaw_ = 0.;
    std::size_t last_reset_ = 0;
    LegBounds bounds_ = kMechanicalBounds;
    bool initialized_ = false;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegStateEstimator, rmcs_executor::Component)
