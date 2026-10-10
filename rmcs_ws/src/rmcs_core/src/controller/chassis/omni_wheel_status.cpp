#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <rmcs_msgs/chassis_motion_state.hpp>

#ifndef RMCS_CHASSIS_MOTION_FILTER_TEST
# include <memory>
# include <string>

# include <rclcpp/node.hpp>
# include <rmcs_description/tf_description.hpp>
# include <rmcs_executor/component.hpp>
#endif

namespace rmcs_core::controller::chassis {

namespace {

class ChassisMotionFilter {
public:
    using Clock = std::chrono::steady_clock;

    struct Config {
        double wheel_radius = 0.07;
        double chassis_radius_x = 0.3;
        double chassis_radius_y = 0.3;
        double linear_response_time = 0.3;
        double angular_response_time = 0.2;
        double linear_acceleration_limit = 3.0;
        double angular_acceleration_limit = 12.0;
        double linear_process_noise = 1.0;
        double angular_process_noise = 1.0;
        double wheel_velocity_noise = 1.0;
        double imu_yaw_rate_noise = 0.03;
        double feedback_timeout = 0.05;
        double prediction_timeout = 0.1;
        double translation_enter = 0.05;
        double translation_exit = 0.03;
        double rotation_enter = 0.1;
        double rotation_exit = 0.06;
    };

    ChassisMotionFilter()
        : ChassisMotionFilter(Config{}) {}

    explicit ChassisMotionFilter(Config config)
        : config_(config) {
        const std::array values{
            config.wheel_radius,
            config.chassis_radius_x,
            config.chassis_radius_y,
            config.linear_response_time,
            config.angular_response_time,
            config.linear_acceleration_limit,
            config.angular_acceleration_limit,
            config.linear_process_noise,
            config.angular_process_noise,
            config.wheel_velocity_noise,
            config.imu_yaw_rate_noise,
            config.feedback_timeout,
            config.prediction_timeout,
            config.translation_enter,
            config.translation_exit,
            config.rotation_enter,
            config.rotation_exit};
        for (const double value : values)
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("Motion filter parameters must be finite and positive");
        if (config.translation_exit > config.translation_enter
            || config.rotation_exit > config.rotation_enter
            || config.prediction_timeout < config.feedback_timeout)
            throw std::invalid_argument("Invalid motion filter hysteresis or feedback timeouts");

        const double length = config.chassis_radius_x + config.chassis_radius_y;
        wheel_model_ << -1, 1, length, -1, -1, length, 1, -1, length, 1, 1, length;
        wheel_model_ *= -1.0 / (std::numbers::sqrt2 * config.wheel_radius);
        odometry_model_ << 1, 1, -1, -1, -1, 1, 1, -1, -1 / length, -1 / length, -1 / length,
            -1 / length;
        odometry_model_ *= std::numbers::sqrt2 * config.wheel_radius / 4;
        if (!wheel_model_.allFinite() || !odometry_model_.allFinite()
            || !std::isfinite(config.wheel_velocity_noise * config.wheel_velocity_noise)
            || !std::isfinite(config.imu_yaw_rate_noise * config.imu_yaw_rate_noise)
            || !std::isfinite(config.linear_process_noise * config.linear_process_noise)
            || !std::isfinite(config.angular_process_noise * config.angular_process_noise)
            || !std::isfinite(1.0 / config.linear_response_time)
            || !std::isfinite(1.0 / config.angular_response_time))
            throw std::invalid_argument(
                "Motion filter geometry, noise, or response rate overflows");
        reset();
    }

    void reset() {
        initialized_ = false;
        translating_ = rotating_ = false;
        velocity_.setZero();
        covariance_.setIdentity();
        wheel_sequence_.fill(0);
        imu_sequence_ = 0;
        last_update_ = {};
    }

    rmcs_msgs::ChassisMotionState update(
        const rmcs_msgs::ChassisMotionFeedback& feedback, const Eigen::Vector3d& previous_command,
        Clock::time_point now) {
        rmcs_msgs::ChassisMotionState result;
        result.timestamp = now;
        std::array<double, 4> ages;
        for (std::size_t i = 0; i < ages.size(); ++i)
            ages[i] = sample_age(
                feedback.wheel_velocity[i], feedback.wheel_sequence[i], feedback.wheel_stamp[i],
                now);
        const double oldest_age = *std::max_element(ages.begin(), ages.end());
        const bool wheels_fresh = oldest_age <= config_.feedback_timeout;
        const double imu_age =
            sample_age(feedback.yaw_rate, feedback.imu_sequence, feedback.imu_stamp, now);
        const bool imu_fresh = imu_age <= config_.feedback_timeout;
        const double dt = initialized_ ? seconds(now - last_update_) : 0.0;
        if (initialized_ && (dt < 0.0 || dt > config_.prediction_timeout)) {
            reset();
            return result;
        }
        // Gyro alone cannot make expired translational feedback valid again.
        if (oldest_age > config_.prediction_timeout) {
            reset();
            return result;
        }
        if (!initialized_) {
            if (!wheels_fresh)
                return result;
            initialize(feedback, ages, imu_fresh, imu_age);
        } else {
            predict(previous_command, dt);
            if (imu_fresh) {
                bool new_wheel_batch = wheels_fresh;
                for (std::size_t i = 0; i < ages.size(); ++i)
                    new_wheel_batch &= feedback.wheel_sequence[i] != wheel_sequence_[i];
                if (new_wheel_batch && feedback.imu_sequence != imu_sequence_) {
                    correct(compensated_measurement(feedback, ages, imu_age));
                    wheel_sequence_ = feedback.wheel_sequence;
                    imu_sequence_ = feedback.imu_sequence;
                } else if (feedback.imu_sequence != imu_sequence_) {
                    correct(
                        Eigen::Vector3d::UnitZ(), feedback.yaw_rate,
                        measurement_variance(config_.imu_yaw_rate_noise, imu_age));
                    imu_sequence_ = feedback.imu_sequence;
                }
            } else {
                // Without fresh gyro, keep the existing wheel-only fallback.
                for (std::size_t i = 0; i < ages.size(); ++i) {
                    if (ages[i] <= config_.feedback_timeout
                        && feedback.wheel_sequence[i] != wheel_sequence_[i]) {
                        correct(
                            wheel_model_.row(i).transpose(), feedback.wheel_velocity[i],
                            measurement_variance(config_.wheel_velocity_noise, ages[i]));
                        wheel_sequence_[i] = feedback.wheel_sequence[i];
                    }
                }
            }
        }
        last_update_ = now;
        if (!velocity_.allFinite() || !covariance_.allFinite()
            || (covariance_.diagonal().array() < 0.0).any()) {
            reset();
            return result;
        }
        result.velocity = velocity_;
        result.covariance = covariance_;
        result.quality = wheels_fresh ? (imu_fresh ? rmcs_msgs::MotionQuality::FUSED
                                                   : rmcs_msgs::MotionQuality::WHEEL_ONLY)
                                      : rmcs_msgs::MotionQuality::PREDICTED;
        if (wheels_fresh)
            result.kind = classify();
        return result;
    }

private:
    struct VelocityMeasurement {
        Eigen::Vector3d velocity;
        Eigen::Matrix3d covariance;
    };

    static double seconds(Clock::duration duration) {
        return std::chrono::duration<double>(duration).count();
    }

    static double sample_age(
        double value, std::uint64_t sequence, Clock::time_point stamp, Clock::time_point now) {
        if (sequence == 0 || !std::isfinite(value) || stamp > now)
            return std::numeric_limits<double>::infinity();
        return seconds(now - stamp);
    }

    double measurement_variance(double noise, double age) const {
        // Reception times are host times. For small sample delays, inflate R at
        // the current estimate time; this is not historical replay or deskewing.
        return noise * noise * (1.0 + age / config_.feedback_timeout);
    }

    VelocityMeasurement compensated_measurement(
        const rmcs_msgs::ChassisMotionFeedback& feedback, const std::array<double, 4>& ages,
        double imu_age) const {
        const double k = std::numbers::sqrt2 * config_.wheel_radius;
        const double length = config_.chassis_radius_x + config_.chassis_radius_y;
        const double rotation = -length * feedback.yaw_rate / k;
        Eigen::Vector4d translation;
        for (std::size_t i = 0; i < ages.size(); ++i)
            translation[i] = feedback.wheel_velocity[i] - rotation;

        // Opposite wheels measure the same translation projection with opposite
        // signs. Select the smaller magnitude after removing rotation, keeping
        // its sign so reverse motion is handled in the same way as forward motion.
        const std::size_t a = std::abs(translation[0]) <= std::abs(translation[2]) ? 0 : 2;
        const std::size_t b = std::abs(translation[1]) <= std::abs(translation[3]) ? 1 : 3;
        const double sign_a = a == 0 ? 1.0 : -1.0;
        const double sign_b = b == 1 ? 1.0 : -1.0;
        const double projection_a = sign_a * k * translation[a];
        const double projection_b = sign_b * k * translation[b];

        VelocityMeasurement result;
        result.velocity = {
            (projection_a + projection_b) / 2.0, (projection_b - projection_a) / 2.0,
            feedback.yaw_rate};
        // The same gyro is used in rotation removal and the yaw observation.
        // Preserve their correlation instead of counting gyro noise twice.
        Eigen::Matrix3d transform;
        transform << k / 2.0 * sign_a, k / 2.0 * sign_b, length / 2.0 * (sign_a + sign_b),
            -k / 2.0 * sign_a, k / 2.0 * sign_b, length / 2.0 * (sign_b - sign_a), 0, 0, 1;
        const Eigen::Vector3d variances{
            measurement_variance(config_.wheel_velocity_noise, ages[a]),
            measurement_variance(config_.wheel_velocity_noise, ages[b]),
            measurement_variance(config_.imu_yaw_rate_noise, imu_age)};
        result.covariance = transform * variances.asDiagonal() * transform.transpose();
        return result;
    }

    void initialize(
        const rmcs_msgs::ChassisMotionFeedback& feedback, const std::array<double, 4>& ages,
        bool imu_fresh, double imu_age) {
        if (imu_fresh) {
            const auto measurement = compensated_measurement(feedback, ages, imu_age);
            velocity_ = measurement.velocity;
            covariance_ = measurement.covariance;
            imu_sequence_ = feedback.imu_sequence;
        } else {
            const Eigen::Map<const Eigen::Vector4d> wheels(feedback.wheel_velocity.data());
            velocity_ = odometry_model_ * wheels;
            Eigen::Vector4d variances;
            for (std::size_t i = 0; i < ages.size(); ++i)
                variances[i] = measurement_variance(config_.wheel_velocity_noise, ages[i]);
            covariance_ = odometry_model_ * variances.asDiagonal() * odometry_model_.transpose();
        }
        // Initialization already consumes these samples; do not fuse them twice.
        wheel_sequence_ = feedback.wheel_sequence;
        initialized_ = true;
    }

    void predict(const Eigen::Vector3d& command, double dt) {
        // Bound integration steps even if the host update loop misses a few ticks.
        const int steps = std::max(1, static_cast<int>(std::ceil(dt / 0.005)));
        const double step = dt / steps;
        for (int iteration = 0; iteration < steps; ++iteration) {
            Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
            Eigen::Matrix2d linear_jacobian = Eigen::Matrix2d::Zero();
            double yaw_acceleration = 0.0, yaw_jacobian = 0.0;
            if (command.allFinite()) {
                const Eigen::Vector2d error = command.head<2>() - velocity_.head<2>();
                if (error.allFinite()) {
                    const double norm = error.stableNorm();
                    if (norm > config_.linear_acceleration_limit * config_.linear_response_time) {
                        // Scale first so even very large finite commands normalize safely.
                        Eigen::Vector2d direction = error / error.cwiseAbs().maxCoeff();
                        direction.normalize();
                        acceleration = config_.linear_acceleration_limit * direction;
                        linear_jacobian =
                            -config_.linear_acceleration_limit / norm
                            * (Eigen::Matrix2d::Identity() - direction * direction.transpose());
                    } else {
                        acceleration = error / config_.linear_response_time;
                        linear_jacobian =
                            -Eigen::Matrix2d::Identity() / config_.linear_response_time;
                    }
                }
                const double error_yaw = command.z() - velocity_.z();
                yaw_acceleration = std::clamp(
                    error_yaw / config_.angular_response_time, -config_.angular_acceleration_limit,
                    config_.angular_acceleration_limit);
                if (std::abs(error_yaw)
                    < config_.angular_acceleration_limit * config_.angular_response_time)
                    yaw_jacobian = -1.0 / config_.angular_response_time;
            }
            // Missing commands preserve inertial motion. The body frame still
            // rotates: dvx/dt = ax + wz*vy, dvy/dt = ay - wz*vx.
            Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
            jacobian.topLeftCorner<2, 2>() = linear_jacobian;
            jacobian(0, 1) += velocity_.z();
            jacobian(1, 0) -= velocity_.z();
            jacobian(0, 2) = velocity_.y();
            jacobian(1, 2) = -velocity_.x();
            jacobian(2, 2) = yaw_jacobian;
            const Eigen::Matrix3d transition = Eigen::Matrix3d::Identity() + step * jacobian;
            const Eigen::Vector3d derivative{
                acceleration.x() + velocity_.z() * velocity_.y(),
                acceleration.y() - velocity_.z() * velocity_.x(), yaw_acceleration};
            velocity_ += step * derivative;
            covariance_ = (transition * covariance_ * transition.transpose()).eval();
            covariance_.diagonal().array() +=
                step
                * Eigen::Array3d{
                    config_.linear_process_noise * config_.linear_process_noise,
                    config_.linear_process_noise * config_.linear_process_noise,
                    config_.angular_process_noise * config_.angular_process_noise};
        }
    }

    void correct(const VelocityMeasurement& measurement) {
        const Eigen::Matrix3d innovation_covariance = covariance_ + measurement.covariance;
        const Eigen::LDLT<Eigen::Matrix3d> decomposition{innovation_covariance};
        if (decomposition.info() != Eigen::Success
            || (decomposition.vectorD().array() <= 0.0).any())
            return;
        const Eigen::Matrix3d gain = decomposition.solve(covariance_.transpose()).transpose();
        velocity_ += (gain * (measurement.velocity - velocity_)).eval();
        const Eigen::Matrix3d residual = Eigen::Matrix3d::Identity() - gain;
        covariance_ = (residual * covariance_ * residual.transpose()
                       + gain * measurement.covariance * gain.transpose())
                          .eval();
        covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();
    }

    void correct(const Eigen::Vector3d& model, double measurement, double variance) {
        const Eigen::Vector3d projected = covariance_ * model;
        const double innovation_variance = model.dot(projected) + variance;
        if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0)
            return;
        const double innovation = measurement - model.dot(velocity_);
        if (!std::isfinite(innovation))
            return;
        const Eigen::Vector3d gain = projected / innovation_variance;
        velocity_ += gain * innovation;
        // Joseph form preserves positive covariance through scalar updates.
        const Eigen::Matrix3d residual = Eigen::Matrix3d::Identity() - gain * model.transpose();
        covariance_ =
            (residual * covariance_ * residual.transpose() + variance * gain * gain.transpose())
                .eval();
        covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();
    }

    rmcs_msgs::MotionKind classify() {
        const double translation = velocity_.head<2>().stableNorm();
        const double rotation = std::abs(velocity_.z());
        translating_ = translating_ ? translation > config_.translation_exit
                                    : translation >= config_.translation_enter;
        rotating_ =
            rotating_ ? rotation > config_.rotation_exit : rotation >= config_.rotation_enter;
        if (translating_ && rotating_)
            return rmcs_msgs::MotionKind::COMBINED;
        if (translating_)
            return rmcs_msgs::MotionKind::TRANSLATING;
        if (rotating_)
            return rmcs_msgs::MotionKind::ROTATING;
        return rmcs_msgs::MotionKind::STATIONARY;
    }

    Config config_;
    Eigen::Matrix<double, 4, 3> wheel_model_;
    Eigen::Matrix<double, 3, 4> odometry_model_;
    Eigen::Vector3d velocity_;
    Eigen::Matrix3d covariance_;
    std::array<std::uint64_t, 4> wheel_sequence_{};
    std::uint64_t imu_sequence_ = 0;
    Clock::time_point last_update_{};
    bool initialized_ = false, translating_ = false, rotating_ = false;
};

} // namespace

#ifndef RMCS_CHASSIS_MOTION_FILTER_TEST
class OmniWheelStatus
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OmniWheelStatus()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , filter_(read_config()) {
        register_input("/chassis/motion/feedback", feedback_);
        register_output("/chassis/motion_state", state_);
        constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
        register_output("/chassis/motion/vx", velocity_x_, kNaN);
        register_output("/chassis/motion/vy", velocity_y_, kNaN);
        register_output("/chassis/motion/wz", yaw_rate_, kNaN);
        register_output("/chassis/motion/speed", speed_, kNaN);

        command_capture_ = create_partner_component<CommandCapture>(
            get_component_name() + "_command_capture", previous_command_);
    }

    void update() override {
        // State estimation is independent of motor enable and controller mode.
        *state_ = filter_.update(*feedback_, previous_command_, Clock::now());
        *velocity_x_ = state_->velocity.x();
        *velocity_y_ = state_->velocity.y();
        *yaw_rate_ = state_->velocity.z();
        *speed_ = std::hypot(state_->velocity.x(), state_->velocity.y());
        if (log_state_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "motion vx=%.3f vy=%.3f wz=%.3f kind=%s quality=%s", state_->velocity.x(),
                state_->velocity.y(), state_->velocity.z(), kind_name(state_->kind),
                quality_name(state_->quality));
        }
    }

private:
    using Clock = rmcs_msgs::ChassisMotionState::Clock;

    class CommandCapture final : public rmcs_executor::Component {
    public:
        explicit CommandCapture(Eigen::Vector3d& previous_command)
            : previous_command_(previous_command) {
            register_input("/chassis/control_velocity", command_, false);
            // This dependency places capture after state estimation. It also
            // permits a future controller to consume motion_state without a cycle.
            register_input("/chassis/motion_state", state_dependency_);
        }

        void update() override {
            if (command_.ready())
                previous_command_ = command_->vector;
            else
                previous_command_.setConstant(std::numeric_limits<double>::quiet_NaN());
        }

    private:
        Eigen::Vector3d& previous_command_;
        InputInterface<rmcs_description::BaseLink::DirectionVector> command_;
        InputInterface<rmcs_msgs::ChassisMotionState> state_dependency_;
    };

    double parameter(const std::string& name, double fallback) {
        return has_parameter(name) ? get_parameter(name).as_double()
                                   : declare_parameter<double>(name, fallback);
    }

    ChassisMotionFilter::Config read_config() {
        auto config = ChassisMotionFilter::Config{};
        config.wheel_radius = parameter("wheel_radius", config.wheel_radius);
        config.chassis_radius_x = parameter("chassis_radius_x", config.chassis_radius_x);
        config.chassis_radius_y = parameter("chassis_radius_y", config.chassis_radius_y);
        config.linear_response_time =
            parameter("linear_response_time", config.linear_response_time);
        config.angular_response_time =
            parameter("angular_response_time", config.angular_response_time);
        config.linear_acceleration_limit =
            parameter("linear_acceleration_limit", config.linear_acceleration_limit);
        config.angular_acceleration_limit =
            parameter("angular_acceleration_limit", config.angular_acceleration_limit);
        config.linear_process_noise =
            parameter("linear_process_noise", config.linear_process_noise);
        config.angular_process_noise =
            parameter("angular_process_noise", config.angular_process_noise);
        config.wheel_velocity_noise =
            parameter("wheel_velocity_noise", config.wheel_velocity_noise);
        config.imu_yaw_rate_noise = parameter("imu_yaw_rate_noise", config.imu_yaw_rate_noise);
        config.feedback_timeout = parameter("feedback_timeout", config.feedback_timeout);
        config.prediction_timeout = parameter("prediction_timeout", config.prediction_timeout);
        config.translation_enter = parameter("translation_enter", config.translation_enter);
        config.translation_exit = parameter("translation_exit", config.translation_exit);
        config.rotation_enter = parameter("rotation_enter", config.rotation_enter);
        config.rotation_exit = parameter("rotation_exit", config.rotation_exit);
        log_state_ = has_parameter("log_state") ? get_parameter("log_state").as_bool()
                                                : declare_parameter<bool>("log_state", true);
        return config;
    }

    static const char* kind_name(rmcs_msgs::MotionKind kind) {
        using rmcs_msgs::MotionKind;
        switch (kind) {
        case MotionKind::UNKNOWN: return "unknown";
        case MotionKind::STATIONARY: return "stationary";
        case MotionKind::TRANSLATING: return "translating";
        case MotionKind::ROTATING: return "rotating";
        case MotionKind::COMBINED: return "combined";
        }
        return "unknown";
    }

    static const char* quality_name(rmcs_msgs::MotionQuality quality) {
        using rmcs_msgs::MotionQuality;
        switch (quality) {
        case MotionQuality::INVALID: return "invalid";
        case MotionQuality::PREDICTED: return "predicted";
        case MotionQuality::WHEEL_ONLY: return "wheel_only";
        case MotionQuality::FUSED: return "fused";
        }
        return "invalid";
    }

    bool log_state_ = true;
    ChassisMotionFilter filter_;
    InputInterface<rmcs_msgs::ChassisMotionFeedback> feedback_;
    OutputInterface<rmcs_msgs::ChassisMotionState> state_;
    OutputInterface<double> velocity_x_, velocity_y_, yaw_rate_, speed_;
    Eigen::Vector3d previous_command_ =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    std::shared_ptr<CommandCapture> command_capture_;
};
#endif

} // namespace rmcs_core::controller::chassis

#ifndef RMCS_CHASSIS_MOTION_FILTER_TEST
# include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::OmniWheelStatus, rmcs_executor::Component)
#endif
