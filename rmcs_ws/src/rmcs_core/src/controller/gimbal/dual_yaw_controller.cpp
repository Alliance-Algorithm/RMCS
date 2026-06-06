
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/gimbal_mode.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

class DualYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
            pid.kp = get_parameter(name + "_kp").as_double();
            pid.ki = get_parameter(name + "_ki").as_double();
            pid.kd = get_parameter(name + "_kd").as_double();
            get_parameter(name + "_integral_min", pid.integral_min);
            get_parameter(name + "_integral_max", pid.integral_max);
            get_parameter(name + "_output_min", pid.output_min);
            get_parameter(name + "_output_max", pid.output_max);
        };
        set_pid_parameter(top_yaw_angle_pid_, "top_yaw_angle");
        set_pid_parameter(top_yaw_velocity_pid_, "top_yaw_velocity");
        set_pid_parameter(bottom_yaw_angle_pid_, "bottom_yaw_angle");
        set_pid_parameter(bottom_yaw_velocity_pid_, "bottom_yaw_velocity");

        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/raw_angle", bottom_yaw_raw_angle_);

        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);
        register_input("/gimbal/yaw_brake/velocity", yaw_brake_velocity_);

        register_input("/gimbal/mode", gimbal_mode_);
        register_input("/gimbal/yaw/control_angle_error", control_angle_error_);
        register_input("/gimbal/yaw/control_angle_shift", control_angle_shift_, false);
        register_input("/gimbal/bottom_yaw/torque", bottom_yaw_torque_);

        register_output("/gimbal/top_yaw/control_torque", top_yaw_control_torque_, 0.0);
        register_output("/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque_, 0.0);
        register_output("/gimbal/bottom_yaw/control_angle", bottom_yaw_control_angle_, nan_);
        register_output("/gimbal/top_yaw/control_angle_shift", top_yaw_control_angle_shift_, nan_);
        register_output("/gimbal/yaw_brake/control_torque", yaw_brake_control_torque_, nan_);

        status_component_ =
            create_partner_component<DualYawStatus>(get_component_name() + "_status");
    }

    void before_updating() override {
        if (!control_angle_shift_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_shift\", set to NaN.");
            control_angle_shift_.bind_directly(nan_);
        }
    }

    void update() override {
        const auto mode = *gimbal_mode_;

        const bool entering_encoder = mode == rmcs_msgs::GimbalMode::ENCODER
                                   && last_gimbal_mode_ != rmcs_msgs::GimbalMode::ENCODER;

        const bool leaving_encoder = mode != rmcs_msgs::GimbalMode::ENCODER
                                  && last_gimbal_mode_ == rmcs_msgs::GimbalMode::ENCODER;

        if (entering_encoder) {
            top_yaw_angle_pid_.reset();
            top_yaw_velocity_pid_.reset();
            bottom_yaw_angle_pid_.reset();
            bottom_yaw_velocity_pid_.reset();

            yaw_brake_engage_count_ = 0;
            yaw_brake_release_elapsed_count_ = 0;
            yaw_brake_release_stop_count_ = 0;
            yaw_brake_release_seen_motion_ = false;
            encoder_state_ = EncoderState::AlignTargetRawAngle;
        }

        if (leaving_encoder) {
            top_yaw_angle_pid_.reset();
            top_yaw_velocity_pid_.reset();
            bottom_yaw_angle_pid_.reset();
            bottom_yaw_velocity_pid_.reset();

            if (encoder_state_ == EncoderState::EngageYawBrake
                || encoder_state_ == EncoderState::BrakeLocked) {
                yaw_brake_release_elapsed_count_ = 0;
                yaw_brake_release_stop_count_ = 0;
                yaw_brake_release_seen_motion_ = false;
                encoder_state_ = EncoderState::ReleaseYawBrake;
            } else {
                yaw_brake_engage_count_ = 0;
                yaw_brake_release_elapsed_count_ = 0;
                yaw_brake_release_stop_count_ = 0;
                yaw_brake_release_seen_motion_ = false;
                *yaw_brake_control_torque_ = nan_;
                encoder_state_ = EncoderState::Idle;
            }
        }
        if (mode == rmcs_msgs::GimbalMode::ENCODER && encoder_state_ == EncoderState::Idle) {
            top_yaw_angle_pid_.reset();
            top_yaw_velocity_pid_.reset();
            bottom_yaw_angle_pid_.reset();
            bottom_yaw_velocity_pid_.reset();

            yaw_brake_engage_count_ = 0;
            yaw_brake_release_elapsed_count_ = 0;
            yaw_brake_release_stop_count_ = 0;
            yaw_brake_release_seen_motion_ = false;
            encoder_state_ = EncoderState::AlignTargetRawAngle;
        }

        // RCLCPP_INFO(get_logger(), "bottom_yaw_raw_angle: %ld", *bottom_yaw_raw_angle_);
        // RCLCPP_INFO(get_logger(), "bottom_yaw_control_torque: %f", *bottom_yaw_control_torque_);
        // RCLCPP_INFO(get_logger(), "bottom_yaw_torque: %f", *bottom_yaw_torque_);
        // RCLCPP_INFO(
        //     get_logger(), "encoder_state: %s",
        //     encoder_state_ == EncoderState::Idle                  ? "Idle"
        //     : encoder_state_ == EncoderState::AlignTargetRawAngle ? "AlignTargetRawAngle"
        //     : encoder_state_ == EncoderState::EngageYawBrake      ? "EngageYawBrake"
        //     : encoder_state_ == EncoderState::BrakeLocked         ? "BrakeLocked"
        //                                                           : "ReleaseYawBrake");
        const bool hold_encoder_for_brake_release = encoder_state_ == EncoderState::ReleaseYawBrake;

        if (mode == rmcs_msgs::GimbalMode::ENCODER || hold_encoder_for_brake_release) {
            *top_yaw_control_torque_ = nan_;
            *bottom_yaw_control_angle_ = nan_;

            auto wrap_raw_delta = [](int64_t diff) -> int64_t {
                diff %= kBottomYawRawAngleModulus;
                if (diff <= -(kBottomYawRawAngleModulus / 2))
                    diff += kBottomYawRawAngleModulus;
                else if (diff > (kBottomYawRawAngleModulus / 2))
                    diff -= kBottomYawRawAngleModulus;
                return diff;
            };

            if (encoder_state_ == EncoderState::AlignTargetRawAngle) {
                *top_yaw_control_angle_shift_ = 0.0;
                *yaw_brake_control_torque_ = nan_;

                if (!bottom_yaw_raw_angle_.ready()) {
                    *bottom_yaw_control_torque_ = nan_;
                } else {
                    const int64_t raw_error_count =
                        wrap_raw_delta(kEncoderBottomYawTargetRawAngle - *bottom_yaw_raw_angle_);
                    const int64_t raw_error_abs =
                        raw_error_count >= 0 ? raw_error_count : -raw_error_count;

                    const bool bottom_yaw_aligned =
                        raw_error_abs <= kEncoderBottomYawTargetToleranceRawAngle
                        && std::abs(*bottom_yaw_velocity_)
                               <= kEncoderBottomYawLockVelocityThreshold;

                    if (bottom_yaw_aligned) {
                        *bottom_yaw_control_torque_ = nan_;
                        bottom_yaw_angle_pid_.reset();
                        bottom_yaw_velocity_pid_.reset();
                        yaw_brake_engage_count_ = 0;
                        encoder_state_ = EncoderState::EngageYawBrake;
                    } else {
                        const double raw_error_angle =
                            kEncoderBottomYawRawAngleErrorSign
                            * static_cast<double>(raw_error_count)
                            / static_cast<double>(kBottomYawRawAngleModulus) * 2.0
                            * std::numbers::pi;

                        const double target_velocity =
                            bottom_yaw_angle_pid_.update(raw_error_angle);
                        const double velocity_error = target_velocity - *bottom_yaw_velocity_;
                        double control_torque = bottom_yaw_velocity_pid_.update(velocity_error);

                        constexpr double kBreakawayTorque = 2.5;
                        constexpr double kBreakawayVelocityThreshold = 0.10;
                        constexpr int64_t kBreakawayErrorThreshold = 80;

                        if (std::abs(*bottom_yaw_velocity_) < kBreakawayVelocityThreshold
                            && std::abs(raw_error_count) > kBreakawayErrorThreshold
                            && std::abs(control_torque) < kBreakawayTorque) {
                            control_torque =
                                velocity_error >= 0.0 ? kBreakawayTorque : -kBreakawayTorque;
                        }

                        *bottom_yaw_control_torque_ = control_torque;
                    }
                }

            } else if (encoder_state_ == EncoderState::EngageYawBrake) {
                *top_yaw_control_angle_shift_ = 0.0;
                *bottom_yaw_control_torque_ = nan_;
                *yaw_brake_control_torque_ = kYawBrakeEngageTorque;

                if (std::abs(*yaw_brake_velocity_) < kYawBrakeEngageVelocityThreshold) {
                    ++yaw_brake_engage_count_;
                } else {
                    yaw_brake_engage_count_ = 0;
                }

                if (yaw_brake_engage_count_ >= kYawBrakeEngageConfirmCount) {
                    yaw_brake_engage_count_ = 0;
                    *yaw_brake_control_torque_ = nan_;
                    encoder_state_ = EncoderState::BrakeLocked;
                }

            } else if (encoder_state_ == EncoderState::BrakeLocked) {
                constexpr double kBrakeLockedHoldTorque = 0.6;
                *bottom_yaw_control_torque_ = kBrakeLockedHoldTorque;
                *yaw_brake_control_torque_ = nan_;
                *top_yaw_control_angle_shift_ = *control_angle_shift_;

            } else if (encoder_state_ == EncoderState::ReleaseYawBrake) {
                *top_yaw_control_angle_shift_ = 0.0;
                *bottom_yaw_control_torque_ = nan_;
                *yaw_brake_control_torque_ = kYawBrakeReleaseTorque;

                ++yaw_brake_release_elapsed_count_;

                if (std::abs(*yaw_brake_velocity_) > kYawBrakeReleaseMotionVelocityThreshold) {
                    yaw_brake_release_seen_motion_ = true;
                }

                if (yaw_brake_release_seen_motion_
                    && std::abs(*yaw_brake_velocity_) < kYawBrakeReleaseStopVelocityThreshold) {
                    ++yaw_brake_release_stop_count_;
                } else {
                    yaw_brake_release_stop_count_ = 0;
                }

                if (yaw_brake_release_stop_count_ >= kYawBrakeReleaseStopConfirmCount) {
                    yaw_brake_engage_count_ = 0;
                    yaw_brake_release_elapsed_count_ = 0;
                    yaw_brake_release_stop_count_ = 0;
                    yaw_brake_release_seen_motion_ = false;
                    *yaw_brake_control_torque_ = nan_;
                    encoder_state_ = EncoderState::Idle;
                } else if (yaw_brake_release_elapsed_count_ >= kYawBrakeReleaseTimeoutCount) {
                    RCLCPP_WARN(
                        get_logger(), "Yaw brake release timed out, stop applying reverse torque "
                                      "for protection.");
                    yaw_brake_engage_count_ = 0;
                    yaw_brake_release_elapsed_count_ = 0;
                    yaw_brake_release_stop_count_ = 0;
                    yaw_brake_release_seen_motion_ = false;
                    *yaw_brake_control_torque_ = nan_;
                    encoder_state_ = EncoderState::Idle;
                }

            } else {
                *top_yaw_control_angle_shift_ = 0.0;
                *bottom_yaw_control_torque_ = nan_;
                *yaw_brake_control_torque_ = nan_;
            }

        } else {
            *yaw_brake_control_torque_ = nan_;

            *top_yaw_control_torque_ = top_yaw_velocity_pid_.update(
                top_yaw_angle_pid_.update(*control_angle_error_) - *gimbal_yaw_velocity_imu_);

            *bottom_yaw_control_torque_ = bottom_yaw_velocity_pid_.update(
                bottom_yaw_angle_pid_.update(bottom_yaw_control_error())
                - bottom_yaw_velocity_imu());

            *bottom_yaw_control_angle_ = nan_;
            *top_yaw_control_angle_shift_ = nan_;
        }

        last_gimbal_mode_ = mode;
    }

private:
    enum class EncoderState {
        Idle,
        AlignTargetRawAngle,
        EngageYawBrake,
        BrakeLocked,
        ReleaseYawBrake
    };
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr int64_t kBottomYawRawAngleModulus = 1 << 16;
    static constexpr int64_t kEncoderBottomYawTargetRawAngle = 2434;
    static constexpr int64_t kEncoderBottomYawTargetToleranceRawAngle = 80;
    static constexpr double kEncoderBottomYawLockVelocityThreshold = 0.15;
    static constexpr double kEncoderBottomYawRawAngleErrorSign = -1.0;

    static constexpr double kYawBrakeEngageTorque = -0.3;
    static constexpr double kYawBrakeEngageVelocityThreshold = 0.1;
    static constexpr int kYawBrakeEngageConfirmCount = 50;

    static constexpr double kYawBrakeReleaseTorque = 0.3;
    static constexpr double kYawBrakeReleaseMotionVelocityThreshold = 0.2;
    static constexpr double kYawBrakeReleaseStopVelocityThreshold = 0.08;
    static constexpr int kYawBrakeReleaseStopConfirmCount = 20;
    static constexpr int kYawBrakeReleaseTimeoutCount = 400;

    double bottom_yaw_control_error() {
        if (!std::isfinite(*top_yaw_angle_) || !std::isfinite(*control_angle_error_))
            return nan_;

        constexpr double alignment = 2 * std::numbers::pi;
        double err =
            std::fmod(*top_yaw_angle_ + *control_angle_error_ + std::numbers::pi, alignment);
        if (err < 0)
            err += alignment;
        err -= std::numbers::pi;
        return err;
    }

    double bottom_yaw_velocity_imu() { return *chassis_yaw_velocity_imu_ + *bottom_yaw_velocity_; }

    InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
    InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;
    InputInterface<int64_t> bottom_yaw_raw_angle_;

    InputInterface<double> gimbal_yaw_velocity_imu_, chassis_yaw_velocity_imu_;
    InputInterface<double> yaw_brake_velocity_;

    InputInterface<rmcs_msgs::GimbalMode> gimbal_mode_;
    InputInterface<double> control_angle_error_, control_angle_shift_;
    InputInterface<double> bottom_yaw_torque_;

    pid::PidCalculator top_yaw_angle_pid_, top_yaw_velocity_pid_;
    pid::PidCalculator bottom_yaw_angle_pid_, bottom_yaw_velocity_pid_;

    OutputInterface<double> top_yaw_control_torque_;
    OutputInterface<double> bottom_yaw_control_torque_;
    OutputInterface<double> bottom_yaw_control_angle_;
    OutputInterface<double> top_yaw_control_angle_shift_;
    OutputInterface<double> yaw_brake_control_torque_;

    rmcs_msgs::GimbalMode last_gimbal_mode_ = rmcs_msgs::GimbalMode::IMU;
    EncoderState encoder_state_ = EncoderState::Idle;

    int yaw_brake_engage_count_ = 0;
    int yaw_brake_release_elapsed_count_ = 0;
    int yaw_brake_release_stop_count_ = 0;
    bool yaw_brake_release_seen_motion_ = false;

    class DualYawStatus : public rmcs_executor::Component {
    public:
        explicit DualYawStatus() {
            register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
            register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
            register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
            register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);

            register_output("/gimbal/yaw/angle", yaw_angle_, 0.0);
            register_output("/gimbal/yaw/velocity", yaw_velocity_, 0.0);
        }

        void update() override {
            double yaw_angle = *top_yaw_angle_ + *bottom_yaw_angle_;
            if (yaw_angle < 0)
                yaw_angle += 2 * std::numbers::pi;
            else if (yaw_angle > 2 * std::numbers::pi)
                yaw_angle -= 2 * std::numbers::pi;
            *yaw_angle_ = yaw_angle;

            *yaw_velocity_ = *top_yaw_velocity_ + *bottom_yaw_velocity_;
        }

    private:
        InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
        InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;

        OutputInterface<double> yaw_angle_, yaw_velocity_;
    };
    std::shared_ptr<DualYawStatus> status_component_;
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawController, rmcs_executor::Component)
