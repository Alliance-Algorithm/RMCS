#include <asm-generic/errno.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/target_snapshot.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

#include "controller/gimbal/planner.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description; // NOLINT(google-build-using-namespace)

namespace {

constexpr double kDegToRad = 1.0 / 57.3;

Eigen::Vector3d planner_direction(double yaw, double pitch) {
    Eigen::Vector3d direction{
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        -std::sin(pitch),
    };
    if (direction.norm() > 1e-9)
        direction.normalize();
    else
        direction.setZero();
    return direction;
}

PlannerConfig load_planner_config(rclcpp::Node& node) {
    PlannerConfig config;
    config.yaw_offset = node.get_parameter("yaw_offset").as_double() * kDegToRad;
    config.pitch_offset = node.get_parameter("pitch_offset").as_double() * kDegToRad;
    config.fire_thresh = node.get_parameter("fire_thresh").as_double();
    config.low_speed_delay_time = node.get_parameter("low_speed_delay_time").as_double();
    config.high_speed_delay_time = node.get_parameter("high_speed_delay_time").as_double();
    config.decision_speed = node.get_parameter("decision_speed").as_double();
    config.max_yaw_acc = node.get_parameter("max_yaw_acc").as_double();
    config.max_pitch_acc = node.get_parameter("max_pitch_acc").as_double();
    return config;
}

double parameter_or_declare(rclcpp::Node& node, const std::string& name, double default_value) {
    if (!node.has_parameter(name))
        node.declare_parameter<double>(name, default_value);
    return node.get_parameter(name).as_double();
}

} // namespace

class FlightTwoAxisGimbalSolver {
    class Operation {
        friend class FlightTwoAxisGimbalSolver;

        virtual PitchLink::DirectionVector update(FlightTwoAxisGimbalSolver& solver) const = 0;
    };

public:
    FlightTwoAxisGimbalSolver(
        rmcs_executor::Component& component, double upper_limit, double lower_limit,
        double yaw_upper_limit, double yaw_lower_limit)
        : upper_limit_(std::cos(upper_limit), -std::sin(upper_limit))
        , lower_limit_(std::cos(lower_limit), -std::sin(lower_limit))
        , yaw_upper_limit_(std::cos(yaw_upper_limit), -std::sin(yaw_upper_limit))
        , yaw_lower_limit_(std::cos(yaw_lower_limit), -std::sin(yaw_lower_limit)) {
        component.register_input("/tf", tf_);
    }

    // Re-anchor the stored control direction to the gimbal's current physical pose.
    // Called on hold-mode exit so that the next SetControlShift starts from "here",
    // not from a stale target that predates the hold period.
    void anchor_to_current_pose() {
        control_direction_ = fast_tf::cast<OdomImu>(
            PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        control_enabled_ = true;
    }

    class SetDisabled : public Operation {
        PitchLink::DirectionVector update(FlightTwoAxisGimbalSolver& solver) const override {
            solver.control_enabled_ = false;
            return {};
        }
    };

    class SetToLevel : public Operation {
        PitchLink::DirectionVector update(FlightTwoAxisGimbalSolver& solver) const override {
            auto odom_direction = fast_tf::cast<OdomImu>(
                PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *solver.tf_);
            if (std::abs(odom_direction->x()) < 1e-6 && std::abs(odom_direction->y()) < 1e-6)
                return {};

            solver.control_enabled_ = true;
            odom_direction->z() = 0;
            auto pitch_direction = fast_tf::cast<PitchLink>(odom_direction, *solver.tf_);
            pitch_direction->normalize();
            return pitch_direction;
        }
    };

    class SetControlDirection : public Operation {
    public:
        explicit SetControlDirection(OdomImu::DirectionVector target)
            : target_(std::move(target)) {}

    private:
        PitchLink::DirectionVector update(FlightTwoAxisGimbalSolver& solver) const override {
            solver.control_enabled_ = true;
            return fast_tf::cast<PitchLink>(target_, *solver.tf_);
        }

        OdomImu::DirectionVector target_;
    };

    class SetControlShift : public Operation {
    public:
        SetControlShift(double yaw_shift, double pitch_shift)
            : yaw_shift_(yaw_shift)
            , pitch_shift_(pitch_shift) {}

    private:
        PitchLink::DirectionVector update(FlightTwoAxisGimbalSolver& solver) const override {
            PitchLink::DirectionVector direction;
            if (!solver.control_enabled_) {
                solver.control_enabled_ = true;
                direction = PitchLink::DirectionVector{Eigen::Vector3d::UnitX()};
            } else {
                direction = fast_tf::cast<PitchLink>(solver.control_direction_, *solver.tf_);
            }

            auto yaw_transform = Eigen::AngleAxisd{yaw_shift_, Eigen::Vector3d::UnitZ()};
            auto pitch_transform = Eigen::AngleAxisd{pitch_shift_, Eigen::Vector3d::UnitY()};
            return PitchLink::DirectionVector{pitch_transform * (yaw_transform * (*direction))};
        }

        double yaw_shift_;
        double pitch_shift_;
    };

    struct AngleError {
        double yaw_angle_error;
        double pitch_angle_error;
    };

    AngleError update(const Operation& operation) {
        update_yaw_axis();

        auto control_direction = operation.update(*this);
        if (!control_enabled_)
            return {kNan, kNan};

        auto [control_direction_yaw_link, pitch] = pitch_link_to_yaw_link(control_direction);
        clamp_control_direction(control_direction_yaw_link);
        if (!control_enabled_)
            return {kNan, kNan};

        control_direction_ =
            fast_tf::cast<OdomImu>(yaw_link_to_pitch_link(control_direction_yaw_link, pitch), *tf_);
        return calculate_control_errors(control_direction_yaw_link, pitch);
    }

    bool enabled() const { return control_enabled_; }

private:
    void update_yaw_axis() {
        auto yaw_axis =
            fast_tf::cast<PitchLink>(YawLink::DirectionVector{Eigen::Vector3d::UnitZ()}, *tf_);
        *yaw_axis_filtered_ += 0.1 * (*fast_tf::cast<OdomImu>(yaw_axis, *tf_));
        yaw_axis_filtered_->normalize();
    }

    auto pitch_link_to_yaw_link(const PitchLink::DirectionVector& direction) const
        -> std::pair<YawLink::DirectionVector, Eigen::Vector2d> {
        std::pair<YawLink::DirectionVector, Eigen::Vector2d> result;
        auto& [direction_yaw_link, pitch] = result;

        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);
        pitch = {yaw_axis->z(), yaw_axis->x()};
        pitch.normalize();

        const auto& [x, y, z] = *direction;
        direction_yaw_link = {x * pitch.x() - z * pitch.y(), y, x * pitch.y() + z * pitch.x()};
        return result;
    }

    static PitchLink::DirectionVector
        yaw_link_to_pitch_link(const YawLink::DirectionVector& direction, const Eigen::Vector2d& pitch) {
        const auto& [x, y, z] = *direction;
        return {x * pitch.x() + z * pitch.y(), y, -x * pitch.y() + z * pitch.x()};
    }

    void clamp_control_direction(YawLink::DirectionVector& control_direction) {
        const auto& [x, y, z] = *control_direction;

        Eigen::Vector2d xy_projection{x, y};
        double xy_norm = xy_projection.norm();
        if (xy_norm <= 0.0) {
            control_enabled_ = false;
            return;
        }
        xy_projection /= xy_norm;

        if (z > upper_limit_.y()) {
            *control_direction << upper_limit_.x() * xy_projection, upper_limit_.y();
        } else if (z < lower_limit_.y()) {
            *control_direction << lower_limit_.x() * xy_projection, lower_limit_.y();
        }

        const auto& [yaw_x, yaw_y, yaw_z] = *control_direction;
        Eigen::Vector2d xz_projection{yaw_x, yaw_z};
        double xz_norm = xz_projection.norm();
        if (xz_norm <= 0.0) {
            control_enabled_ = false;
            return;
        }
        xz_projection /= xz_norm;

        if (yaw_y > yaw_upper_limit_.y()) {
            *control_direction << yaw_upper_limit_.x() * xz_projection.x(),
                yaw_upper_limit_.y(), yaw_upper_limit_.x() * xz_projection.y();
        } else if (yaw_y < yaw_lower_limit_.y()) {
            *control_direction << yaw_lower_limit_.x() * xz_projection.x(),
                yaw_lower_limit_.y(), yaw_lower_limit_.x() * xz_projection.y();
        }
    }

    static AngleError calculate_control_errors(
        const YawLink::DirectionVector& control_direction, const Eigen::Vector2d& pitch) {
        const auto& [x, y, z] = *control_direction;
        const auto& [c, s] = pitch;

        AngleError result;
        result.yaw_angle_error = std::atan2(y, x);
        double x_projected = std::sqrt(x * x + y * y);
        result.pitch_angle_error = -std::atan2(z * c - x_projected * s, z * s + x_projected * c);
        return result;
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    const Eigen::Vector2d upper_limit_;
    const Eigen::Vector2d lower_limit_;
    const Eigen::Vector2d yaw_upper_limit_;
    const Eigen::Vector2d yaw_lower_limit_;

    rmcs_executor::Component::InputInterface<Tf> tf_;

    OdomImu::DirectionVector yaw_axis_filtered_{Eigen::Vector3d::UnitZ()};
    bool control_enabled_ = false;
    OdomImu::DirectionVector control_direction_;
};

class FlightGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FlightGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , hold_level_threshold_(get_parameter("hold_level_threshold").as_double())
        , solver_(
              *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double(),
              get_parameter("yaw_upper_limit").as_double(),
              get_parameter("yaw_lower_limit").as_double())
        , planner_(load_planner_config(*this))
        , yaw_vel_ff_gain_(parameter_or_declare(*this, "yaw_vel_ff_gain", 0.0))
        , yaw_acc_ff_gain_(parameter_or_declare(*this, "yaw_acc_ff_gain", 0.0))
        , pitch_vel_ff_gain_(parameter_or_declare(*this, "pitch_vel_ff_gain", 1.0)) {
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/gimbal/yaw/velocity", yaw_velocity_encoder_);
        register_input("/gimbal/pitch/velocity", pitch_velocity_encoder_);
        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);

        // Target snapshot + planner dependencies
        register_input("/gimbal/auto_aim/target_snapshot", target_snapshot_, false);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/referee/shooter/initial_speed", bullet_speed_, false);

        // Hold feedforward inputs (optional — HoldController may not be loaded)
        register_input("/gimbal/yaw/hold_feedforward", hold_yaw_ff_, false);
        register_input("/gimbal/pitch/hold_feedforward", hold_pitch_ff_, false);

        // Optional: when HoldController is not loaded these stay unbound,
        // update_manual_or_hold_control() falls through to the pre-hold behavior.
        register_input("/gimbal/hold/desired", hold_desired_, false);

        if (!has_parameter("bullet_speed_fallback"))
            declare_parameter<double>("bullet_speed_fallback", 23.0);
        if (!has_parameter("result_timeout"))
            declare_parameter<double>("result_timeout", 0.2);

        planner_result_timeout_ =
            std::chrono::duration<double>(get_parameter("result_timeout").as_double());
        bullet_speed_fallback_storage_ =
            static_cast<float>(get_parameter("bullet_speed_fallback").as_double());

        // Planner outputs
        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/fire_control", fire_control_, false);
        register_output("/gimbal/auto_aim/laser_distance", laser_distance_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw", plan_yaw_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch", plan_pitch_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_velocity", plan_yaw_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_acceleration", plan_yaw_acceleration_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_velocity", plan_pitch_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_acceleration", plan_pitch_acceleration_, 0.0);

        // Unified feedforward outputs (replaces separate Mixer component)
        register_output("/gimbal/yaw/feedforward", yaw_ff_output_, 0.0);
        register_output("/gimbal/pitch/feedforward", pitch_ff_output_, 0.0);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, kNan);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, kNan);
        register_output("/gimbal/hold/active", hold_active_output_, false);
    }

    void before_updating() override {
        if (!bullet_speed_.ready())
            bullet_speed_.bind_directly(bullet_speed_fallback_storage_);
        if (!target_snapshot_.ready())
            target_snapshot_.make_and_bind_directly(rmcs_msgs::TargetSnapshot{});
        if (!hold_yaw_ff_.ready())
            hold_yaw_ff_.bind_directly(zero_ff_);
        if (!hold_pitch_ff_.ready())
            hold_pitch_ff_.bind_directly(zero_ff_);
        clear_planner_outputs();
        *yaw_ff_output_ = 0.0;
        *pitch_ff_output_ = 0.0;
    }

    void update() override {
        // Phase 1: unconditionally update planner outputs (aligned with omni)
        const PlannerResult planner_result = update_planner_outputs();
        const bool planner_active = auto_aim_requested() && planner_result.control;

        // Phase 2: angle control
        const auto angle_error =
            planner_active ? update_auto_aim_control(*control_direction_)
                           : update_manual_or_hold_control();

        if (std::isfinite(angle_error.yaw_angle_error)
            && std::abs(angle_error.yaw_angle_error)<kYawDeadband){
            *yaw_angle_error_=0.0;
            }else {
                *yaw_angle_error_ =angle_error.yaw_angle_error;
            }

        // Phase 3: feedforward routing
        update_feedforward(planner_active);

        // Phase 4: pitch deadband
        if (!std::isfinite(angle_error.pitch_angle_error)) {
            *pitch_angle_error_ = angle_error.pitch_angle_error;
        } else if (std::abs(angle_error.pitch_angle_error) < kPitchDeadband) {
            *pitch_angle_error_ = 0.0;
        } 
        *hold_active_output_ = hold_active_;

    }

private:
    bool auto_aim_requested() const {
        return *switch_right_ == rmcs_msgs::Switch::UP;
    }

    PlannerResult update_planner_outputs() {
        const auto now = *timestamp_;
        const auto snapshot = *target_snapshot_;

        const double age_s = std::chrono::duration<double>(now - snapshot.timestamp).count();
        const bool fresh = snapshot.valid && age_s <= planner_result_timeout_.count();
        if (!fresh) {
            clear_planner_outputs();
            return {};
        }

        const PlannerResult result = planner_.plan(
            std::optional<rmcs_msgs::TargetSnapshot>{snapshot}, now,
            static_cast<double>(*bullet_speed_));
        if (!result.control) {
            clear_planner_outputs();
            return {};
        }

        *control_direction_ = planner_direction(result.yaw, result.pitch);
        *fire_control_ = result.fire;
        *laser_distance_ = result.control_xyza.head<3>().norm();
        *plan_yaw_ = result.yaw;
        *plan_pitch_ = result.pitch;
        *plan_yaw_velocity_ = result.yaw_velocity;
        *plan_yaw_acceleration_ = result.yaw_acceleration;
        *plan_pitch_velocity_ = result.pitch_velocity;
        *plan_pitch_acceleration_ = result.pitch_acceleration;
        return result;
    }

    void clear_planner_outputs() {
        *control_direction_ = Eigen::Vector3d::Zero();
        *fire_control_ = false;
        *laser_distance_ = 0.0;
        *plan_yaw_ = 0.0;
        *plan_pitch_ = 0.0;
        *plan_yaw_velocity_ = 0.0;
        *plan_yaw_acceleration_ = 0.0;
        *plan_pitch_velocity_ = 0.0;
        *plan_pitch_acceleration_ = 0.0;
    }

    void update_feedforward(bool planner_active) {
        if (planner_active) {
            *yaw_ff_output_ =
                yaw_vel_ff_gain_ * *plan_yaw_velocity_
                + yaw_acc_ff_gain_ * *plan_yaw_acceleration_;
            *pitch_ff_output_ = pitch_vel_ff_gain_ * *plan_pitch_velocity_;
        } else if (hold_active_) {
            *yaw_ff_output_ = *hold_yaw_ff_;
            *pitch_ff_output_ = *hold_pitch_ff_;
        } else {
            *yaw_ff_output_ = 0.0;
            *pitch_ff_output_ = 0.0;
        }
    }

    FlightTwoAxisGimbalSolver::AngleError
        update_auto_aim_control(const Eigen::Vector3d& direction) {
        if (hold_active_) {
            hold_active_ = false;
        }
        return solver_.update(FlightTwoAxisGimbalSolver::SetControlDirection(
            OdomImu::DirectionVector{direction}));
    }

    FlightTwoAxisGimbalSolver::AngleError update_manual_or_hold_control() {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        using namespace rmcs_msgs; // NOLINT(google-build-using-namespace)

        // DISABLED
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            level_reached_ = false;
            hold_active_ = false;
            return solver_.update(FlightTwoAxisGimbalSolver::SetDisabled{});
        }

        // LEVELING
        if (!solver_.enabled()) {
            level_reached_ = false;
            hold_active_ = false;
            return solver_.update(FlightTwoAxisGimbalSolver::SetToLevel{});
        }

        if (!level_reached_) {
            hold_active_ = false;
            auto err = solver_.update(FlightTwoAxisGimbalSolver::SetToLevel{});
            if (std::isfinite(err.yaw_angle_error) && std::isfinite(err.pitch_angle_error)
                && std::abs(err.yaw_angle_error) < hold_level_threshold_
                && std::abs(err.pitch_angle_error) < hold_level_threshold_) {
                level_reached_ = true;
            }
            return err;
        }

        // HOLD
        const bool want_hold = hold_desired_.ready() && *hold_desired_;
        if (want_hold) {
            if (!hold_active_) {
                hold_active_ = true;
                hold_target_yaw_ = *gimbal_yaw_angle_;
                hold_target_pitch_ = *gimbal_pitch_angle_;
            }
            return {
                wrap_pi(hold_target_yaw_ - *gimbal_yaw_angle_),
                wrap_pi(hold_target_pitch_ - *gimbal_pitch_angle_)};
        }

        if (hold_active_) {
            hold_active_ = false;
            solver_.anchor_to_current_pose();
        }

        // MANUAL
        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;
        double yaw_shift =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift =
            -joystick_sensitivity * joystick_left_->x() + mouse_sensitivity * mouse_velocity_->x();

        return solver_.update(FlightTwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
    }

    static double wrap_pi(double x) {
        return std::remainder(x, 2.0 * std::numbers::pi);
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kPitchDeadband = 1e-1;
    static constexpr double kYawDeadband = 1e-1;

    // TODO: temporary auto-aim diagnostics — remove after verification

    const double hold_level_threshold_;
    bool level_reached_ = false;
    bool hold_active_ = false;
    double hold_target_yaw_ = 0.0;
    double hold_target_pitch_ = 0.0;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<double> yaw_velocity_encoder_;
    InputInterface<double> pitch_velocity_encoder_;
    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<double> gimbal_pitch_angle_;
    InputInterface<bool> hold_desired_;

    // Planner inputs
    InputInterface<rmcs_msgs::TargetSnapshot> target_snapshot_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<float> bullet_speed_;

    // Hold feedforward inputs (optional, from HoldController)
    InputInterface<double> hold_yaw_ff_;
    InputInterface<double> hold_pitch_ff_;

    // Planner instance + config
    Planner planner_;
    std::chrono::duration<double> planner_result_timeout_{0.2};
    float bullet_speed_fallback_storage_ = 23.0F;
    double zero_ff_ = 0.0;

    // FF gain parameters
    const double yaw_vel_ff_gain_;
    const double yaw_acc_ff_gain_;
    const double pitch_vel_ff_gain_;

    // Planner outputs
    OutputInterface<Eigen::Vector3d> control_direction_;
    OutputInterface<bool> fire_control_;
    OutputInterface<double> laser_distance_;
    OutputInterface<double> plan_yaw_;
    OutputInterface<double> plan_pitch_;
    OutputInterface<double> plan_yaw_velocity_;
    OutputInterface<double> plan_yaw_acceleration_;
    OutputInterface<double> plan_pitch_velocity_;
    OutputInterface<double> plan_pitch_acceleration_;

    FlightTwoAxisGimbalSolver solver_;

    // Unified feedforward outputs (replaces Mixer)
    OutputInterface<double> yaw_ff_output_;
    OutputInterface<double> pitch_ff_output_;

    OutputInterface<double> yaw_angle_error_;
    OutputInterface<double> pitch_angle_error_;
    OutputInterface<bool> hold_active_output_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::FlightGimbalController, rmcs_executor::Component)
