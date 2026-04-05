#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description; // NOLINT(google-build-using-namespace)

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
        component.register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        component.register_input("/tf", tf_);
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

    rmcs_executor::Component::InputInterface<double> gimbal_pitch_angle_;
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
        , solver_(
              *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double(),
              get_parameter("yaw_upper_limit").as_double(),
              get_parameter("yaw_lower_limit").as_double()) {
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, kNan);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, kNan);
    }

    void update() override {
        auto angle_error = calculate_angle_error();
        *yaw_angle_error_ = angle_error.yaw_angle_error;
        if (std::abs(angle_error.pitch_angle_error) < kPitchDeadband) {
            *pitch_angle_error_ = 0.0;
        } else {
            *pitch_angle_error_ = angle_error.pitch_angle_error;
        }
    }

private:
    auto calculate_angle_error() -> FlightTwoAxisGimbalSolver::AngleError {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto mouse = *mouse_;

        using namespace rmcs_msgs; // NOLINT(google-build-using-namespace)
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            return solver_.update(FlightTwoAxisGimbalSolver::SetDisabled{});
        }

        if (auto_aim_control_direction_.ready() && (mouse.right || switch_right == Switch::UP)
            && !auto_aim_control_direction_->isZero()) {
            return solver_.update(FlightTwoAxisGimbalSolver::SetControlDirection(
                OdomImu::DirectionVector(*auto_aim_control_direction_)));
        }

        if (!solver_.enabled())
            return solver_.update(FlightTwoAxisGimbalSolver::SetToLevel{});

        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;
        double yaw_shift =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift =
            -joystick_sensitivity * joystick_left_->x() + mouse_sensitivity * mouse_velocity_->x();

        return solver_.update(FlightTwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kPitchDeadband = 8e-3;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    OutputInterface<double> yaw_angle_error_;
    OutputInterface<double> pitch_angle_error_;

    FlightTwoAxisGimbalSolver solver_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::FlightGimbalController, rmcs_executor::Component)
