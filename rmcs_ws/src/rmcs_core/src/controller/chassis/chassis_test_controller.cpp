#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class ChassisTestController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisTestController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , body_length_(get_parameter_or("body_length", 0.001))
        , body_width_(get_parameter_or("body_width", 0.001))
        , arm_length_(get_parameter_or("arm_length", 0.155))
        , pivot_offset_(get_parameter_or("pivot_offset", 0.17389))
        , pivot_z_(get_parameter_or("pivot_z", 0.14))
        , alpha_nom_deg_(get_parameter_or("alpha_nom_deg", 30.0))
        , alpha_min_deg_(get_parameter_or("alpha_min_deg", 11.0))
        , alpha_max_deg_(get_parameter_or("alpha_max_deg", 48.0))
        , max_tilt_deg_(get_parameter_or("max_tilt_deg", 10.0))
        , print_debug_(get_parameter_or("print_debug", false)) {

        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/switch/left", switch_left_, false);

        register_output("/chassis/test/body_height", body_height_, nan_);

        register_output(
            "/chassis/test/left_front_joint/target_angle", left_front_target_angle_, nan_);
        register_output("/chassis/test/left_back_joint/target_angle", left_back_target_angle_, nan_);
        register_output("/chassis/test/right_back_joint/target_angle", right_back_target_angle_, nan_);
        register_output(
            "/chassis/test/right_front_joint/target_angle", right_front_target_angle_, nan_);
    }

    void update() override {
        if (!joystick_left_.ready() || !switch_right_.ready() || !switch_left_.ready()) {
            publish_nan_outputs_();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Missing remote inputs. Waiting for /remote/joystick/left, "
                "/remote/switch/right and /remote/switch/left.");
            return;
        }

        if (!std::isfinite(joystick_left_->x()) || !std::isfinite(joystick_left_->y())) {
            publish_nan_outputs_();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Received non-finite joystick value(s). Output is set to NaN.");
            return;
        }

        if (!parameters_are_valid_()) {
            publish_nan_outputs_();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Invalid solver parameters. Check geometry lengths and angle limits.");
            return;
        }

        const BodyAttitudeInput body_attitude_input = build_body_attitude_input_();

        double solved_body_height = nan_;
        const std::array<double, leg_count_> angles_deg =
            compute_leg_angles_deg_(body_attitude_input, solved_body_height);

        *body_height_ = solved_body_height;

        *left_front_target_angle_ = angles_deg[k_left_front_];
        *left_back_target_angle_  = angles_deg[k_left_back_];
        *right_back_target_angle_ = angles_deg[k_right_back_];
        *right_front_target_angle_ = angles_deg[k_right_front_];

        if (print_debug_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "chassis_test_solver: roll=%.2f, pitch=%.2f, h=%.4f, lf=%.3f, lb=%.3f, rb=%.3f, "
                "rf=%.3f",
                body_attitude_input.roll_deg, body_attitude_input.pitch_deg, *body_height_,
                *left_front_target_angle_, *left_back_target_angle_, *right_back_target_angle_,
                *right_front_target_angle_);
        }
    }

private:
    static constexpr int leg_count_ = 4;
    static constexpr int k_left_front_ = 0;
    static constexpr int k_right_front_ = 1;
    static constexpr int k_left_back_ = 2;
    static constexpr int k_right_back_ = 3;

    static constexpr double pi_  = std::numbers::pi;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    struct BodyAttitudeInput {
        double roll_deg;
        double pitch_deg;
        double yaw_deg;
    };

    struct RobotParams {
        double body_length;
        double body_width;
        double arm_length;
        double pivot_offset;
        double pivot_z;
        double alpha_nom_deg;
        double alpha_min_deg;
        double alpha_max_deg;
    };

    struct AngleSolveData {
        double a = 0.0;
        double b = 0.0;
        double d = 0.0;
    };

    bool parameters_are_valid_() const {
        return std::isfinite(body_length_) && std::isfinite(body_width_) && std::isfinite(arm_length_)
            && std::isfinite(pivot_offset_) && std::isfinite(pivot_z_) && std::isfinite(alpha_nom_deg_)
            && std::isfinite(alpha_min_deg_) && std::isfinite(alpha_max_deg_)
            && std::isfinite(max_tilt_deg_) && body_length_ > 0.0 && body_width_ > 0.0
            && arm_length_ > 0.0 && alpha_min_deg_ <= alpha_max_deg_ && max_tilt_deg_ >= 0.0;
    }

    RobotParams build_params_() const {
        return RobotParams{
            .body_length = body_length_,
            .body_width = body_width_,
            .arm_length = arm_length_,
            .pivot_offset = pivot_offset_,
            .pivot_z = pivot_z_,
            .alpha_nom_deg = alpha_nom_deg_,
            .alpha_min_deg = alpha_min_deg_,
            .alpha_max_deg = alpha_max_deg_,
        };
    }

    void publish_nan_outputs_() {
        *body_height_ = nan_;

        *left_front_target_angle_ = nan_;
        *left_back_target_angle_  = nan_;
        *right_back_target_angle_ = nan_;
        *right_front_target_angle_ = nan_;
    }

    BodyAttitudeInput build_body_attitude_input_() const {
        BodyAttitudeInput input{
            .roll_deg = 0.0,
            .pitch_deg = 0.0,
            .yaw_deg = 0.0,
        };

        if (*switch_right_ != rmcs_msgs::Switch::UP || *switch_left_ != rmcs_msgs::Switch::DOWN) {
            return input;
        }

        Eigen::Vector2d tilt_ratio = *joystick_left_;
        if (tilt_ratio.norm() > 1.0) {
            tilt_ratio.normalize();
        }

        input.roll_deg = clamp_(tilt_ratio.x(), -1.0, 1.0) * max_tilt_deg_;
        input.pitch_deg = clamp_(tilt_ratio.y(), -1.0, 1.0) * max_tilt_deg_;
        return input;
    }

    static double deg_to_rad_(double deg) {
        return deg * pi_ / 180.0;
    }

    static double rad_to_deg_(double rad) {
        return rad * 180.0 / pi_;
    }

    static double clamp_(double value, double lower, double upper) {
        return std::max(lower, std::min(value, upper));
    }

    static double normalize_pi_(double rad) {
        while (rad <= -pi_) {
            rad += 2.0 * pi_;
        }
        while (rad > pi_) {
            rad -= 2.0 * pi_;
        }
        return rad;
    }

    static double median4_(std::array<double, leg_count_> values) {
        std::sort(values.begin(), values.end());
        return 0.5 * (values[1] + values[2]);
    }

    static Eigen::Matrix3d euler_zyx_to_rotation_matrix_(
        double roll_deg, double pitch_deg, double yaw_deg) {
        const double roll  = deg_to_rad_(roll_deg);
        const double pitch = deg_to_rad_(pitch_deg);
        const double yaw   = deg_to_rad_(yaw_deg);

        const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());

        return (rz * ry * rx).toRotationMatrix();
    }

    static std::array<Eigen::Vector3d, leg_count_> build_body_corners_(const RobotParams& params) {
        const double hx = 0.5 * params.body_length;
        const double hy = 0.5 * params.body_width;

        return {
            Eigen::Vector3d(+hx, +hy, 0.0), // left_front
            Eigen::Vector3d(+hx, -hy, 0.0), // right_front
            Eigen::Vector3d(-hx, +hy, 0.0), // left_back
            Eigen::Vector3d(-hx, -hy, 0.0), // right_back
        };
    }

    static std::array<Eigen::Vector3d, leg_count_>
        build_radials_(const std::array<Eigen::Vector3d, leg_count_>& corners) {
        std::array<Eigen::Vector3d, leg_count_> radials{};

        for (int i = 0; i < leg_count_; ++i) {
            Eigen::Vector3d radial(corners[i].x(), corners[i].y(), 0.0);
            const double norm = radial.norm();

            if (norm < 1e-9) {
                return {};
            }

            radials[i] = radial / norm;
        }

        return radials;
    }

    static std::array<Eigen::Vector3d, leg_count_> build_pivots_(
        const std::array<Eigen::Vector3d, leg_count_>& corners,
        const std::array<Eigen::Vector3d, leg_count_>& radials,
        const RobotParams& params) {

        std::array<Eigen::Vector3d, leg_count_> pivots{};

        for (int i = 0; i < leg_count_; ++i) {
            pivots[i] =
                corners[i] + params.pivot_offset * radials[i] + Eigen::Vector3d(0.0, 0.0, params.pivot_z);
        }

        return pivots;
    }

    static std::optional<double> solve_leg_angle_rad_(
        const AngleSolveData& data, double alpha_min_rad, double alpha_max_rad,
        double alpha_nominal_rad) {

        const double radius = std::hypot(data.a, data.b);

        if (radius < 1e-12) {
            if (std::abs(data.d) < 1e-12) {
                return clamp_(alpha_nominal_rad, alpha_min_rad, alpha_max_rad);
            }
            return std::nullopt;
        }

        if (std::abs(data.d) > radius + 1e-9) {
            return std::nullopt;
        }

        const double ratio = clamp_(data.d / radius, -1.0, 1.0);
        const double base  = std::asin(ratio);
        const double gamma = std::atan2(data.a, data.b);

        std::vector<double> candidates;
        candidates.reserve(10);

        const std::array<double, 2> seeds = {base - gamma, (pi_ - base) - gamma};

        for (double seed : seeds) {
            seed = normalize_pi_(seed);

            for (int k = -2; k <= 2; ++k) {
                const double candidate = seed + 2.0 * pi_ * static_cast<double>(k);
                if (candidate >= alpha_min_rad - 1e-9 && candidate <= alpha_max_rad + 1e-9) {
                    candidates.push_back(candidate);
                }
            }
        }

        if (candidates.empty()) {
            return std::nullopt;
        }

        const auto best_it = std::min_element(
            candidates.begin(), candidates.end(), [alpha_nominal_rad](double lhs, double rhs) {
                return std::abs(lhs - alpha_nominal_rad) < std::abs(rhs - alpha_nominal_rad);
            });

        return *best_it;
    }

    std::array<double, leg_count_>
        compute_leg_angles_deg_(const BodyAttitudeInput& body_attitude_input, double& body_height_out)
            const {

        const RobotParams params = build_params_();
        const Eigen::Matrix3d rotation_world_body =
            euler_zyx_to_rotation_matrix_(
                body_attitude_input.roll_deg, body_attitude_input.pitch_deg,
                body_attitude_input.yaw_deg);

        const auto corners = build_body_corners_(params);
        const auto radials = build_radials_(corners);
        const auto pivots  = build_pivots_(corners, radials, params);

        const double alpha_nominal_rad = deg_to_rad_(params.alpha_nom_deg);
        const double alpha_min_rad     = deg_to_rad_(params.alpha_min_deg);
        const double alpha_max_rad     = deg_to_rad_(params.alpha_max_deg);

        const Eigen::Vector3d b_vec(0.0, 0.0, -params.arm_length);

        std::array<double, leg_count_> h_candidates{};
        for (int i = 0; i < leg_count_; ++i) {
            const Eigen::Vector3d a_vec = params.arm_length * radials[i];
            const Eigen::Vector3d foot_body =
                pivots[i] + std::cos(alpha_nominal_rad) * a_vec + std::sin(alpha_nominal_rad) * b_vec;

            h_candidates[i] = -(rotation_world_body * foot_body).z();
        }
        body_height_out = median4_(h_candidates);

        std::array<double, leg_count_> output_angles_deg{};
        for (int i = 0; i < leg_count_; ++i) {
            const Eigen::Vector3d a_vec = params.arm_length * radials[i];

            AngleSolveData equation{};
            equation.a = (rotation_world_body * a_vec).z();
            equation.b = (rotation_world_body * b_vec).z();
            equation.d = -body_height_out - (rotation_world_body * pivots[i]).z();

            const std::optional<double> angle_rad = solve_leg_angle_rad_(
                equation, alpha_min_rad, alpha_max_rad, alpha_nominal_rad);

            if (angle_rad.has_value()) {
                output_angles_deg[i] = rad_to_deg_(*angle_rad);
            } else {
                output_angles_deg[i] = nan_;
            }
        }

        return output_angles_deg;
    }

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    OutputInterface<double> body_height_;

    OutputInterface<double> left_front_target_angle_;
    OutputInterface<double> left_back_target_angle_;
    OutputInterface<double> right_back_target_angle_;
    OutputInterface<double> right_front_target_angle_;

    double body_length_;
    double body_width_;
    double arm_length_;
    double pivot_offset_;
    double pivot_z_;
    double alpha_nom_deg_;
    double alpha_min_deg_;
    double alpha_max_deg_;
    double max_tilt_deg_;

    bool print_debug_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisTestController, rmcs_executor::Component)
