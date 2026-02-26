#include <cmath>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {

class DualYawSolver
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawSolver()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/gimbal/yaw/control_angle_error", control_angle_error_);
        register_input("/gimbal/yaw/control_angle_shift", control_angle_shift_, false);
        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_velocity", control_angle_velocity_, false);

        register_output("/gimbal/top_yaw/target_angle_error", top_yaw_target_error_, 0.0);
        register_output("/gimbal/bottom_yaw/target_angle_error", bottom_yaw_target_error_, 0.0);

        register_output("/gimbal/top_yaw/target_angle_velocity", top_yaw_target_velocity_, 0.0);
        register_output(
            "/gimbal/bottom_yaw/target_angle_velocity", bottom_yaw_target_velocity_, 0.0);

        register_output("/gimbal/top_yaw/control_angle", top_yaw_control_angle_, 0.0);
        register_output(
            "/gimbal/bottom_yaw/control_angle_shift", bottom_yaw_control_angle_shift_, 0.0);

        status_component_ =
            create_partner_component<DualYawStatus>(get_component_name() + "_status");
    }

    void before_updating() override {
        if (!control_angle_shift_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_shift\", set to NaN.");
            control_angle_shift_.bind_directly(nan_);
        }
        if (!control_angle_velocity_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_velocity\", set to 0.");
            control_angle_velocity_.bind_directly(0.0);
        }
    }

    void update() override {
        constexpr double TOP_YAW_LIMIT = std::numbers::pi / 3.0;

        if (std::isnan(*control_angle_error_)) {
            *top_yaw_target_error_ = nan_;
            *bottom_yaw_target_error_ = nan_;
            *top_yaw_target_velocity_ = nan_;
            *bottom_yaw_target_velocity_ = nan_;

        } else {
            double e_total = std::remainder(*control_angle_error_, 2.0 * std::numbers::pi);

            double e_bot = std::remainder(e_total + *top_yaw_angle_, 2.0 * std::numbers::pi);

            double top_target = std::clamp(e_bot, -TOP_YAW_LIMIT, TOP_YAW_LIMIT);

            *top_yaw_target_error_ = top_target - *top_yaw_angle_;
            *bottom_yaw_target_error_ = e_bot;

            *bottom_yaw_target_velocity_ = *control_angle_velocity_;
            *top_yaw_target_velocity_ = 0.0;
        }

        if (std::isnan(*control_angle_shift_)) {
            *top_yaw_control_angle_ = nan_;
            *bottom_yaw_control_angle_shift_ = nan_;
        } else {
            *top_yaw_control_angle_ = 0.0;
            *bottom_yaw_control_angle_shift_ = *control_angle_shift_;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> control_angle_error_, control_angle_shift_;
    InputInterface<double> top_yaw_angle_;
    InputInterface<double> control_angle_velocity_;

    OutputInterface<double> top_yaw_target_error_, bottom_yaw_target_error_;
    OutputInterface<double> top_yaw_target_velocity_, bottom_yaw_target_velocity_;
    OutputInterface<double> top_yaw_control_angle_, bottom_yaw_control_angle_shift_;

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
            yaw_angle = std::fmod(yaw_angle, 2.0 * std::numbers::pi);
            if (yaw_angle < 0)
                yaw_angle += 2.0 * std::numbers::pi;
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
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawSolver, rmcs_executor::Component)