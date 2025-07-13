#include <cmath>

#include <limits>
#include <string>

#include <eigen3/Eigen/Dense>
#include <fmt/chrono.h>
#include <fmt/format.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/shoot_status.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/fps_counter.hpp>

namespace rmcs_core::controller::shooting {

class FrictionWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FrictionWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

        auto friction_wheels = get_parameter("friction_wheels").as_string_array();
        auto friction_working_velocities = get_parameter("friction_velocities").as_double_array();
        if (friction_wheels.size() != friction_working_velocities.size())
            throw std::runtime_error(
                "Mismatch in array sizes: "
                "'friction_wheels' and 'friction_velocities' must have the same length!");
        else if (friction_wheels.size() == 0)
            throw std::runtime_error(
                "Empty array error: 'friction_wheels' and 'friction_velocities' cannot be empty!");

        friction_count_ = friction_wheels.size();
        friction_working_velocities_ = std::make_unique<double[]>(friction_count_);
        friction_velocities_ = std::make_unique<InputInterface<double>[]>(friction_count_);
        friction_control_velocities_ = std::make_unique<OutputInterface<double>[]>(friction_count_);
        for (size_t i = 0; i < friction_count_; i++) {
            friction_working_velocities_[i] = friction_working_velocities[i];
            register_input(friction_wheels[i] + "/velocity", friction_velocities_[i]);
            register_output(
                friction_wheels[i] + "/control_velocity", friction_control_velocities_[i], nan_);
        }

        friction_soft_start_stop_step_ =
            (1 / 1000.0) / get_parameter("friction_soft_start_stop_time").as_double();

        register_output("/gimbal/friction_ready", friction_ready_, false);
        register_output("/gimbal/friction_jammed", friction_jammed_, false);
        register_output("/gimbal/bullet_fired", bullet_fired_, false);
    }

    void update() override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        if (switch_right != Switch::DOWN) {
            if ((!last_keyboard_.v && keyboard.v)
                || (last_switch_left_ == Switch::MIDDLE && switch_left == Switch::UP)) {
                friction_enabled_ = !friction_enabled_;
            }

            update_friction_velocities();
            update_friction_status();
            if (*friction_jammed_)
                RCLCPP_INFO(logger_, "Friction Jammed!");
            if (*bullet_fired_)
                RCLCPP_INFO(logger_, "Bullet Fired!");

            last_switch_right_ = switch_right;
            last_switch_left_ = switch_left;
            last_keyboard_ = keyboard;
        }
    }

private:
    void reset_all_controls() {
        friction_enabled_ = false;

        last_primary_friction_velocity_ = nan_;
        primary_friction_velocity_decrease_integral_ = 0;

        friction_soft_start_stop_percentage_ = nan_;

        for (size_t i = 0; i < friction_count_; i++)
            *friction_control_velocities_[i] = nan_;

        *friction_ready_ = *friction_jammed_ = *bullet_fired_ = false;
    }

    void update_friction_velocities() {
        if (std::isnan(friction_soft_start_stop_percentage_)) {
            friction_soft_start_stop_percentage_ = 0.0;
            for (size_t i = 0; i < friction_count_; i++)
                friction_soft_start_stop_percentage_ +=
                    *friction_velocities_[i] / friction_working_velocities_[i];
            friction_soft_start_stop_percentage_ /= static_cast<double>(friction_count_);
        }
        friction_soft_start_stop_percentage_ +=
            friction_enabled_ ? friction_soft_start_stop_step_ : -friction_soft_start_stop_step_;
        friction_soft_start_stop_percentage_ =
            std::clamp(friction_soft_start_stop_percentage_, 0.0, 1.0);

        for (size_t i = 0; i < friction_count_; i++)
            *friction_control_velocities_[i] =
                friction_soft_start_stop_percentage_ * friction_working_velocities_[i];
    }

    void update_friction_status() {
        *friction_ready_ = *friction_jammed_ = *bullet_fired_ = false;

        if (!friction_enabled_)
            return;
        if (friction_soft_start_stop_percentage_ < 1.0)
            return;

        if (detect_friction_faulty()) {
            if (friction_faulty_count_ == 200) {
                friction_enabled_ = false;
                *friction_jammed_ = true;
            } else {
                friction_faulty_count_++;
                *friction_ready_ = true;
            }
            return;
        }

        *friction_ready_ = true;
        *bullet_fired_ = detect_bullet_fire();
    }

    bool detect_friction_faulty() {
        for (size_t i = 0; i < friction_count_; i++) {
            if (*friction_velocities_[i] < *friction_control_velocities_[i] * 0.5)
                return true;
        }
        return false;
    }

    rmcs_utility::FpsCounter fps_counter_;

    bool detect_bullet_fire() {
        bool fired = false;

        // The first friction wheel in the list is considered the primary one, meaning we only
        // monitor the speed drop of this wheel to detect whether a bullet has been fired.
        if (!std::isnan(last_primary_friction_velocity_)) {
            double differential = *friction_velocities_[0] - last_primary_friction_velocity_;
            if (differential < 0.1)
                primary_friction_velocity_decrease_integral_ += differential;
            else {
                if (primary_friction_velocity_decrease_integral_ < -14.0
                    && last_primary_friction_velocity_ < friction_working_velocities_[0] - 20.0)
                    fired = true;

                primary_friction_velocity_decrease_integral_ = 0;
            }
        }
        last_primary_friction_velocity_ = *friction_velocities_[0];

        return fired;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    size_t friction_count_;

    std::unique_ptr<double[]> friction_working_velocities_;

    std::unique_ptr<InputInterface<double>[]> friction_velocities_;

    bool friction_enabled_ = false;

    double friction_soft_start_stop_step_;
    double friction_soft_start_stop_percentage_ = nan_;
    std::unique_ptr<OutputInterface<double>[]> friction_control_velocities_;

    OutputInterface<bool> friction_ready_;

    int friction_faulty_count_ = 0;
    OutputInterface<bool> friction_jammed_;

    double last_primary_friction_velocity_ = nan_;
    double primary_friction_velocity_decrease_integral_ = 0;
    OutputInterface<bool> bullet_fired_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::FrictionWheelController, rmcs_executor::Component)