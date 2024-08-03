#include <cmath>

#include <limits>

#include <eigen3/Eigen/Dense>
#include <fast_tf/rcl.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class ShootingController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ShootingController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        friction_working_velocity        = get_parameter("friction_velocity").as_double();
        double shot_frequency            = get_parameter("shot_frequency").as_double();
        bullet_feeder_working_velocity   = shot_frequency / 8 * 2 * std::numbers::pi;
        double safe_shot_frequency       = get_parameter("safe_shot_frequency").as_double();
        bullet_feeder_safe_shot_velocity = safe_shot_frequency / 8 * 2 * std::numbers::pi;

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/right_friction/velocity", right_friction_velocity_);

        register_input("/referee/shooter/cooling", shooter_cooling_);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_);

        register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);

        register_output(
            "/gimbal/left_friction/control_velocity", left_friction_control_velocity_, nan);
        register_output(
            "/gimbal/right_friction/control_velocity", right_friction_control_velocity_, nan);
        register_output(
            "/gimbal/bullet_feeder/control_velocity", bullet_feeder_control_velocity_, nan);
    }

    void update() override {
        update_muzzle_heat();

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            if (switch_right != Switch::DOWN) {
                if ((!last_keyboard_.v && keyboard.v)
                    || (last_switch_left_ == Switch::MIDDLE && switch_left == Switch::UP)) {
                    friction_enabled_ = !friction_enabled_;
                }
                bullet_feeder_enabled_ = mouse.left || switch_left == Switch::DOWN;
                if (keyboard.f || keyboard.g) {
                    if (bullet_count_limited_by_single_shot_ < 0)
                        bullet_count_limited_by_single_shot_ = 0;
                    if (!last_mouse_.left && mouse.left) {
                        ++bullet_count_limited_by_single_shot_;
                        RCLCPP_INFO(get_logger(), "%ld", bullet_count_limited_by_single_shot_);
                    }
                    bullet_feeder_enabled_ |= bullet_count_limited_by_single_shot_ > 0;
                } else {
                    bullet_count_limited_by_single_shot_ = -1;
                }
            }
            update_friction_velocities();
            update_bullet_feeder_velocity();
        }

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
        last_mouse_        = mouse;
        last_keyboard_     = keyboard;
    }

private:
    void reset_all_controls() {
        friction_enabled_                 = false;
        *left_friction_control_velocity_  = nan;
        *right_friction_control_velocity_ = nan;
        bullet_feeder_enabled_            = false;
        *bullet_feeder_control_velocity_  = nan;
    }

    void update_muzzle_heat() {
        shooter_heat_ -= *shooter_cooling_;
        if (shooter_heat_ < 0)
            shooter_heat_ = 0;

        if (friction_enabled_ && !std::isnan(last_left_friction_velocity_)) {
            double differential = *left_friction_velocity_ - last_left_friction_velocity_;
            if (differential < 0.1)
                friction_velocity_decrease_integral_ += differential;
            else {
                if (friction_velocity_decrease_integral_ < -14.0
                    && last_left_friction_velocity_ < friction_working_velocity - 20.0) {
                    // Heat with 1/1000 tex
                    shooter_heat_ += 10'000 + 10;
                    // Decrease single-shot bullet allowance
                    --bullet_count_limited_by_single_shot_;
                }
                friction_velocity_decrease_integral_ = 0;
            }
        }

        last_left_friction_velocity_ = *left_friction_velocity_;

        bullet_count_limited_by_shooter_heat_ =
            (*shooter_heat_limit_ - shooter_heat_ - 10'000) / 10'000;
        if (bullet_count_limited_by_shooter_heat_ < 0)
            bullet_count_limited_by_shooter_heat_ = 0;
    }

    void update_friction_velocities() {
        double control_velocity           = friction_enabled_ ? friction_working_velocity : 0.0;
        *left_friction_control_velocity_  = control_velocity;
        *right_friction_control_velocity_ = control_velocity;
    }

    void update_bullet_feeder_velocity() {
        auto bullet_allowance = bullet_count_limited_by_shooter_heat_;
        if (0 <= bullet_count_limited_by_single_shot_
            && bullet_count_limited_by_single_shot_ < bullet_allowance)
            bullet_allowance = bullet_count_limited_by_single_shot_;

        if (!friction_enabled_ || !bullet_feeder_enabled_ || bullet_allowance == 0) {
            bullet_feeder_working_status_    = 0;
            *bullet_feeder_control_velocity_ = 0.0;
            return;
        }

        update_jam_detection();

        if (bullet_feeder_cool_down_ > 0) {
            bullet_feeder_cool_down_--;
            return;
        }

        double new_control_velocity = bullet_allowance > 1 ? bullet_feeder_working_velocity
                                                           : bullet_feeder_safe_shot_velocity;
        if (new_control_velocity > *bullet_feeder_control_velocity_)
            bullet_feeder_working_status_ = std::min(0, bullet_feeder_working_status_);
        *bullet_feeder_control_velocity_ = new_control_velocity;
    }

    void update_jam_detection() {
        auto control_velocity = *bullet_feeder_control_velocity_;
        if (control_velocity > 0.0) {
            auto velocity = *bullet_feeder_velocity_;
            if (velocity > control_velocity / 2) {
                if (bullet_feeder_working_status_ < 0) {
                    bullet_feeder_working_status_ = 0;
                } else if (bullet_feeder_working_status_ < 500) {
                    bullet_feeder_working_status_++;
                } else {
                    bullet_feeder_jammed_count_ = 0;
                }
            } else {
                if (bullet_feeder_working_status_ == 500) {
                    enter_jam_protection();
                    RCLCPP_INFO(logger_, "Instant jammed! Count = %d", bullet_feeder_jammed_count_);
                } else if (bullet_feeder_working_status_ > 0) {
                    bullet_feeder_working_status_ = 0;
                } else if (bullet_feeder_working_status_ > -500) {
                    bullet_feeder_working_status_--;
                } else {
                    enter_jam_protection();
                    RCLCPP_INFO(logger_, "Jammed! Count = %d", bullet_feeder_jammed_count_);
                }
            }
        }
    }

    void enter_jam_protection() {
        bullet_feeder_working_status_ = 0;
        if (++bullet_feeder_jammed_count_ <= 2) {
            *bullet_feeder_control_velocity_ = -2 * std::numbers::pi / 16 / 0.05;
            bullet_feeder_cool_down_         = 50;
        } else {
            *bullet_feeder_control_velocity_ = -2 * std::numbers::pi / 8 / 0.2;
            bullet_feeder_cool_down_         = 200;
        }
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    double friction_working_velocity;
    double bullet_feeder_working_velocity, bullet_feeder_safe_shot_velocity;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_         = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();

    InputInterface<double> left_friction_velocity_, right_friction_velocity_;
    double last_left_friction_velocity_ = nan, friction_velocity_decrease_integral_ = 0;

    InputInterface<int64_t> shooter_cooling_, shooter_heat_limit_;
    int64_t shooter_heat_                         = 0;
    int64_t bullet_count_limited_by_shooter_heat_ = 0;
    int64_t bullet_count_limited_by_single_shot_  = -1;

    bool friction_enabled_ = false, bullet_feeder_enabled_ = false;

    InputInterface<double> bullet_feeder_velocity_;
    int bullet_feeder_working_status_ = 0;
    int bullet_feeder_jammed_count_   = 0;
    int bullet_feeder_cool_down_      = 0;

    OutputInterface<double> left_friction_control_velocity_, right_friction_control_velocity_;
    OutputInterface<double> bullet_feeder_control_velocity_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::ShootingController, rmcs_executor::Component)