#include <cmath>

#include <limits>

#include <eigen3/Eigen/Dense>
#include <fast_tf/rcl.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/shoot_status.hpp>
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

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/referee/shooter/cooling", shooter_cooling_, false);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_, false);

        auto friction_wheels     = get_parameter("friction_wheels").as_string_array();
        auto friction_velocities = get_parameter("friction_velocities").as_double_array();
        if (friction_wheels.size() != friction_wheels.size())
            throw std::runtime_error(
                "Mismatch in array sizes: "
                "'friction_wheels' and 'friction_velocities' must have the same length!");
        else if (friction_wheels.size() == 0)
            throw std::runtime_error(
                "Empty array error: 'friction_wheels' and 'friction_velocities' cannot be empty!");

        friction_count_              = friction_wheels.size();
        friction_working_velocities_ = std::make_unique<double[]>(friction_count_);
        friction_velocities_         = std::make_unique<InputInterface<double>[]>(friction_count_);
        friction_control_velocities_ = std::make_unique<OutputInterface<double>[]>(friction_count_);
        for (size_t i = 0; i < friction_count_; i++) {
            friction_working_velocities_[i] = friction_velocities[i];
            register_input(friction_wheels[i] + "/velocity", friction_velocities_[i]);
            register_output(
                friction_wheels[i] + "/control_velocity", friction_control_velocities_[i]);
        }

        is_42mm_ = get_parameter("is_42mm").as_bool();

        double bullets_per_feeder_turn      = get_parameter("bullets_per_feeder_turn").as_double();
        bullet_feeder_angle_per_bullet_     = 2 * std::numbers::pi / bullets_per_feeder_turn;
        bullet_feeder_angle_per_1_5_bullet_ = bullet_feeder_angle_per_bullet_ * 1.5;

        double shot_frequency            = get_parameter("shot_frequency").as_double();
        bullet_feeder_working_velocity   = bullet_feeder_angle_per_bullet_ * shot_frequency;
        double safe_shot_frequency       = get_parameter("safe_shot_frequency").as_double();
        bullet_feeder_safe_shot_velocity = bullet_feeder_angle_per_bullet_ * safe_shot_frequency;
        double precise_shot_frequency    = 0;
        if (get_parameter("precise_shot_frequency", precise_shot_frequency))
            bullet_feeder_precise_shot_velocity =
                bullet_feeder_angle_per_bullet_ * precise_shot_frequency;

        double eject_frequency        = get_parameter("eject_frequency").as_double();
        bullet_feeder_eject_velocity_ = -bullet_feeder_angle_per_bullet_ * eject_frequency;
        bullet_feeder_eject_time_ =
            static_cast<int>(std::round(1000.0 * get_parameter("eject_time").as_double()));

        double deep_eject_frequency = get_parameter("deep_eject_frequency").as_double();
        bullet_feeder_deep_eject_velocity_ =
            -bullet_feeder_angle_per_bullet_ * deep_eject_frequency;
        bullet_feeder_deep_eject_time_ =
            static_cast<int>(std::round(1000.0 * get_parameter("deep_eject_time").as_double()));

        single_shot_max_stop_delay_ = static_cast<int>(
            std::round(1000.0 * get_parameter("single_shot_max_stop_delay").as_double()));

        register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);
        register_input("/gimbal/bullet_feeder/angle", bullet_feeder_multi_turn_angle_);
        register_output(
            "/gimbal/bullet_feeder/control_velocity", bullet_feeder_control_velocity_, nan_);

        register_output("/gimbal/shooter/mode", shoot_mode_, default_shoot_mode());
        register_output(
            "/gimbal/shooter/status", shoot_status_, rmcs_msgs::ShootStatus{false, 0, 0, 0, 0});
    }

    void before_updating() override {
        constexpr int64_t safe_shooter_cooling_17mm = 10;
        constexpr int64_t safe_heat_limit_17mm      = 50'000;

        constexpr int64_t safe_shooter_cooling_42mm = 40;
        constexpr int64_t safe_heat_limit_42mm      = 200'000;

        if (!shooter_cooling_.ready()) {
            auto safe_shooter_cooling =
                is_42mm_ ? safe_shooter_cooling_42mm : safe_shooter_cooling_17mm;
            shooter_cooling_.make_and_bind_directly(safe_shooter_cooling);
            RCLCPP_WARN(
                get_logger(),
                "Failed to fetch \"/referee/shooter/cooling\". Set to safe value %ld.",
                safe_shooter_cooling);
        }
        if (!shooter_heat_limit_.ready()) {
            auto safe_heat_limit = is_42mm_ ? safe_heat_limit_42mm : safe_heat_limit_17mm;
            shooter_heat_limit_.make_and_bind_directly(safe_heat_limit);
            RCLCPP_WARN(
                get_logger(),
                "Failed to fetch \"/referee/shooter/heat_limit\". Set to safe value %ld.",
                safe_heat_limit);
        }
    }

    void update() override {
        update_muzzle_heat();

        auto& shoot_mode = *shoot_mode_;

        const auto switch_right = *switch_right_;
        const auto switch_left  = *switch_left_;
        const auto mouse        = *mouse_;
        const auto keyboard     = *keyboard_;

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

                const auto default_mode     = default_shoot_mode();
                const auto alternative_mode = alternative_shoot_mode();

                if (keyboard.f)
                    shoot_mode = alternative_mode;
                else if (shoot_mode == alternative_mode)
                    shoot_mode = default_mode;

                if (!last_keyboard_.g && keyboard.g) {
                    shoot_mode = shoot_mode == rmcs_msgs::ShootMode::PRECISE
                                   ? default_mode
                                   : rmcs_msgs::ShootMode::PRECISE;
                }
                if (is_42mm_) {
                    if (switch_right == Switch::UP)
                        shoot_mode = rmcs_msgs::ShootMode::PRECISE;
                    else if (shoot_mode == rmcs_msgs::ShootMode::PRECISE)
                        shoot_mode = default_mode;
                }

                if (shoot_mode == rmcs_msgs::ShootMode::SINGLE
                    || shoot_mode == rmcs_msgs::ShootMode::PRECISE) {
                    if (bullet_count_limited_by_single_shot_ < 0)
                        bullet_count_limited_by_single_shot_ = 0;
                    if ((!last_mouse_.left && mouse.left)
                        || (last_switch_left_ == rmcs_msgs::Switch::MIDDLE
                            && switch_left == rmcs_msgs::Switch::DOWN)) {
                        bullet_count_limited_by_single_shot_++;
                        single_shot_delayed_stop_counter_ = 0;
                    }

                    if (!bullet_feeder_enabled_ && bullet_count_limited_by_single_shot_) {
                        if (++single_shot_delayed_stop_counter_ != single_shot_max_stop_delay_) {
                            bullet_feeder_enabled_ = true;
                        } else {
                            bullet_count_limited_by_single_shot_ = 0;
                            single_shot_delayed_stop_counter_    = 0;
                            shoot_status_->single_shot_cancelled_count++;
                        }
                    }
                } else {
                    bullet_count_limited_by_single_shot_ = -1;
                    single_shot_delayed_stop_counter_    = 0;
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
        *shoot_mode_         = default_shoot_mode();
        shoot_status_->ready = false;

        friction_enabled_ = false;
        for (size_t i = 0; i < friction_count_; i++)
            *friction_control_velocities_[i] = nan_;

        bullet_feeder_enabled_           = false;
        *bullet_feeder_control_velocity_ = nan_;
        bullet_feeder_last_shoot_angle_  = *bullet_feeder_multi_turn_angle_;
    }

    rmcs_msgs::ShootMode default_shoot_mode() const {
        return is_42mm_ ? rmcs_msgs::ShootMode::SINGLE : rmcs_msgs::ShootMode::AUTOMATIC;
    }

    rmcs_msgs::ShootMode alternative_shoot_mode() const {
        return is_42mm_ ? rmcs_msgs::ShootMode::AUTOMATIC : rmcs_msgs::ShootMode::SINGLE;
    }

    void update_muzzle_heat() {
        shooter_heat_ -= *shooter_cooling_;
        if (shooter_heat_ < 0)
            shooter_heat_ = 0;

        int64_t heat_per_shot = (is_42mm_ ? 100'000 : 10'000);

        // The first friction wheel in the list is considered the primary one, meaning we only
        // monitor the speed drop of this wheel to detect whether a bullet has been fired.
        if (friction_enabled_ && !std::isnan(last_primary_friction_velocity_)) {
            double differential = *friction_velocities_[0] - last_primary_friction_velocity_;
            if (differential < 0.1)
                primary_friction_velocity_decrease_integral_ += differential;
            else {
                if (primary_friction_velocity_decrease_integral_ < -14.0
                    && last_primary_friction_velocity_ < friction_working_velocities_[0] - 20.0) {
                    // Heat with 1/1000 tex
                    shooter_heat_ += heat_per_shot + 10;

                    // Decrease single-shot bullet allowance
                    --bullet_count_limited_by_single_shot_;
                    single_shot_delayed_stop_counter_ = 0;

                    shoot_status_->fired_count++;

                    bullet_feeder_last_shoot_angle_ = *bullet_feeder_multi_turn_angle_;
                }
                primary_friction_velocity_decrease_integral_ = 0;
            }
        }

        last_primary_friction_velocity_ = *friction_velocities_[0];

        bullet_count_limited_by_shooter_heat_ =
            (*shooter_heat_limit_ - shooter_heat_ - 10'000) / heat_per_shot;
        if (bullet_count_limited_by_shooter_heat_ < 0)
            bullet_count_limited_by_shooter_heat_ = 0;
    }

    void update_friction_velocities() {
        shoot_status_->ready = friction_enabled_;
        if (friction_enabled_) {
            for (size_t i = 0; i < friction_count_; i++)
                *friction_control_velocities_[i] = friction_working_velocities_[i];
        } else {
            for (size_t i = 0; i < friction_count_; i++)
                *friction_control_velocities_[i] = 0.0;
        }
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
        if (*shoot_mode_ == rmcs_msgs::ShootMode::PRECISE)
            new_control_velocity =
                std::min(new_control_velocity, bullet_feeder_precise_shot_velocity);
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
            *bullet_feeder_control_velocity_ = bullet_feeder_eject_velocity_;
            bullet_feeder_cool_down_         = bullet_feeder_eject_time_;
        } else {
            *bullet_feeder_control_velocity_ = bullet_feeder_deep_eject_velocity_;
            bullet_feeder_cool_down_         = bullet_feeder_deep_eject_time_;
        }

        shoot_status_->jammed_count++;
    }

    void update_dry_feed_detection() {
        while (*bullet_feeder_multi_turn_angle_ - bullet_feeder_last_shoot_angle_
               >= bullet_feeder_angle_per_1_5_bullet_) {
            bullet_feeder_last_shoot_angle_ += bullet_feeder_angle_per_bullet_;
            shoot_status_->dry_fed_count++;
        }
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    size_t friction_count_;
    bool is_42mm_;

    std::unique_ptr<double[]> friction_working_velocities_;
    double bullet_feeder_working_velocity, bullet_feeder_safe_shot_velocity,
        bullet_feeder_precise_shot_velocity;
    double bullet_feeder_eject_velocity_, bullet_feeder_deep_eject_velocity_;
    int bullet_feeder_eject_time_, bullet_feeder_deep_eject_time_;

    int single_shot_delayed_stop_counter_ = 0, single_shot_max_stop_delay_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_         = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();

    std::unique_ptr<InputInterface<double>[]> friction_velocities_;
    double last_primary_friction_velocity_              = nan_;
    double primary_friction_velocity_decrease_integral_ = 0;

    InputInterface<int64_t> shooter_cooling_, shooter_heat_limit_;
    int64_t shooter_heat_                         = 0;
    int64_t bullet_count_limited_by_shooter_heat_ = 0;
    int64_t bullet_count_limited_by_single_shot_  = -1;

    bool friction_enabled_ = false, bullet_feeder_enabled_ = false;

    InputInterface<double> bullet_feeder_velocity_;
    int bullet_feeder_working_status_ = 0;
    int bullet_feeder_jammed_count_   = 0;
    int bullet_feeder_cool_down_      = 0;

    InputInterface<double> bullet_feeder_multi_turn_angle_;
    double bullet_feeder_angle_per_bullet_, bullet_feeder_angle_per_1_5_bullet_;
    double bullet_feeder_last_shoot_angle_ = 0;

    std::unique_ptr<OutputInterface<double>[]> friction_control_velocities_;
    OutputInterface<double> bullet_feeder_control_velocity_;

    OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;
    OutputInterface<rmcs_msgs::ShootStatus> shoot_status_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::ShootingController, rmcs_executor::Component)