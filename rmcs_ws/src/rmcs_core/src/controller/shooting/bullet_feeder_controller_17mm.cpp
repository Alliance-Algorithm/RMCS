#include <cmath>

#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::shooting {

class BulletFeederController17mm
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    BulletFeederController17mm()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        double bullets_per_feeder_turn = get_parameter("bullets_per_feeder_turn").as_double();
        double bullet_feeder_angle_per_bullet = 2 * std::numbers::pi / bullets_per_feeder_turn;

        double shot_frequency = get_parameter("shot_frequency").as_double();
        bullet_feeder_working_velocity = bullet_feeder_angle_per_bullet * shot_frequency;
        double safe_shot_frequency = get_parameter("safe_shot_frequency").as_double();
        bullet_feeder_safe_shot_velocity = bullet_feeder_angle_per_bullet * safe_shot_frequency;

        double eject_frequency = get_parameter("eject_frequency").as_double();
        bullet_feeder_eject_velocity_ = -bullet_feeder_angle_per_bullet * eject_frequency;
        bullet_feeder_eject_time_ =
            static_cast<int>(std::round(1000.0 * get_parameter("eject_time").as_double()));

        double deep_eject_frequency = get_parameter("deep_eject_frequency").as_double();
        bullet_feeder_deep_eject_velocity_ = -bullet_feeder_angle_per_bullet * deep_eject_frequency;
        bullet_feeder_deep_eject_time_ =
            static_cast<int>(std::round(1000.0 * get_parameter("deep_eject_time").as_double()));

        single_shot_max_stop_delay_ = static_cast<int>(
            std::round(1000.0 * get_parameter("single_shot_max_stop_delay").as_double()));

        register_input("/gimbal/friction_ready", friction_ready_);
        register_input("/gimbal/bullet_fired", bullet_fired_);
        register_input(
            "/gimbal/control_bullet_allowance/limited_by_heat",
            control_bullet_allowance_limited_by_heat_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/auto_aim/fire_control", fire_control_, false);

        register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);
        register_output(
            "/gimbal/bullet_feeder/control_velocity", bullet_feeder_control_velocity_, nan_);

        register_output("/gimbal/shooter/mode", shoot_mode_, rmcs_msgs::ShootMode::AUTOMATIC);
    }

    void before_updating() override {
        if (!fire_control_.ready())
            fire_control_.bind_directly(false);
    }

    void update() override {
        auto& shoot_mode = *shoot_mode_;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto mouse = *mouse_;
        const auto keyboard = *keyboard_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            int64_t bullet_allowance = 0;

            if (switch_right != Switch::DOWN) {
                shoot_mode = keyboard.f ? ShootMode::SINGLE : ShootMode::AUTOMATIC;

                single_shot_stop_counter_ = std::max(0, single_shot_stop_counter_ - 1);
                temporary_single_shot_counter_ = std::max(0, temporary_single_shot_counter_ - 1);

                if (!last_mouse_.left && mouse.left)
                    single_shot_stop_counter_ = single_shot_max_stop_delay_;
                else if (last_switch_left_ != Switch::DOWN && switch_left == Switch::DOWN) {
                    single_shot_stop_counter_ = single_shot_max_stop_delay_;
                    temporary_single_shot_counter_ = 500;
                }

                shoot_mode = temporary_single_shot_counter_ > 0 ? ShootMode::SINGLE : shoot_mode;

                if (*bullet_fired_)
                    single_shot_stop_counter_ = 0;

                if (*friction_ready_) {
                    if (shoot_mode == ShootMode::AUTOMATIC) {
                        bool triggered = mouse.left || switch_left == Switch::DOWN
                                      || (switch_right == Switch::UP && *fire_control_);
                        bullet_allowance =
                            triggered ? *control_bullet_allowance_limited_by_heat_ : 0;
                    } else {
                        bool triggered = single_shot_stop_counter_ > 0;
                        bullet_allowance =
                            triggered && (*control_bullet_allowance_limited_by_heat_ > 0);
                    }
                }
            }

            update_bullet_feeder_velocity(bullet_allowance);
        }

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_mouse_ = mouse;
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        *shoot_mode_ = rmcs_msgs::ShootMode::AUTOMATIC;

        *bullet_feeder_control_velocity_ = nan_;
    }

    void update_bullet_feeder_velocity(int64_t bullet_allowance) {
        if (bullet_allowance <= 0) {
            bullet_feeder_working_status_ = 0;
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
            *bullet_feeder_control_velocity_ = bullet_feeder_eject_velocity_;
            bullet_feeder_cool_down_ = bullet_feeder_eject_time_;
        } else {
            *bullet_feeder_control_velocity_ = bullet_feeder_deep_eject_velocity_;
            bullet_feeder_cool_down_ = bullet_feeder_deep_eject_time_;
        }
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    double bullet_feeder_working_velocity, bullet_feeder_safe_shot_velocity;
    double bullet_feeder_eject_velocity_, bullet_feeder_deep_eject_velocity_;
    int bullet_feeder_eject_time_, bullet_feeder_deep_eject_time_;

    int single_shot_max_stop_delay_, single_shot_stop_counter_ = 0;

    InputInterface<bool> friction_ready_;
    InputInterface<bool> bullet_fired_;
    InputInterface<int64_t> control_bullet_allowance_limited_by_heat_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    InputInterface<bool> fire_control_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> bullet_feeder_velocity_;
    int bullet_feeder_working_status_ = 0;
    int bullet_feeder_jammed_count_ = 0;
    int bullet_feeder_cool_down_ = 0;

    int temporary_single_shot_counter_ = 0;
    OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;

    OutputInterface<double> bullet_feeder_control_velocity_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::BulletFeederController17mm, rmcs_executor::Component)