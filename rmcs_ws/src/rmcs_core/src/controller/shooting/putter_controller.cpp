#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_condiction.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"
#include <rmcs_msgs/gimbal_mode.hpp>

namespace rmcs_core::controller::shooting {

/**
 * @class PutterController
 * @brief Putter mechanism controller
 *
 * Firing mechanism notes:
 * Since the photoelectric sensor is placed at the chamber opening, testing showed that
 * the double-middle action first triggers the putter to retract, and then stall detection
 * confirms that the reset is complete.
 * By default, a small holding force is applied so the putter does not slide down. The entire
 * process uses angle-loop advancement, and grayscale detection determines whether to feed.
 * Bullet firing is detected from two sources: the friction wheels and the putter stroke.
 * The full scheme completed stress testing before the summer break.
 */
class PutterController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PutterController()
        : Node(
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

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/friction_ready", friction_ready_);

        register_input("/gimbal/bullet_feeder/angle", bullet_feeder_angle_);
        register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);

        register_input(
            "/gimbal/control_bullet_allowance/limited_by_heat",
            control_bullet_allowance_limited_by_heat_);

        register_input("/gimbal/photoelectric_sensor", photoelectric_sensor_status_);
        register_input("/gimbal/grayscale_sensor", grayscale_sensor_status_);
        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_input("/gimbal/putter/angle", putter_angle_);
        register_input("/gimbal/putter/velocity", putter_velocity_);

        set_pid_parameter(bullet_feeder_velocity_pid_, "bullet_feeder_velocity");
        set_pid_parameter(putter_return_velocity_pid_, "putter_return_velocity");

        register_output(
            "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, nan_);
        register_output("/gimbal/putter/control_torque", putter_control_torque_, nan_);

        register_output("/gimbal/shoot/delay_ms", shoot_delay_ms_, nan_);

        // auto_aim
        register_input("/auto_aim/should_shoot", should_shoot_, false);

        register_output("/gimbal/shooter/mode", shoot_mode_, rmcs_msgs::ShootMode::SINGLE);
        register_output("/gimbal/shooter/condiction", shoot_condiction_);
        register_output("/gimbal/shooter/preloaded_ready", preloaded_ready_, false);
    }

    void before_updating() override {
        if (!should_shoot_.ready())
            should_shoot_.bind_directly(false);
    }

    void update() override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto mouse = *mouse_;
        const auto keyboard = *keyboard_;

        using namespace rmcs_msgs;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        // Normal control flow after the putter has been initialized.
        if (putter_initialized) {
            // Handling during bullet-feeder jam-protection cooldown.
            if (bullet_feeder_reverse_end_ > 0) {
                bullet_feeder_reverse_end_--;

                // Early cooldown stage: reverse the feeder to clear the jam.
                if (bullet_feeder_reverse_end_ > 300)
                    *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                        -low_latency_velocity_ / 2 - *bullet_feeder_velocity_);
                else {
                    // Late cooldown stage: stop control.
                    bullet_feeder_velocity_pid_.reset();
                    *bullet_feeder_control_torque_ = 0.0;
                }

                if (!bullet_feeder_reverse_end_ && shoot_stage_ == ShootStage::PRELOADED)
                // RCLCPP_INFO(get_logger(), "Reverse finished");
                {
                    *preloaded_ready_ = true;
                } else {
                    *preloaded_ready_ = false;
                }

            } else {
                // Normal operating mode: only fire when the friction wheels are ready.
                if (*friction_ready_) {
                    // Detect fire triggers.
                    if (switch_right != Switch::DOWN) {

                        const auto now = std::chrono::steady_clock::now();
                        const bool left_click_edge = (!last_mouse_.left && mouse.left);
                        if (left_click_edge) {
                            if (now - last_click_time_ < std::chrono::milliseconds(500)) {
                                click_count_++;
                            } else {
                                click_count_ = 1;
                            }
                            last_click_time_ = now;
                        }

                        const bool manual_trigger =
                            (!last_mouse_.left && mouse.left && !mouse.right)
                            || (last_switch_left_ == rmcs_msgs::Switch::MIDDLE
                                && switch_left == rmcs_msgs::Switch::DOWN);

                        const bool auto_fire_now =
                            (switch_right == Switch::UP || mouse.right) && *should_shoot_;

                        const bool auto_trigger_emergence = mouse.right && (click_count_ >= 2);

                        const bool auto_trigger =
                            auto_fire_now
                            && (now - last_fire_time_ > std::chrono::milliseconds(1000));

                        if (manual_trigger || auto_trigger || auto_trigger_emergence) {
                            if (*control_bullet_allowance_limited_by_heat_ > 0
                                && (shoot_stage_ == ShootStage::PRELOADED || shoot_first)) {
                                set_shooting();
                                last_fire_time_ = now;
                                shoot_first = false;
                            }
                        }
                        if (auto_trigger_emergence) {
                            click_count_ = 0;
                        }
                    }

                    if (shoot_stage_ == ShootStage::PRELOADING) {

                        *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                            low_latency_velocity_ - *bullet_feeder_velocity_); // Velocity loop.

                        update_locked_detection();
                        // This includes the photoelectric-sensor logic: if triggered, switch to
                        // preloaded; otherwise reverse briefly and continue rotating.
                    }

                    if (shoot_stage_ == ShootStage::SHOOTING) {
                        // Firing state: detect whether the bullet has been fired.
                        // if (*bullet_fired_ && !shooted) {
                        //     RCLCPP_INFO(get_logger(), "DETECT: Bullet fired!");
                        //     shooted = true;
                        // }

                        // if (*putter_angle_ - putter_startpoint >= putter_stroke_ && !shooted) {
                        //     RCLCPP_INFO(get_logger(), "DETECT: Putter stroke completed!");
                        //     shooted = true;
                        // }

                        if (shooted) {
                            // Bullet fired: return the putter.
                            *putter_control_torque_ =
                                putter_return_velocity_pid_.update(-50. - *putter_velocity_);
                            putter_timeout_detection();
                        } else {
                            // Bullet not fired yet: continue advancing.
                            *putter_control_torque_ =
                                putter_return_velocity_pid_.update(120. - *putter_velocity_);
                            update_putter_jam_detection();
                        }
                    }
                } else {
                    // Friction wheels not ready: stop the bullet feeder.
                    *bullet_feeder_control_torque_ = 0.;
                }

                // Non-firing state: apply a small holding force to the putter.
                if (shoot_stage_ != ShootStage::SHOOTING)
                    *putter_control_torque_ = -0.02;
            }
        } else {
            // Putter not initialized: perform the reset procedure.
            *putter_control_torque_ = putter_return_velocity_pid_.update(-80. - *putter_velocity_);
            update_putter_jam_detection();
        }

        // Save the current state for the next comparison.
        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_mouse_ = mouse;
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_mouse_ = rmcs_msgs::Mouse::zero();
        last_keyboard_ = rmcs_msgs::Keyboard::zero();

        bullet_feeder_velocity_pid_.reset();
        *bullet_feeder_control_torque_ = nan_;

        shoot_stage_ = ShootStage::PRELOADED;

        putter_initialized = false;
        putter_startpoint = nan_;
        putter_return_velocity_pid_.reset();
        *putter_control_torque_ = nan_;

        bullet_feeder_faulty_count_ = 0;

        *shoot_delay_ms_ = nan_;
    }

    void set_preloading() {
        RCLCPP_INFO(get_logger(), "PRELOADING");
        shoot_stage_ = ShootStage::PRELOADING;
    }

    void set_preloaded() {
        RCLCPP_INFO(get_logger(), "PRELOADED");
        shoot_stage_ = ShootStage::PRELOADED;
    }

    void set_shooting() {
        RCLCPP_INFO(get_logger(), "SHOOTING");
        shoot_stage_ = ShootStage::SHOOTING;
    }

    void update_locked_detection() {
        // If feeder speed is near zero and the photoelectric sensor is triggered,
        // treat it as locked and start reversing.
        if (*bullet_feeder_velocity_ < 0.5 && *bullet_feeder_control_torque_ > 0.1) {
            locked_detect_count_++;
        } else {
            locked_detect_count_ = 0;
        }

        if (locked_detect_count_ > 150) {
            if (*photoelectric_sensor_status_) {
                set_preloaded();
            }
            // If the photoelectric sensor was not triggered, treat it as a simple jam,
            // reverse briefly, then continue until stall.
            locked_detect_count_ = 0;
            enter_reverse_protection();
        }
    }

    void update_putter_jam_detection() {
        if (std::abs(*putter_velocity_) > 0.1 || std::isnan(*putter_control_torque_)) {
            putter_faulty_count_ = 0;
        } else {
            putter_faulty_count_++;
        }

        // Accumulate a fault count when the torque is abnormal.
        if (putter_faulty_count_ >= 50) {
            putter_faulty_count_ = 0;
            if (shoot_stage_ != ShootStage::SHOOTING) {
                // Stall detected outside the firing state: the putter is in position,
                // so mark it initialized.
                putter_initialized = true;
                putter_startpoint = *putter_angle_;
            } else {
                // Stall detected during firing: treat the bullet as fired.
                RCLCPP_INFO(get_logger(), "DETECT: Putter freezed");
                shooted = true;
            }
        }
    }

    void putter_timeout_detection() {
        // If the putter stays in the firing state too long without extending,
        // treat it as finished and move to the next state.
        if (shoot_stage_ == ShootStage::SHOOTING) {
            if (shooted) {
                if (putter_timeout_count_ < 400)
                    ++putter_timeout_count_;
                else {
                    putter_timeout_count_ = 0;
                    RCLCPP_INFO(get_logger(), "PUTTER TIMEOUT");
                    set_preloading();
                    shooted = false;
                }
            }
        }
    }

    void enter_reverse_protection() {
        locked_detect_count_ = 0;
        bullet_feeder_faulty_count_ = 0;
        bullet_feeder_reverse_end_ = 400;
        bullet_feeder_velocity_pid_.reset();
    }

    static constexpr double nan_ =
        std::numeric_limits<double>::quiet_NaN(); ///< Not-a-number constant.
    static constexpr double inf_ = std::numeric_limits<double>::infinity(); ///< Infinity constant.

    static constexpr double putter_stroke_ = 12.0; ///< Putter stroke length.

    static constexpr double max_bullet_feeder_control_torque_ = 0.1;
    static constexpr double bullet_feeder_angle_per_bullet_ = 2 * std::numbers::pi / 6;
    static constexpr double low_latency_velocity_ = 5.0;

    InputInterface<bool> photoelectric_sensor_status_;
    InputInterface<bool> grayscale_sensor_status_;
    InputInterface<bool> bullet_fired_;
    bool shooted{false};
    bool shoot_first{true};

    InputInterface<bool> friction_ready_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> bullet_feeder_angle_;
    InputInterface<double> bullet_feeder_velocity_;

    InputInterface<int64_t> control_bullet_allowance_limited_by_heat_;

    bool putter_initialized = false;
    int putter_faulty_count_ = 0;
    int putter_timeout_count_ = 0;
    double putter_startpoint = nan_;
    pid::PidCalculator putter_return_velocity_pid_;
    InputInterface<double> putter_velocity_;

    enum class ShootStage { PRELOADING, PRELOADED, SHOOTING };
    ShootStage shoot_stage_ = ShootStage::PRELOADING;

    pid::PidCalculator bullet_feeder_velocity_pid_;

    OutputInterface<double> bullet_feeder_control_torque_;

    InputInterface<double> putter_angle_;
    OutputInterface<double> putter_control_torque_;

    int bullet_feeder_faulty_count_ = 0;

    OutputInterface<double> shoot_delay_ms_;

    InputInterface<bool> should_shoot_;
    std::chrono::steady_clock::time_point last_fire_time_{};
    std::chrono::steady_clock::time_point last_click_time_{};
    int click_count_ = 0;

    int locked_detect_count_ = 0;
    int bullet_feeder_reverse_end_ = 0;

    InputInterface<double> bullet_feeder_torque;
    InputInterface<double> putter_torque;

    OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;
    OutputInterface<rmcs_msgs::ShootCondiction> shoot_condiction_;
    OutputInterface<bool> preloaded_ready_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::PutterController, rmcs_executor::Component)
