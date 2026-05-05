#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::shooting {

class BulletFeederController42mm
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    BulletFeederController42mm()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

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

        bullet_feeder_velocity_pid_.kp           = 50.0;
        bullet_feeder_velocity_pid_.ki           = 10.0;
        bullet_feeder_velocity_pid_.kd           = 0.0;
        bullet_feeder_velocity_pid_.integral_max = 60.0;
        bullet_feeder_velocity_pid_.integral_min = 0.0;

        bullet_feeder_angle_pid_.kp = 50.0;
        bullet_feeder_angle_pid_.ki = 0.0;
        bullet_feeder_angle_pid_.kd = 2.0;

        register_output(
            "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, nan_);

        // For compatibility
        register_output("/gimbal/shooter/mode", shoot_mode_, rmcs_msgs::ShootMode::SINGLE);
    }

    void update() override {
        const auto switch_right = *switch_right_;
        const auto switch_left  = *switch_left_;
        const auto mouse        = *mouse_;
        const auto keyboard     = *keyboard_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        overdrive_mode_ = keyboard.f;
        if (keyboard.ctrl && !last_keyboard_.r && keyboard.r)
            low_latency_mode_ = !low_latency_mode_;

        if (bullet_feeder_cool_down_ > 0) {
            bullet_feeder_cool_down_--;

            if (bullet_feeder_cool_down_ > 500)
                *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                    -5.0 * bullet_feeder_angle_per_bullet_ - *bullet_feeder_velocity_);
            else {
                bullet_feeder_velocity_pid_.reset();
                *bullet_feeder_control_torque_ = 0.0;
            }

            bullet_feeder_angle_pid_.reset();

            if (!bullet_feeder_cool_down_)
                RCLCPP_INFO(get_logger(), "Jamming Solved!");
        } else {
            if (!*friction_ready_ || std::isnan(bullet_feeder_control_angle_)) {
                bullet_feeder_control_angle_ = *bullet_feeder_angle_;
                shoot_stage_                 = ShootStage::PRELOADED;
                bullet_fed_count_            = static_cast<int>(
                    (*bullet_feeder_angle_ - bullet_feeder_compressed_zero_point_ - 0.1)
                    / bullet_feeder_angle_per_bullet_);
            }

            if (*friction_ready_) {
                if (switch_right != Switch::DOWN) {
                    if ((!last_mouse_.left && mouse.left)
                        || (last_switch_left_ == rmcs_msgs::Switch::MIDDLE
                            && switch_left == rmcs_msgs::Switch::DOWN)) {
                        if (*control_bullet_allowance_limited_by_heat_ > 0)
                            set_shooting();
                    }
                }

                double err_abs = std::abs(bullet_feeder_control_angle_ - *bullet_feeder_angle_);
                // RCLCPP_INFO(
                //     get_logger(), "%.2f %.2f %.2f", bullet_feeder_control_angle_,
                //     *bullet_feeder_angle_, err_abs);
                if (shoot_stage_ == ShootStage::PRELOADING) {
                    if (err_abs < 0.1)
                        set_preloaded();
                }
                if (shoot_stage_ == ShootStage::PRELOADED) {
                    if (low_latency_mode_)
                        set_compressing();
                }
                if (shoot_stage_ == ShootStage::COMPRESSING) {
                    if (err_abs < 0.1)
                        set_compressed();
                }
                if (shoot_stage_ == ShootStage::SHOOTING) {
                    if (err_abs < 0.1)
                        set_preloading();
                }
            }

            double velocity_err = bullet_feeder_angle_pid_.update(
                                      bullet_feeder_control_angle_ - *bullet_feeder_angle_)
                                - *bullet_feeder_velocity_;
            // bullet_feeder_velocity_pid_.integral_max = std::clamp(1000.0 * velocity_err,
            // 0.0, 60.0);
            *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(velocity_err);
            // RCLCPP_INFO(
            //     get_logger(), "%.2f %.2f", velocity_err, *bullet_feeder_control_torque_);

            update_jam_detection();
        }

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
        last_mouse_        = mouse;
        last_keyboard_     = keyboard;
    }

private:
    void reset_all_controls() {
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
        last_mouse_        = rmcs_msgs::Mouse::zero();
        last_keyboard_     = rmcs_msgs::Keyboard::zero();

        overdrive_mode_ = low_latency_mode_ = false;

        shoot_stage_      = ShootStage::PRELOADED;
        bullet_fed_count_ = std::numeric_limits<int>::min();

        bullet_feeder_control_angle_        = nan_;
        bullet_feeder_angle_pid_.output_max = inf_;

        bullet_feeder_velocity_pid_.reset();
        bullet_feeder_angle_pid_.reset();
        *bullet_feeder_control_torque_ = nan_;

        bullet_feeder_faulty_count_ = 0;
        bullet_feeder_cool_down_    = 0;
    }

    void set_preloading() {
        RCLCPP_INFO(get_logger(), "PRELOADING");
        bullet_fed_count_++;
        shoot_stage_                 = ShootStage::PRELOADING;
        bullet_feeder_control_angle_ = bullet_feeder_compressed_zero_point_
                                     + (bullet_fed_count_ + 0.5) * bullet_feeder_angle_per_bullet_;
        bullet_feeder_angle_pid_.output_max = 1.0;
    }

    void set_preloaded() {
        RCLCPP_INFO(get_logger(), "PRELOADED");
        shoot_stage_ = ShootStage::PRELOADED;
    }

    void set_compressing() {
        RCLCPP_INFO(get_logger(), "COMPRESSING");
        shoot_stage_                 = ShootStage::COMPRESSING;
        bullet_feeder_control_angle_ = bullet_feeder_compressed_zero_point_
                                     + (bullet_fed_count_ + 1) * bullet_feeder_angle_per_bullet_;
        bullet_feeder_angle_pid_.output_max = 0.8;
    }

    void set_compressed() {
        RCLCPP_INFO(get_logger(), "COMPRESSED");
        shoot_stage_ = ShootStage::COMPRESSED;
    }

    void set_shooting() {
        RCLCPP_INFO(get_logger(), "SHOOTING");
        shoot_stage_                 = ShootStage::SHOOTING;
        bullet_feeder_control_angle_ = bullet_feeder_compressed_zero_point_
                                     + (bullet_fed_count_ + 1.2) * bullet_feeder_angle_per_bullet_;
        bullet_feeder_angle_pid_.output_max = 1.0;
    }

    void update_jam_detection() {
        // RCLCPP_INFO(get_logger(), "%.2f --", *bullet_feeder_control_torque_);
        if (*bullet_feeder_control_torque_ < 300.0) {
            bullet_feeder_faulty_count_ = 0;
            return;
        }

        if (bullet_feeder_faulty_count_ < 1000)
            bullet_feeder_faulty_count_++;
        else {
            bullet_feeder_faulty_count_ = 0;
            enter_jam_protection();
        }
    }

    void enter_jam_protection() {
        bullet_feeder_control_angle_ = nan_;
        bullet_feeder_cool_down_     = 1000;
        bullet_feeder_angle_pid_.reset();
        bullet_feeder_velocity_pid_.reset();
        RCLCPP_INFO(get_logger(), "Jammed!");
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double bullet_feeder_compressed_zero_point_ = 0.58;
    static constexpr double bullet_feeder_angle_per_bullet_      = 2 * std::numbers::pi / 6;

    InputInterface<bool> friction_ready_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_         = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();

    bool overdrive_mode_ = false, low_latency_mode_ = false;

    InputInterface<double> bullet_feeder_angle_;
    InputInterface<double> bullet_feeder_velocity_;

    InputInterface<int64_t> control_bullet_allowance_limited_by_heat_;

    enum class ShootStage { PRELOADING, PRELOADED, COMPRESSING, COMPRESSED, SHOOTING };
    ShootStage shoot_stage_             = ShootStage::PRELOADED;
    int bullet_fed_count_               = std::numeric_limits<int>::min();
    double bullet_feeder_control_angle_ = nan_;

    pid::PidCalculator bullet_feeder_velocity_pid_;
    pid::PidCalculator bullet_feeder_angle_pid_;
    OutputInterface<double> bullet_feeder_control_torque_;

    int bullet_feeder_faulty_count_ = 0;
    int bullet_feeder_cool_down_    = 0;

    OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::BulletFeederController42mm, rmcs_executor::Component)