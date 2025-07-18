#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "librmcs/utility/logging.hpp"

namespace rmcs_core::controller::shooting {

class PutterController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PutterController()
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

        register_input("/gimbal/photoelectric_sensor", photoelectric_sensor_status_);
        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_input("/gimbal/putter/angle", putter_angle_);
        register_input("/gimbal/putter/velocity", putter_velocity_);

        last_preload_flag_ = false;
        bullet_feeder_velocity_pid_.kp = 50.0;
        bullet_feeder_velocity_pid_.ki = 10.0;
        bullet_feeder_velocity_pid_.kd = 0.0;
        bullet_feeder_velocity_pid_.integral_max = 60.0;
        bullet_feeder_velocity_pid_.integral_min = 0.0;

        bullet_feeder_angle_pid_.kp = 8.0;
        bullet_feeder_angle_pid_.ki = 0.0;
        bullet_feeder_angle_pid_.kd = 2.0;

        puttter_return_velocity_pid_.kp = 0.0015;
        puttter_return_velocity_pid_.ki = 0.00005;
        puttter_return_velocity_pid_.kd = 0.;
        puttter_return_velocity_pid_.integral_max = 0.;
        puttter_return_velocity_pid_.integral_min = -0.03;

        putter_velocity_pid_.kp = 0.004;
        putter_velocity_pid_.ki = 0.0001;
        putter_velocity_pid_.kd = 0.001;
        puttter_return_velocity_pid_.integral_max = 0.03;
        puttter_return_velocity_pid_.integral_min = 0.;

        putter_return_angle_pid.kp = 0.0001;
        // putter_return_angle_pid.ki = 0.000001;
        putter_return_angle_pid.kd = 0.;

        register_output(
            "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, nan_);
        register_output("/gimbal/putter/control_torque", putter_control_torque_, nan_);

        // For compatibility
        register_output("/gimbal/shooter/mode", shoot_mode_, rmcs_msgs::ShootMode::SINGLE);
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

        if (putter_initialized) {
            if (bullet_feeder_cool_down_ > 0) {
                bullet_feeder_cool_down_--;

                if (bullet_feeder_cool_down_ > 500)
                    *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                        -bullet_feeder_angle_per_bullet_ - *bullet_feeder_velocity_);
                else {
                    bullet_feeder_velocity_pid_.reset();
                    *bullet_feeder_control_torque_ = 0.0;
                }

                bullet_feeder_angle_pid_.reset();

                if (!bullet_feeder_cool_down_)
                    RCLCPP_INFO(get_logger(), "Jamming Solved!");
            } else {
                if (*friction_ready_)
                    if (switch_right != Switch::DOWN) {
                        if ((!last_mouse_.left && mouse.left)
                            || (last_switch_left_ == rmcs_msgs::Switch::MIDDLE
                                && switch_left == rmcs_msgs::Switch::DOWN)) {
                            //*control_bullet_allowance_limited_by_heat_ > 0&&
                            if (shoot_stage_ == ShootStage::PRELOADED)
                                set_shooting();
                        }
                    }

                if (*friction_ready_) {
                    if (shoot_stage_ == ShootStage::PRELOADING) {
                        if (last_preload_flag_) {
                            const auto angle_err =
                                bullet_feeder_control_angle_ - *bullet_feeder_angle_;
                            if (angle_err < 0.1) {
                                set_preloaded();
                            }
                            double velocity_err =
                                bullet_feeder_angle_pid_.update(
                                    bullet_feeder_control_angle_ - *bullet_feeder_angle_)
                                - *bullet_feeder_velocity_;
                            *bullet_feeder_control_torque_ =
                                bullet_feeder_velocity_pid_.update(velocity_err);
                        } else {
                            if (*photoelectric_sensor_status_) {
                                last_preload_flag_ = true;
                                bullet_feeder_control_angle_ =
                                    *bullet_feeder_angle_ + bullet_feeder_angle_per_bullet_ * 1.2;
                            } else
                                *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                                    low_latency_velocity_ - *bullet_feeder_velocity_);
                        }
                        update_jam_detection();
                    } else {
                        double velocity_err =
                            bullet_feeder_angle_pid_.update(
                                bullet_feeder_control_angle_ - *bullet_feeder_angle_)
                            - *bullet_feeder_velocity_;
                        *bullet_feeder_control_torque_ =
                            bullet_feeder_velocity_pid_.update(velocity_err);
                    }

                    if (shoot_stage_ == ShootStage::PRELOADED) {}
                    if (shoot_stage_ == ShootStage::SHOOTING) {
                        if (*bullet_fired_
                            || *putter_angle_ - putter_startpoint >= putter_stroke_) {
                            shooted = true;
                        }

                        if (shooted) {
                            const auto angle_err = putter_startpoint - *putter_angle_;
                            if (angle_err > -0.05) {
                                *putter_control_torque_ = 0.;
                                set_preloading();
                                shooted = false;
                            }

                            *putter_control_torque_ =
                                puttter_return_velocity_pid_.update(-80. - *putter_velocity_);
                        } else {
                            *putter_control_torque_ =
                                puttter_return_velocity_pid_.update(60. - *putter_velocity_);
                        }
                    }
                }

                if (shoot_stage_ != ShootStage::SHOOTING)
                    *putter_control_torque_ = -0.02;
            }
        } else {
            *putter_control_torque_ = puttter_return_velocity_pid_.update(-80. - *putter_velocity_);
            update_putter_jam_detection();
        }
        // 3.387030 -9.177040
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

        overdrive_mode_ = false;

        bullet_feeder_control_angle_ = nan_;
        bullet_feeder_angle_pid_.output_max = inf_;

        bullet_feeder_velocity_pid_.reset();
        bullet_feeder_angle_pid_.reset();
        *bullet_feeder_control_torque_ = nan_;

        putter_initialized = false;
        putter_startpoint = nan_;
        puttter_return_velocity_pid_.reset();

        putter_velocity_pid_.reset();

        putter_return_angle_pid.reset();
        *putter_control_torque_ = nan_;

        last_preload_flag_ = false;
        last_photoelectric_sensor_status_ = false;

        bullet_feeder_faulty_count_ = 0;
        bullet_feeder_cool_down_ = 0;
    }

    void set_preloading() {
        RCLCPP_INFO(get_logger(), "PRELOADING");
        shoot_stage_ = ShootStage::PRELOADING;
    }

    void set_preloaded() {
        RCLCPP_INFO(get_logger(), "PRELOADED");
        shoot_stage_ = ShootStage::PRELOADED;
        last_preload_flag_ = false;
        bullet_feeder_control_angle_ = *bullet_feeder_angle_ - 0.3;
    }

    void set_shooting() {
        RCLCPP_INFO(get_logger(), "SHOOTING");
        shoot_stage_ = ShootStage::SHOOTING;
    }

    void update_jam_detection() {
        // RCLCPP_INFO(get_logger(), "%.2f --", *bullet_feeder_control_torque_);
        if (*bullet_feeder_control_torque_ < 300.0 || std::isnan(*bullet_feeder_control_torque_)) {
            bullet_feeder_faulty_count_ = 0;
            return;
        }

        if (bullet_feeder_faulty_count_ < 1000)
            bullet_feeder_faulty_count_++;
        else {
            bullet_feeder_faulty_count_ = 0;
            if (last_preload_flag_)
                set_preloaded();
            else
                enter_jam_protection();
        }
    }

    void update_putter_jam_detection() {
        if (*putter_control_torque_ > -0.03 || std::isnan(*putter_control_torque_)) {
            putter_faulty_count_ = 0;
            return;
        }

        if (putter_faulty_count_ < 500)
            ++putter_faulty_count_;
        else {
            putter_faulty_count_ = 0;
            putter_initialized = true;
            putter_startpoint = *putter_angle_;
        }
    }

    void enter_jam_protection() {
        bullet_feeder_control_angle_ = nan_;
        bullet_feeder_cool_down_ = 1000;
        bullet_feeder_angle_pid_.reset();
        bullet_feeder_velocity_pid_.reset();
        RCLCPP_INFO(get_logger(), "Jammed!");
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double low_latency_velocity_ = 2.5;

    static constexpr double putter_stroke_ = 11.5;

    static constexpr double max_bullet_feeder_control_torque_ = 0.1;
    static constexpr double bullet_feeder_angle_per_bullet_ = 2 * std::numbers::pi / 6;

    // static constexpr double qian=0.6;
    // static constexpr double hou=-3.8;

    InputInterface<bool> photoelectric_sensor_status_;
    bool last_photoelectric_sensor_status_;
    InputInterface<bool> bullet_fired_;
    bool shooted{false};

    InputInterface<bool> friction_ready_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    bool overdrive_mode_ = false;

    InputInterface<double> bullet_feeder_angle_;
    InputInterface<double> bullet_feeder_velocity_;

    InputInterface<int64_t> control_bullet_allowance_limited_by_heat_;

    bool last_preload_flag_ = false;

    bool putter_initialized = false;
    int putter_faulty_count_ = 0;
    double putter_startpoint = nan_;
    pid::PidCalculator puttter_return_velocity_pid_;
    InputInterface<double> putter_velocity_;

    pid::PidCalculator putter_velocity_pid_;

    enum class ShootStage { PRELOADING, PRELOADED, SHOOTING };
    ShootStage shoot_stage_ = ShootStage::PRELOADING;
    double bullet_feeder_control_angle_ = nan_;

    pid::PidCalculator bullet_feeder_velocity_pid_;
    pid::PidCalculator bullet_feeder_angle_pid_;
    OutputInterface<double> bullet_feeder_control_torque_;

    InputInterface<double> putter_angle_;
    pid::PidCalculator putter_return_angle_pid;
    OutputInterface<double> putter_control_torque_;

    int bullet_feeder_faulty_count_ = 0;
    int bullet_feeder_cool_down_ = 0;

    OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::PutterController, rmcs_executor::Component)