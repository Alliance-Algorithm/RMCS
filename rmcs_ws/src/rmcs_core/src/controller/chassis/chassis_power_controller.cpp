#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::controller::chassis {

using namespace referee::app;

class ChassisPowerController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisPowerController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/chassis/control_mode", mode_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);

        register_input("/chassis/power", chassis_power_);
        register_input("/chassis/supercap/voltage", supercap_voltage_);
        register_input("/chassis/supercap/enabled", supercap_enabled_);

        register_input("/referee/chassis/power_limit", chassis_power_limit_referee_);
        register_input("/referee/chassis/buffer_energy", chassis_buffer_energy_referee_);

        register_output("/chassis/supercap/charge_power_limit", supercap_charge_power_limit_, 0.0);
        register_output("/chassis/control_power_limit", chassis_control_power_limit_, 0.0);

        register_output(
            "/chassis/supercap/voltage/control_line", supercap_voltage_control_line_, 12.5);
        register_output("/chassis/supercap/voltage/base_line", supercap_voltage_base_line_, 12.0);
        register_output("/chassis/supercap/voltage/dead_line", supercap_voltage_dead_line_, 11.0);
    }

    void update() override {
        update_charging_power_limit();

        update_ui();

        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;
        auto rotary_knob  = *rotary_knob_;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_power_control();
            return;
        }

        update_virtual_buffer_energy();

        boost_mode_ = keyboard.shift || rotary_knob < -0.9;
        update_control_power_limit();
    }

private:
    void update_charging_power_limit() {
        // Maximum excess power when buffer energy is sufficient.
        constexpr double excess_power_limit = 35;

        //                     charging_power_limit =
        constexpr double buffer_energy_control_line = 120; // = referee + excess
        constexpr double buffer_energy_base_line    = 30;  // = referee
        constexpr double buffer_energy_dead_line    = 0;   // = 0

        *supercap_charge_power_limit_ =
            *chassis_power_limit_referee_
                * std::clamp(
                    (*chassis_buffer_energy_referee_ - buffer_energy_dead_line)
                        / (buffer_energy_base_line - buffer_energy_dead_line),
                    0.0, 1.0)
            + excess_power_limit
                  * std::clamp(
                      (*chassis_buffer_energy_referee_ - buffer_energy_base_line)
                          / (buffer_energy_control_line - buffer_energy_base_line),
                      0.0, 1.0);
    }

    void reset_power_control() {
        virtual_buffer_energy_        = virtual_buffer_energy_limit_;
        boost_mode_                   = false;
        *chassis_control_power_limit_ = 0.0;
    }

    void update_virtual_buffer_energy() {
        constexpr double dt = 1e-3;
        virtual_buffer_energy_ += dt * (chassis_power_limit_expected_ - *chassis_power_);
        virtual_buffer_energy_ = std::clamp(
            virtual_buffer_energy_, 0.0,
            std::min(*chassis_buffer_energy_referee_, virtual_buffer_energy_limit_));
    }

    void update_control_power_limit() {
        double power_limit;

        if (boost_mode_ && *supercap_enabled_)
            power_limit = *mode_ == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                            ? inf_
                            : *chassis_power_limit_referee_ + 80.0;
        else
            power_limit = *chassis_power_limit_referee_;
        chassis_power_limit_expected_ = power_limit;

        //                 chassis_control_power_limit =
        constexpr double supercap_voltage_control_line = 12.5; // = supercap
        constexpr double supercap_voltage_base_line    = 12.0; // = referee
        power_limit                                    = *chassis_power_limit_referee_
                    + (power_limit - *chassis_power_limit_referee_)
                          * std::clamp(
                              (*supercap_voltage_ - supercap_voltage_base_line)
                                  / (supercap_voltage_control_line - supercap_voltage_base_line),
                              0.0, 1.0);

        // Maximum excess power when virtual buffer energy is full.
        constexpr double excess_power_limit = 15;

        power_limit += excess_power_limit;
        power_limit *= virtual_buffer_energy_ / virtual_buffer_energy_limit_;

        *chassis_control_power_limit_ = power_limit;
    }

    void update_ui() {
        chassis_power_ui_.set_value(static_cast<int32_t>(std::round(*chassis_power_)));
        chassis_control_power_limit_ui_.set_value(
            static_cast<int32_t>(std::round(*chassis_control_power_limit_)));
    }

    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::ChassisMode> mode_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;

    InputInterface<double> chassis_power_;
    static constexpr double virtual_buffer_energy_limit_ = 30.0;
    double virtual_buffer_energy_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_enabled_;

    InputInterface<double> chassis_power_limit_referee_;
    InputInterface<double> chassis_buffer_energy_referee_;

    bool boost_mode_ = false;
    OutputInterface<double> supercap_charge_power_limit_;
    double chassis_power_limit_expected_;
    OutputInterface<double> chassis_control_power_limit_;

    OutputInterface<double> supercap_voltage_control_line_;
    OutputInterface<double> supercap_voltage_base_line_;
    OutputInterface<double> supercap_voltage_dead_line_;

    ui::Integer chassis_power_ui_{ui::Shape::Color::WHITE, 15, 2, ui::x_center, 100, 0};
    ui::Integer chassis_control_power_limit_ui_{
        ui::Shape::Color::WHITE, 15, 2, ui::x_center, 150, 0};
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisPowerController, rmcs_executor::Component)