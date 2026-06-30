#include <cmath>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "rmcs_msgs/chassis_leveling_phase.hpp"

namespace rmcs_core::controller::dart {

class DartChassisLeveling
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartChassisLeveling()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , pitch_angle_pid_(
              get_parameter("pitch_angle_kp").as_double(),
              get_parameter("pitch_angle_ki").as_double(),
              get_parameter("pitch_angle_kd").as_double())
        , roll_angle_pid_(
              get_parameter("roll_angle_kp").as_double(),
              get_parameter("roll_angle_ki").as_double(),
              get_parameter("roll_angle_kd").as_double())
        , contact_stage_angle_increment_(get_parameter("contact_stage_angle_increment").as_double())
        , roll_pair_stage_angle_increment_(
              get_parameter("roll_pair_stage_angle_increment").as_double())
        , pitch_pair_stage_angle_increment_(
              get_parameter("pitch_pair_stage_angle_increment").as_double()) {
        register_input("/imu/catapult_pitch_angle", pitch_angle_);
        register_input("/imu/catapult_roll_angle", roll_angle_);
        register_input("/dart_manager/chassis_leveling/phase", chassis_leveling_phase_, false);
        register_input(
            "/dart_manager/chassis_leveling/front_left/target_velocity",
            front_left_target_velocity_, false);
        register_input(
            "/dart_manager/chassis_leveling/front_right/target_velocity",
            front_right_target_velocity_, false);
        register_input(
            "/dart_manager/chassis_leveling/rear_left/target_velocity",
            rear_left_target_velocity_, false);
        register_input(
            "/dart_manager/chassis_leveling/rear_right/target_velocity",
            rear_right_target_velocity_, false);

        // register_input("/dart/leveling_feet/front_left/encoder_angle",
        // front_left_encoder_angle_);
        // register_input("/dart/leveling_feet/front_right/encoder_angle",
        // front_right_encoder_angle_);
        // register_input("/dart/leveling_feet/rear_left/encoder_angle", rear_left_encoder_angle_);
        // register_input("/dart/leveling_feet/rear_right/encoder_angle",
        // rear_right_encoder_angle_); register_input("/predefined/update_rate", update_rate_,
        // false);

        register_output(
            "/dart/leveling_feet/front_left/control_velocity", front_left_control_velocity_, 0.0);
        register_output(
            "/dart/leveling_feet/front_right/control_velocity", front_right_control_velocity_, 0.0);
        register_output(
            "/dart/leveling_feet/rear_left/control_velocity", rear_left_control_velocity_, 0.0);
        register_output(
            "/dart/leveling_feet/rear_right/control_velocity", rear_right_control_velocity_, 0.0);

        configure_pid_limits("pitch_angle", pitch_angle_pid_);
        configure_pid_limits("roll_angle", roll_angle_pid_);
    }

    void update() override {
        set_hold_velocity_targets();

        const auto phase = active_phase();
        switch (phase) {
        // case rmcs_msgs::ChassisLevelingPhase::LEGACY_ROLL:
        //     pitch_angle_pid_.reset();
        //     apply_legacy_roll();
        //     break;
        // case rmcs_msgs::ChassisLevelingPhase::LEGACY_PITCH:
        //     roll_angle_pid_.reset();
        //     apply_legacy_pitch();
        //     break;
        // case rmcs_msgs::ChassisLevelingPhase::STAGE_CONTACT:
        //     reset_pid_controllers();
        //     apply_stage_contact();
        //     break;
        // case rmcs_msgs::ChassisLevelingPhase::STAGE_ROLL:
        //     reset_pid_controllers();
        //     apply_stage_roll_pair();
        //     break;
        // case rmcs_msgs::ChassisLevelingPhase::STAGE_PITCH:
        //     reset_pid_controllers();
        //     apply_stage_pitch_pair();
        //     break;
        case rmcs_msgs::ChassisLevelingPhase::IDLE:
        default: reset_pid_controllers(); break;
        }

        if (phase == rmcs_msgs::ChassisLevelingPhase::IDLE) {
            apply_manual_target_velocity();
        }
    }

private:
    void configure_pid_limits(const std::string& prefix, pid::PidCalculator& pid) {
        const auto parameter_name = [&prefix](const char* suffix) { return prefix + suffix; };

        (void)get_parameter(parameter_name("_integral_min"), pid.integral_min);
        (void)get_parameter(parameter_name("_integral_max"), pid.integral_max);
        (void)get_parameter(parameter_name("_integral_split_min"), pid.integral_split_min);
        (void)get_parameter(parameter_name("_integral_split_max"), pid.integral_split_max);
        (void)get_parameter(parameter_name("_output_min"), pid.output_min);
        (void)get_parameter(parameter_name("_output_max"), pid.output_max);
    }

    void set_hold_velocity_targets() {
        *front_left_control_velocity_ = 0.0;
        *front_right_control_velocity_ = 0.0;
        *rear_left_control_velocity_ = 0.0;
        *rear_right_control_velocity_ = 0.0;
    }

    void reset_pid_controllers() {
        pitch_angle_pid_.reset();
        roll_angle_pid_.reset();
    }

    rmcs_msgs::ChassisLevelingPhase active_phase() const {
        if (!chassis_leveling_phase_.ready()) {
            return rmcs_msgs::ChassisLevelingPhase::IDLE;
        }
        return *chassis_leveling_phase_;
    }

    void apply_manual_target_velocity() {
        *front_left_control_velocity_ = manual_target_velocity_or_zero(front_left_target_velocity_);
        *front_right_control_velocity_ =
            manual_target_velocity_or_zero(front_right_target_velocity_);
        *rear_left_control_velocity_ = manual_target_velocity_or_zero(rear_left_target_velocity_);
        *rear_right_control_velocity_ =
            manual_target_velocity_or_zero(rear_right_target_velocity_);
    }

    static double manual_target_velocity_or_zero(const rmcs_executor::Component::InputInterface<double>& velocity_input) {
        if (!velocity_input.ready() || !std::isfinite(*velocity_input)) {
            return 0.0;
        }
        return *velocity_input;
    }

    // void apply_legacy_roll() {
    //     const double roll_increment = update_axis_increment(roll_angle_, roll_angle_pid_);
    //     *front_left_control_velocity_ += roll_increment;
    //     *rear_left_control_velocity_ += roll_increment;
    // }

    // void apply_legacy_pitch() {
    //     const double pitch_increment = update_axis_increment(pitch_angle_, pitch_angle_pid_);
    //     *front_left_control_velocity_ += pitch_increment;
    //     *front_right_control_velocity_ += pitch_increment;
    // }

    // void apply_stage_contact() {
    //     *front_left_control_velocity_ += contact_stage_angle_increment_;
    //     *front_right_control_velocity_ += contact_stage_angle_increment_;
    //     *rear_left_control_velocity_ += contact_stage_angle_increment_;
    //     *rear_right_control_velocity_ += contact_stage_angle_increment_;
    // }

    // void apply_stage_roll_pair() {
    //     if (!roll_angle_.ready() || !std::isfinite(*roll_angle_)) {
    //         return;
    //     }

    //     if (*roll_angle_ < 0.0) {
    //         *front_left_control_velocity_ += roll_pair_stage_angle_increment_;
    //         *rear_left_control_velocity_ += roll_pair_stage_angle_increment_;
    //     } else if (*roll_angle_ > 0.0) {
    //         *front_right_control_velocity_ += roll_pair_stage_angle_increment_;
    //         *rear_right_control_velocity_ += roll_pair_stage_angle_increment_;
    //     }
    // }

    // void apply_stage_pitch_pair() {
    //     if (!pitch_angle_.ready() || !std::isfinite(*pitch_angle_)) {
    //         return;
    //     }

    //     if (*pitch_angle_ < 0.0) {
    //         *front_left_control_velocity_ += pitch_pair_stage_angle_increment_;
    //         *front_right_control_velocity_ += pitch_pair_stage_angle_increment_;
    //     } else if (*pitch_angle_ > 0.0) {
    //         *rear_left_control_velocity_ += pitch_pair_stage_angle_increment_;
    //         *rear_right_control_velocity_ += pitch_pair_stage_angle_increment_;
    //     }
    // }

    // static double update_axis_increment(
    //     const rmcs_executor::Component::InputInterface<double>& angle_interface, pid::PidCalculator& pid) {
    //     if (!angle_interface.ready() || !std::isfinite(*angle_interface)) {
    //         pid.reset();
    //         return 0.0;
    //     }

    //     const double increment = pid.update(-*angle_interface);
    //     if (!std::isfinite(increment)) {
    //         pid.reset();
    //         return 0.0;
    //     }

    //     return increment;
    // }

    pid::PidCalculator pitch_angle_pid_;
    pid::PidCalculator roll_angle_pid_;
    double contact_stage_angle_increment_;
    double roll_pair_stage_angle_increment_;
    double pitch_pair_stage_angle_increment_;

    rmcs_executor::Component::InputInterface<double> pitch_angle_;
    rmcs_executor::Component::InputInterface<double> roll_angle_;
    rmcs_executor::Component::InputInterface<rmcs_msgs::ChassisLevelingPhase> chassis_leveling_phase_;
    rmcs_executor::Component::InputInterface<double> front_left_target_velocity_;
    rmcs_executor::Component::InputInterface<double> front_right_target_velocity_;
    rmcs_executor::Component::InputInterface<double> rear_left_target_velocity_;
    rmcs_executor::Component::InputInterface<double> rear_right_target_velocity_;

    rmcs_executor::Component::InputInterface<double> front_left_encoder_angle_;
    rmcs_executor::Component::InputInterface<double> front_right_encoder_angle_;
    rmcs_executor::Component::InputInterface<double> rear_left_encoder_angle_;
    rmcs_executor::Component::InputInterface<double> rear_right_encoder_angle_;
    rmcs_executor::Component::InputInterface<double> update_rate_;

    rmcs_executor::Component::OutputInterface<double> front_left_control_velocity_;
    rmcs_executor::Component::OutputInterface<double> front_right_control_velocity_;
    rmcs_executor::Component::OutputInterface<double> rear_left_control_velocity_;
    rmcs_executor::Component::OutputInterface<double> rear_right_control_velocity_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartChassisLeveling, rmcs_executor::Component)
