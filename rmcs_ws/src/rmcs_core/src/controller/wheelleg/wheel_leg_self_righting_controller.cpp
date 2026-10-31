#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "identification/wheel_leg_arm_dwell.hpp"
#include "recovery_geometry_v202.hpp"

namespace rmcs_core::controller::wheelleg {

// Acyclic path: measured sensors -> recovery -> trajectory gate / sole drive
// bridge. The exact same SelfRightingCore is called by the native Isaac runner.
class WheelLegSelfRightingController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegSelfRightingController()
        : Node{get_component_name(), node::options()} {
        enabled_ = node::param_or("enabled", false);
        simulation_only_ = node::param_or("simulation_only", true);
        const bool mapping_verified = node::param_or("hardware_mapping_verified", false);
        if (enabled_ && !simulation_only_ && !mapping_verified)
            throw std::invalid_argument("Self-righting hardware axis mapping has not been verified");
        handoff_enabled_ = node::param_or("handoff_enabled", false);
        motor_velocity_servo_ = node::param_or("motor_velocity_servo", true);
        dt_ = node::param_or("dt", .001);
        arm_dwell_ = node::param_or("arm_dwell_seconds", 1.);
        auto signs = node::param_or("cad_axis_sign", std::vector<double>{1., 1., 1., 1.});
        auto zeros = node::param_or("cad_axis_offset_rad", std::vector<double>{0., 0., 0., 0.});
        if (signs.size()!=4 || zeros.size()!=4 || !std::isfinite(dt_) || dt_<=0. || dt_>.01
            || !std::isfinite(arm_dwell_) || arm_dwell_<0.)
            throw std::invalid_argument("Invalid self-righting timing or CAD calibration");
        for (std::size_t i=0; i<4; ++i) {
            if ((signs[i]!=1. && signs[i]!=-1.) || !std::isfinite(zeros[i]))
                throw std::invalid_argument("Invalid self-righting CAD axis calibration");
            signs_[i]=signs[i]; offsets_[i]=zeros[i];
        }
        SelfRightingCore::Settings settings;
        settings.cruise_speed = node::param_or("cruise_speed_rad_s", 1.5);
        settings.speed_limit = node::param_or("speed_limit_rad_s", 3.14);
        settings.acceleration = node::param_or("acceleration_rad_s2", 3.);
        settings.velocity_gain = node::param_or("velocity_gain", 5.);
        velocity_gain_=settings.velocity_gain;
        if (motor_velocity_servo_ && (velocity_gain_<=0. || velocity_gain_>5.))
            throw std::invalid_argument("DM velocity servo requires 0 < Kd <= 5");
        settings.position_gain = node::param_or("position_gain", 12.);
        settings.integral_gain = node::param_or("integral_gain", 50.);
        settings.integral_limit = node::param_or("integral_limit_nm", 30.);
        settings.torque_limit = node::param_or("torque_limit_nm", 40.);
        settings.timeout_seconds = node::param_or("timeout_seconds", 30.);
        settings.fold_knee = node::param_or("fold_knee_deg", 75.);
        settings.flip_knee = node::param_or("flip_knee_deg", 105.);
        settings.flip_orbit = node::param_or("flip_orbit_deg", 0.);
        settings.sweep_orbit = node::param_or("sweep_orbit_deg", 180.);
        settings.flip_asymmetry = node::param_or("flip_asymmetry_knee_deg", 0.);
        settings.stand_knee = recovery_v202::stand_knee;
        settings.stand_orbit = recovery_v202::stand_orbit;
        settings.pitch_trim_gain = node::param_or("pitch_trim_gain", 0.);
        settings.yaw_rate_gain = node::param_or("yaw_rate_gain", .6);
        settings.balance_kp = node::param_or("balance_kp", 35.);
        settings.balance_kd = node::param_or("balance_kd", 5.);
        settings.balance_kv = node::param_or("balance_kv", 2.);
        settings.support_gain = node::param_or("support_gain", 1.);
        core_ = std::make_unique<SelfRightingCore>(recovery_v202::geometry, settings, recovery_v202::support_compensation);

        constexpr std::array joints{"left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (std::size_t i=0; i<4; ++i) {
            const std::string name=joints[i];
            register_input("/wheel_leg/"+name+"/angle", position_[i], false);
            register_input("/wheel_leg/"+name+"/velocity", velocity_[i], false);
            register_output("/wheelleg/recovery/"+name+"/torque", torque_[i], 0.);
            register_output("/wheelleg/recovery/"+name+"/reference_rad", reference_[i], 0.);
            register_output("/wheelleg/recovery/"+name+"/reference_rad_s", reference_velocity_[i], 0.);
            register_output("/wheelleg/recovery/"+name+"/servo_rad_s", servo_velocity_[i], 0.);
            register_output("/wheelleg/recovery/"+name+"/servo_feedforward_nm", servo_feedforward_[i], 0.);
        }
        register_output("/wheelleg/recovery/motor_velocity_servo", velocity_servo_out_, false);
        register_output("/wheelleg/recovery/velocity_gain", velocity_gain_out_, velocity_gain_);
        constexpr std::array axes{"x", "y", "z"};
        constexpr std::array rates{"roll_rate", "pitch_velocity", "yaw_rate"};
        for (std::size_t i=0; i<3; ++i) {
            register_input(std::string{"/wheelleg/state/gravity_"}+axes[i], gravity_[i], false);
            register_input(std::string{"/wheelleg/state/"}+rates[i], gyro_[i], false);
        }
        constexpr std::array wheels{"left_wheel", "right_wheel"};
        for (std::size_t i=0; i<2; ++i) {
            register_input(std::string{"/wheel_leg/"}+wheels[i]+"/velocity", wheel_velocity_[i], false);
            register_output(std::string{"/wheelleg/recovery/"}+wheels[i]+"/torque", wheel_torque_[i], 0.);
        }
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_, false);
        register_input("/wheel_leg/dm_control_ready", dm_ready_, false);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/wheelleg/recovery/request", request_, false);
        register_output("/wheelleg/recovery/active", active_, false);
        register_output("/wheelleg/recovery/ready", ready_, false);
        register_output("/wheelleg/recovery/phase", phase_, std::uint8_t{0});
        register_output("/wheelleg/recovery/fault", fault_, std::uint8_t{0});
    }

    void before_updating() override { reset_(); }

    void update() override {
        using Switch=rmcs_msgs::Switch;
        bool wheels_allowed=simulation_only_;
        if (switch_left_.ready() && switch_right_.ready()) {
            const bool fresh=remote_fresh_.ready() && *remote_fresh_;
            const auto left=*switch_left_, right=*switch_right_;
            if (!fresh) { reset_(); return; }
            if (left==Switch::DOWN && right==Switch::DOWN) {
                armed_=released_=had_ready_=false; core_->reset(); clear_outputs_();
                arm_.update(Arm::Clock::now(), fresh, left, right, arm_dwell_);
                return;
            }
            if (arm_.update(Arm::Clock::now(), fresh, left, right, arm_dwell_)) armed_=true;
            wheels_allowed=left==Switch::MIDDLE && right==Switch::UP;
        } else {
            // Simulation needs an explicit request producer. Never auto-arm a
            // hardware graph merely because its DR16 interfaces are absent.
            armed_=simulation_only_ && request_.ready() && *request_;
        }
        if (!enabled_ || !armed_) { clear_outputs_(); core_->reset(); had_ready_=false; return; }
        if (released_) { *active_=false; *ready_=true; return; }
        *active_=true;
        if (*fault_!=0) { clear_torques_(); return; }
        // Maintain active while waiting for DM readiness so the bridge performs
        // its enable handshake without falling back to ordinary references.
        if (!dm_ready_.ready() || !*dm_ready_) {
            clear_torques_(); core_->reset();
            if (had_ready_) { *fault_=9; *phase_=255; *ready_=false; }
            return;
        }
        had_ready_=true;
        SelfRightingCore::Feedback f;
        bool fresh=feedback_fresh_.ready() && *feedback_fresh_;
        for (std::size_t i=0; i<4; ++i) {
            fresh=fresh && position_[i].ready() && velocity_[i].ready();
            if (position_[i].ready()) f.position[i]=signs_[i]**position_[i]+offsets_[i];
            if (velocity_[i].ready()) f.velocity[i]=signs_[i]**velocity_[i];
        }
        for (std::size_t i=0; i<3; ++i) {
            fresh=fresh && gravity_[i].ready() && gyro_[i].ready();
            if (gravity_[i].ready()) f.gravity[i]=*gravity_[i];
            if (gyro_[i].ready()) f.gyro[i]=*gyro_[i];
        }
        for (std::size_t i=0; i<2; ++i) {
            fresh=fresh && wheel_velocity_[i].ready();
            if (wheel_velocity_[i].ready()) f.wheel_velocity[i]=*wheel_velocity_[i];
        }
        f.spring_compensation=recovery_v202::compensation(f.position);
        const auto out=core_->step(f,dt_,true,fresh);
        *phase_=static_cast<std::uint8_t>(out.phase);
        *fault_=static_cast<std::uint8_t>(out.fault);
        *ready_=out.phase==SelfRightingCore::Complete;
        *velocity_servo_out_=motor_velocity_servo_ && out.fault==0;
        for (std::size_t i=0; i<4; ++i) {
            *torque_[i]=signs_[i]*out.torque[i];
            *reference_[i]=signs_[i]*(out.position[i]-offsets_[i]);
            *reference_velocity_[i]=signs_[i]*out.velocity[i];
            *servo_velocity_[i]=signs_[i]*out.servo_velocity[i];
            *servo_feedforward_[i]=signs_[i]*out.servo_feedforward[i];
        }
        for (std::size_t i=0; i<2; ++i) *wheel_torque_[i]=wheels_allowed ? out.wheel_torque[i] : 0.;
        if (*ready_ && handoff_enabled_) { released_=true; *active_=false; clear_torques_(); }
    }

private:
    using Arm=rmcs_core::controller::identification::WheelLegArmDwell;
    void clear_torques_() {
        for (auto& value:torque_) *value=0.;
        for (auto& value:wheel_torque_) *value=0.;
        for (auto& value:servo_velocity_) *value=0.;
        for (auto& value:servo_feedforward_) *value=0.;
        *velocity_servo_out_=false;
    }
    void clear_outputs_() {
        clear_torques_(); *active_=false; *ready_=false; *phase_=0; *fault_=0;
        for (auto& value:reference_) *value=0.;
        for (auto& value:reference_velocity_) *value=0.;
    }
    void reset_() { arm_.reset(); armed_=released_=had_ready_=false; core_->reset(); clear_outputs_(); }
    std::array<InputInterface<double>,4> position_,velocity_;
    std::array<InputInterface<double>,3> gravity_,gyro_;
    std::array<InputInterface<double>,2> wheel_velocity_;
    InputInterface<bool> feedback_fresh_,dm_ready_,remote_fresh_,request_;
    InputInterface<rmcs_msgs::Switch> switch_left_,switch_right_;
    std::array<OutputInterface<double>,4> torque_,reference_,reference_velocity_,servo_velocity_,servo_feedforward_;
    std::array<OutputInterface<double>,2> wheel_torque_;
    OutputInterface<bool> active_,ready_,velocity_servo_out_;
    OutputInterface<double> velocity_gain_out_;
    OutputInterface<std::uint8_t> phase_,fault_;
    std::array<double,4> signs_{},offsets_{};
    std::unique_ptr<SelfRightingCore> core_;
    Arm arm_;
    double dt_=0.,arm_dwell_=0.,velocity_gain_=0.;
    bool enabled_=false,simulation_only_=true,handoff_enabled_=false,motor_velocity_servo_=true,
         armed_=false,released_=false,had_ready_=false;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::wheelleg::WheelLegSelfRightingController, rmcs_executor::Component)
