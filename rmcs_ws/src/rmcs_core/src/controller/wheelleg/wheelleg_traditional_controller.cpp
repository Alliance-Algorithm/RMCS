#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/wheelleg/referee_power_gate.hpp"
#include "controller/wheelleg/traditional_hardware_guard.hpp"
#include "controller/wheelleg/wheelleg_gas_spring_v202.hpp"
#include "identification/wheel_leg_arm_dwell.hpp"

extern "C" {
void* wheelleg_traditional_create(const double*, int, const double*, int, const double*, int);
void wheelleg_traditional_destroy(void*);
void wheelleg_traditional_reset(void*);
int wheelleg_traditional_step(
    void*, std::int64_t, std::int64_t, unsigned, const double*, int, const double*, int, double*,
    int);
}

namespace rmcs_core::controller::wheelleg {

// Hardware host of the exact C ABI loaded by Isaac Sim. CAD/API calibration is
// an explicit admission gate; a newly generated research YAML cannot arm a robot.
class WheelLegTraditionalController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegTraditionalController()
        : Node{get_component_name(), node::options()}
        , core_(nullptr, wheelleg_traditional_destroy) {
        if (get_parameter("abi_version").as_int() != 1)
            throw std::invalid_argument("Unsupported traditional control ABI");
        const auto parameters = get_parameter("parameters").as_double_array();
        const auto rows = get_parameter("posture_rows").as_double_array();
        const auto curve = get_parameter("spring_force_rows").as_double_array();
        if (parameters.size() != 28 || rows.size() < 54 || rows.size() % 27 || curve.size() < 4
            || curve.size() % 2)
            throw std::invalid_argument("Invalid traditional control table dimensions");
        for (const auto* values : {&parameters, &rows, &curve})
            for (double value : *values)
                if (!std::isfinite(value))
                    throw std::invalid_argument("Nonfinite traditional control table");
        core_.reset(wheelleg_traditional_create(
            parameters.data(), parameters.size(), rows.data(), rows.size() / 27, curve.data(),
            curve.size() / 2));
        if (!core_)
            throw std::invalid_argument("Invalid traditional control parameters");
        const auto model_hash = get_parameter("model_spec_sha256").as_string();
        const auto posture_hash = get_parameter("posture_sha256").as_string();
        const auto gas_hash = get_parameter("gas_spring_model_sha256").as_string();
        if (model_hash != rmcs_core::wheelleg::gas_spring_v202::model_spec_sha256
            || gas_hash != rmcs_core::wheelleg::gas_spring_v202::model_sha256
            || posture_hash.size() != 64)
            throw std::invalid_argument("Traditional model/posture/gas-spring SHA256 mismatch");
        info("Traditional model={} postures={} gas-spring={}", model_hash, posture_hash, gas_hash);
        mapping_verified_ = get_parameter_or("hardware_mapping_verified", false);
        commissioning_ = get_parameter_or("hardware_commissioning_enabled", false);
        simulation_only_ = get_parameter_or("simulation_only", true);
        const auto calibration = get_parameter_or("hardware_calibration_id", std::string{});
        if ((mapping_verified_ || commissioning_) && calibration.empty())
            throw std::invalid_argument("Hardware mapping requires a calibration receipt ID");
        signs_ =
            get_parameter_or("hardware_to_cad_sign", std::vector<double>{1., 1., 1., 1., 1., 1.});
        zeros_ = get_parameter_or("hardware_zero_rad", std::vector<double>(6, 0.0));
        if (signs_.size() != 6 || zeros_.size() != 6)
            throw std::invalid_argument("Six explicit calibration axes required");
        for (std::size_t i = 0; i < 6; ++i)
            if (std::abs(signs_[i]) != 1.0 || !std::isfinite(zeros_[i]))
                throw std::invalid_argument("Invalid CAD/API calibration");
        length_min_ = rows[0];
        length_max_ = rows[rows.size() - 27];
        length_nominal_ = parameters[17];
        leg_limit_ = parameters[3];
        wheel_limit_ = parameters[2];
        posture_rows_ = rows;
        const auto lower = get_parameter_or(
            "mechanical_relative_min_rad",
            std::vector<double>{
                std::min(rows[2] - rows[1], rows[rows.size() - 25] - rows[rows.size() - 26]),
                std::min(rows[4] - rows[3], rows[rows.size() - 23] - rows[rows.size() - 24])});
        const auto upper = get_parameter_or(
            "mechanical_relative_max_rad",
            std::vector<double>{
                std::max(rows[2] - rows[1], rows[rows.size() - 25] - rows[rows.size() - 26]),
                std::max(rows[4] - rows[3], rows[rows.size() - 23] - rows[rows.size() - 24])});
        relative_tolerance_ = get_parameter_or("feedback_relative_limit_tolerance_rad", .01);
        if (lower.size() != 2 || upper.size() != 2 || !std::isfinite(relative_tolerance_)
            || relative_tolerance_ < 0.0 || relative_tolerance_ > .03)
            throw std::invalid_argument(
                "Invalid mechanical relative-angle limit dimensions/tolerance");
        for (std::size_t side = 0; side < 2; ++side) {
            relative_min_[side] = lower[side];
            relative_max_[side] = upper[side];
            if (!std::isfinite(lower[side]) || !std::isfinite(upper[side])
                || lower[side] >= upper[side])
                throw std::invalid_argument("Invalid mechanical relative-angle interval");
            for (std::size_t row = 0; row < rows.size(); row += 27) {
                const auto hip = row + 1 + side * 2;
                const double relative = rows[hip + 1] - rows[hip];
                if (relative < lower[side] || relative > upper[side])
                    throw std::invalid_argument(
                        "Command posture lies outside mechanical relative-angle stops");
            }
        }
        maximum_tilt_ = parameters[14];
        idle_power_ = parameters[18];
        maximum_gap_s_ = get_parameter_or("maximum_control_gap_s", .0025);
        enable_timeout_s_ = get_parameter_or("drive_enable_timeout_s", 1.0);
        startup_tilt_ = get_parameter_or("startup_maximum_tilt_rad", .15);
        startup_joint_error_ = get_parameter_or("startup_maximum_joint_error_rad", .35);
        velocity_max_ = get_parameter_or("command_velocity_max_m_s", .5);
        yaw_max_ = get_parameter_or("command_yaw_rate_max_rad_s", 1.0);
        arm_dwell_ = get_parameter_or("arm_dwell_seconds", .5);
        require_recovery_ready_ = get_parameter_or("require_recovery_ready", false);
        allow_motion_ = get_parameter_or("allow_motion_commands", true);
        for (double value :
             {maximum_gap_s_, enable_timeout_s_, startup_tilt_, startup_joint_error_, velocity_max_,
              yaw_max_, arm_dwell_})
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("Invalid hardware commissioning limit");
        if (maximum_gap_s_ < .001 || maximum_gap_s_ > .02 || startup_tilt_ > maximum_tilt_
            || velocity_max_ > .5 || yaw_max_ > 1.0)
            throw std::invalid_argument("Hardware commissioning limits exceed reviewed envelope");
        power_budget_ = get_parameter("power_budget_w").as_double();
        require_referee_ = get_parameter_or("require_referee_power_limit", true);
        if (!std::isfinite(power_budget_) || power_budget_ < 0.0)
            throw std::invalid_argument("Invalid power budget");
        for (std::size_t i = 0; i < 6; ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + axes_[i];
            register_input(prefix + "/angle", position_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/feedback_steady_ns", feedback_time_[i]);
            register_input(prefix + "/max_torque", max_torque_[i]);
            register_output(prefix + "/control_torque", torque_[i], 0.0);
            register_output(
                std::string{"/wheelleg/traditional/cad_position/"} + axes_[i], cad_position_[i],
                0.0);
            const std::string recovery = std::string{"/wheelleg/recovery/"} + axes_[i];
            register_input(recovery + "/torque", recovery_torque_[i], false);
        }
        register_input("/wheelleg/recovery/active", recovery_active_, false);
        register_input("/wheelleg/recovery/ready", recovery_ready_, false);
        register_input("/wheelleg/recovery/fault", recovery_fault_, false);
        register_input("/wheel_leg/imu/quaternion", quaternion_);
        register_input("/wheel_leg/imu/angular_velocity", angular_velocity_);
        register_input("/wheel_leg/imu/acceleration", acceleration_);
        register_input("/wheel_leg/imu/last_steady_ns", imu_time_);
        register_input("/wheel_leg/imu/acceleration_steady_ns", acceleration_time_);
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
        register_input("/wheel_leg/dm_control_ready", dm_ready_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/predefined/timestamp", scheduled_time_);
        register_input("/predefined/update_rate", update_rate_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/joystick/right", joystick_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/rotary_knob", knob_, false);
        register_input("/wheelleg/move/velocity", move_velocity_, false);
        register_input("/wheelleg/move/yaw_rate", move_yaw_, false);
        register_input("/wheelleg/move/leg_length", move_length_, false);
        register_input("/referee/chassis/power_limit", referee_limit_, false);
        register_input("/referee/chassis/power_limit_steady_ns", referee_time_, false);
        register_input("/referee/chassis/output_status", referee_output_, false);
        register_output("/wheel_leg/enable_request", enable_, false);
        register_output("/wheel_leg/clear_error_request", clear_error_, false);
        constexpr std::array auxiliary_names{
            "/gimbal/yaw", "/gimbal/pitch", "/gimbal/left_friction",
            "/gimbal/right_friction", "/gimbal/bullet_feeder"};
        for (std::size_t i = 0; i < auxiliary_names.size(); ++i)
            register_output(std::string{auxiliary_names[i]} + "/control_torque", auxiliary_torque_[i], 0.);
        register_output("/wheelleg/traditional/failure", failure_, 0.0);
        register_output(
            "/wheelleg/traditional/mapping_admitted", admitted_,
            (mapping_verified_ || commissioning_) && !simulation_only_);
        register_output("/wheelleg/state/velocity", estimated_velocity_, 0.0);
        register_output("/wheelleg/reference/leg_length", reference_length_, length_nominal_);
        register_output("/wheelleg/power/torque_scale", power_scale_, 0.0);
        register_output("/wheelleg/power/predicted_command", predicted_power_, 0.0);
        register_output("/wheelleg/safety/support_state", support_, 0.0);
        register_output("/wheelleg/traditional/armed", armed_output_, 0.0);
        register_output("/wheelleg/traditional/fault_latched", fault_output_, 0.0);
        register_output("/wheelleg/traditional/period_ms", period_output_, 0.0);
        register_output("/wheelleg/traditional/steady_time_s", time_output_, 0.0);
        register_output("/wheelleg/traditional/feedback_age_ms", age_output_, 0.0);
        register_output("/wheelleg/traditional/drive_ready", ready_output_, 0.0);
        register_output("/wheelleg/traditional/enable_requested", enable_output_, 0.0);
        register_output("/wheelleg/traditional/pitch", pitch_output_, 0.0);
        register_output("/wheelleg/traditional/roll", roll_output_, 0.0);
        register_output("/wheelleg/traditional/command_velocity", velocity_command_, 0.0);
        register_output("/wheelleg/traditional/command_yaw_rate", yaw_command_, 0.0);
        register_output(
            "/wheelleg/traditional/command_leg_length", length_command_, length_nominal_);
        register_output("/wheelleg/traditional/relative_angle_left", relative_left_, 0.0);
        register_output("/wheelleg/traditional/relative_angle_right", relative_right_, 0.0);
        info(
            "Traditional hardware admission={} calibration={} max_gap={}ms",
            (mapping_verified_ || commissioning_) && !simulation_only_, calibration,
            maximum_gap_s_ * 1000.0);
        parameter_callback_ =
            add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = false;
                result.reason =
                    "Traditional calibration, gains and limits require YAML export and restart";
                return result;
            });
    }

    void before_updating() override {
        arm_ = {};
        guard_ = {};
        tick_ns_ = 0;
        previous_schedule_ns_ = 0;
        recovery_was_active_ = false;
        wheelleg_traditional_reset(core_.get());
        safe_outputs_();
        *clear_error_ = false;
    }

    void update() override {
        if (*update_rate_ != 1000.0)
            throw std::invalid_argument("Traditional hardware control requires a 1000 Hz executor");
        using Switch = rmcs_msgs::Switch;
        safe_outputs_();
        *clear_error_ = false;
        const bool remote_fresh = *remote_fresh_;
        const auto left = *switch_left_, right = *switch_right_;
        const bool down = remote_fresh && left == Switch::DOWN && right == Switch::DOWN;
        *clear_error_ = down;
        const auto arm_now = Arm::Clock::now();
        const bool arm_event = arm_.update(arm_now, remote_fresh, left, right, arm_dwell_);
        const bool stand =
            left == Switch::MIDDLE && (right == Switch::MIDDLE || right == Switch::UP);
        const bool wheels = stand && right == Switch::UP;
        const bool recovering = recovery_active_.ready() && *recovery_active_;
        const bool recovery_arm = recovering && !recovery_was_active_;
        recovery_was_active_ = recovering;
        const bool control_mode = stand
                               && (!require_recovery_ready_ || recovering
                                   || (recovery_ready_.ready() && *recovery_ready_));
        const auto now =
            std::chrono::duration_cast<std::chrono::nanoseconds>(arm_now.time_since_epoch())
                .count();
        const auto scheduled_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      scheduled_time_->time_since_epoch())
                                      .count();
        const bool schedule_valid =
            previous_schedule_ns_ == 0 || scheduled_ns - previous_schedule_ns_ == 1000000;
        previous_schedule_ns_ = scheduled_ns;
        std::uint64_t oldest = static_cast<std::uint64_t>(now);
        bool finite = true;
        std::array<double, 17> observation{};
        for (std::size_t i = 0; i < 6; ++i) {
            finite = finite && std::isfinite(*position_[i]) && std::isfinite(*velocity_[i])
                  && std::isfinite(*max_torque_[i]) && *max_torque_[i] > 0.
                  && *feedback_time_[i] > 0
                  && *feedback_time_[i] <= static_cast<std::uint64_t>(now);
            oldest = std::min(oldest, *feedback_time_[i]);
            observation[i] = signs_[i] * (*position_[i] - zeros_[i]);
            observation[i + 6] = signs_[i] * *velocity_[i];
            *cad_position_[i] = observation[i];
        }
        const auto* quaternion = &*quaternion_;
        const bool quaternion_valid =
            quaternion->coeffs().allFinite() && quaternion->norm() > .9 && quaternion->norm() < 1.1;
        finite = finite && quaternion_valid && acceleration_->allFinite()
              && angular_velocity_->allFinite();
        double roll = 0.0;
        if (quaternion_valid) {
            const auto rotation = quaternion->normalized().toRotationMatrix();
            observation[12] = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
            observation[14] = std::atan2(rotation(1, 0), rotation(0, 0));
            roll = std::atan2(rotation(2, 1), rotation(2, 2));
        }
        observation[13] = (*angular_velocity_)[1];
        observation[15] = (*angular_velocity_)[2];
        observation[16] = acceleration_->norm();
        finite = finite && *acceleration_time_ > 0
              && *acceleration_time_ <= static_cast<std::uint64_t>(now);
        oldest = std::min(oldest, *acceleration_time_);
        if (imu_time_.ready()) {
            finite = finite && *imu_time_ > 0 && *imu_time_ <= static_cast<std::uint64_t>(now);
            oldest = std::min(oldest, *imu_time_);
        } else {
            finite = false;
        }
        const bool fresh =
            finite && *feedback_fresh_ && static_cast<std::uint64_t>(now) - oldest <= 20000000;
        const bool referee_available =
            referee_limit_.ready() && referee_time_.ready() && referee_output_.ready();
        const double budget = rmcs_core::wheelleg::referee_power_budget(
            power_budget_, require_referee_, referee_available,
            referee_available ? *referee_limit_ : 0.0, referee_available && *referee_output_,
            referee_available ? *referee_time_ : 0, static_cast<std::uint64_t>(now));
        std::array<double, 4> command{0., 0., length_nominal_, budget};
        const auto channel = [](double x) {
            return std::isfinite(x) && std::abs(x) > .08 ? std::clamp(x, -1.0, 1.0) : 0.0;
        };
        if (wheels && allow_motion_) {
            command[0] = move_velocity_.ready()  ? *move_velocity_
                       : joystick_right_.ready() ? velocity_max_ * channel(joystick_right_->x())
                                                 : 0.0;
            command[1] = move_yaw_.ready()      ? *move_yaw_
                       : joystick_left_.ready() ? yaw_max_ * channel(joystick_left_->y())
                                                : 0.0;
        }
        if (allow_motion_ && move_length_.ready())
            command[2] = *move_length_;
        else if (allow_motion_ && knob_.ready() && std::isfinite(*knob_)) {
            const double knob = channel(*knob_);
            command[2] = length_nominal_
                       + knob
                             * (knob >= 0.0 ? length_max_ - length_nominal_
                                            : length_nominal_ - length_min_);
        }
        for (double value : command)
            finite = finite && std::isfinite(value);
        command[0] = std::clamp(command[0], -velocity_max_, velocity_max_);
        command[1] = std::clamp(command[1], -yaw_max_, yaw_max_);
        command[2] = std::clamp(command[2], length_min_, length_max_);
        std::array<double, 6> cad_position{};
        std::copy_n(observation.begin(), 6, cad_position.begin());
        const bool geometry_valid = rmcs_core::wheelleg::traditional_relative_angles_valid(
            cad_position, relative_min_, relative_max_, relative_tolerance_);
        const bool attitude_valid =
            finite
            && (recovering
                || (std::abs(observation[12]) <= maximum_tilt_ && std::abs(roll) <= maximum_tilt_));
        const bool startup_valid = recovering || (recovery_ready_.ready() && *recovery_ready_)
                                || (attitude_valid && std::abs(observation[12]) <= startup_tilt_
                                    && std::abs(roll) <= startup_tilt_
                                    && rmcs_core::wheelleg::traditional_start_pose_valid(
                                        posture_rows_, cad_position, startup_joint_error_));
        const auto gate = guard_.step(
            static_cast<std::uint64_t>(now),
            {.reset = down,
             .arm = arm_event || recovery_arm,
             .remote_fresh = remote_fresh,
             .control_mode = control_mode,
             .admitted = (mapping_verified_ || commissioning_) && !simulation_only_,
             .feedback_fresh = fresh && finite,
             .geometry_valid = geometry_valid,
             .attitude_valid = attitude_valid,
             .power_valid = budget > 0.0 && budget >= idle_power_,
             .startup_pose_valid = startup_valid,
             .drives_ready = *dm_ready_,
             .schedule_valid = schedule_valid},
            static_cast<std::uint64_t>(maximum_gap_s_ * 1e9),
            static_cast<std::uint64_t>(enable_timeout_s_ * 1e9));
        *enable_ = gate.enable;
        *failure_ = gate.failure;
        *period_output_ = guard_.period_ns() * 1e-6;
        *time_output_ = now * 1e-9;
        *age_output_ = (static_cast<std::uint64_t>(now) - oldest) * 1e-6;
        *pitch_output_ = observation[12];
        *roll_output_ = roll;
        *ready_output_ = *dm_ready_;
        *velocity_command_ = command[0];
        *yaw_command_ = command[1];
        *length_command_ = command[2];
        *relative_left_ = cad_position[1] - cad_position[0];
        *relative_right_ = cad_position[3] - cad_position[2];
        if (recovering && (!recovery_fault_.ready() || *recovery_fault_ != 0)) {
            guard_.latch(19);
            *enable_ = false;
            *failure_ = 19;
            publish_guard_();
            return;
        }
        if (!gate.compute) {
            wheelleg_traditional_reset(core_.get());
            tick_ns_ = 0;
            publish_guard_();
            return;
        }
        if (recovering) {
            wheelleg_traditional_reset(core_.get());
            tick_ns_ = 0;
            if (!publish_recovery_(wheels)) {
                safe_outputs_();
                guard_.latch(19);
                *failure_ = 19;
            }
            publish_guard_();
            return;
        }
        if (tick_ns_ == 0)
            tick_ns_ = now;
        std::array<double, 16> result{};
        const unsigned flags = 3u | (wheels ? 8u : 0u);
        const auto age = static_cast<std::int64_t>(static_cast<std::uint64_t>(now) - oldest);
        const int code = wheelleg_traditional_step(
            core_.get(), tick_ns_, tick_ns_ - age, flags, observation.data(), 17, command.data(), 4,
            result.data(), 16);
        tick_ns_ += 1000000;
        *failure_ = code;
        if (code == 0) {
            for (std::size_t i = 0; i < 6; ++i)
                *torque_[i] = std::clamp(signs_[i] * result[i], -*max_torque_[i], *max_torque_[i]);
            *reference_length_ = result[6];
            *estimated_velocity_ = result[9];
            *power_scale_ = result[12];
            *predicted_power_ = result[13];
            *support_ = result[14];
        } else {
            *enable_ = false;
            guard_.latch(code);
        }
        publish_guard_();
    }

private:
    std::array<OutputInterface<double>, 5> auxiliary_torque_;
    bool publish_recovery_(bool wheels) {
        std::array<double, 6> commands{};
        for (std::size_t i = 0; i < 6; ++i) {
            if (!recovery_torque_[i].ready() || !std::isfinite(*recovery_torque_[i]))
                return false;
            const double limit = std::min(*max_torque_[i], i < 4 ? leg_limit_ : wheel_limit_);
            commands[i] = i < 4 || wheels ? std::clamp(*recovery_torque_[i], -limit, limit) : 0.;
        }
        for (std::size_t i = 0; i < 6; ++i)
            *torque_[i] = commands[i];
        return true;
    }

    void publish_guard_() {
        *armed_output_ = guard_.armed();
        *fault_output_ = guard_.faulted();
        *enable_output_ = *enable_;
        const int code = static_cast<int>(*failure_);
        if (code != last_failure_) {
            info(
                "Traditional status={} armed={} enable={} fault={} period={}ms feedback_age={}ms",
                code, guard_.armed(), *enable_, guard_.faulted(), *period_output_, *age_output_);
            last_failure_ = code;
        }
    }
    void safe_outputs_() {
        for (auto& torque : torque_)
            *torque = 0.0;
        *enable_ = false;
        *power_scale_ = *predicted_power_ = *support_ = *estimated_velocity_ = 0.0;
        *reference_length_ = length_nominal_;
    }
    using Arm = rmcs_core::controller::identification::WheelLegArmDwell;
    static constexpr std::array axes_{"left_hip_joint",   "left_knee_joint", "right_hip_joint",
                                      "right_knee_joint", "left_wheel",      "right_wheel"};
    std::array<InputInterface<double>, 6> position_, velocity_;
    std::array<InputInterface<double>, 6> max_torque_, recovery_torque_;
    InputInterface<bool> recovery_active_, recovery_ready_;
    InputInterface<std::uint8_t> recovery_fault_;
    std::array<InputInterface<std::uint64_t>, 6> feedback_time_;
    std::array<OutputInterface<double>, 6> torque_;
    InputInterface<Eigen::Quaterniond> quaternion_;
    InputInterface<Eigen::Vector3d> acceleration_, angular_velocity_;
    InputInterface<Eigen::Vector2d> joystick_left_, joystick_right_;
    InputInterface<std::uint64_t> imu_time_, acceleration_time_, referee_time_;
    InputInterface<std::chrono::steady_clock::time_point> scheduled_time_;
    InputInterface<double> update_rate_;
    InputInterface<bool> feedback_fresh_, dm_ready_, remote_fresh_, referee_output_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<double> knob_, move_velocity_, move_yaw_, move_length_, referee_limit_;
    OutputInterface<bool> enable_, clear_error_, admitted_;
    std::array<OutputInterface<double>, 6> cad_position_;
    OutputInterface<double> armed_output_, fault_output_, period_output_, time_output_, age_output_;
    OutputInterface<double> ready_output_, enable_output_, pitch_output_, roll_output_;
    OutputInterface<double> velocity_command_, yaw_command_, length_command_;
    OutputInterface<double> relative_left_, relative_right_;
    OutputInterface<double> failure_, estimated_velocity_, reference_length_, power_scale_,
        predicted_power_, support_;
    std::unique_ptr<void, void (*)(void*)> core_;
    bool mapping_verified_ = false, simulation_only_ = true, require_referee_ = true;
    bool commissioning_ = false, require_recovery_ready_ = false, allow_motion_ = true;
    bool recovery_was_active_ = false;
    double leg_limit_ = 40., wheel_limit_ = 4.5, arm_dwell_ = .5;
    double length_min_ = 0., length_max_ = 0., length_nominal_ = 0., power_budget_ = 0.;
    std::int64_t tick_ns_ = 0, previous_schedule_ns_ = 0;
    double maximum_tilt_ = 0., idle_power_ = 0., maximum_gap_s_ = 0., enable_timeout_s_ = 0.;
    double startup_tilt_ = 0., startup_joint_error_ = 0., velocity_max_ = 0., yaw_max_ = 0.;
    std::array<double, 2> relative_min_{}, relative_max_{};
    double relative_tolerance_ = 0.;
    int last_failure_ = -1;
    std::vector<double> signs_, zeros_, posture_rows_;
    Arm arm_;
    rmcs_core::wheelleg::TraditionalHardwareGuard guard_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegTraditionalController, rmcs_executor::Component)
