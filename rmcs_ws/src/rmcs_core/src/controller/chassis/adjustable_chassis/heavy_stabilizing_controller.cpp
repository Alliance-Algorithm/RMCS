#include "controller/pid/pid_calculator.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace rmcs_core::controller::chassis {

/// Gravity compensation + IMU self-leveling for the four master_lift motors of the
/// square adjustable chassis. Structured like controller/arm/arm_solver.cpp: every
/// term returns a TorqueVec and the terms named in `controller_list` are summed.
///
/// Leg order: 0 LF, 1 LB, 2 RB, 3 RF. Body frame: x forward, y left, z up.
/// Corners at (±a, ±a), a = half_side. Lifts are linear: h_i = dir * r * theta_i.
///
/// Modal coordinates (B: 4x4, entries ±1, B^T B = 4I), columns:
///   heave [+,+,+,+], pitch [+,-,-,+] (front +), roll [+,+,-,-] (left +), warp [+,-,+,-]
///   q = B^T h / 4,   leg force f = B Q / 4,   leg torque tau_i = dir * r * f_i
class HeavyStabilizingController final
    : public rmcs_executor::Component
    , public rclcpp::Node {

    static constexpr std::size_t num_leg = 4;
    using TorqueVec = Eigen::Array<double, num_leg, 1>;
    using ModalVec = Eigen::Vector4d;

    enum Mode : std::size_t { kHeave = 0, kPitch = 1, kRoll = 2, kWarp = 3 };

public:
    HeavyStabilizingController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , mass_(get_parameter("mass").as_double())
        , com_x_(get_parameter("com_x").as_double())
        , com_y_(get_parameter("com_y").as_double())
        , com_z0_(get_parameter("com_z0").as_double())
        , half_side_(get_parameter("half_side").as_double())
        , lift_lead_(get_parameter("lift_lead").as_double())
        , lift_direction_(get_parameter("lift_direction").as_double() >= 0.0 ? 1.0 : -1.0)
        , height_min_(get_parameter("height_min").as_double())
        , height_max_(get_parameter("height_max").as_double())
        , max_level_offset_(get_parameter("max_level_offset").as_double())
        , roll_sign_(get_parameter("roll_sign").as_double() >= 0.0 ? 1.0 : -1.0)
        , pitch_sign_(get_parameter("pitch_sign").as_double() >= 0.0 ? 1.0 : -1.0)
        , friction_load_coefficient_(get_parameter("friction_load_coefficient").as_double())
        , friction_speed_threshold_(get_parameter("friction_speed_threshold").as_double())
        , modal_pos_pid_{
              pid::make_pid_calculator(*this, "heave_pos_"),
              pid::make_pid_calculator(*this, "pitch_pos_"),
              pid::make_pid_calculator(*this, "roll_pos_"),
              pid::make_pid_calculator(*this, "warp_pos_"),
          }
        , modal_vel_pid_{
              pid::make_pid_calculator(*this, "heave_vel_"),
              pid::make_pid_calculator(*this, "pitch_vel_"),
              pid::make_pid_calculator(*this, "roll_vel_"),
              pid::make_pid_calculator(*this, "warp_vel_"),
          } {
        // clang-format off
        modal_basis_ << 1,  1,  1,  1,
                        1, -1,  1, -1,
                        1, -1, -1,  1,
                        1,  1, -1, -1;
        // clang-format on

        const auto friction = get_parameter("friction_torque").as_double_array();
        if (friction.size() != num_leg)
            throw std::runtime_error("Parameter 'friction_torque' must contain 4 values");
        for (std::size_t i = 0; i < num_leg; ++i)
            friction_torque_(static_cast<Eigen::Index>(i)) = friction[i];
        if (!(friction_speed_threshold_ > 0.0))
            throw std::runtime_error("Parameter 'friction_speed_threshold' must be positive");

        if (height_max_ - height_min_ < 4.0 * max_level_offset_)
            RCLCPP_WARN(
                get_logger(),
                "Lift travel (%.3f m) is smaller than 4 * max_level_offset (%.3f m); "
                "the heave target will be pinned to the middle of travel.",
                height_max_ - height_min_, 4.0 * max_level_offset_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/chassis/roll_imu_angle", roll_imu_angle_);
        register_input("/chassis/pitch_imu_angle", pitch_imu_angle_);

        register_input("/chassis/lift/target_height", target_height_, false);
        register_input("/chassis/lift/target_roll", target_roll_, false);
        register_input("/chassis/lift/target_pitch", target_pitch_, false);

        static const std::array<std::string, num_leg> master_lift_prefix{
            "/chassis/lift/master_left_front",
            "/chassis/lift/master_left_back",
            "/chassis/lift/master_right_back",
            "/chassis/lift/master_right_front",
        };
        for (std::size_t i = 0; i < num_leg; ++i) {
            register_input(master_lift_prefix[i] + "/angle", master_lift_angle_[i]);
            register_input(master_lift_prefix[i] + "/target_angle", master_lift_target_angle_[i]);
            register_input(master_lift_prefix[i] + "/velocity", master_lift_velocity_[i]);
            register_output(
                master_lift_prefix[i] + "/control_torque", master_lift_control_torque_[i], NAN);
        }

        // Slave lifts are out of scope for now: always unpowered.
        static const std::array<std::string, 2> slave_lift_prefix{
            "/chassis/lift/slave_left", "/chassis/lift/slave_right"};
        for (std::size_t i = 0; i < slave_lift_prefix.size(); ++i) {
            register_input(slave_lift_prefix[i] + "/angle", slave_lift_angle_[i]);
            register_input(slave_lift_prefix[i] + "/target_angle", slave_lift_target_angle_[i]);
            register_input(slave_lift_prefix[i] + "/velocity", slave_lift_velocity_[i]);
            register_output(
                slave_lift_prefix[i] + "/control_torque", slave_lift_control_torque_[i], NAN);
        }

        load_controller_list(get_parameter("controller_list").as_string_array());
    }

    void before_updating() override {
        if (!target_height_.ready()) {
            use_latched_height_ = true;
            target_height_.make_and_bind_directly(NAN);
        }
        if (!target_roll_.ready())
            target_roll_.make_and_bind_directly(0.0);
        if (!target_pitch_.ready())
            target_pitch_.make_and_bind_directly(0.0);
    }

    void update() override {
        using rmcs_msgs::Switch;
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;

        update_state();

        bool enable = false;
        if (!initial_check_done_) {
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN)
                initial_check_done_ = true;
        } else {
            enable =
                !((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                  || (switch_left == Switch::DOWN && switch_right == Switch::DOWN));
        }

        TorqueVec tau_cmd;
        if (enable) {
            if (!last_enable_) {
                for (auto& pid : modal_pos_pid_)
                    pid.reset();
                for (auto& pid : modal_vel_pid_)
                    pid.reset();
                latched_height_ = q_(kHeave);
            }
            tau_cmd.setZero();
            for (auto fn : controller_list_)
                tau_cmd += (this->*fn)();
        } else {
            tau_cmd = calculate_zero_torque();
            target_modal_velocity_.setZero();
        }
        last_enable_ = enable;

        for (std::size_t i = 0; i < num_leg; ++i)
            *master_lift_control_torque_[i] = tau_cmd(static_cast<Eigen::Index>(i));
        for (auto& torque : slave_lift_control_torque_)
            *torque = NAN;
    }

private:
    using controller_type = TorqueVec (HeavyStabilizingController::*)();

    // Refresh leg heights/velocities, modal state and (sign-corrected) IMU attitude.
    void update_state() {
        ModalVec h, v;
        for (std::size_t i = 0; i < num_leg; ++i) {
            const auto idx = static_cast<Eigen::Index>(i);
            h(idx) = lift_direction_ * lift_lead_ * *master_lift_angle_[i];
            v(idx) = lift_direction_ * lift_lead_ * *master_lift_velocity_[i];
        }
        q_ = modal_basis_.transpose() * h / 4.0;
        dq_ = modal_basis_.transpose() * v / 4.0;

        roll_ = std::isfinite(*roll_imu_angle_) ? roll_sign_ * *roll_imu_angle_ : 0.0;
        pitch_ = std::isfinite(*pitch_imu_angle_) ? pitch_sign_ * *pitch_imu_angle_ : 0.0;
    }

    // Modal generalized force Q [N] -> output-shaft torque per leg [N*m].
    TorqueVec modal_force_to_torque(const ModalVec& modal_force) const {
        const ModalVec leg_force = modal_basis_ * modal_force / 4.0;
        return (lift_direction_ * lift_lead_ * leg_force).array();
    }

    TorqueVec calculate_gravity_compensation() {
        return modal_force_to_torque(gravity_modal_force());
    }

    // Modal force that statically supports the body weight; lateral components are
    // taken by the wheels. Shared by the gravity and friction terms.
    ModalVec gravity_modal_force() const {
        static constexpr double g = 9.81;
        const double mg = mass_ * g;

        // Gravity in body frame (ZYX Euler).
        const double w_x = mg * std::sin(pitch_);
        const double w_y = -mg * std::cos(pitch_) * std::sin(roll_);
        const double w_z = -mg * std::cos(pitch_) * std::cos(roll_);

        // CoM measured from the wheel-contact plane, rises with heave.
        const double c_x = com_x_;
        const double c_y = com_y_;
        const double c_z = com_z0_ + q_(kHeave);

        const double force = -w_z;                     // sum f_i
        const double moment_x = c_z * w_y - c_y * w_z; // sum y_i f_i
        const double moment_y = c_z * w_x - c_x * w_z; // sum x_i f_i

        ModalVec modal_force;
        modal_force << force, moment_y / half_side_, moment_x / half_side_, 0.0;
        return modal_force;
    }

    TorqueVec calculate_pid() {
        ModalVec target = q_;

        // Heave: external target if provided, otherwise the height latched on enable.
        const double heave_lower = height_min_ + 2.0 * max_level_offset_;
        const double heave_upper = height_max_ - 2.0 * max_level_offset_;
        const double heave_target = (use_latched_height_ || !std::isfinite(*target_height_))
                                      ? latched_height_
                                      : *target_height_;
        target(kHeave) = heave_lower <= heave_upper
                           ? std::clamp(heave_target, heave_lower, heave_upper)
                           : 0.5 * (height_min_ + height_max_);

        // Pitch / roll: turn the IMU attitude error into a modal height correction.
        const double target_pitch = std::isfinite(*target_pitch_) ? *target_pitch_ : 0.0;
        const double target_roll = std::isfinite(*target_roll_) ? *target_roll_ : 0.0;
        const double delta_pitch = half_side_ * std::tan(pitch_ - target_pitch);
        const double delta_roll = -half_side_ * std::tan(roll_ - target_roll);
        target(kPitch) =
            std::clamp(q_(kPitch) + delta_pitch, -max_level_offset_, max_level_offset_);
        target(kRoll) = std::clamp(q_(kRoll) + delta_roll, -max_level_offset_, max_level_offset_);

        // Warp: compliant, keeps all four wheels loaded on uneven ground.
        target(kWarp) = 0.0;

        ModalVec modal_force;
        for (std::size_t m = 0; m < num_leg; ++m) {
            const auto idx = static_cast<Eigen::Index>(m);
            const double target_vel = modal_pos_pid_[m].update(target(idx) - q_(idx));
            modal_force(idx) = modal_vel_pid_[m].update(target_vel - dq_(idx));
            target_modal_velocity_(idx) = std::isfinite(target_vel) ? target_vel : 0.0;
        }
        return modal_force_to_torque(modal_force);
    }

    TorqueVec calculate_friction_compensation() {
        // Load-scaled Coulomb friction, smoothed with tanh:
        //   tau_f,i = dir * (tau_c0,i + mu * |tau_g,i|) * tanh(v_i / v0)
        // tau_g,i is the leg's gravity-support torque, v_i the leg extension speed [m/s].
        // Near standstill the measured speed is blended with the pid term's commanded speed
        // so the compensation also helps the leg break away; once moving, measured wins.
        const ModalVec leg_load = modal_basis_ * gravity_modal_force() / 4.0;      // [N]
        const ModalVec commanded_velocity = modal_basis_ * target_modal_velocity_; // [m/s]
        const double v0 = friction_speed_threshold_;

        TorqueVec tau_f;
        for (std::size_t i = 0; i < num_leg; ++i) {
            const auto idx = static_cast<Eigen::Index>(i);
            const double measured = lift_direction_ * lift_lead_ * *master_lift_velocity_[i];
            const double weight = std::min(std::abs(measured) / v0, 1.0);
            const double velocity = weight * measured + (1.0 - weight) * commanded_velocity(idx);

            const double magnitude =
                friction_torque_(idx)
                + friction_load_coefficient_ * lift_lead_ * std::abs(leg_load(idx));
            tau_f(idx) = lift_direction_ * magnitude * std::tanh(velocity / v0);
        }
        return tau_f;
    }

    TorqueVec calculate_zero_torque() {
        TorqueVec tau;
        tau.fill(NAN);
        return tau;
    }

    static constexpr std::array<std::tuple<std::string_view, controller_type>, 4> term_table_{
        {{"gravity", &HeavyStabilizingController::calculate_gravity_compensation},
         {"pid", &HeavyStabilizingController::calculate_pid},
         {"friction", &HeavyStabilizingController::calculate_friction_compensation},
         {"zero_torque", &HeavyStabilizingController::calculate_zero_torque}}};

    void load_controller_list(const std::vector<std::string>& list) {
        controller_list_.clear();
        controller_list_.reserve(list.size());
        // record's string_views point into list's elements; list must outlive record
        std::unordered_set<std::string_view> record;
        record.reserve(list.size());

        for (const auto& l : list) {
            const std::string_view sv{l};
            if (!record.insert(sv).second)
                continue;

            bool matched = false;
            for (const auto& [name, fn] : term_table_) {
                if (sv == name) {
                    controller_list_.push_back(fn);
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Unknown controller_list entry: '%s' (allowed: gravity, pid, friction, "
                    "zero_torque)",
                    l.c_str());
            }
        }

        // The friction term reads the velocity commanded by the pid term in the same tick,
        // so always evaluate it last (the sum itself is order-independent).
        std::stable_partition(
            controller_list_.begin(), controller_list_.end(), [](controller_type fn) {
                return fn != &HeavyStabilizingController::calculate_friction_compensation;
            });
    }

    // Parameters
    const double mass_, com_x_, com_y_, com_z0_;
    const double half_side_, lift_lead_, lift_direction_;
    const double height_min_, height_max_, max_level_offset_;
    const double roll_sign_, pitch_sign_;
    const double friction_load_coefficient_, friction_speed_threshold_;
    TorqueVec friction_torque_;

    Eigen::Matrix4d modal_basis_;
    std::vector<controller_type> controller_list_;
    std::array<pid::PidCalculator, num_leg> modal_pos_pid_;
    std::array<pid::PidCalculator, num_leg> modal_vel_pid_;

    // State
    ModalVec q_ = ModalVec::Zero();
    ModalVec dq_ = ModalVec::Zero();
    ModalVec target_modal_velocity_ = ModalVec::Zero(); // set by the pid term [m/s]
    double roll_ = 0.0, pitch_ = 0.0;
    double latched_height_ = 0.0;
    bool use_latched_height_ = false;
    bool initial_check_done_ = false;
    bool last_enable_ = false;

    // Interfaces
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    InputInterface<double> roll_imu_angle_;
    InputInterface<double> pitch_imu_angle_;

    InputInterface<double> target_height_;
    InputInterface<double> target_roll_;
    InputInterface<double> target_pitch_;

    std::array<InputInterface<double>, num_leg> master_lift_angle_;
    std::array<InputInterface<double>, num_leg> master_lift_target_angle_;
    std::array<InputInterface<double>, num_leg> master_lift_velocity_;
    std::array<OutputInterface<double>, num_leg> master_lift_control_torque_;

    std::array<InputInterface<double>, 2> slave_lift_angle_;
    std::array<InputInterface<double>, 2> slave_lift_target_angle_;
    std::array<InputInterface<double>, 2> slave_lift_velocity_;
    std::array<OutputInterface<double>, 2> slave_lift_control_torque_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::HeavyStabilizingController, rmcs_executor::Component)
