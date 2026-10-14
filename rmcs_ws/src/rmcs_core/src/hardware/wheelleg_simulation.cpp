#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "controller/wheelleg/leg_transmission.hpp"

namespace rmcs_core::hardware {

// In-process MuJoCo simulation bridge for the wheel-leg chassis.
//
// It replaces the physical hardware component (see flight.cpp) with a MuJoCo
// model stepped inside the executor loop. The reduced serial wheel-leg
// controller (rmcs_core::controller::wheelleg::WheelLegTrajectoryController)
// consumes the /wheelleg/state/* interfaces and produces /wheelleg/reference/*,
// exactly like it would on real hardware.
//
// Like Flight, the component is split in two so the executor's dependency
// graph has no cycle:
//   * WheelLegSimulation        -> emits /wheelleg/state/*, steps the physics
//   * WheelLegSimulationCommand -> consumes /wheelleg/reference/*, emits
//                                  /wheelleg/command/*
//
// NOTE: the mapping between the reduced state used by the controller
// (shank/beta/pitch) and the raw joint space of the MJCF still needs
// calibration; every channel has a `state_*_sign` / `state_*_offset` parameter
// for that purpose (see the TODO in publish_state_()).

class WheelLegSimulation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegSimulation()
        : Node{get_component_name(), node::options()} {
        load_parameters_();
        load_model_();
        setup_leg_linkages_();

        command_component_ = create_partner_component<WheelLegSimulationCommand>(
            get_component_name() + "_command", *this);

        register_output("/wheelleg/state/wheel_velocity", state_wheel_velocity_, 0.0);
        register_output("/wheelleg/state/shank_angle", state_shank_angle_, 0.0);
        register_output("/wheelleg/state/shank_velocity", state_shank_velocity_, 0.0);
        register_output("/wheelleg/state/knee_angle", state_knee_angle_, 0.0);
        register_output("/wheelleg/state/knee_velocity", state_knee_velocity_, 0.0);
        register_output("/wheelleg/state/pitch", state_pitch_, 0.0);
        register_output("/wheelleg/state/pitch_velocity", state_pitch_velocity_, 0.0);
        // Measured reduced hip (== raw L_joint1 in the V5.0 convention) and the
        // base height, used by the self-righting recovery controller.
        register_output("/wheelleg/state/hip_angle", state_hip_angle_, 0.0);
        register_output("/wheelleg/state/base_height", state_base_height_, 0.0);
        // Task-space state (see 执行路线2 §5): leg length / leg angle and the
        // vehicle CoM relative to the wheel axle (static-stability offset d).
        register_output("/wheelleg/state/leg_length", state_leg_length_, 0.0);
        register_output("/wheelleg/state/leg_angle", state_leg_angle_, 0.0);
        register_output("/wheelleg/state/leg_length_l", state_leg_length_l_, 0.0);
        register_output("/wheelleg/state/leg_length_r", state_leg_length_r_, 0.0);
        register_output("/wheelleg/state/leg_angle_l", state_leg_angle_l_, 0.0);
        register_output("/wheelleg/state/leg_angle_r", state_leg_angle_r_, 0.0);
        register_output("/wheelleg/state/com_offset", state_com_offset_, 0.0);
        register_output("/wheelleg/state/com_x", state_com_x_, 0.0);
        register_output("/wheelleg/state/com_z", state_com_z_, 0.0);
        // Yaw (heading) and yaw rate of the base, for the differential-steering
        // (turning) loop. yaw is about the world up axis; yaw_rate is the world
        // z angular velocity of base_link.
        register_output("/wheelleg/state/yaw", state_yaw_, 0.0);
        register_output("/wheelleg/state/yaw_rate", state_yaw_rate_, 0.0);
        // Roll (lateral tilt) and roll rate, for the roll control loop.
        register_output("/wheelleg/state/roll", state_roll_, 0.0);
        register_output("/wheelleg/state/roll_rate", state_roll_rate_, 0.0);
        // Body-forward speed of the base (m/s), heading-independent. Used by the
        // outer speed loop so that turning does not change the plant gain.
        register_output("/wheelleg/state/velocity", state_velocity_, 0.0);
        // Body-forward odometry distance (m), the integral of the body-forward
        // speed. The outer loop is a second-order plant on this coordinate
        // (s_ddot = b0 * tau + f), matching the ADRC's second-order ESO, and it
        // is heading-independent (unlike the world-frame com_x).
        register_output("/wheelleg/state/distance", state_distance_, 0.0);

        // Wheel-ground friction diagnostics (phase-0 observation). These expose
        // the *actual* contact state so the friction margin can be measured:
        //   contact_normal_*        instantaneous normal force N per wheel (N)
        //   slip_*                  contact-point slip speed (m/s, ~0 = rolling)
        //   friction_utilization_*  |Ft| / (mu*N), 1.0 = at the friction cone
        //   friction_torque_limit_* mu*N*r, the traction torque the contact can
        //                           actually transmit (Nm), vs wheel_torque_*
        register_output("/wheelleg/state/contact_normal_l", state_contact_normal_l_, 0.0);
        register_output("/wheelleg/state/contact_normal_r", state_contact_normal_r_, 0.0);
        register_output("/wheelleg/state/slip_l", state_slip_l_, 0.0);
        register_output("/wheelleg/state/slip_r", state_slip_r_, 0.0);
        register_output("/wheelleg/state/friction_utilization_l", state_friction_utilization_l_, 0.0);
        register_output("/wheelleg/state/friction_utilization_r", state_friction_utilization_r_, 0.0);
        register_output("/wheelleg/state/friction_torque_limit_l", state_friction_torque_limit_l_, 0.0);
        register_output("/wheelleg/state/friction_torque_limit_r", state_friction_torque_limit_r_, 0.0);
        register_output("/wheelleg/state/wheel_torque_l", state_wheel_torque_l_, 0.0);
        register_output("/wheelleg/state/wheel_torque_r", state_wheel_torque_r_, 0.0);

        setup_state_publisher_();

        initialize_pose_();
        step_count_ = 0;
        info("MuJoCo wheel-leg simulation ready ({} actuators, {} dof)", model_->nu, model_->nv);
        info(
            "motor torque limits: hip {:.1f} Nm, crank {:.1f} Nm, wheel {:.1f} Nm "
            "(motor x 268/17)",
            hip_torque_limit_, crank_torque_limit_, wheel_torque_limit_);
        info(
            "wheel friction torque: nominal static-weight ceiling {:.2f} Nm "
            "(mu={:.2f}, r={:.3f} m); online per-wheel ceiling is mu*N(t)*r from the "
            "contact solver, motor cap {:.2f} Nm",
            wheel_friction_torque_limit_, friction_coefficient_, wheel_radius_, wheel_torque_limit_);

        start_viewer_();
    }

    ~WheelLegSimulation() override {
        stop_viewer_();

        if (data_)
            mj_deleteData(data_);
        if (model_)
            mj_deleteModel(model_);
    }

    void before_updating() override {
        if (!reference_ready_warned_ && !command_component_->reference_ready()) {
            warn("no /wheelleg/reference/* input bound; simulation will stay passive");
            reference_ready_warned_ = true;
        }
    }

    void update() override {
        apply_control_();
        apply_constraints_();
        apply_gas_spring_();

        // Commissioning disturbance: a forward force impulse on the base.
        if (base_body_id_ >= 0) {
            const double t = step_count_ * dt_;
            const bool active = disturbance_start_ >= 0.0 && t >= disturbance_start_
                             && t < disturbance_start_ + disturbance_duration_;
            data_->xfrc_applied[base_body_id_ * 6 + 1] = active ? disturbance_force_ : 0.0;
        }

        mj_step(model_, data_);
        enforce_knee_hard_stop_();
        publish_state_();
        maybe_publish_state_();

        if (step_count_ % 100 == 0) {
            const auto leg_l = leg_length_angle_(kLJoint1, kLJoint3);
            const auto leg_r = leg_length_angle_(kRJoint1, kRJoint3);
            const auto com = vehicle_com_();
            const double axle_y =
                0.5
                * (data_->xanchor[3 * joint_ids_[kLJoint3] + 1]
                   + data_->xanchor[3 * joint_ids_[kRJoint3] + 1]);
            const double* base_rotation = data_->xmat + 9 * base_body_id_;
            const double n_l = l_contact_.normal;
            const double s_l = l_contact_.slip;
            const double n_r = r_contact_.normal;
            const double s_r = r_contact_.slip;
            // Roll-axis generalized torque on the floating base: project the
            // base angular dof forces onto the body forward axis. Splits the
            // roll source into bias (gyroscopic/gravity), constraint
            // (contacts/equalities) and applied (actuators).
            const int bd = base_dof_adr_;
            const double* fwd = base_rotation + 3; // body y axis in world
            const auto roll_torque = [&](const double* vec) {
                return vec[bd + 3] * fwd[0] + vec[bd + 4] * fwd[1] + vec[bd + 5] * fwd[2];
            };
            const double qbR = roll_torque(data_->qfrc_bias);
            const double qaR = roll_torque(data_->qfrc_applied);
            const double qbX = data_->qfrc_bias[bd + 3];
            const double qbY = data_->qfrc_bias[bd + 4];
            const double qbZ = data_->qfrc_bias[bd + 5];
            const double qcX = data_->qfrc_constraint[bd + 3];
            const double qcY = data_->qfrc_constraint[bd + 4];
            const double qcZ = data_->qfrc_constraint[bd + 5];
            const double tauHR = data_->ctrl[actuator_ids_[kActuatorHipR]];
            const double tauCR = data_->ctrl[actuator_ids_[kActuatorCrankR]];
            // Gravity part of the base angular bias: sum over bodies of
            // (r_i - base_origin) x (m_i g), rotated into the body frame.
            double gtx = 0.0, gty = 0.0;
            const double* bp = data_->xpos + 3 * base_body_id_;
            for (int bi = 1; bi < model_->nbody; ++bi) {
                const double bm = model_->body_mass[bi];
                if (bm <= 0.0)
                    continue;
                const double* ri = data_->xipos + 3 * bi;
                const double rx = ri[0] - bp[0];
                const double ry = ri[1] - bp[1];
                gtx += -bm * 9.81 * ry;
                gty += bm * 9.81 * rx;
            }
            const double* Rm = data_->xmat + 9 * base_body_id_;
            const double qgX = Rm[0] * gtx + Rm[1] * gty;
            const double qgY = Rm[3] * gtx + Rm[4] * gty;
            // CoM offset from the base origin, in the body frame (lateral x /
            // forward y). x_offset * M g is the gravity roll torque.
            const double ox = com[0] - bp[0];
            const double oy = com[1] - bp[1];
            const double oz = com[2] - bp[2];
            const double dxb = ox * Rm[0] + oy * Rm[1] + oz * Rm[2];
            const double dyb = ox * Rm[3] + oy * Rm[4] + oz * Rm[5];
            info(
                "t={:.1f} L0L={:.4f} L0R={:.4f} aL={:.4f} aR={:.4f} d={:.4f} pitch={:.4f} "
                "tauH={:.2f} tauC={:.2f} tauW={:.2f} tauWR={:.2f} cH={:.1f} cC={:.1f} dqC={:.3f} "
                "hL={:.3f} hR={:.3f} kL={:.3f} kR={:.3f} "
                "bz={:.3f} az={:.3f} by={:.4f} dqW={:.3f} dqWR={:.3f} cx={:.4f} "
                "roll={:.3f} vw={:.2f} vbody={:.2f} tcmn={:.2f} tdif={:.2f} "
                "nL={:.1f} sL={:.2f} nR={:.1f} sR={:.2f} uL={:.2f} uR={:.2f} "
                "flL={:.2f} flR={:.2f} "
                "qbR={:.3f} qaR={:.3f} qbX={:.3f} qbY={:.3f} qbZ={:.3f} "
                "qcX={:.3f} qcY={:.3f} qcZ={:.3f} qgX={:.3f} qgY={:.3f} "
                "cX={:.4f} cZ={:.4f} dxb={:.4f} dyb={:.4f} tHR={:.2f} tCR={:.2f} "
                "bq={:.3f},{:.3f},{:.3f},{:.3f}",
                step_count_ * dt_, leg_l[0], leg_r[0], leg_l[1], leg_r[1], com[1] - axle_y,
                body_pitch_(), dbg_hip_torque_, dbg_crank_torque_, dbg_wheel_torque_,
                dbg_wheel_torque_r_,
                data_->ctrl[actuator_ids_[kActuatorHipL]], data_->ctrl[actuator_ids_[kActuatorCrankL]],
                joint_velocity_(kLCrank),
                joint_position_(kLJoint1), joint_position_(kRJoint1),
                joint_position_(kLCrank), joint_position_(kRCrank),
                data_->qpos[base_qpos_adr_ + 2],
                data_->xanchor[3 * joint_ids_[kLJoint3] + 2], data_->qpos[base_qpos_adr_ + 1],
                joint_velocity_(kLJoint3), joint_velocity_(kRJoint3), com[1],
                std::atan2(base_rotation[6], base_rotation[8]),
                -0.5 * (joint_velocity_(kLJoint3) + joint_velocity_(kRJoint3)) * wheel_radius_,
                *state_velocity_,
                0.5 * (dbg_wheel_torque_ + dbg_wheel_torque_r_),
                0.5 * (dbg_wheel_torque_ - dbg_wheel_torque_r_),
                n_l, s_l, n_r, s_r, l_contact_.utilization, r_contact_.utilization,
                dbg_wheel_friction_limit_l_, dbg_wheel_friction_limit_r_,
                qbR, qaR, qbX, qbY, qbZ, qcX, qcY, qcZ, qgX, qgY,
                com[0], com[2], dxb, dyb, tauHR, tauCR,
                data_->qpos[base_qpos_adr_ + 3], data_->qpos[base_qpos_adr_ + 4],
                data_->qpos[base_qpos_adr_ + 5], data_->qpos[base_qpos_adr_ + 6]);
        }

        const double sim_time = step_count_ * dt_;
        (void)sim_time;

        ++step_count_;
    }

private:
    void load_parameters_() {
        dt_ = node::param_or("dt", 0.001);
        initial_height_ = node::param_or("initial_height", 0.39217);
        fix_base_ = node::param_or("fix_base", false);
        lock_leg_ = node::param_or("lock_leg", false);
        fix_base_hold_z_ = node::param_or("fix_base_hold_z", true);
        fix_base_planar_ = node::param_or("fix_base_planar", false);
        fix_base_release_time_ = node::param_or("fix_base_release_time", -1.0);

        model_path_ = node::param_or<std::string>("mjcf_path", std::string{});
        if (model_path_.empty()) {
            model_path_ = ament_index_cpp::get_package_share_directory("rmcs_core")
                        + "/models/wheelleg/wheelleg.xml";
        }

        hip_kp_ = node::param_or("hip_kp", 400.0);
        hip_kd_ = node::param_or("hip_kd", 8.0);
        wheel_kp_ = node::param_or("wheel_kp", 20.0);
        open_loop_wheel_torque_ =
            node::param_or("open_loop_wheel_torque", std::numeric_limits<double>::quiet_NaN());
        open_loop_wheel_same_sign_ = node::param_or("open_loop_wheel_same_sign", false);
        open_loop_roll_bias_ = node::param_or("open_loop_roll_bias", kNaN);
        open_loop_wheel_start_ = node::param_or("open_loop_wheel_start", 0.0);
        disturbance_force_ = node::param_or("disturbance_force", 0.0);
        disturbance_start_ = node::param_or("disturbance_start", -1.0);
        disturbance_duration_ = node::param_or("disturbance_duration", 0.0);
        wheel_velocity_damping_ = node::param_or("wheel_velocity_damping", 0.0);
        gravity_feedforward_ = node::param_or("gravity_feedforward", true);
        gravity_ff_analytic_ = node::param_or("gravity_ff_analytic", true);

        // Physical motor torque limits. In the closed-chain model the two
        // coaxial hip motors (thigh, crank) are separate joints at 40 Nm each;
        // the wheel motor is 3 Nm through a 268:17 reducer.
        hip_torque_limit_ = node::param_or("hip_torque_limit", 40.0);
        crank_torque_limit_ = node::param_or("crank_torque_limit", 40.0);

        // Closed-chain dynamics calibration (see apply_calibrated_dynamics_).
        // hip_armature is the reflected rotor inertia J_rotor * N^2, estimated
        // from the 9:1 hip motor; joint_damping is the calibrated viscous term.
        joint_damping_ = node::param_or("joint_damping", 0.1);
        hip_armature_ = node::param_or("hip_armature", 5.0e-3);
        // Leg Coulomb friction from the common-mode identification (hip+crank
        // co-mode): 2.11 Nm common -> ~1.05 Nm per driven joint (each joint
        // carries half when both move together). Applied to the driven joints
        // only; the passive linkage joints are not identified.
        leg_frictionloss_ = node::param_or("leg_frictionloss", 1.05);
        // Wheel joint dynamics from the real-robot identification
        // (wheels-r15p62): reflected M3508/gearbox inertia (armature), bearing
        // viscous damping and Coulomb friction, all at the wheel output shaft.
        // Wheels are NOT covered by joint_damping (that is the leg limit-cycle
        // term); with no armature/damping here the sim wheel is far lighter and
        // less damped than the real one.
        wheel_armature_ = node::param_or("wheel_armature", 4.7e-3);
        wheel_joint_damping_ = node::param_or("wheel_joint_damping", 1.0e-3);
        wheel_frictionloss_ = node::param_or("wheel_frictionloss", 9.0e-2);
        // Wheel motor speed limit at the output shaft [rad/s]; M3508 rated
        // ~469 rpm => 49.1 rad/s. 0/negative disables the cap.
        wheel_speed_limit_ = node::param_or("wheel_speed_limit", 49.1);
        constraint_timeconst_ = node::param_or("constraint_timeconst", 0.002);
        constraint_dampratio_ = node::param_or("constraint_dampratio", 1.0);
        contact_timeconst_ = node::param_or("contact_timeconst", 0.002);
        // The crank joint has a small reflected inertia, so it needs its own
        // (much lower) PD gains than the serial knee it replaced.
        crank_kp_ = node::param_or("crank_kp", 150.0);
        crank_kd_ = node::param_or("crank_kd", 2.0);
        // Wheel output-shaft torque limit [Nm]. The sim applies the command
        // directly at the output joint (no gearbox is modelled), so this is the
        // real motor's OUTPUT limit (DjiMotor::max_torque ≈ 4.88 Nm at 20 A),
        // NOT "motor torque x ratio". The online friction cone mu*N*r caps it
        // further per wheel while in contact.
        wheel_torque_limit_ = node::param_or("wheel_torque_limit", 4.88);

        // Ground friction limit on the wheel: |tau| <= mu * N * r, with the
        // normal force taken as the static weight. This caps the wheel torque
        // below the motor limit, since friction slips first.
        friction_coefficient_ = node::param_or("friction_coefficient", 2.0);
        wheel_radius_ = node::param_or("wheel_radius", 0.06);
        // Relation between the reduced knee angle and the linkage knee angle:
        //   knee_rel = knee_rel_sign * knee_reduced + knee_rel_offset
        // V5.0 uses the interior knee angle directly, so the sign is -1 (the
        // trajectory controller's branch) and the offset is zero.
        knee_rel_sign_ = node::param_or("knee_rel_sign", -1.0);

        // V5.0 reduced-model geometry (thigh/shank segments, see the MJCF).
        thigh_length_ = node::param_or("thigh_length", 0.21);
        shank_length_ = node::param_or("shank_length", 0.25005);

        // Analytic joint<->reduced-model zero offsets. hip_zero is the reduced
        // hip angle at q1 = 0; hip_sign flips the joint convention (V5.0 hip is
        // opposite to the trajectory controller's reduced-hip sense).
        hip_sign_ = node::param_or("hip_sign", -1.0);
        hip_zero_ = node::param_or("hip_zero", std::numbers::pi);
        // The motors are zeroed at (knee interior angle = 110 deg, thigh
        // perpendicular to the chassis). The model has the thigh perpendicular
        // at q_hip = 0, so the hip zero is q_hip = 0 and the crank zero is the
        // tabulated crank at 110 deg. knee_zero_* keeps the reduced knee zero
        // (= 110 deg) for the real-hardware port.
        knee_zero_l_ = node::param_or("knee_zero_l", wheelleg::LegLinkage::knee_motor_zero());
        knee_zero_r_ = node::param_or("knee_zero_r", wheelleg::LegLinkage::knee_motor_zero());

        // Mechanical knee limits (interior thigh-shank angle). These are hard
        // limits: the leg is designed not to straighten past 110 deg and not to
        // fold under 40 deg. Applied to the commanded knee and, optionally, as a
        // post-step hard stop on the relative crank.
        knee_limit_min_ = node::param_or(
            "knee_limit_min", wheelleg::LegLinkage::knee_limit_min());
        knee_limit_max_ = node::param_or(
            "knee_limit_max", wheelleg::LegLinkage::knee_limit_max());
        knee_hard_stop_ = node::param_or("knee_hard_stop", true);

        // Sign of the wheel command: target qdot_L3 = -wheel_dir * omega_ref.
        wheel_dir_ = node::param_or("wheel_dir", 1.0);

        state_pitch_sign_ = node::param_or("state_pitch_sign", 1.0);
        state_shank_sign_ = node::param_or("state_shank_sign", 1.0);
        // Reduced hip reported to the self-righting controller (V5.0: the raw
        // L_joint1 with an optional sign).
        state_hip_sign_ = node::param_or("state_hip_sign", 1.0);
        // V5.0 reports the interior knee angle with the controller's -1 branch,
        // i.e. negative when bent.
        state_knee_sign_ = node::param_or("state_knee_sign", -1.0);
        state_wheel_sign_ = node::param_or("state_wheel_sign", 1.0);
        state_yaw_sign_ = node::param_or("state_yaw_sign", 1.0);
        state_velocity_sign_ = node::param_or("state_velocity_sign", 1.0);
        state_distance_sign_ = node::param_or("state_distance_sign", 1.0);
        state_roll_sign_ = node::param_or("state_roll_sign", 1.0);

        viewer_state_rate_ = node::param_or("viewer_state_rate", 100.0);
        state_topic_ =
            node::param_or<std::string>("state_topic", "/wheelleg/simulation/state");
        viewer_enabled_ = node::param_or("viewer", true);
        viewer_script_ = node::param_or<std::string>("viewer_script", std::string{});
        viewer_track_body_ =
            node::param_or<std::string>("viewer_track_body", "base_link");
        viewer_follow_ = node::param_or("viewer_follow", true);
        viewer_distance_ = node::param_or("viewer_distance", 2.5);
        viewer_azimuth_ = node::param_or("viewer_azimuth", 90.0);
        viewer_elevation_ = node::param_or("viewer_elevation", -20.0);

        // Gas spring (nitrogen spring) as an external force along the two pin
        // anchors, see apply_gas_spring_. N(u) = c0 + c2*u^2 + c3*u^3 [N], with
        // u = full_extension - pin_distance clamped to [0, stroke]. The
        // gas_spring_sign flips the push direction so it can be confirmed by
        // running once.
        gas_spring_enabled_ = node::param_or("gas_spring_enabled", true);
        gas_spring_sign_ = node::param_or("gas_spring_sign", 1.0);
        gas_spring_full_extension_ = node::param_or("gas_spring_full_extension", 0.232);
        gas_spring_stroke_ = node::param_or("gas_spring_stroke", 0.08);
        gas_spring_c0_ = node::param_or("gas_spring_c0", 280.0);
        gas_spring_c2_ = node::param_or("gas_spring_c2", 122.7359);
        gas_spring_c3_ = node::param_or("gas_spring_c3", 53.9698);

        if (dt_ <= 0.0)
            throw std::runtime_error{"wheelleg_simulation: dt must be positive"};

        if (viewer_state_rate_ > 0.0) {
            const auto period = static_cast<long>(
                std::llround((1.0 / viewer_state_rate_) / dt_));
            publish_period_steps_ = static_cast<std::uint64_t>(std::max(period, 1L));
        }
    }

    void setup_state_publisher_() {
        if (viewer_state_rate_ <= 0.0) {
            info("viewer state publishing disabled (viewer_state_rate <= 0)");
            return;
        }

        state_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            state_topic_, rclcpp::QoS{1});
        info(
            "publishing MuJoCo state on '{}' at ~{} Hz", state_topic_, viewer_state_rate_);
    }

    // Launch .script/launch-mujoco-viewer as a child process so the graphical
    // viewer comes up together with the simulation, without a manual step.
    void start_viewer_() {
        if (!viewer_enabled_)
            return;

        if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
            warn("viewer requested but no DISPLAY/WAYLAND_DISPLAY is set; skipping");
            return;
        }
        if (viewer_state_rate_ <= 0.0) {
            warn("viewer requested but viewer_state_rate <= 0; skipping");
            return;
        }

        const auto script = resolve_viewer_script_();
        if (script.empty()) {
            warn(
                "viewer requested but launch-mujoco-viewer was not found "
                "(set the 'viewer_script' parameter)");
            return;
        }
        info(
            "viewer cfg: follow={} track_body='{}' distance={} azimuth={} elevation={}",
            viewer_follow_, viewer_follow_ ? viewer_track_body_ : std::string{},
            viewer_distance_, viewer_azimuth_, viewer_elevation_);

        std::vector<std::string> arguments{
            script,
            model_path_,
            "--topic",
            state_topic_,
            "--track-body",
            viewer_follow_ ? viewer_track_body_ : std::string{},
            "--distance",
            std::to_string(viewer_distance_),
            "--azimuth",
            std::to_string(viewer_azimuth_),
            "--elevation",
            std::to_string(viewer_elevation_)};
        std::vector<char*> argv;
        argv.reserve(arguments.size() + 1);
        for (auto& argument : arguments)
            argv.push_back(argument.data());
        argv.push_back(nullptr);

        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        pid_t pid = 0;
        const int result =
            posix_spawn(&pid, script.c_str(), &actions, nullptr, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions);

        if (result != 0) {
            warn("failed to start MuJoCo viewer: {}", std::strerror(result));
            return;
        }

        viewer_pid_ = pid;
        info("started MuJoCo viewer (pid {})", static_cast<long>(pid));
    }

    void stop_viewer_() {
        if (viewer_pid_ <= 0)
            return;

        // The viewer is detached-ish; ask it to close and reap it best-effort.
        kill(viewer_pid_, SIGTERM);
        waitpid(viewer_pid_, nullptr, 0);
        viewer_pid_ = -1;
    }

    std::string resolve_viewer_script_() const {
        if (!viewer_script_.empty())
            return viewer_script_;

        std::vector<std::string> candidates;
        if (const char* rmcs_path = std::getenv("RMCS_PATH"))
            candidates.emplace_back(std::string{rmcs_path} + "/.script/launch-mujoco-viewer");
        candidates.emplace_back(".script/launch-mujoco-viewer");
        candidates.emplace_back("/workspaces/RMCS/.script/launch-mujoco-viewer");

        for (const auto& candidate : candidates) {
            if (access(candidate.c_str(), X_OK) == 0)
                return candidate;
        }
        return {};
    }

    // Experimental dynamics calibration for the closed chain (执行路线2 §11):
    // reflected hip-motor rotor inertia, joint viscous damping, and the
    // four-bar loop-constraint stiffness. Applied after load so the MJCF stays
    // a pure geometric/kinematic artifact and the values can be tuned in yaml.
    void apply_calibrated_dynamics_() {
        for (const char* name : {"L_joint1", "R_joint1", "L_crank", "R_crank"}) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, name);
            if (id >= 0)
                model_->dof_armature[model_->jnt_dofadr[id]] = hip_armature_;
        }
        // Driven-joint Coulomb friction (common-mode identification, halved to
        // each joint). Passive linkage joints are left unfrictioned.
        for (const char* name : {"L_joint1", "R_joint1", "L_crank", "R_crank"}) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, name);
            if (id >= 0)
                model_->dof_frictionloss[model_->jnt_dofadr[id]] = leg_frictionloss_;
        }
        for (const char* name :
             {"L_joint1", "R_joint1", "L_joint2", "R_jonit2", "L_crank", "R_crank", "LL_joint2",
              "LL_joint3", "LL_link4", "RR_joint2", "RR_joint3", "RR_joint4"}) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, name);
            if (id >= 0)
                model_->dof_damping[model_->jnt_dofadr[id]] = joint_damping_;
        }
        // Wheel joints: reflected inertia + bearing viscous/Coulomb friction
        // from the r15p62 identification. Deliberately separate from the leg
        // joint_damping (0.3), which the wheels must not inherit.
        for (const char* name : {"L_joint3", "R_joint3"}) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, name);
            if (id < 0)
                continue;
            const int dof = model_->jnt_dofadr[id];
            model_->dof_armature[dof] = wheel_armature_;
            model_->dof_damping[dof] = wheel_joint_damping_;
            model_->dof_frictionloss[dof] = wheel_frictionloss_;
        }
        for (int e = 0; e < model_->neq; ++e) {
            model_->eq_solref[2 * e] = constraint_timeconst_;
            model_->eq_solref[2 * e + 1] = constraint_dampratio_;
        }
        // Stiff ground contacts (default 0.02 s is too soft for this mass and
        // lets the wheel sink, tipping the robot).
        for (int g = 0; g < model_->ngeom; ++g) {
            model_->geom_solref[2 * g] = contact_timeconst_;
            model_->geom_solref[2 * g + 1] = 1.0;
        }
        if (model_->ngeom > 0) {
            model_->geom_solimp[0] = 0.95;
            model_->geom_solimp[1] = 0.99;
            model_->geom_solimp[3] = 0.001;
            model_->geom_solimp[4] = 0.5;
        }
    }

    void load_model_() {
        std::array<char, 1024> error{};
        model_ = mj_loadXML(model_path_.c_str(), nullptr, error.data(), error.size());
        if (!model_)
            throw std::runtime_error(
                "wheelleg_simulation: failed to load '" + model_path_ + "': " + error.data());
        data_ = mj_makeData(model_);
        if (!data_) {
            mj_deleteModel(model_);
            model_ = nullptr;
            throw std::runtime_error{"wheelleg_simulation: failed to allocate mjData"};
        }

        model_->opt.timestep = dt_;

        joint_ids_.fill(-1);
        actuator_ids_.fill(-1);
        for (std::size_t i = 0; i < kJointNames.size(); ++i) {
            joint_ids_[i] = mj_name2id(model_, mjOBJ_JOINT, kJointNames[i]);
            if (joint_ids_[i] < 0)
                throw std::runtime_error{
                    std::string{"wheelleg_simulation: joint '"} + kJointNames[i]
                    + "' not found in " + model_path_};
        }
        for (std::size_t i = 0; i < kActuatorNames.size(); ++i) {
            actuator_ids_[i] = mj_name2id(model_, mjOBJ_ACTUATOR, kActuatorNames[i]);
            if (actuator_ids_[i] < 0)
                throw std::runtime_error{
                    std::string{"wheelleg_simulation: actuator '"} + kActuatorNames[i]
                    + "' not found in " + model_path_};
        }

        // Widen the leg actuators' ctrlrange so the sim-side torque limit is the
        // only cap (the MJCF default +/-100 would otherwise clip it). This lets
        // commissioning stand in for the real robot's unmodelled gas spring.
        const double leg_ctrl_range = std::max(hip_torque_limit_, crank_torque_limit_);
        for (const std::size_t idx : {kActuatorHipL, kActuatorCrankL, kActuatorHipR, kActuatorCrankR}) {
            const int actuator = actuator_ids_[idx];
            model_->actuator_ctrlrange[2 * actuator] = -leg_ctrl_range;
            model_->actuator_ctrlrange[2 * actuator + 1] = leg_ctrl_range;
        }
        info(
            "leg actuator ctrlrange widened to +/-{:.1f} (hip limit {:.1f}, crank limit {:.1f})",
            leg_ctrl_range, hip_torque_limit_, crank_torque_limit_);

        apply_calibrated_dynamics_();

        // Commissioning constraints authored by models/wheelleg/build_closed_chain.py:
        // real equalities (weld / joint-lock) that replace post-step qpos clamping.
        // The leg closed-chain constraints must ALWAYS be active; zeroing
        // eq_active (below) must not disable them.
        closure_eq_ids_.fill(-1);
        for (std::size_t i = 0; i < closure_eq_ids_.size(); ++i)
            closure_eq_ids_[i] = mj_name2id(model_, mjOBJ_EQUALITY, kClosureEqNames[i]);
        fix_base_z_eq_ = mj_name2id(model_, mjOBJ_EQUALITY, "fix_base_z");
        fix_base_planar_eq_ = mj_name2id(model_, mjOBJ_EQUALITY, "fix_base_planar");
        lock_zgantry_eq_ = mj_name2id(model_, mjOBJ_EQUALITY, "lock_zgantry_slide_z");
        planar_slide_y_id_ = mj_name2id(model_, mjOBJ_JOINT, "planar_slide_y");
        planar_slide_z_id_ = mj_name2id(model_, mjOBJ_JOINT, "planar_slide_z");
        planar_pitch_id_ = mj_name2id(model_, mjOBJ_JOINT, "planar_pitch");
        zgantry_slide_z_id_ = mj_name2id(model_, mjOBJ_JOINT, "zgantry_slide_z");

        base_free_joint_id_ = mj_name2id(model_, mjOBJ_JOINT, "base_free");
        base_body_id_ = mj_name2id(model_, mjOBJ_BODY, "base_link");
        l_wheel_body_id_ = mj_name2id(model_, mjOBJ_BODY, "L_link3");
        r_wheel_body_id_ = mj_name2id(model_, mjOBJ_BODY, "R_link3");
        l_wheel_geom_id_ = mj_name2id(model_, mjOBJ_GEOM, "L_link3_collision");
        r_wheel_geom_id_ = mj_name2id(model_, mjOBJ_GEOM, "R_link3_collision");
        // Gas-spring anchors live on the thigh and shank meshes (see
        // apply_gas_spring_).
        l_thigh_body_id_ = mj_name2id(model_, mjOBJ_BODY, "L_link1");
        l_shank_body_id_ = mj_name2id(model_, mjOBJ_BODY, "L_link2");
        r_thigh_body_id_ = mj_name2id(model_, mjOBJ_BODY, "R_link1");
        r_shank_body_id_ = mj_name2id(model_, mjOBJ_BODY, "R_link2");
        if (base_free_joint_id_ < 0 || base_body_id_ < 0)
            throw std::runtime_error{
                "wheelleg_simulation: model must expose a base_free joint on base_link"};
        base_dof_adr_ = model_->jnt_dofadr[base_free_joint_id_];

        const double total_mass = mj_getTotalmass(model_);
        const double gravity = std::abs(model_->opt.gravity[2]);
        wheel_friction_torque_limit_ =
            friction_coefficient_ * total_mass * gravity * wheel_radius_;

        // Analytic-gravity parameters: prefer explicit params, else model truth.
        total_mass_ = node::param_or("total_mass", total_mass);
        gravity_ = node::param_or("gravity", gravity);
    }

    void setup_leg_linkages_() {
        // V5.0 six-bar transmission is the same on both legs: the relative
        // crank is c = q_crank - q_hip (see leg_transmission.hpp).
        linkage_l_ = std::make_unique<wheelleg::LegLinkage>();
        linkage_r_ = std::make_unique<wheelleg::LegLinkage>();

        // Relative-crank bounds for the mechanical knee limits, and the crank
        // motor zero (encoder reads 0 there). Used by the command clamp and the
        // optional post-step hard stop.
        (void)linkage_l_->crank_from_knee(knee_limit_min_, crank_limit_min_);
        (void)linkage_l_->crank_from_knee(knee_limit_max_, crank_limit_max_);
        crank_motor_zero_ = linkage_l_->crank_motor_zero();
        info(
            "knee limits: {:.1f}..{:.1f} deg -> crank c {:.4f}..{:.4f} rad; "
            "motor zero (knee {:.0f} deg): hip q={:.4f}, crank c={:.4f}",
            knee_limit_min_ * 180.0 / std::numbers::pi,
            knee_limit_max_ * 180.0 / std::numbers::pi, crank_limit_min_, crank_limit_max_,
            wheelleg::LegLinkage::kKneeMotorZeroDeg, hip_motor_zero_, crank_motor_zero_);
    }

    void initialize_pose_() {
        mj_resetData(model_, data_);
        const auto base_qpos = model_->jnt_qposadr[base_free_joint_id_];
        base_qpos_adr_ = base_qpos;
        data_->qpos[base_qpos + 2] = initial_height_;
        data_->qpos[base_qpos + 3] = 1.0;
        mj_forward(model_, data_);

        // V5.0 maps the reduced knee directly: q_crank = c(beta) + q_hip, so no
        // per-leg calibration offset is needed.
        knee_rel_offset_l_ = 0.0;
        knee_rel_offset_r_ = 0.0;

        info(
            "leg linkage: reduced knee range [{:.3f}, {:.3f}] rad, assembly knee {:.3f}",
            linkage_l_->knee_min(), linkage_l_->knee_max(), linkage_l_->assembly_knee());

        // Start pose, selected by `initial_pose`:
        //   "standing" (default): V5.0 standing pose, hip height ~0.39 m.
        //   "motor_zero": the CAD pose at the motor/encoder zero -- thigh
        //     perpendicular to the chassis (vertical), knee interior ~105 deg,
        //     legs up; only the chassis is placed on the ground. Use this to
        //     inspect / verify the zero calibration against the real robot.
        //   "legs_up": chassis grounded, legs folded in the air.
        //   "crouched": the RL recovery `crouched`/`fold` pose (frozen V6
        //     profile): hip L_joint1 = -0.252107, crank LL_joint1 = -0.117909,
        //     interior knee 50 deg, wheels down. Passive closed-chain joints are
        //     seeded from the standing pose and settle via the closure.
        const std::string initial_pose =
            node::param_or<std::string>("initial_pose", std::string{"standing"});
        std::array<std::pair<const char*, double>, 12> pose;
        if (initial_pose == "motor_zero") {
            pose = {{
                {"L_joint1", 1.64841}, {"L_crank", 3.01539}, {"L_joint2", 1.04927},
                {"LL_joint2", -1.00733}, {"LL_joint3", 0.72236}, {"LL_link4", -1.17689},
                {"R_joint1", 1.65073}, {"R_crank", 3.02434}, {"R_jonit2", 1.08313},
                {"RR_joint2", -1.01574}, {"RR_joint3", 0.73583}, {"RR_joint4", -1.17987},
            }};
            data_->qpos[base_qpos + 2] = node::param_or("initial_pose_height", 0.173);
        } else if (initial_pose == "legs_up") {
            pose = {{
                {"L_joint1", 2.52126}, {"L_crank", 0.55199}, {"L_joint2", -0.52882},
                {"LL_joint2", 0.60156}, {"LL_joint3", 0.69273}, {"LL_link4", 0.80236},
                {"R_joint1", 2.52335}, {"R_crank", 0.49563}, {"R_jonit2", -0.54813},
                {"RR_joint2", 0.61024}, {"RR_joint3", 0.73363}, {"RR_joint4", 0.80705},
            }};
            data_->qpos[base_qpos + 2] = node::param_or("initial_pose_height", 0.173);
        } else if (initial_pose == "crouched") {
            // RL recovery crouched/fold pose (frozen V6 profile). Our reduced
            // hip == raw L_joint1, so the same value applies to both legs; the
            // crank is the relative crank for a 50 deg knee plus the hip. The
            // passive joints reuse the standing seed and close via the model
            // constraints.
            pose = {{
                {"L_joint1", -0.252107}, {"L_crank", -0.110}, {"L_joint2", 0.83313},
                {"LL_joint2", -0.78961}, {"LL_joint3", 0.53888}, {"LL_link4", -0.92236},
                {"R_joint1", -0.252107}, {"R_crank", -0.110}, {"R_jonit2", 0.84273},
                {"RR_joint2", -0.79012}, {"RR_joint3", 0.54198}, {"RR_joint4", -0.92513},
            }};
            data_->qpos[base_qpos + 2] = node::param_or("initial_pose_height", 0.26);
        } else {
            pose = {{
                {"L_joint1", -0.60000}, {"L_crank", 0.50000}, {"L_joint2", 0.83313},
                {"LL_joint2", -0.78961}, {"LL_joint3", 0.53888}, {"LL_link4", -0.92236},
                {"R_joint1", -0.60000}, {"R_crank", 0.50000}, {"R_jonit2", 0.84273},
                {"RR_joint2", -0.79012}, {"RR_joint3", 0.54198}, {"RR_joint4", -0.92513},
            }};
        }
        for (const auto& [name, value] : pose) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, name);
            if (id >= 0)
                data_->qpos[model_->jnt_qposadr[id]] = value;
        }
        mj_forward(model_, data_);

        // Hand-authored poses (e.g. "crouched") may leave the passive
        // closed-chain joints inconsistent with the actuated ones. Settle them
        // against the structural closure with gravity off and the base held
        // still (zeroed velocities each step), then restore. Without this the
        // first frame snaps and can flip the robot.
        if (initial_pose == "crouched") {
            const double gx = model_->opt.gravity[0];
            const double gy = model_->opt.gravity[1];
            const double gz = model_->opt.gravity[2];
            model_->opt.gravity[0] = model_->opt.gravity[1] = model_->opt.gravity[2] = 0.0;
            for (int i = 0; i < model_->neq; ++i)
                data_->eq_active[i] = 0;
            for (const int eq : closure_eq_ids_)
                if (eq >= 0)
                    data_->eq_active[eq] = 1;
            for (int k = 0; k < 2000; ++k) {
                mj_step(model_, data_);
                std::fill_n(data_->qvel, model_->nv, 0.0);
            }
            for (int i = 0; i < model_->neq; ++i)
                data_->eq_active[i] = 0;
            std::fill_n(data_->qvel, model_->nv, 0.0);
            model_->opt.gravity[0] = gx;
            model_->opt.gravity[1] = gy;
            model_->opt.gravity[2] = gz;
            mj_forward(model_, data_);
        }

        leg_lock_ids_.fill(-1);
        for (std::size_t i = 0; i < pose.size(); ++i) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, pose[i].first);
            leg_lock_ids_[i] = id;
            leg_lock_qpos_[i] = id >= 0 ? data_->qpos[model_->jnt_qposadr[id]] : 0.0;

            // Point the joint-lock equality at the start angle.
            const std::string lock_name = std::string{"lock_"} + pose[i].first;
            leg_lock_eq_ids_[i] = mj_name2id(model_, mjOBJ_EQUALITY, lock_name.c_str());
            if (leg_lock_eq_ids_[i] >= 0)
                model_->eq_data[leg_lock_eq_ids_[i] * mjNEQDATA] = leg_lock_qpos_[i];
        }

        // Move the commissioning carriers to the initial base pose so their
        // welds are already satisfied the moment they are activated.
        const auto place_carrier = [&](int joint_id, double value) {
            if (joint_id >= 0)
                data_->qpos[model_->jnt_qposadr[joint_id]] = value;
        };
        place_carrier(planar_slide_y_id_, data_->qpos[base_qpos + 1]);
        place_carrier(planar_slide_z_id_, data_->qpos[base_qpos + 2]);
        place_carrier(planar_pitch_id_, 0.0);
        place_carrier(zgantry_slide_z_id_, data_->qpos[base_qpos + 2]);
        if (lock_zgantry_eq_ >= 0)
            model_->eq_data[lock_zgantry_eq_ * mjNEQDATA] = data_->qpos[base_qpos + 2];
        if (data_)
            std::fill_n(data_->eq_active, model_->neq, static_cast<mjtByte>(0));

        const auto leg_l = leg_length_angle_(kLJoint1, kLJoint3);
        const auto leg_r = leg_length_angle_(kRJoint1, kRJoint3);
        const auto com = vehicle_com_();
        const double axle_y =
            0.5
            * (data_->xanchor[3 * joint_ids_[kLJoint3] + 1]
               + data_->xanchor[3 * joint_ids_[kRJoint3] + 1]);
        info(
            "initial task state: L0={:.4f} m, alpha_L={:.4f} rad, d={:.4f} m, "
            "com=({:.4f}, {:.4f})",
            0.5 * (leg_l[0] + leg_r[0]), 0.5 * (leg_l[1] + leg_r[1]), com[1] - axle_y, com[1],
            com[2]);

        // Seed the online friction ceiling so the very first control step does
        // not see a zero (uninitialised) normal force.
        l_contact_ = wheel_contact_state_(l_wheel_geom_id_, l_wheel_body_id_);
        r_contact_ = wheel_contact_state_(r_wheel_geom_id_, r_wheel_body_id_);
    }

    // Commissioning aid: pin the floating base / lock the leg using the model's
    // real equalities (welds and joint-locks authored by build_closed_chain.py),
    // toggled through eq_active. Clamping qpos/qvel after mj_step is *not* a
    // constraint — the free DOFs would still evolve unconstrained inside each
    // step — so the isolated loops would not be physically faithful.
    void apply_constraints_() {
        if (model_ == nullptr || data_ == nullptr)
            return;

        // Optionally release the base after a delay so the loops can settle
        // while it is held (commissioning).
        const bool base_held =
            fix_base_ && !(fix_base_release_time_ > 0.0 && step_count_ * dt_ >= fix_base_release_time_);

        if (fix_base_z_eq_ >= 0)
            data_->eq_active[fix_base_z_eq_] =
                static_cast<mjtByte>(base_held && !fix_base_planar_);
        if (fix_base_planar_eq_ >= 0)
            data_->eq_active[fix_base_planar_eq_] =
                static_cast<mjtByte>(base_held && fix_base_planar_);
        // Full fix = the vertical gantry weld plus a lock on the gantry's slide.
        if (lock_zgantry_eq_ >= 0)
            data_->eq_active[lock_zgantry_eq_] = static_cast<mjtByte>(
                base_held && !fix_base_planar_ && fix_base_hold_z_);

        const auto leg_locked = static_cast<mjtByte>(lock_leg_);
        for (const int eq : leg_lock_eq_ids_)
            if (eq >= 0)
                data_->eq_active[eq] = leg_locked;

        // Closed-chain constraints are structural: always on.
        for (const int eq : closure_eq_ids_)
            if (eq >= 0)
                data_->eq_active[eq] = 1;
    }

    double joint_position_(std::size_t index) const {
        return data_->qpos[model_->jnt_qposadr[joint_ids_[index]]];
    }

    double joint_velocity_(std::size_t index) const {
        return data_->qvel[model_->jnt_dofadr[joint_ids_[index]]];
    }

    // Analytic mapping to the reduced serial-leg model of
    // 执行路线1_数学解析.md (derivation in the wheel-leg analysis notes):
    //   phi  = gamma(AK)            (shank angle, forward = world +y, up = +z)
    //   beta = gamma(KH) - phi + pi (knee, straight ~= pi)
    //   theta = atan2(body_up . fwd, body_up . up)
    //   alpha = pi - theta - beta - phi  and  alpha = hip_zero + q1_L
    //   alpha = hip_zero - q1_R ; beta = knee_zero_L - q2_L ; beta = knee_zero_R + q2_R
    static double wrap_to_pi(double angle) {
        return std::remainder(angle, 2.0 * std::numbers::pi);
    }

    double shank_angle_(std::size_t knee, std::size_t wheel) const {
        const double* A = data_->xanchor + 3 * joint_ids_[wheel];
        const double* K = data_->xanchor + 3 * joint_ids_[knee];
        return std::atan2(K[2] - A[2], K[1] - A[1]);
    }

    double knee_angle_(std::size_t hip, std::size_t knee, std::size_t wheel) const {
        const double* H = data_->xanchor + 3 * joint_ids_[hip];
        const double* K = data_->xanchor + 3 * joint_ids_[knee];
        const double psi1 = std::atan2(H[2] - K[2], H[1] - K[1]);
        return wrap_to_pi(psi1 - shank_angle_(knee, wheel) + std::numbers::pi);
    }

    double body_pitch_() const {
        const double* rotation = data_->xmat + 9 * base_body_id_;
        // Fore-aft pitch from the body forward (y) axis vertical component.
        // MuJoCo stores xmat row-major with the body axes as its *columns*, so
        // the forward axis is R[:,1] and its world-z component is R[2][1]
        // (flat index 7). The old code read R[1][2] (flat index 5) -- the body
        // z-axis's world-y component -- which is not the pitch and whose gain
        // collapses as cos(yaw) (blind at yaw = 90deg), decoupling the balance
        // loop from the true fore-aft tilt.
        // asin(forward_z) is independent of roll, unlike atan2(forward_z, up_z)
        // which is biased by roll (and would fool the pitch balance loop).
        // Negated so nose-up is negative, matching the balance loop's sign
        // convention (was asin(R[1][2]) == -theta at yaw = 0).
        return -std::asin(std::clamp(rotation[7], -1.0, 1.0));
    }

    // Instantaneous wheel-ground contact diagnostics. `normal` is the total
    // normal force, `slip` the speed of the contact point (0 while rolling
    // without slip, large when slipping), `traction` the transmitted tangential
    // force, `friction_limit` the cone bound sum(mu_i * N_i) and `utilization`
    // the ratio traction/limit (1.0 == at the friction limit).
    struct WheelContact {
        double normal = 0.0;
        double slip = 0.0;
        double traction = 0.0;
        double friction_limit = 0.0;
        double utilization = 0.0;
    };

    [[nodiscard]] WheelContact wheel_contact_state_(int geom_id, int body_id) const {
        WheelContact out;
        if (geom_id < 0 || body_id < 0)
            return out;
        double vel[6] = {};
        mj_objectVelocity(model_, data_, mjOBJ_BODY, body_id, vel, 0);
        const double* com = data_->xipos + 3 * body_id;
        for (int i = 0; i < data_->ncon; ++i) {
            const auto& c = data_->contact[i];
            if (c.geom1 != geom_id && c.geom2 != geom_id)
                continue;
            double force[6] = {};
            mj_contactForce(model_, data_, i, force);
            const double normal = std::abs(force[0]);
            out.normal += normal;
            out.friction_limit += c.friction[0] * normal;
            out.traction += std::hypot(force[1], force[2]);
            const double rx = c.pos[0] - com[0];
            const double ry = c.pos[1] - com[1];
            const double rz = c.pos[2] - com[2];
            const double vx = vel[3] + vel[1] * rz - vel[2] * ry;
            const double vy = vel[4] + vel[2] * rx - vel[0] * rz;
            const double vz = vel[5] + vel[0] * ry - vel[1] * rx;
            out.slip += std::sqrt(vx * vx + vy * vy + vz * vz);
        }
        if (out.friction_limit > 1e-9)
            out.utilization = out.traction / out.friction_limit;
        return out;
    }

    void apply_control_() {
        const auto command = command_component_->control();

        if (!command.valid) {
            std::fill(data_->ctrl, data_->ctrl + model_->nu, 0.0);
            return;
        }

        // Controller joint references -> MJCF joint targets (inverse of the mapping above).
        // V5.0: hip and crank joints mirror on the same sign (same value = mirrored
        // pose), so both legs use the same target; only the wheels are opposite.
        const double hip_l = wrap_to_pi(hip_sign_ * (command.hip_angle - hip_zero_));
        const double hip_r = hip_l;

        // Desired relative crank (thigh frame) for the reduced knee reference,
        // then the crank joint target. For the V5.0 six-bar,
        //   c = q_crank - q_hip   =>   q_crank = c + q_hip   (both legs)
        auto crank_target = [&](const wheelleg::LegLinkage& linkage, double offset,
                                double assembly_crank, std::size_t thigh_joint) -> double {
            const double knee_rel = knee_rel_sign_ * command.knee_angle + offset;
            const double clamped =
                std::clamp(knee_rel, knee_limit_min_, knee_limit_max_);
            double crank = 0.0;
            if (!linkage.crank_from_knee(clamped, crank))
                return kNaN;
            const double thigh = joint_position_(thigh_joint);
            return crank - assembly_crank + thigh;
        };

        double hip_torque_l = 0.0;
        double crank_torque_l = 0.0;
        double hip_torque_r = 0.0;
        double crank_torque_r = 0.0;

        {
            const double roll_bias = std::isfinite(open_loop_roll_bias_)
                                        ? open_loop_roll_bias_
                                        : (command.roll_bias_valid ? command.roll_bias : 0.0);
            const double crank_l = crank_target(
                *linkage_l_, knee_rel_offset_l_ + roll_bias, linkage_l_->assembly_crank(), kLJoint1);
            const double crank_r = crank_target(
                *linkage_r_, knee_rel_offset_r_ - roll_bias, linkage_r_->assembly_crank(), kRJoint1);

            hip_torque_l = hip_kp_ * (hip_l - joint_position_(kLJoint1))
                         - hip_kd_ * joint_velocity_(kLJoint1);
            crank_torque_l = crank_kp_ * (crank_l - joint_position_(kLCrank))
                           - crank_kd_ * joint_velocity_(kLCrank);
            hip_torque_r = hip_kp_ * (hip_r - joint_position_(kRJoint1))
                         - hip_kd_ * joint_velocity_(kRJoint1);
            crank_torque_r = crank_kp_ * (crank_r - joint_position_(kRCrank))
                           - crank_kd_ * joint_velocity_(kRCrank);

            if (gravity_feedforward_) {
                // 数学解析.md §7.6: leg torque = Kp e + Kd e_dot + tau_grav.
                if (gravity_ff_analytic_) {
                    // Analytic leg gravity (the closed-form used on the real robot),
                    // not the model truth qfrc_bias.
                    const auto gl = leg_gravity_torque_(*linkage_l_, kLJoint1, kLJoint3);
                    const auto gr = leg_gravity_torque_(*linkage_r_, kRJoint1, kRJoint3);
                    hip_torque_l += gl[0];
                    crank_torque_l += gl[1];
                    hip_torque_r += gr[0];
                    crank_torque_r += gr[1];
                } else {
                    hip_torque_l += data_->qfrc_bias[model_->jnt_dofadr[joint_ids_[kLJoint1]]];
                    crank_torque_l += data_->qfrc_bias[model_->jnt_dofadr[joint_ids_[kLCrank]]];
                    hip_torque_r += data_->qfrc_bias[model_->jnt_dofadr[joint_ids_[kRJoint1]]];
                    crank_torque_r += data_->qfrc_bias[model_->jnt_dofadr[joint_ids_[kRCrank]]];
                }
            }
            // Optional leg feedforward (stand-up gas-spring / static-load comp).
            hip_torque_l += command.hip_torque_ff_l;
            hip_torque_r += command.hip_torque_ff_r;
            crank_torque_l += command.crank_torque_ff_l;
            crank_torque_r += command.crank_torque_ff_r;
            if (step_count_ % 500 == 0)
                info(
                    "reffemode hipL_tgt={:.3f} hipL={:.3f} crankL_tgt={:.3f} crankL={:.3f} "
                    "(cmd hip={:.3f} knee={:.3f})",
                    hip_l, joint_position_(kLJoint1), crank_l, joint_position_(kLCrank),
                    command.hip_angle, command.knee_angle);
        }

        // Wheel command. When the balance ring is present it outputs the wheel
        // torque directly (torque-form ADRC, model theta_ddot = f + b0*tau);
        // otherwise fall back to a velocity servo on the forward spin rate.
        // Applied as ctrl[L wheel] = +tau, ctrl[R wheel] = -tau: this produces
        // omega > 0 and forward motion.
        double wheel_torque = 0.0;
        double wheel_torque_l = 0.0;
        double wheel_torque_r = 0.0;
        if (command.per_leg_wheel_valid) {
            wheel_torque_l = command.wheel_torque_l;
            wheel_torque_r = command.wheel_torque_r;
        } else {
            if (command.torque_valid) {
                wheel_torque = command.wheel_torque;
            } else {
                const double omega = 0.5 * (joint_velocity_(kLJoint3) + joint_velocity_(kRJoint3));
                const double omega_reference = wheel_dir_ * command.wheel_velocity;
                wheel_torque = wheel_kp_ * (omega_reference - omega);
            }
            wheel_torque_l = wheel_torque;
            wheel_torque_r = wheel_torque;
        }

        // Commissioning: optional wheel-velocity damping (outer-loop surrogate)
        // to stop the base run-away of a pitch-only balance loop.
        if (wheel_velocity_damping_ != 0.0) {
            const double omega = joint_velocity_(kLJoint3);
            wheel_torque_l -= wheel_velocity_damping_ * omega;
            wheel_torque_r -= wheel_velocity_damping_ * omega;
        }

        // Open-loop wheel torque injection (system identification): overrides
        // whatever the controllers asked for.
        if (std::isfinite(open_loop_wheel_torque_)
            && step_count_ * dt_ >= open_loop_wheel_start_) {
            wheel_torque_l = open_loop_wheel_torque_;
            wheel_torque_r = open_loop_wheel_same_sign_ ? open_loop_wheel_torque_
                                                         : -open_loop_wheel_torque_;
        }

        // A5 acceleration feedforward removed: it consumed /wheelleg/command/*
        // and would make the bridge depend on the motion planner while the
        // planner depends on /wheelleg/state/yaw (circular dependency).

        // Motor speed limit (M3508 rated ~469 rpm at the wheel output ~= 49.1
        // rad/s): as the wheel approaches the limit the ACCELERATING torque
        // tapers linearly to zero, so a free / lightly-damped wheel cannot spin
        // up without bound and a fast re-contact cannot explode the contact
        // solver. Braking (torque opposed to motion) is never limited, matching
        // the real current loop. wheel_speed_limit <= 0 disables the cap.
        const auto motor_speed_limit = [&](double omega, double torque) {
            if (wheel_speed_limit_ <= 0.0 || omega * torque <= 0.0)
                return torque;
            const double frac = std::clamp(std::abs(omega) / wheel_speed_limit_, 0.0, 1.0);
            return torque * (1.0 - frac);
        };
        wheel_torque_l = motor_speed_limit(joint_velocity_(kLJoint3), wheel_torque_l);
        wheel_torque_r = motor_speed_limit(joint_velocity_(kRJoint3), wheel_torque_r);

        // Wheel torque is capped by the motor and, while in contact, by the
        // friction cone mu*N*r evaluated online from the last solved contact
        // state. Unlike the old static total-weight estimate (a fixed constant
        // per wheel) this tracks the instantaneous normal force, so a
        // lightly-loaded wheel -- e.g. the inside wheel of a turn or a wheel
        // about to lift -- gets proportionally less torque authority. The cone
        // force comes straight from the contact solver, so mu and N match the
        // actual contact. With no contact there is no friction to limit the
        // wheel: only the motor cap applies (an airborne wheel is free to spin
        // and still reacts on the body through its motor).
        const auto wheel_limit_for = [&](const WheelContact& contact) {
            if (contact.normal <= kContactNormalEps)
                return wheel_torque_limit_;
            return std::min(wheel_torque_limit_, contact.friction_limit * wheel_radius_);
        };
        const double wheel_limit_l = wheel_limit_for(l_contact_);
        const double wheel_limit_r = wheel_limit_for(r_contact_);
        dbg_wheel_friction_limit_l_ = wheel_limit_l;
        dbg_wheel_friction_limit_r_ = wheel_limit_r;

        set_torque_(kActuatorHipL, hip_torque_l, hip_torque_limit_);
        set_torque_(kActuatorCrankL, crank_torque_l, crank_torque_limit_);
        set_torque_(kActuatorWheelL, wheel_torque_l, wheel_limit_l);
        set_torque_(kActuatorHipR, hip_torque_r, hip_torque_limit_);
        set_torque_(kActuatorCrankR, crank_torque_r, crank_torque_limit_);
        set_torque_(kActuatorWheelR, wheel_torque_r, wheel_limit_r);

        dbg_hip_torque_ = hip_torque_l;
        dbg_crank_torque_ = crank_torque_l;
        dbg_wheel_torque_ = wheel_torque_l;
        dbg_wheel_torque_r_ = wheel_torque_r;
    }

    void set_torque_(std::size_t index, double torque, double limit) {
        data_->ctrl[actuator_ids_[index]] = std::clamp(torque, -limit, limit);
    }

    // Mechanical knee hard stop: the relative crank c = q_crank - q_hip must
    // stay within [crank(40deg), crank(110deg)]. Enforced after the step as a
    // position/velocity projection, standing in for the physical stop; the
    // commanded target is clamped separately in crank_target.
    void enforce_knee_hard_stop_() {
        if (!knee_hard_stop_ || model_ == nullptr || data_ == nullptr)
            return;
        const std::array<std::size_t, 2> hips{kLJoint1, kRJoint1};
        const std::array<std::size_t, 2> cranks{kLCrank, kRCrank};
        bool clamped = false;
        for (int i = 0; i < 2; ++i) {
            const int hip_joint = joint_ids_[hips[i]];
            const int crank_joint = joint_ids_[cranks[i]];
            const int hip_q = model_->jnt_qposadr[hip_joint];
            const int crank_q = model_->jnt_qposadr[crank_joint];
            const int hip_d = model_->jnt_dofadr[hip_joint];
            const int crank_d = model_->jnt_dofadr[crank_joint];
            const double q_hip = data_->qpos[hip_q];
            const double c = data_->qpos[crank_q] - q_hip;
            if (c < crank_limit_min_ || c > crank_limit_max_) {
                data_->qpos[crank_q] = std::clamp(c, crank_limit_min_, crank_limit_max_) + q_hip;
                const double d_c = data_->qvel[crank_d] - data_->qvel[hip_d];
                const bool moving_out =
                    (c > crank_limit_max_ && d_c > 0.0) || (c < crank_limit_min_ && d_c < 0.0);
                if (moving_out)
                    data_->qvel[crank_d] -= d_c;
                clamped = true;
            }
        }
        if (clamped)
            mj_forward(model_, data_);
    }

    // Transform a point given in a body's local frame to world coordinates.
    // MuJoCo stores xmat row-major with the body axes as its columns, so
    // world = R @ local with R[i][k] = xmat[3*i + k].
    void body_local_to_world_(int body, const double* local, double* world) const {
        const double* rotation = data_->xmat + 9 * body;
        for (int i = 0; i < 3; ++i)
            world[i] = data_->xpos[3 * body + i] + rotation[3 * i] * local[0]
                     + rotation[3 * i + 1] * local[1] + rotation[3 * i + 2] * local[2];
    }

    // Write a force acting at a world point into xfrc_applied as a wrench about
    // the body CoM (xfrc_applied is applied at xipos).
    void apply_point_force_(int body, const double* point, const double* force) const {
        const double* com = data_->xipos + 3 * body;
        const double rx = point[0] - com[0];
        const double ry = point[1] - com[1];
        const double rz = point[2] - com[2];
        double* wrench = data_->xfrc_applied + 6 * body;
        wrench[0] = force[0];
        wrench[1] = force[1];
        wrench[2] = force[2];
        wrench[3] = ry * force[2] - rz * force[1];
        wrench[4] = rz * force[0] - rx * force[2];
        wrench[5] = rx * force[1] - ry * force[0];
    }

    // Gas spring (nitrogen spring) as an external force along the two pin
    // anchors. The SolidWorks rigid links are not modelled (they would
    // over-constrain the six-bar), so the spring is applied here as a pure
    // force: the pins are pushed apart with the measured characteristic
    //   N(u) = c0 + c2*u^2 + c3*u^3 [N],  u = full_extension - pin_distance,
    // clamped to the stroke. The anchors are the V5 URDF joint origins on the
    // thigh (L_link1/R_link1) and shank (L_link2/R_link2); gas_spring_sign flips
    // the direction so it can be confirmed by a single run.
    void apply_gas_spring_() {
        if (!gas_spring_enabled_ || model_ == nullptr || data_ == nullptr)
            return;

        const std::array<int, 2> thigh_body{l_thigh_body_id_, r_thigh_body_id_};
        const std::array<int, 2> shank_body{l_shank_body_id_, r_shank_body_id_};
        const std::array<std::array<double, 3>, 2> thigh_anchor{{
            {0.01559, -0.048559, -0.002}, {0.01559, -0.048559, 0.002}}};
        const std::array<std::array<double, 3>, 2> shank_anchor{{
            {0.050754, 0.005, -0.002}, {0.050754, 0.005, 0.002}}};

        for (int i = 0; i < 2; ++i) {
            const int tb = thigh_body[i];
            const int sb = shank_body[i];
            if (tb < 0 || sb < 0)
                continue;

            double p_thigh[3];
            double p_shank[3];
            body_local_to_world_(tb, thigh_anchor[i].data(), p_thigh);
            body_local_to_world_(sb, shank_anchor[i].data(), p_shank);

            const double dx = p_shank[0] - p_thigh[0];
            const double dy = p_shank[1] - p_thigh[1];
            const double dz = p_shank[2] - p_thigh[2];
            const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (length < 1e-9)
                continue;

            const double u =
                std::clamp(gas_spring_full_extension_ - length, 0.0, gas_spring_stroke_);
            const double force =
                gas_spring_c0_ + gas_spring_c2_ * u * u + gas_spring_c3_ * u * u * u;
            const double scale = gas_spring_sign_ * force / length;
            const double f[3] = {scale * dx, scale * dy, scale * dz};
            const double nf[3] = {-f[0], -f[1], -f[2]};

            // Apply each force at its anchor, expressed as a wrench at the
            // body CoM (xfrc_applied acts at xipos).
            apply_point_force_(sb, p_shank, f);
            apply_point_force_(tb, p_thigh, nf);

            if (i == 0) {
                gas_spring_length_ = length;
                gas_spring_force_ = force;
                gas_spring_compression_ = u;
            }
        }
        if (step_count_ % 500 == 0)
            info(
                "gas spring L: pin_len={:.4f} u={:.4f} F={:.1f} N (sign={:+.0f})",
                gas_spring_length_, gas_spring_compression_, gas_spring_force_,
                gas_spring_sign_);
    }

    // World-frame vehicle CoM from the body inertials (exact, both legs).
    [[nodiscard]] std::array<double, 3> vehicle_com_() const {
        std::array<double, 3> com{0.0, 0.0, 0.0};
        double total = 0.0;
        for (int i = 1; i < model_->nbody; ++i) {
            const double mass = model_->body_mass[i];
            if (mass <= 0.0)
                continue;
            for (int k = 0; k < 3; ++k)
                com[static_cast<std::size_t>(k)] += mass * data_->xipos[3 * i + k];
            total += mass;
        }
        if (total > 0.0)
            for (auto& value : com)
                value /= total;
        return com;
    }

    // (leg length L0, leg angle alpha_L) for one leg from the world anchors.
    // The sagittal plane of this model is (forward, up) = (y, z).
    [[nodiscard]] std::array<double, 2> leg_length_angle_(
        std::size_t hip, std::size_t wheel) const {
        const double* H = data_->xanchor + 3 * joint_ids_[hip];
        const double* A = data_->xanchor + 3 * joint_ids_[wheel];
        const double dy = H[1] - A[1];
        const double dz = H[2] - A[2];
        return {std::hypot(dy, dz), std::atan2(dz, dy)};
    }

    // Interior thigh-shank angle from the leg length (cosine law), matching the
    // reduced knee convention used by the trajectory controller.
    [[nodiscard]] double reduced_knee_(double leg_length) const {
        const double cosine =
            (thigh_length_ * thigh_length_ + shank_length_ * shank_length_
             - leg_length * leg_length)
            / (2.0 * thigh_length_ * shank_length_);
        return std::acos(std::clamp(cosine, -1.0, 1.0));
    }

    // Reduced-leg Jacobian (dL0/dq, dgamma/dq) folding the crank so the mapping
    // to hip/crank torques is tau_hip = -a F_L + (1-b) T_a, tau_crank = a F_L + b T_a.
    // Mirrors wheelleg_decoupled_leg_controller's analytic Jacobian.
    [[nodiscard]] std::pair<double, double> leg_jacobian_(
        const wheelleg::LegLinkage& linkage, double leg_length) const {
        const double L1 = thigh_length_;
        const double L2 = shank_length_;
        const double cos_beta =
            std::clamp((L1 * L1 + L2 * L2 - leg_length * leg_length) / (2.0 * L1 * L2), -1.0, 1.0);
        const double beta = std::acos(cos_beta);
        const double d_leg_length_d_beta = L1 * L2 * std::sin(beta) / std::max(leg_length, 1e-9);
        const double d_gamma_d_beta = (L1 * L2 * cos_beta - L2 * L2) / (leg_length * leg_length);

        double d_beta_d_crank = 0.0;
        constexpr double kDelta = 1e-3;
        double cp = 0.0;
        double cm = 0.0;
        if (linkage.crank_from_knee(beta + kDelta, cp) && linkage.crank_from_knee(beta - kDelta, cm)
            && std::abs(cp - cm) > 1e-9)
            d_beta_d_crank = 2.0 * kDelta / (cp - cm);

        return {d_leg_length_d_beta * d_beta_d_crank, -d_gamma_d_beta * d_beta_d_crank};
    }

    // Analytic leg gravity generalized force mapped to hip/crank torques
    // (the real-robot feedforward; no model-truth qfrc_bias).
    [[nodiscard]] std::array<double, 2> leg_gravity_torque_(
        const wheelleg::LegLinkage& linkage, std::size_t hip, std::size_t wheel) const {
        const auto [leg_length, alpha] = leg_length_angle_(hip, wheel);
        const double m_half = 0.5 * total_mass_;
        const double bias_L = m_half * gravity_ * std::sin(alpha);
        const double bias_a = m_half * gravity_ * leg_length * std::cos(alpha);
        const auto [a, b] = leg_jacobian_(linkage, leg_length);
        return {-a * bias_L + (1.0 - b) * bias_a, a * bias_L + b * bias_a};
    }

    void publish_state_() {
        const auto leg_l = leg_length_angle_(kLJoint1, kLJoint3);
        const auto leg_r = leg_length_angle_(kRJoint1, kRJoint3);
        const double leg_length = 0.5 * (leg_l[0] + leg_r[0]);
        const double shank = 0.5 * (shank_angle_(kLJoint2, kLJoint3)
                                    + shank_angle_(kRJoint2, kRJoint3));
        // Reduced knee angle: interior thigh-shank angle from the cosine law.
        const double knee = reduced_knee_(leg_length);
        const double pitch = body_pitch_();
        const double omega = 0.5 * (joint_velocity_(kLJoint3) + joint_velocity_(kRJoint3));

        double shank_rate = 0.0;
        double knee_rate = 0.0;
        double pitch_rate = 0.0;
        if (has_previous_state_) {
            shank_rate = (shank - previous_shank_) / dt_;
            knee_rate = (knee - previous_knee_) / dt_;
            pitch_rate = (pitch - previous_pitch_) / dt_;
        }
        previous_shank_ = shank;
        previous_knee_ = knee;
        previous_pitch_ = pitch;
        has_previous_state_ = true;

        *state_pitch_ = state_pitch_sign_ * pitch;
        *state_pitch_velocity_ = state_pitch_sign_ * pitch_rate;
        *state_shank_angle_ = state_shank_sign_ * shank;
        *state_shank_velocity_ = state_shank_sign_ * shank_rate;
        *state_knee_angle_ = state_knee_sign_ * knee;
        *state_knee_velocity_ = state_knee_sign_ * knee_rate;
        *state_wheel_velocity_ = state_wheel_sign_ * omega;
        // Reduced hip (raw L_joint1) and base height for the self-righting
        // recovery controller.
        *state_hip_angle_ = state_hip_sign_ * joint_position_(kLJoint1);
        *state_base_height_ = data_->xpos[3 * base_body_id_ + 2];

        // Task-space state (执行路线2 §5/§8): leg length / angle and the CoM
        // offset d = (C_veh - A)_forward (static stability condition of §6).
        const auto com = vehicle_com_();
        const double axle_y =
            0.5
            * (data_->xanchor[3 * joint_ids_[kLJoint3] + 1]
               + data_->xanchor[3 * joint_ids_[kRJoint3] + 1]);
        const double axle_x =
            0.5
            * (data_->xanchor[3 * joint_ids_[kLJoint3] + 0]
               + data_->xanchor[3 * joint_ids_[kRJoint3] + 0]);

        // Horizontal body heading (body +y axis projected on the floor).
        const double* base_rotation = data_->xmat + 9 * base_body_id_;
        // Body forward (y) axis = xmat column 1 (MuJoCo columns are body axes),
        // i.e. (R[0][1], R[1][1]) = base_rotation[1], base_rotation[4]. The old
        // base_rotation[3] was the body x-axis's world-y component, which is the
        // forward heading mirrored in x (180deg wrong at yaw = 90deg).
        double heading_x = base_rotation[1];
        double heading_y = base_rotation[4];
        const double heading_norm = std::hypot(heading_x, heading_y);
        if (heading_norm > 1e-9) {
            heading_x /= heading_norm;
            heading_y /= heading_norm;
        } else {
            heading_x = 0.0;
            heading_y = 1.0;
        }

        *state_leg_length_ = leg_length;
        *state_leg_angle_ = 0.5 * (leg_l[1] + leg_r[1]);
        *state_leg_length_l_ = leg_l[0];
        *state_leg_length_r_ = leg_r[0];
        *state_leg_angle_l_ = leg_l[1];
        *state_leg_angle_r_ = leg_r[1];
        *state_com_x_ = com[1];
        *state_com_z_ = com[2];
        // CoM offset d in the axle (body-forward) frame: heading-independent, so
        // the posture loop stays valid while turning (was a world-y difference).
        *state_com_offset_ = (com[0] - axle_x) * heading_x + (com[1] - axle_y) * heading_y;

        // Yaw from the base quaternion (rotation about world +z), and yaw rate
        // from the base body's world-frame angular velocity.
        const double qw = data_->qpos[base_qpos_adr_ + 3];
        const double qx = data_->qpos[base_qpos_adr_ + 4];
        const double qy = data_->qpos[base_qpos_adr_ + 5];
        const double qz = data_->qpos[base_qpos_adr_ + 6];
        const double yaw_wrapped =
            std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        // Unwrap so a continuous spin yields a continuous heading (the yaw-angle
        // ADRC cannot handle the +/-pi branch cut).
        if (has_previous_yaw_) {
            double dyaw = yaw_wrapped - previous_yaw_;
            while (dyaw > std::numbers::pi)
                dyaw -= 2.0 * std::numbers::pi;
            while (dyaw < -std::numbers::pi)
                dyaw += 2.0 * std::numbers::pi;
            unwrapped_yaw_ += dyaw;
        } else {
            unwrapped_yaw_ = yaw_wrapped;
            has_previous_yaw_ = true;
        }
        previous_yaw_ = yaw_wrapped;
        double base_velocity[6] = {};
        mj_objectVelocity(model_, data_, mjOBJ_BODY, base_body_id_, base_velocity, 0);
        *state_yaw_ = state_yaw_sign_ * unwrapped_yaw_;
        *state_yaw_rate_ = state_yaw_sign_ * base_velocity[2];

        // Roll: lateral tilt about the body forward axis (roll-independent of
        // pitch). Rate is finite-differenced like the other rate states.
        // Roll about the body forward axis: the world-up projected on the body
        // x-z plane. With MuJoCo xmat columns as body axes, that is
        // atan2(R[2][0], R[2][2]) = atan2(base_rotation[6], base_rotation[8])
        // (body x-axis world-z vs body z-axis world-z). The old
        // atan2(-base_rotation[2], base_rotation[8]) used the world x-z
        // projection, which is roll about the world x-axis and therefore
        // contaminated by yaw (gain collapsed as cos(yaw)).
        const double roll = std::atan2(base_rotation[6], base_rotation[8]);
        double roll_rate = 0.0;
        if (has_previous_roll_)
            roll_rate = (roll - previous_roll_) / dt_;
        previous_roll_ = roll;
        has_previous_roll_ = true;
        *state_roll_ = state_roll_sign_ * roll;
        *state_roll_rate_ = state_roll_sign_ * roll_rate;

        // Body-forward CoM position and speed: the world CoM velocity projected
        // on the horizontal body heading, integrated. For yaw = 0 this reduces
        // exactly to com_x (so the outer loop matches the proven straight-line
        // behaviour), while for yaw != 0 it stays in the body frame and its
        // plant gain does not collapse with the heading.
        double com_velocity[3] = {0.0, 0.0, 0.0};
        if (has_previous_com_) {
            com_velocity[0] = (com[0] - previous_com_[0]) / dt_;
            com_velocity[1] = (com[1] - previous_com_[1]) / dt_;
            com_velocity[2] = (com[2] - previous_com_[2]) / dt_;
        }
        previous_com_ = com;
        has_previous_com_ = true;

        const double body_forward_speed =
            heading_x * com_velocity[0] + heading_y * com_velocity[1];
        body_distance_ += body_forward_speed * dt_;
        *state_velocity_ = state_velocity_sign_ * body_forward_speed;
        *state_distance_ = state_distance_sign_ * body_distance_;

        // Wheel-ground friction diagnostics (phase 0): sample the contact state
        // once per step and expose it so the friction margin can be measured
        // against the applied wheel torque.
        l_contact_ = wheel_contact_state_(l_wheel_geom_id_, l_wheel_body_id_);
        r_contact_ = wheel_contact_state_(r_wheel_geom_id_, r_wheel_body_id_);
        *state_contact_normal_l_ = l_contact_.normal;
        *state_contact_normal_r_ = r_contact_.normal;
        *state_slip_l_ = l_contact_.slip;
        *state_slip_r_ = r_contact_.slip;
        *state_friction_utilization_l_ = l_contact_.utilization;
        *state_friction_utilization_r_ = r_contact_.utilization;
        *state_friction_torque_limit_l_ = l_contact_.friction_limit * wheel_radius_;
        *state_friction_torque_limit_r_ = r_contact_.friction_limit * wheel_radius_;
        *state_wheel_torque_l_ = dbg_wheel_torque_;
        *state_wheel_torque_r_ = dbg_wheel_torque_r_;
    }

    // Mirror the raw MuJoCo state to ROS so an out-of-process passive viewer
    // (.script/launch-mujoco-viewer) can render the exact controlled simulation.
    // Layout: [qpos (nq), qvel (nv)].
    void maybe_publish_state_() {
        if (!state_publisher_ || publish_period_steps_ == 0
            || (step_count_ % publish_period_steps_) != 0)
            return;

        std_msgs::msg::Float64MultiArray message;
        message.data.resize(static_cast<std::size_t>(model_->nq + model_->nv));
        std::copy_n(data_->qpos, model_->nq, message.data.begin());
        std::copy_n(data_->qvel, model_->nv, message.data.begin() + model_->nq);
        state_publisher_->publish(message);
    }

    class WheelLegSimulationCommand : public rmcs_executor::Component {
    public:
        struct Control {
            bool valid = false;
            double hip_angle = 0.0;
            double knee_angle = 0.0;
            double wheel_velocity = 0.0;
            bool torque_valid = false;
            double wheel_torque = 0.0;
            bool per_leg_wheel_valid = false;
            double wheel_torque_l = 0.0;
            double wheel_torque_r = 0.0;
            // Roll loop actuation: a differential reduced-knee bias (rad) applied
            // as +bias on the left leg and -bias on the right, which tilts the
            // body laterally (differential leg length) to control roll.
            bool roll_bias_valid = false;
            double roll_bias = 0.0;
            // Optional leg feedforward torques (added on top of the leg PD),
            // e.g. the gas-spring / static-load compensation from the stand-up
            // script. NaN when unused.
            double hip_torque_ff_l = 0.0;
            double hip_torque_ff_r = 0.0;
            double crank_torque_ff_l = 0.0;
            double crank_torque_ff_r = 0.0;
        };

        explicit WheelLegSimulationCommand(WheelLegSimulation& simulation)
            : simulation_(simulation) {
            register_input("/wheelleg/reference/hip_angle", hip_angle_, false);
            register_input("/wheelleg/reference/knee_angle", knee_angle_, false);
            register_input("/wheelleg/reference/wheel_velocity", wheel_velocity_, false);
            register_input("/wheelleg/control/wheel_torque", wheel_torque_, false);
            register_input("/wheelleg/control/wheel_torque_l", wheel_torque_l_, false);
            register_input("/wheelleg/control/wheel_torque_r", wheel_torque_r_, false);
            register_input("/wheelleg/control/roll_bias", roll_bias_, false);
            register_input("/wheelleg/control/hip_torque_ff_l", hip_torque_ff_l_, false);
            register_input("/wheelleg/control/hip_torque_ff_r", hip_torque_ff_r_, false);
            register_input("/wheelleg/control/crank_torque_ff_l", crank_torque_ff_l_, false);
            register_input("/wheelleg/control/crank_torque_ff_r", crank_torque_ff_r_, false);
        }

        void update() override {
            if (hip_angle_.ready() && std::isfinite(*hip_angle_))
                control_.hip_angle = *hip_angle_;
            if (knee_angle_.ready() && std::isfinite(*knee_angle_))
                control_.knee_angle = *knee_angle_;
            if (wheel_velocity_.ready() && std::isfinite(*wheel_velocity_))
                control_.wheel_velocity = *wheel_velocity_;
            control_.torque_valid = wheel_torque_.ready() && std::isfinite(*wheel_torque_);
            if (control_.torque_valid)
                control_.wheel_torque = *wheel_torque_;
            control_.per_leg_wheel_valid = wheel_torque_l_.ready() && wheel_torque_r_.ready()
                                        && std::isfinite(*wheel_torque_l_)
                                        && std::isfinite(*wheel_torque_r_);
            if (control_.per_leg_wheel_valid) {
                control_.wheel_torque_l = *wheel_torque_l_;
                control_.wheel_torque_r = *wheel_torque_r_;
            }
            control_.roll_bias_valid = roll_bias_.ready() && std::isfinite(*roll_bias_);
            if (control_.roll_bias_valid)
                control_.roll_bias = *roll_bias_;
            if (hip_torque_ff_l_.ready() && std::isfinite(*hip_torque_ff_l_))
                control_.hip_torque_ff_l = *hip_torque_ff_l_;
            if (hip_torque_ff_r_.ready() && std::isfinite(*hip_torque_ff_r_))
                control_.hip_torque_ff_r = *hip_torque_ff_r_;
            if (crank_torque_ff_l_.ready() && std::isfinite(*crank_torque_ff_l_))
                control_.crank_torque_ff_l = *crank_torque_ff_l_;
            if (crank_torque_ff_r_.ready() && std::isfinite(*crank_torque_ff_r_))
                control_.crank_torque_ff_r = *crank_torque_ff_r_;
            control_.valid = hip_angle_.ready() && knee_angle_.ready()
                          && std::isfinite(*hip_angle_) && std::isfinite(*knee_angle_);
        }

        [[nodiscard]] bool reference_ready() const { return hip_angle_.ready(); }
        [[nodiscard]] const Control& control() const { return control_; }

    private:
        WheelLegSimulation& simulation_;

        InputInterface<double> hip_angle_;
        InputInterface<double> knee_angle_;
        InputInterface<double> wheel_velocity_;
        InputInterface<double> wheel_torque_;
        InputInterface<double> wheel_torque_l_;
        InputInterface<double> wheel_torque_r_;
        InputInterface<double> roll_bias_;
        InputInterface<double> hip_torque_ff_l_;
        InputInterface<double> hip_torque_ff_r_;
        InputInterface<double> crank_torque_ff_l_;
        InputInterface<double> crank_torque_ff_r_;

        Control control_;
    };

    enum JointIndex : std::size_t {
        kLJoint1,
        kLJoint2,
        kLJoint3,
        kRJoint1,
        kRJoint2,
        kRJoint3,
        kLCrank,
        kRCrank,
    };

    enum ActuatorIndex : std::size_t {
        kActuatorHipL,
        kActuatorCrankL,
        kActuatorWheelL,
        kActuatorHipR,
        kActuatorCrankR,
        kActuatorWheelR,
    };

    // State joints (knee L_joint2/R_jonit2 are now passive) plus the two crank
    // joints that carry the leg actuators in the closed-chain model.
    static constexpr std::array<const char*, 8> kJointNames = {
        "L_joint1", "L_joint2", "L_joint3", "R_joint1", "R_jonit2",
        "R_joint3", "L_crank", "R_crank"};
    static constexpr std::array<const char*, 6> kActuatorNames = {
        "L_joint1_motor", "L_crank_motor", "L_joint3_motor",
        "R_joint1_motor", "R_crank_motor", "R_joint3_motor"};

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    // Below this normal force the wheel is treated as airborne: no contact, so
    // no friction ceiling (only the motor cap applies).
    static constexpr double kContactNormalEps = 1e-3;

    std::shared_ptr<WheelLegSimulationCommand> command_component_;

    std::unique_ptr<wheelleg::LegLinkage> linkage_l_;
    std::unique_ptr<wheelleg::LegLinkage> linkage_r_;

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    int base_free_joint_id_ = -1;
    int base_body_id_ = -1;
    int l_wheel_body_id_ = -1;
    int r_wheel_body_id_ = -1;
    int l_wheel_geom_id_ = -1;
    int r_wheel_geom_id_ = -1;
    int l_thigh_body_id_ = -1;
    int l_shank_body_id_ = -1;
    int r_thigh_body_id_ = -1;
    int r_shank_body_id_ = -1;
    int base_dof_adr_ = -1;
    int base_qpos_adr_ = -1;
    std::array<int, 12> leg_lock_ids_{};
    std::array<double, 12> leg_lock_qpos_{};
    std::array<int, 12> leg_lock_eq_ids_{};
    std::array<int, 4> closure_eq_ids_{};
    static constexpr std::array<const char*, 4> kClosureEqNames = {"L_P", "L_E", "R_P", "R_E"};
    int fix_base_z_eq_ = -1;
    int fix_base_planar_eq_ = -1;
    int lock_zgantry_eq_ = -1;
    int planar_slide_y_id_ = -1;
    int planar_slide_z_id_ = -1;
    int planar_pitch_id_ = -1;
    int zgantry_slide_z_id_ = -1;
    bool lock_leg_ = false;
    bool fix_base_ = false;
    bool fix_base_hold_z_ = true;
    bool fix_base_planar_ = false;
    double fix_base_release_time_ = -1.0;
    std::array<int, 8> joint_ids_{};
    std::array<int, 6> actuator_ids_{};

    std::string model_path_;
    double dt_ = 0.001;
    double initial_height_ = 0.4008;

    double hip_kp_ = 400.0, hip_kd_ = 8.0;
    double crank_kp_ = 150.0, crank_kd_ = 2.0;
    double wheel_kp_ = 20.0;
    double open_loop_wheel_torque_ = std::numeric_limits<double>::quiet_NaN();
    bool open_loop_wheel_same_sign_ = false;
    double open_loop_roll_bias_ = std::numeric_limits<double>::quiet_NaN();
    double open_loop_wheel_start_ = 0.0;
    double wheel_velocity_damping_ = 0.0;
    double disturbance_force_ = 0.0;
    double disturbance_start_ = -1.0;
    double disturbance_duration_ = 0.0;
    bool gravity_feedforward_ = true;
    bool gravity_ff_analytic_ = true;
    bool gas_spring_enabled_ = true;
    double gas_spring_sign_ = 1.0;
    double gas_spring_full_extension_ = 0.232;
    double gas_spring_stroke_ = 0.08;
    double gas_spring_c0_ = 280.0;
    double gas_spring_c2_ = 122.7359;
    double gas_spring_c3_ = 53.9698;
    double gas_spring_length_ = 0.0;
    double gas_spring_compression_ = 0.0;
    double gas_spring_force_ = 0.0;
    double total_mass_ = 13.702;
    double gravity_ = 9.81;
    double dbg_hip_torque_ = 0.0;
    double dbg_crank_torque_ = 0.0;
    double dbg_wheel_torque_ = 0.0;
    double dbg_wheel_torque_r_ = 0.0;
    double dbg_wheel_friction_limit_l_ = 0.0;
    double dbg_wheel_friction_limit_r_ = 0.0;

    double joint_damping_ = 0.1;
    double hip_armature_ = 5.0e-3;
    double leg_frictionloss_ = 1.05;
    double wheel_armature_ = 4.7e-3;
    double wheel_joint_damping_ = 1.0e-3;
    double wheel_frictionloss_ = 9.0e-2;
    double wheel_speed_limit_ = 49.1;
    double constraint_timeconst_ = 0.002;
    double constraint_dampratio_ = 1.0;
    double contact_timeconst_ = 0.002;

    double hip_torque_limit_ = 40.0;
    double crank_torque_limit_ = 40.0;
    double wheel_torque_limit_ = 4.88;
    double friction_coefficient_ = 2.0;
    double wheel_radius_ = 0.06;
    double wheel_friction_torque_limit_ = 0.0;
    double knee_rel_sign_ = -1.0;
    double knee_rel_offset_l_ = 0.0;
    double knee_rel_offset_r_ = 0.0;

    double hip_sign_ = -1.0;
    double hip_zero_ = std::numbers::pi;
    double knee_zero_l_ = 1.9198621771937625;
    double knee_zero_r_ = 1.9198621771937625;
    double knee_limit_min_ = 0.6981317007977318;
    double knee_limit_max_ = 1.9198621771937625;
    bool knee_hard_stop_ = true;
    double crank_limit_min_ = 0.0;
    double crank_limit_max_ = 0.0;
    double hip_motor_zero_ = 0.0;
    double crank_motor_zero_ = 0.0;
    double thigh_length_ = 0.21;
    double shank_length_ = 0.25005;
    double wheel_dir_ = 1.0;

    double state_pitch_sign_ = 1.0;
    double state_shank_sign_ = 1.0;
    double state_hip_sign_ = 1.0;
    double state_knee_sign_ = 1.0;
    double state_wheel_sign_ = 1.0;
    double state_yaw_sign_ = 1.0;
    double state_velocity_sign_ = 1.0;
    double state_distance_sign_ = 1.0;
    double state_roll_sign_ = 1.0;
    bool has_previous_roll_ = false;
    double previous_roll_ = 0.0;
    double body_distance_ = 0.0;

    bool has_previous_state_ = false;
    double previous_shank_ = 0.0;
    double previous_knee_ = 0.0;
    double previous_pitch_ = 0.0;
    bool has_previous_com_ = false;
    std::array<double, 3> previous_com_{0.0, 0.0, 0.0};
    bool has_previous_yaw_ = false;
    double previous_yaw_ = 0.0;
    double unwrapped_yaw_ = 0.0;

    double viewer_state_rate_ = 100.0;
    std::string state_topic_ = "/wheelleg/simulation/state";
    std::uint64_t publish_period_steps_ = 0;

    bool viewer_enabled_ = true;
    std::string viewer_script_;
    std::string viewer_track_body_ = "base_link";
    bool viewer_follow_ = true;
    double viewer_distance_ = 2.5;
    double viewer_azimuth_ = 90.0;
    double viewer_elevation_ = -20.0;
    pid_t viewer_pid_ = -1;

    bool reference_ready_warned_ = false;
    std::uint64_t step_count_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;

    OutputInterface<double> state_wheel_velocity_;
    OutputInterface<double> state_shank_angle_;
    OutputInterface<double> state_shank_velocity_;
    OutputInterface<double> state_knee_angle_;
    OutputInterface<double> state_knee_velocity_;
    OutputInterface<double> state_pitch_;
    OutputInterface<double> state_pitch_velocity_;
    OutputInterface<double> state_hip_angle_;
    OutputInterface<double> state_base_height_;
    OutputInterface<double> state_leg_length_;
    OutputInterface<double> state_leg_angle_;
    OutputInterface<double> state_leg_length_l_;
    OutputInterface<double> state_leg_length_r_;
    OutputInterface<double> state_leg_angle_l_;
    OutputInterface<double> state_leg_angle_r_;
    OutputInterface<double> state_com_offset_;
    OutputInterface<double> state_com_x_;
    OutputInterface<double> state_com_z_;
    OutputInterface<double> state_yaw_;
    OutputInterface<double> state_yaw_rate_;
    OutputInterface<double> state_roll_;
    OutputInterface<double> state_roll_rate_;
    OutputInterface<double> state_velocity_;
    OutputInterface<double> state_distance_;

    WheelContact l_contact_;
    WheelContact r_contact_;
    OutputInterface<double> state_contact_normal_l_;
    OutputInterface<double> state_contact_normal_r_;
    OutputInterface<double> state_slip_l_;
    OutputInterface<double> state_slip_r_;
    OutputInterface<double> state_friction_utilization_l_;
    OutputInterface<double> state_friction_utilization_r_;
    OutputInterface<double> state_friction_torque_limit_l_;
    OutputInterface<double> state_friction_torque_limit_r_;
    OutputInterface<double> state_wheel_torque_l_;
    OutputInterface<double> state_wheel_torque_r_;

};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegSimulation, rmcs_executor::Component)
