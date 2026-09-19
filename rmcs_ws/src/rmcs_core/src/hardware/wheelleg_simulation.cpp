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

#include "controller/wheelleg/leg_kinematics.hpp"
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

        // Emitted here (not on the command partner) so the trajectory controller,
        // which consumes them, does not form a dependency cycle with the
        // component that consumes its references.
        register_output("/wheelleg/command/velocity", command_velocity_, 0.0);
        register_output("/wheelleg/command/acceleration", command_acceleration_, 0.0);
        register_output("/wheelleg/command/leg_length", command_leg_length_, kNaN);

        setup_state_publisher_();

        initialize_pose_();
        step_count_ = 0;
        info("MuJoCo wheel-leg simulation ready ({} actuators, {} dof)", model_->nu, model_->nv);
        info(
            "motor torque limits: hip {:.1f} Nm, crank {:.1f} Nm, wheel {:.1f} Nm "
            "(motor x 268/17)",
            hip_torque_limit_, crank_torque_limit_, wheel_torque_limit_);
        info(
            "wheel friction torque limit: {:.2f} Nm (mu={:.2f}, static weight, r={:.3f} m); "
            "effective wheel limit {:.2f} Nm",
            wheel_friction_torque_limit_, friction_coefficient_, wheel_radius_,
            std::min(wheel_torque_limit_, wheel_friction_torque_limit_));

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

        // Commissioning disturbance: a forward force impulse on the base.
        if (base_body_id_ >= 0) {
            const double t = step_count_ * dt_;
            const bool active = disturbance_start_ >= 0.0 && t >= disturbance_start_
                             && t < disturbance_start_ + disturbance_duration_;
            data_->xfrc_applied[base_body_id_ * 6 + 1] = active ? disturbance_force_ : 0.0;
        }

        mj_step(model_, data_);
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
            info(
                "t={:.1f} L0L={:.4f} L0R={:.4f} aL={:.4f} aR={:.4f} d={:.4f} pitch={:.4f} "
                "tauH={:.2f} tauC={:.2f} tauW={:.2f} cH={:.1f} cC={:.1f} dqC={:.3f} "
                "bz={:.3f} az={:.3f} by={:.4f} dqW={:.3f} dqWR={:.3f} cx={:.4f}",
                step_count_ * dt_, leg_l[0], leg_r[0], leg_l[1], leg_r[1], com[1] - axle_y,
                body_pitch_(), dbg_hip_torque_, dbg_crank_torque_, dbg_wheel_torque_,
                data_->ctrl[actuator_ids_[kActuatorHipL]], data_->ctrl[actuator_ids_[kActuatorCrankL]],
                joint_velocity_(kLCrank), data_->qpos[base_qpos_adr_ + 2],
                data_->xanchor[3 * joint_ids_[kLJoint3] + 2], data_->qpos[base_qpos_adr_ + 1],
                joint_velocity_(kLJoint3), joint_velocity_(kRJoint3), com[1]);
        }

        *command_velocity_ = command_velocity_target_;
        *command_acceleration_ = command_acceleration_target_;
        *command_leg_length_ = command_leg_length_target_;

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
        sync_kd_ = node::param_or("sync_kd", 0.0);
        sync_kp_ = node::param_or("sync_kp", 0.0);
        direct_hip_kd_ = node::param_or("direct_hip_kd", 0.0);
        direct_crank_kd_ = node::param_or("direct_crank_kd", 0.0);
        knee_kp_ = node::param_or("knee_kp", 800.0);
        knee_kd_ = node::param_or("knee_kd", 40.0);
        wheel_kp_ = node::param_or("wheel_kp", 20.0);
        open_loop_wheel_torque_ =
            node::param_or("open_loop_wheel_torque", std::numeric_limits<double>::quiet_NaN());
        open_loop_wheel_same_sign_ = node::param_or("open_loop_wheel_same_sign", false);
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
        constraint_timeconst_ = node::param_or("constraint_timeconst", 0.002);
        constraint_dampratio_ = node::param_or("constraint_dampratio", 1.0);
        contact_timeconst_ = node::param_or("contact_timeconst", 0.002);
        // The crank joint has a small reflected inertia, so it needs its own
        // (much lower) PD gains than the serial knee it replaced.
        crank_kp_ = node::param_or("crank_kp", 150.0);
        crank_kd_ = node::param_or("crank_kd", 2.0);
        const double wheel_motor_torque_limit =
            node::param_or("wheel_motor_torque_limit", 3.0);
        const double wheel_gear_ratio = node::param_or("wheel_gear_ratio", 268.0 / 17.0);
        wheel_torque_limit_ = wheel_motor_torque_limit * wheel_gear_ratio;

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
        knee_zero_l_ = node::param_or("knee_zero_l", 0.784469);
        knee_zero_r_ = node::param_or("knee_zero_r", 0.784469);

        // Sign of the wheel command: target qdot_L3 = -wheel_dir * omega_ref.
        wheel_dir_ = node::param_or("wheel_dir", 1.0);

        state_pitch_sign_ = node::param_or("state_pitch_sign", 1.0);
        state_shank_sign_ = node::param_or("state_shank_sign", 1.0);
        // V5.0 reports the interior knee angle with the controller's -1 branch,
        // i.e. negative when bent.
        state_knee_sign_ = node::param_or("state_knee_sign", -1.0);
        state_wheel_sign_ = node::param_or("state_wheel_sign", 1.0);

        command_velocity_target_ = node::param_or("command_velocity", 0.0);
        command_acceleration_target_ = node::param_or("command_acceleration", 0.0);
        command_leg_length_target_ = node::param_or("command_leg_length", kNaN);

        viewer_state_rate_ = node::param_or("viewer_state_rate", 100.0);
        state_topic_ =
            node::param_or<std::string>("state_topic", "/wheelleg/simulation/state");
        viewer_enabled_ = node::param_or("viewer", true);
        viewer_script_ = node::param_or<std::string>("viewer_script", std::string{});

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

        std::vector<std::string> arguments{
            script, model_path_, "--topic", state_topic_};
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
        for (const char* name :
             {"L_joint1", "R_joint1", "L_joint2", "R_jonit2", "L_crank", "R_crank", "LL_joint2",
              "LL_joint3", "LL_link4", "RR_joint2", "RR_joint3", "RR_joint4", "L_joint3",
              "R_joint3"}) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, name);
            if (id >= 0)
                model_->dof_damping[model_->jnt_dofadr[id]] = joint_damping_;
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

        // Start at a V5.0 standing pose (hip height ~0.39 m, leg near vertical):
        // the actuated hip/crank plus the passive six-bar closure solved at that
        // configuration (models/wheelleg/calibrate_kinematics.py).
        const std::array<std::pair<const char*, double>, 12> standing_pose = {{
            {"L_joint1", -0.60000},
            {"L_crank", 0.50000},
            {"L_joint2", 0.83313},
            {"LL_joint2", -0.78961},
            {"LL_joint3", 0.53888},
            {"LL_link4", -0.92236},
            {"R_joint1", -0.60000},
            {"R_crank", 0.50000},
            {"R_jonit2", 0.84273},
            {"RR_joint2", -0.79012},
            {"RR_joint3", 0.54198},
            {"RR_joint4", -0.92513},
        }};
        for (const auto& [name, value] : standing_pose) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, name);
            if (id >= 0)
                data_->qpos[model_->jnt_qposadr[id]] = value;
        }
        mj_forward(model_, data_);

        leg_lock_ids_.fill(-1);
        for (std::size_t i = 0; i < standing_pose.size(); ++i) {
            const int id = mj_name2id(model_, mjOBJ_JOINT, standing_pose[i].first);
            leg_lock_ids_[i] = id;
            leg_lock_qpos_[i] = id >= 0 ? data_->qpos[model_->jnt_qposadr[id]] : 0.0;

            // Point the joint-lock equality at the standing angle.
            const std::string lock_name = std::string{"lock_"} + standing_pose[i].first;
            leg_lock_eq_ids_[i] = mj_name2id(model_, mjOBJ_EQUALITY, lock_name.c_str());
            if (leg_lock_eq_ids_[i] >= 0)
                model_->eq_data[leg_lock_eq_ids_[i] * mjNEQDATA] = leg_lock_qpos_[i];
        }
        for (int i = 0; i < 7; ++i)
            base_hold_qpos_[static_cast<std::size_t>(i)] = data_->qpos[base_qpos + i];

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
        // Third column of the body rotation is the body "up" axis in world.
        return std::atan2(rotation[5], rotation[8]);
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
                std::clamp(knee_rel, linkage.knee_min(), linkage.knee_max());
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

        if (command.per_leg_torque_valid) {
            // Per-leg direct torque: each leg gets its own command (no shared
            // interface, so no antisymmetric mode).
            hip_torque_l = command.hip_torque_l;
            hip_torque_r = command.hip_torque_r;
            crank_torque_l = command.crank_torque_l;
            crank_torque_r = command.crank_torque_r;

            if (direct_hip_kd_ > 0.0) {
                hip_torque_l -= direct_hip_kd_ * joint_velocity_(kLJoint1);
                hip_torque_r -= direct_hip_kd_ * joint_velocity_(kRJoint1);
            }
            if (direct_crank_kd_ > 0.0) {
                crank_torque_l -= direct_crank_kd_ * joint_velocity_(kLCrank);
                crank_torque_r -= direct_crank_kd_ * joint_velocity_(kRCrank);
            }
        } else if (command.leg_torque_valid) {
            // Direct reduced-joint torque (执行路线2 §8). V5.0 hip/crank mirror
            // on the same sign, so both legs receive the same command.
            hip_torque_l = command.hip_torque;
            crank_torque_l = command.crank_torque;
            hip_torque_r = command.hip_torque;
            crank_torque_r = command.crank_torque;

            // Differential sync damping: the direct-torque interface commands
            // both legs identically, so the antisymmetric mode is unactuated.
            // Damp the L/R velocity difference to keep the legs together.
            if (sync_kp_ > 0.0 || sync_kd_ > 0.0) {
                const double hip_sync =
                    sync_kp_ * (joint_position_(kLJoint1) - joint_position_(kRJoint1))
                    + sync_kd_ * (joint_velocity_(kLJoint1) - joint_velocity_(kRJoint1));
                const double crank_sync =
                    sync_kp_ * (joint_position_(kLCrank) - joint_position_(kRCrank))
                    + sync_kd_ * (joint_velocity_(kLCrank) - joint_velocity_(kRCrank));
                hip_torque_l -= hip_sync;
                hip_torque_r += hip_sync;
                crank_torque_l -= crank_sync;
                crank_torque_r += crank_sync;
            }

            // Motor velocity damping (back-EMF-like): the direct-torque path
            // skips the joint PD, so add explicit viscous damping on each joint
            // to damp the leg's structural mode (~5-8 Hz).
            if (direct_hip_kd_ > 0.0) {
                hip_torque_l -= direct_hip_kd_ * joint_velocity_(kLJoint1);
                hip_torque_r -= direct_hip_kd_ * joint_velocity_(kRJoint1);
            }
            if (direct_crank_kd_ > 0.0) {
                crank_torque_l -= direct_crank_kd_ * joint_velocity_(kLCrank);
                crank_torque_r -= direct_crank_kd_ * joint_velocity_(kRCrank);
            }
        } else {
            const double crank_l = crank_target(
                *linkage_l_, knee_rel_offset_l_, linkage_l_->assembly_crank(), kLJoint1);
            const double crank_r =
                crank_target(*linkage_r_, knee_rel_offset_r_, linkage_r_->assembly_crank(), kRJoint1);

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
        if (std::isfinite(open_loop_wheel_torque_)) {
            wheel_torque_l = open_loop_wheel_torque_;
            wheel_torque_r = open_loop_wheel_same_sign_ ? open_loop_wheel_torque_
                                                         : -open_loop_wheel_torque_;
        }

        // A5 acceleration feedforward: F = M*a -> tau = M*a*r (commissioning test).
        if (std::abs(command_acceleration_target_) > 1e-6) {
            const double accel_torque =
                total_mass_ * command_acceleration_target_ * wheel_radius_;
            wheel_torque_l += accel_torque;
            wheel_torque_r += accel_torque;
        }

        // Wheel torque is capped by both the motor and the ground friction.
        const double wheel_limit = std::min(wheel_torque_limit_, wheel_friction_torque_limit_);

        set_torque_(kActuatorHipL, hip_torque_l, hip_torque_limit_);
        set_torque_(kActuatorCrankL, crank_torque_l, crank_torque_limit_);
        set_torque_(kActuatorWheelL, wheel_torque_l, wheel_limit);
        set_torque_(kActuatorHipR, hip_torque_r, hip_torque_limit_);
        set_torque_(kActuatorCrankR, crank_torque_r, crank_torque_limit_);
        set_torque_(kActuatorWheelR, wheel_torque_r, wheel_limit);

        dbg_hip_torque_ = hip_torque_l;
        dbg_crank_torque_ = crank_torque_l;
        dbg_wheel_torque_ = wheel_torque_l;
    }

    void set_torque_(std::size_t index, double torque, double limit) {
        data_->ctrl[actuator_ids_[index]] = std::clamp(torque, -limit, limit);
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

        // Task-space state (执行路线2 §5/§8): leg length / angle and the CoM
        // offset d = (C_veh - A)_forward (static stability condition of §6).
        const auto com = vehicle_com_();
        const double axle_y =
            0.5
            * (data_->xanchor[3 * joint_ids_[kLJoint3] + 1]
               + data_->xanchor[3 * joint_ids_[kRJoint3] + 1]);

        *state_leg_length_ = leg_length;
        *state_leg_angle_ = 0.5 * (leg_l[1] + leg_r[1]);
        *state_leg_length_l_ = leg_l[0];
        *state_leg_length_r_ = leg_r[0];
        *state_leg_angle_l_ = leg_l[1];
        *state_leg_angle_r_ = leg_r[1];
        *state_com_x_ = com[1];
        *state_com_z_ = com[2];
        *state_com_offset_ = com[1] - axle_y;
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
            // Direct reduced-joint torques (执行路线2 §8). When both are finite
            // the leg PD is bypassed and these are applied (mirrored to R).
            // In the closed-chain model the second channel is the crank motor.
            bool leg_torque_valid = false;
            double hip_torque = 0.0;
            double crank_torque = 0.0;
            // Per-leg direct torques (per-leg control).
            bool per_leg_torque_valid = false;
            double hip_torque_l = 0.0;
            double hip_torque_r = 0.0;
            double crank_torque_l = 0.0;
            double crank_torque_r = 0.0;
            bool per_leg_wheel_valid = false;
            double wheel_torque_l = 0.0;
            double wheel_torque_r = 0.0;
        };

        explicit WheelLegSimulationCommand(WheelLegSimulation& simulation)
            : simulation_(simulation) {
            register_input("/wheelleg/reference/hip_angle", hip_angle_, false);
            register_input("/wheelleg/reference/knee_angle", knee_angle_, false);
            register_input("/wheelleg/reference/wheel_velocity", wheel_velocity_, false);
            register_input("/wheelleg/control/wheel_torque", wheel_torque_, false);
            register_input("/wheelleg/control/hip_torque", hip_torque_, false);
            register_input("/wheelleg/control/crank_torque", crank_torque_, false);
            register_input("/wheelleg/control/hip_torque_l", hip_torque_l_, false);
            register_input("/wheelleg/control/hip_torque_r", hip_torque_r_, false);
            register_input("/wheelleg/control/crank_torque_l", crank_torque_l_, false);
            register_input("/wheelleg/control/crank_torque_r", crank_torque_r_, false);
            register_input("/wheelleg/control/wheel_torque_l", wheel_torque_l_, false);
            register_input("/wheelleg/control/wheel_torque_r", wheel_torque_r_, false);
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
            control_.leg_torque_valid = hip_torque_.ready() && crank_torque_.ready()
                                     && std::isfinite(*hip_torque_)
                                     && std::isfinite(*crank_torque_);
            if (control_.leg_torque_valid) {
                control_.hip_torque = *hip_torque_;
                control_.crank_torque = *crank_torque_;
            }
            control_.per_leg_torque_valid = hip_torque_l_.ready() && hip_torque_r_.ready()
                                         && crank_torque_l_.ready() && crank_torque_r_.ready()
                                         && std::isfinite(*hip_torque_l_)
                                         && std::isfinite(*hip_torque_r_)
                                         && std::isfinite(*crank_torque_l_)
                                         && std::isfinite(*crank_torque_r_);
            if (control_.per_leg_torque_valid) {
                control_.hip_torque_l = *hip_torque_l_;
                control_.hip_torque_r = *hip_torque_r_;
                control_.crank_torque_l = *crank_torque_l_;
                control_.crank_torque_r = *crank_torque_r_;
            }
            control_.per_leg_wheel_valid = wheel_torque_l_.ready() && wheel_torque_r_.ready()
                                        && std::isfinite(*wheel_torque_l_)
                                        && std::isfinite(*wheel_torque_r_);
            if (control_.per_leg_wheel_valid) {
                control_.wheel_torque_l = *wheel_torque_l_;
                control_.wheel_torque_r = *wheel_torque_r_;
            }
            control_.valid = (hip_angle_.ready() && knee_angle_.ready())
                          || control_.leg_torque_valid || control_.per_leg_torque_valid;
        }

        [[nodiscard]] bool reference_ready() const { return hip_angle_.ready(); }
        [[nodiscard]] const Control& control() const { return control_; }

    private:
        WheelLegSimulation& simulation_;

        InputInterface<double> hip_angle_;
        InputInterface<double> knee_angle_;
        InputInterface<double> wheel_velocity_;
        InputInterface<double> wheel_torque_;
        InputInterface<double> hip_torque_;
        InputInterface<double> crank_torque_;
        InputInterface<double> hip_torque_l_;
        InputInterface<double> hip_torque_r_;
        InputInterface<double> crank_torque_l_;
        InputInterface<double> crank_torque_r_;
        InputInterface<double> wheel_torque_l_;
        InputInterface<double> wheel_torque_r_;

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

    std::shared_ptr<WheelLegSimulationCommand> command_component_;

    std::unique_ptr<wheelleg::LegLinkage> linkage_l_;
    std::unique_ptr<wheelleg::LegLinkage> linkage_r_;

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    int base_free_joint_id_ = -1;
    int base_body_id_ = -1;
    int base_dof_adr_ = -1;
    int base_qpos_adr_ = -1;
    std::array<double, 7> base_hold_qpos_{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
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
    double sync_kd_ = 0.0;
    double sync_kp_ = 0.0;
    double direct_hip_kd_ = 0.0;
    double direct_crank_kd_ = 0.0;
    double knee_kp_ = 800.0, knee_kd_ = 40.0;
    double crank_kp_ = 150.0, crank_kd_ = 2.0;
    double wheel_kp_ = 20.0;
    double open_loop_wheel_torque_ = std::numeric_limits<double>::quiet_NaN();
    bool open_loop_wheel_same_sign_ = false;
    double wheel_velocity_damping_ = 0.0;
    double disturbance_force_ = 0.0;
    double disturbance_start_ = -1.0;
    double disturbance_duration_ = 0.0;
    bool gravity_feedforward_ = true;
    bool gravity_ff_analytic_ = true;
    double total_mass_ = 13.702;
    double gravity_ = 9.81;
    double dbg_hip_torque_ = 0.0;
    double dbg_crank_torque_ = 0.0;
    double dbg_wheel_torque_ = 0.0;

    double joint_damping_ = 0.1;
    double hip_armature_ = 5.0e-3;
    double constraint_timeconst_ = 0.002;
    double constraint_dampratio_ = 1.0;
    double contact_timeconst_ = 0.002;

    double hip_torque_limit_ = 40.0;
    double crank_torque_limit_ = 40.0;
    double wheel_torque_limit_ = 3.0 * 268.0 / 17.0;
    double friction_coefficient_ = 2.0;
    double wheel_radius_ = 0.06;
    double wheel_friction_torque_limit_ = 0.0;
    double knee_rel_sign_ = -1.0;
    double knee_rel_offset_l_ = 0.0;
    double knee_rel_offset_r_ = 0.0;

    double hip_sign_ = -1.0;
    double hip_zero_ = std::numbers::pi;
    double knee_zero_l_ = 0.784469;
    double knee_zero_r_ = 0.784469;
    double thigh_length_ = 0.21;
    double shank_length_ = 0.25005;
    double wheel_dir_ = 1.0;

    double state_pitch_sign_ = 1.0;
    double state_shank_sign_ = 1.0;
    double state_knee_sign_ = 1.0;
    double state_wheel_sign_ = 1.0;

    bool has_previous_state_ = false;
    double previous_shank_ = 0.0;
    double previous_knee_ = 0.0;
    double previous_pitch_ = 0.0;

    double command_velocity_target_ = 0.0;
    double command_acceleration_target_ = 0.0;
    double command_leg_length_target_ = kNaN;

    double viewer_state_rate_ = 100.0;
    std::string state_topic_ = "/wheelleg/simulation/state";
    std::uint64_t publish_period_steps_ = 0;

    bool viewer_enabled_ = true;
    std::string viewer_script_;
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
    OutputInterface<double> state_leg_length_;
    OutputInterface<double> state_leg_angle_;
    OutputInterface<double> state_leg_length_l_;
    OutputInterface<double> state_leg_length_r_;
    OutputInterface<double> state_leg_angle_l_;
    OutputInterface<double> state_leg_angle_r_;
    OutputInterface<double> state_com_offset_;
    OutputInterface<double> state_com_x_;
    OutputInterface<double> state_com_z_;

    OutputInterface<double> command_velocity_;
    OutputInterface<double> command_acceleration_;
    OutputInterface<double> command_leg_length_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegSimulation, rmcs_executor::Component)
