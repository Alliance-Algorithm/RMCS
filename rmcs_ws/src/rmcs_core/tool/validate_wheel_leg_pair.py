#!/usr/bin/env python3
"""Run the production pair servo through the V5 PhysX closed chain, offline.

Use the Python in the installed Isaac Lab environment. Only reset writes passive
positions; every later state is integrated by PhysX from motor efforts. The DM
velocity PI below is an explicit test assumption, not an identified motor model.
"""
import argparse
import ctypes
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--bundle", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--device", default="cuda:0")
parser.add_argument("--envs", type=int, default=16)
parser.add_argument("--seconds-per-target", type=float, default=6.0)
parser.add_argument("--dm-feedback", action="store_true", help="Run CAN frames through actual DmMotor and controller component in ROS container")
parser.add_argument("--container", default="rmcs_devcontainer-rmcs-develop-1")
parser.add_argument("--direction-comparison", action="store_true", help="Four identical-drive cases: production pair routing versus an independent-short-arc counterexample")
args = parser.parse_args()
if args.direction_comparison:
    args.dm_feedback = True
    args.envs = 4

import numpy as np
import yaml

core = Path(__file__).resolve().parents[1]
config = yaml.safe_load((core.parent / "rmcs_bringup/config/wheel-leg-infantry-rl.yaml").read_text())
p = config["wheel_leg_joint_velocity_controller"]["ros__parameters"]
parameters = np.array([p[n] for n in ("angle_kp", "max_joint_velocity", "max_joint_acceleration",
                                     "min_motor_difference", "max_motor_difference",
                                     "motor_difference_margin", "motor_difference_tolerance")], dtype=np.float64)
hardware = config["wheel_leg_infantry_rl"]["ros__parameters"]
joint_keys = ["left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"]
offsets = np.array([hardware[n + "_angle_offset"] for n in joint_keys])
consumer = config["wheel_leg_rl_consumer"]["ros__parameters"]
spec = json.loads((args.bundle / "model_spec.json").read_text())
manifest = json.loads((args.bundle / "manifest.json").read_text())
sys.path.insert(0, str(args.bundle / "tools"))
from build_v5_closedchain import closure_error, solve_pose
from v5_mechanism import fk

active = ["L_joint1", "LL_joint1", "R_joint1", "RR_joint1"]
zero_inner = math.pi - 2.3573

def closed(inner, hip):
    q2 = math.radians(inner) - zero_inner
    seed = dict(spec["nominal_joint_pos"])
    for name in ("L_joint1", "LL_joint1"):
        seed[name] += hip - .42
    for name in ("R_joint1", "RR_joint1"):
        seed[name] -= hip - .42
    return solve_pose(spec, {"L_joint1": hip, "L_joint2": q2, "L_joint3": 0.,
                             "R_joint1": -hip, "R_jonit2": -q2, "R_joint3": 0.}, seed)

qcal = solve_pose(spec, dict(zip(active, offsets)) | {"L_joint3": 0., "R_joint3": 0.}, closed(105., -1.6))
frames = fk(spec, qcal)
thigh = frames["L_link2"][:3, 3] - frames["L_link1"][:3, 3]
calibration = {"offsets": offsets.tolist(), "inner_angle_deg": [math.degrees(zero_inner + qcal["L_joint2"]),
               math.degrees(zero_inner - qcal["R_jonit2"])], "left_thigh_direction": (thigh / np.linalg.norm(thigh)).tolist()}
geometry = []
for inner in (30., 45., 70., 90., 100., 105., 110., 115., 120.):
    q = closed(inner, .42)
    geometry.append({"inner_deg": inner, "left_difference": q["L_joint1"] - q["LL_joint1"],
                     "right_difference": q["RR_joint1"] - q["R_joint1"],
                     "max_closure_error_m": float(np.max(np.abs(closure_error(spec, q))))})

# Seeded random headings plus both sides of the calibration seam.
rng = np.random.default_rng(20260926)
headings = rng.uniform(-math.pi, math.pi, args.envs)
headings[:min(4, args.envs)] = [-1.6 - .05, -1.6 + .05, -math.pi + .01, math.pi - .01][:min(4, args.envs)]
initial_angles = np.resize(np.array([100., 105., 110., 120.]), args.envs)
if args.direction_comparison:
    headings = np.array([-2.1, -2.1, -1.6, -1.6])
    initial_angles = np.full(4, 105.)
elif args.dm_feedback:
    if args.envs != 16:
        raise ValueError("DM feedback suite requires 16 paired unsigned/signed cases")
    headings = np.repeat(np.array([-1.6, -2.1, -1.65, -1.55, -math.pi + .01, math.pi - .01, -2.9, 2.6]), 2)
    initial_angles = np.repeat(np.array([105., 105., 100., 110., 105., 105., 100., 110.]), 2)
initial = [closed(float(angle), float(hip)) for angle, hip in zip(initial_angles, headings)]
if args.direction_comparison:
    initial[2:] = [dict(qcal), dict(qcal)]
elif args.dm_feedback:
    initial[:2] = [dict(qcal), dict(qcal)]

with tempfile.TemporaryDirectory(prefix="rmcs-v5-servo-") as temp:
    library_path = Path(temp) / "pair.so"
    subprocess.run(["g++", "-std=c++23", "-shared", "-fPIC", "-O2", "-I", str(core / "src"),
                    str(core / "test/wheel_leg_pair_sim_bridge.cpp"), "-o", str(library_path)], check=True)
    library = ctypes.CDLL(str(library_path))
    pointer = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    library.wheel_leg_pair_step.argtypes = [pointer, pointer, pointer, pointer, ctypes.c_int, ctypes.c_double]
    library.wheel_leg_pair_step.restype = ctypes.c_int

    dm_bridge = None
    if args.dm_feedback:
        from wheel_leg_dm_feedback import DmFeedbackBridge
        args.output.parent.mkdir(parents=True, exist_ok=True)
        dm_bridge = DmFeedbackBridge(args.envs, offsets, args.container, args.output.with_suffix(".bridge.log"), args.direction_comparison)

    from isaaclab.app import AppLauncher
    launcher = AppLauncher({"headless": True, "device": args.device, "enable_cameras": False})
    try:
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation, ArticulationCfg
        from isaaclab.actuators import IdealPDActuatorCfg
        from pxr import UsdGeom, UsdPhysics

        torch.set_num_threads(4)
        dt = .001
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            device=args.device, dt=dt, render_interval=100,
            physx=sim_utils.PhysxCfg(enable_external_forces_every_iteration=True)))
        for i in range(args.envs):
            UsdGeom.Xform.Define(sim.stage, f"/World/envs/env_{i}").AddTranslateOp().Set((i * 3., 0., 0.))
        springs = manifest["spring_joint_names"]
        wheels = ["L_joint3", "R_joint3"]
        passive = [n for n in manifest["tree_joint_names"] if n not in active + springs + wheels]
        robot = Articulation(ArticulationCfg(
            prim_path="/World/envs/env_.*/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=str(args.bundle / "robot.usda"),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True,
                    enabled_self_collisions=False, solver_position_iteration_count=32, solver_velocity_iteration_count=8)),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0., 0., 1.), joint_pos=manifest["nominal_joint_pos"], joint_vel={".*": 0.}),
            actuators={
                "active": IdealPDActuatorCfg(joint_names_expr=active, stiffness=0., damping=0.,
                    effort_limit=40., effort_limit_sim=40., velocity_limit_sim=45., armature=.02),
                "passive": IdealPDActuatorCfg(joint_names_expr=passive, stiffness=0., damping=.002,
                    effort_limit=100., effort_limit_sim=100., velocity_limit_sim=1000.),
                "wheel": IdealPDActuatorCfg(joint_names_expr=wheels, stiffness=0., damping=.2,
                    effort_limit=4., effort_limit_sim=4., velocity_limit_sim=1000.),
                "spring": IdealPDActuatorCfg(joint_names_expr=springs, stiffness=0., damping=.002,
                    effort_limit=1000., effort_limit_sim=1000., velocity_limit_sim=100.)},
            soft_joint_pos_limit_factor=1.0))
        loops = [prim for prim in sim.stage.Traverse() if prim.IsA(UsdPhysics.SphericalJoint)]
        assert len(loops) == 6 * args.envs
        for prim in sim.stage.Traverse():
            if prim.GetName() in ("L_joint2", "R_jonit2"):
                limits = [30. - math.degrees(zero_inner), 120. - math.degrees(zero_inner)]
                if prim.GetName() == "R_jonit2":
                    limits = [-limits[1], -limits[0]]
                joint = UsdPhysics.RevoluteJoint(prim)
                joint.GetLowerLimitAttr().Set(limits[0])
                joint.GetUpperLimitAttr().Set(limits[1])
            if prim.GetName() in springs:
                UsdPhysics.PrismaticJoint(prim).GetLowerLimitAttr().Set(-.012)
        sim.reset()
        names = robot.joint_names
        active_ids = [names.index(n) for n in active]
        spring_ids = [names.index(n) for n in springs]
        initial_q = torch.tensor([[q[n] for n in names] for q in initial], device=args.device, dtype=torch.float32)
        robot.write_joint_state_to_sim(initial_q, torch.zeros_like(initial_q))
        robot.reset()
        robot.update(dt)
        effort = torch.zeros_like(initial_q)
        integral = torch.zeros((args.envs, 4), device=args.device)
        commands = np.zeros((args.envs, 4), dtype=np.float64)
        s0 = torch.tensor([spec["spring_binding"][n]["compression_at_q_zero_m"] for n in springs], device=args.device)
        gas_enabled = torch.tensor([[float((i // 2) % 2 if args.dm_feedback else i % 2)] for i in range(args.envs)], device=args.device)
        if args.direction_comparison:
            gas_enabled.zero_()
        gain_scale = np.ones((args.envs, 4))
        if dm_bridge is not None:
            gain_scale[dm_bridge.stressed] = [1.0, .6, 1.2, .7]
        kp_velocity = torch.tensor(10. * gain_scale, device=args.device, dtype=torch.float32)
        ki_velocity = torch.tensor(50. * gain_scale, device=args.device, dtype=torch.float32)
        last_torque = np.zeros((args.envs, 4), dtype=np.float64)
        simulation_time = 1.
        fault_steps = 0
        faults_per_environment = np.zeros(args.envs, dtype=np.int64)
        stall_seconds = np.zeros((args.envs, 2))
        max_stall_seconds = 0.
        trace = []
        trace_times = []
        max_gap = 0.
        max_command = 0.
        max_command_step = 0.
        invalid_feedback_steps = 0
        max_torque = 0.
        inner_min, inner_max = 180., 0.
        seam_crossings = np.zeros(4, dtype=np.int64)
        previous_raw = None
        phases = []
        # Fixed modes, nominal policy pose, and a return through motor raw=0.
        targets = [consumer["urdf_zero_leg_position"], consumer["calibrated_zero_leg_position"],
                   consumer["policy_leg_default_position"],
                   (offsets + np.array([.08, .08, -.08, -.08])).tolist(),
                   (offsets + np.array([-.08, -.08, .08, .08])).tolist()]
        phases_to_run = [(f"target_{i}", value, True, args.seconds_per_target) for i, value in enumerate(targets)]
        if dm_bridge is not None:
            phases_to_run = [("power_on_disabled", None, False, .02),
                             ("enable_hold", None, True, args.seconds_per_target),
                             ("urdf_zero", targets[0], True, args.seconds_per_target),
                             ("calibrated_zero", targets[1], True, args.seconds_per_target),
                             ("policy_nominal", targets[2], True, args.seconds_per_target),
                             ("calibration_positive_side", targets[3], True, args.seconds_per_target),
                             ("calibration_negative_side", targets[4], True, args.seconds_per_target),
                             ("explicit_disable", None, False, .02),
                             ("rearm_hold", None, True, args.seconds_per_target),
                             ("return_calibration", targets[1], True, args.seconds_per_target)]
        if args.direction_comparison:
            phases_to_run = [("power_on_disabled", None, False, .02),
                             ("enable_hold", None, True, 1.),
                             ("urdf_zero", targets[0], True, args.seconds_per_target)]
        reset_count = 0
        for phase_index, (phase_name, target, requested, duration) in enumerate(phases_to_run):
            phase_start = simulation_time - 1.
            if not requested:
                reset_count += 1
            if target is None:
                target_array = np.ascontiguousarray(robot.data.joint_pos[:, active_ids].cpu().numpy(), dtype=np.float64)
            else:
                target_array = np.ascontiguousarray(np.tile(target, (args.envs, 1)), dtype=np.float64)
            phase_saturation = 0
            for tick in range(round(duration / dt)):
                q = robot.data.joint_pos
                dq = robot.data.joint_vel
                q_active = q[:, active_ids].cpu().numpy().astype(np.float64)
                old_commands = commands.copy()
                if dm_bridge is not None:
                    measured, measured_velocity, commands, applied, active_flags, healthy_flags = dm_bridge.step(
                        simulation_time, q_active, dq[:, active_ids].cpu().numpy().astype(np.float64),
                        last_torque, target_array, requested, reset_count)
                    fault_steps += int((~healthy_flags).sum()) if requested else 0
                    faults_per_environment += ~healthy_flags if requested else 0
                    enabled_tensor = torch.tensor(dm_bridge.status == 1, device=args.device)
                else:
                    raw = np.remainder(offsets - q_active, 2. * math.pi)
                    if previous_raw is not None:
                        seam_crossings += (np.abs(raw - previous_raw) > math.pi).sum(axis=0)
                    previous_raw = raw.copy()
                    quantized = np.rint((raw + 12.5) / 25. * 65535.) / 65535. * 25. - 12.5
                    measured = np.ascontiguousarray((offsets - quantized + math.pi) % (2. * math.pi) - math.pi)
                    invalid_feedback_steps += library.wheel_leg_pair_step(
                        measured, target_array, commands, parameters, 2 * args.envs, dt)
                    applied = commands
                    enabled_tensor = torch.ones((args.envs, 1), device=args.device, dtype=torch.bool)
                    active_flags = np.ones(args.envs, dtype=bool)
                max_command = max(max_command, float(np.max(np.abs(commands))))
                max_command_step = max(max_command_step, float(np.max(np.abs(commands - old_commands))))
                velocity_error = torch.tensor(applied, device=args.device, dtype=torch.float32) - dq[:, active_ids]
                proposed_integral = (integral + dt * velocity_error).clamp(-.8, .8)
                proposed_torque = kp_velocity * velocity_error + ki_velocity * proposed_integral
                integral = torch.where((proposed_torque.abs() <= 40.) | (proposed_torque * velocity_error < 0.), proposed_integral, integral)
                integral *= enabled_tensor
                torque = (kp_velocity * velocity_error + ki_velocity * integral).clamp(-40., 40.) * enabled_tensor
                last_torque = torque.cpu().numpy().astype(np.float64)
                phase_saturation += int((np.abs(last_torque) >= 39.99).sum())
                phase_error = (target_array - q_active + np.pi) % (2. * np.pi) - np.pi
                pair_error = np.abs(phase_error).reshape(args.envs, 2, 2).max(axis=2)
                pair_speed = dq[:, active_ids].abs().cpu().numpy().reshape(args.envs, 2, 2).max(axis=2)
                pair_torque = np.abs(last_torque).reshape(args.envs, 2, 2).max(axis=2)
                stalled = active_flags[:, None] & (pair_error > .15) & (pair_speed < .03) & (pair_torque > 8.)
                stall_seconds = np.where(stalled, stall_seconds + dt, 0.)
                max_stall_seconds = max(max_stall_seconds, float(stall_seconds.max()))
                if dm_bridge is not None and tick % 20 == 0:
                    trace.append(np.stack([q_active, measured, target_array, commands, last_torque,
                                           dm_bridge.previous_raw, dq[:, active_ids].cpu().numpy()], axis=1).copy())
                    trace_times.append(simulation_time - 1.)
                effort.zero_()
                effort[:, active_ids] = torque
                compression = s0 - q[:, spring_ids]
                u = compression.clamp(0., .08) / .08
                effort[:, spring_ids] = gas_enabled * (280. + 122.735918491 * u ** 2 + 53.969804409 * u ** 3)
                robot.set_joint_effort_target(effort)
                robot.write_data_to_sim()
                sim.step(render=False)
                robot.update(dt)
                simulation_time += dt
                max_torque = max(max_torque, float(torque.abs().max()))
                inner = torch.stack((q[:, names.index("L_joint2")] + zero_inner,
                                     zero_inner - q[:, names.index("R_jonit2")]), dim=1) * (180. / math.pi)
                inner_min = min(inner_min, float(inner.min()))
                inner_max = max(inner_max, float(inner.max()))
                if tick % 50 == 0:
                    poses = robot.data.body_link_pose_w.cpu().numpy()
                    assert np.isfinite(poses).all(), "nonfinite PhysX body state"
                    def world(body, local):
                        pose = poses[:, robot.body_names.index(body), :]
                        vec = np.tile(local, (args.envs, 1))
                        cross = np.cross(pose[:, 4:7], vec)
                        return pose[:, :3] + vec + 2. * (pose[:, 3:4] * cross + np.cross(pose[:, 4:7], cross))
                    for constraint in spec["constraints"]:
                        gap = np.linalg.norm(world(constraint["body0"], constraint["local_pos0_m"])
                                             - world(constraint["body1"], constraint["local_pos1_m"]), axis=1)
                        max_gap = max(max_gap, float(gap.max()))
            final_q = robot.data.joint_pos[:, active_ids].cpu().numpy()
            error = (final_q - target_array + math.pi) % (2. * math.pi) - math.pi
            phases.append({"name": phase_name, "start_time_s": phase_start, "duration_s": duration,
                           "requested_enabled": requested, "target": target, "torque_saturation_joint_steps": phase_saturation, "max_motor_error_rad": float(np.abs(error).max()),
                           "per_environment_max_error_rad": np.abs(error).max(axis=1).tolist(),
                           "max_final_motor_velocity_rad_s": float(robot.data.joint_vel[:, active_ids].abs().max()),
                           "max_final_motor_torque_nm": float(np.abs(last_torque).max()),
                           "max_final_command_rad_s": float(np.abs(commands).max())})
            print("V5_PAIR_PHASE", json.dumps(phases[-1]), flush=True)
        report = {"method": "production C++ phase-pair velocity servo, quantized DM feedback, PhysX dynamics",
                  "device": args.device,
                  "isaaclab_version": importlib.metadata.version("isaaclab"),
                  "isaacsim_version": importlib.metadata.version("isaacsim"),
                  "source_hashes": {n: hashlib.sha256((args.bundle / n).read_bytes()).hexdigest()
                                    for n in ["source_urdf_v5.0.urdf", "model_spec.json", "robot.usda"]},
                  "controller_sha256": hashlib.sha256((core / "src/controller/chassis/wheel_leg_joint_pair_geometry.hpp").read_bytes()).hexdigest(),
                  "calibration": calibration, "geometry": geometry, "environments": args.envs,
                  "initial_hip_rad": headings.tolist(), "initial_inner_deg": initial_angles.tolist(),
                  "parameters": p, "dt": dt, "seconds_per_target": args.seconds_per_target,
                  "physx_external_forces_every_iteration": True,
                  "fixture": "fixed base, 19 bodies and 6 loop constraints per robot, gravity enabled, alternating gas springs on/off",
                  "motor_model_assumptions": {"velocity_kp": 10., "velocity_ki": 50., "torque_limit_nm": 40., "armature_kg_m2": .02},
                  "temporary_model_edits": {"inner_limits_deg": [30, 120], "spring_lower_limit_m": -.012},
                  "passive_pose_writes_after_reset": 0, "kinematic_solver_calls_during_physics": 0,
                  "invalid_feedback_steps": invalid_feedback_steps, "max_loop_gap_m": max_gap,
                  "inner_angle_range_deg": [inner_min, inner_max], "max_command_rad_s": max_command,
                  "max_command_step_rad_s": max_command_step, "max_motor_torque_nm": max_torque,
                  "calibration_zero_crossings": seam_crossings.tolist(), "phases": phases}
        report["passed"] = (invalid_feedback_steps == 0 and max_gap < .002
                            and inner_min > 29.5 and inner_max < 120.5
                            and max_command <= parameters[1] + 1e-6
                            and all(v["max_motor_error_rad"] < .04 for v in phases if v["requested_enabled"])
                            and (dm_bridge is not None or bool((seam_crossings > 0).all())))
        if dm_bridge is not None:
            report["method"] = "PhysX -> simulated DM encoder/CAN feedback -> production DmMotor -> actual WheelLegJointVelocityController component -> DmJointEnableSequence -> production VEL bytes -> simulated velocity PI -> PhysX"
            report["calibration_zero_crossings"] = dm_bridge.wrap_crossings[~dm_bridge.wrap_signed].sum(axis=0).tolist()
            report["invalid_feedback_steps"] = fault_steps
            report["calibrated_feedback"] = {
                "measured_hardware_trace_used": False,
                "encoder_resolution_bits": 14, "position_field_bits": 16, "velocity_field_bits": 12,
                "feedback_limits": {"P_MAX": 12.5, "V_MAX": 45., "T_MAX": 54.},
                "signed_wrap_per_environment": dm_bridge.wrap_signed.tolist(),
                "feedback_delay_ms": dm_bridge.feedback_delay.tolist(),
                "velocity_command_delay_ms": dm_bridge.command_delay.tolist(),
                "velocity_gain_scale": gain_scale.tolist(),
                "independent_short_arc_per_environment": dm_bridge.independent_short_arc.tolist(),
                "max_decode_phase_error_rad": dm_bridge.max_phase_error,
                "initial_and_zero_samples": dm_bridge.encoder_samples,
                "velocity_command_samples": dm_bridge.command_samples,
                "wrap_crossings_per_environment": dm_bridge.wrap_crossings.tolist(),
                "startup_nonzero_frames": dm_bridge.startup_nonzero_frames,
                "disable_commands_while_requested": dm_bridge.disable_commands_while_requested,
                "system_command_counts": dm_bridge.system_counts,
                "first_controller_fault": dm_bridge.first_fault,
                "unhealthy_environment_steps": fault_steps,
                "unhealthy_steps_per_environment": faults_per_environment.tolist(),
                "longest_stall_condition_s": max_stall_seconds,
                "stall_definition": "per leg: active, some position error >0.15rad, both motor speeds <0.03rad/s, some torque >8Nm",
            }
            report["implementation_hashes"] = {str(path.relative_to(core)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in [core / "src/hardware/device/dm_motor.hpp", core / "src/controller/chassis/wheel_leg_joint_velocity_controller.cpp",
                             core / "src/hardware/device/dm_joint_enable_sequence.hpp", core / "test/wheel_leg_dm_sim_bridge.cpp"]}
            report["passed"] &= (fault_steps == 0 and dm_bridge.startup_nonzero_frames == 0
                                 and dm_bridge.disable_commands_while_requested == 0
                                 and dm_bridge.max_phase_error < .0006 and max_stall_seconds < .5
                                 and bool((np.asarray(report["calibration_zero_crossings"]) > 0).all()))
            if args.direction_comparison:
                final_error = np.array(phases[-1]["per_environment_max_error_rad"])
                report["fixture"] = "fixed base, 19 bodies and 6 loop constraints per robot, gravity enabled, gas springs off"
                report["direction_comparison"] = {
                    "only_control_difference": "angular error selection; speed, acceleration, boundary limiter, PI gains, delays, geometry, gravity, encoder and offsets are identical",
                    "counterexample_is_historical_code_replay": False,
                    "cases": ["paired route, knee phase wraps", "independent arcs, knee phase wraps",
                              "paired route, calibration pose", "independent arcs, calibration pose"],
                    "production_passed": bool((final_error[[0, 2]] < .04).all() and (faults_per_environment[[0, 2]] == 0).all()),
                    "counterexample_reproduced": bool(final_error[1] > .15 and final_error[3] < .04),
                }
                report["passed"] = (report["direction_comparison"]["production_passed"]
                                    and report["direction_comparison"]["counterexample_reproduced"]
                                    and max_gap < .002 and inner_min > 29.5 and inner_max < 120.5
                                    and dm_bridge.startup_nonzero_frames == 0 and dm_bridge.max_phase_error < .0006)
            np.savez_compressed(args.output.with_suffix(".trace.npz"),
                                trace=np.array(trace), time_s=np.array(trace_times), sample_period_s=.020,
                                signals=np.array(["urdf", "decoded_feedback", "target", "command", "torque", "raw_encoder", "velocity"]))
            dm_bridge.close()
            dm_bridge = None
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print("V5_PAIR_RESULT", json.dumps({k: report[k] for k in ["passed", "max_loop_gap_m", "inner_angle_range_deg", "invalid_feedback_steps"]}), flush=True)
        if not report["passed"]:
            raise RuntimeError("V5 pair validation failed; inspect report")
    finally:
        if dm_bridge is not None:
            dm_bridge.close()
        # Isaac Lab's STOP callback otherwise renders until Play is pressed,
        # including in this headless script. The public cleanup API removes it.
        if "sim" in locals():
            sim_utils.SimulationContext.clear_instance()
        launcher.app.close(wait_for_replicator=False)
