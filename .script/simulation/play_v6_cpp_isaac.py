#!/usr/bin/env python3
"""Run production RMCS components against the pinned free-base V6 plant.

Isaac owns physics and passive springs. The socket peer owns remote decoding,
state transitions, observations, ONNX inference and every active motor effort.
No Python PD/actor, CAN device or training worker is used by this entry point.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import sys
import time
import traceback

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import numpy as np

RMCS = Path(__file__).resolve().parents[2]
POLICY = RMCS / "rmcs_ws/src/rmcs_rl/models/wheel_leg/policy.onnx"
POLICY_NAMES = ("L_joint1", "LL_joint1", "R_joint1", "RR_joint1", "L_joint3", "R_joint3")
OFFSET = np.array([1.6, 2.93, -1.6, -2.93, 0., 0.])
SIGN = np.array([-1., -1., -1., -1., 1., 1.])
CASES = {
    "stand": (16., 0., 0., 0.),
    "forward_stop": (20., 1., 0., 0.),
    "backward_stop": (20., -1., 0., 0.),
    "yaw_positive": (16., 0., 1., 0.),
    "yaw_negative": (16., 0., -1., 0.),
    "spin_negative": (16., 1., 1., 0.),
    "spin_positive": (16., 1., 1., 0.),
    "height_low": (22., 0., 0., -1.),
    "height_high": (22., 0., 0., 1.),
    "disable": (5.5, 0., 0., 0.),
}
INITIAL_PROFILES = {
    "nominal": (0., 0., 0.),
    "native_prepare": (0., 0., 0.),
    "pitch_forward": (.10, .4, .10),
    "pitch_backward": (-.10, -.4, -.10),
}


def initialize_endpoint(env, name, policy_ids, torch, prepared_reference):
    """Engineering end-state perturbations, applied only at episode reset.

    Rotate the complete closed mechanism; never move only its active axes.
    Native 0.324 m release height stays above the three validated CAD ground
    poses (0.3203..0.3227 m with 3 mm clearance). This is not script success.
    """
    if name == "nominal":
        return
    if name == "native_prepare":
        pose = env.robot.data.root_link_pose_w.torch.clone()
        pose[0, 2] = env.origins[0, 2] + prepared_reference["height_m"]
        pose[0, 3:] = pose.new_tensor(prepared_reference["root_quaternion_xyzw"])
        joints = env.nominal.clone()
        joints[0] = joints.new_tensor([prepared_reference["full_joint_positions"][n] for n in env.robot.joint_names])
        env.robot.write_root_link_pose_to_sim_index(root_pose=pose)
        env.robot.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros(1, 6))
        env.robot.write_joint_position_to_sim_index(position=joints)
        env.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joints))
        env.robot.update(env.dt)
        return
    pitch, pitch_rate, vx = INITIAL_PROFILES[name]
    pose = env.robot.data.root_link_pose_w.torch.clone()
    pose[0, 3:] = pose.new_tensor([0., math.sin(pitch / 2), 0., math.cos(pitch / 2)])
    velocity = torch.zeros(1, 6, device=env.device)
    velocity[0, 0], velocity[0, 2], velocity[0, 4] = vx * math.cos(pitch), -vx * math.sin(pitch), pitch_rate
    joint_velocity = torch.zeros_like(env.nominal)
    joint_velocity[0, policy_ids[4]] = vx / .06
    joint_velocity[0, policy_ids[5]] = -vx / .06
    env.robot.write_root_link_pose_to_sim_index(root_pose=pose)
    env.robot.write_root_com_velocity_to_sim_index(root_velocity=velocity)
    env.robot.write_joint_velocity_to_sim_index(velocity=joint_velocity)
    env.robot.update(env.dt)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


class ComponentClient:
    def __init__(self, path):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(10.)
        self.socket.connect(str(path))
        self.stream = self.socket.makefile("rwb")

    def request(self, value):
        self.stream.write(json.dumps(value, allow_nan=False).encode() + b"\n")
        self.stream.flush()
        line = self.stream.readline()
        if not line:
            raise RuntimeError("C++ component bridge disconnected")
        response = json.loads(line)
        if not response.get("ok"):
            raise RuntimeError(f"C++ component bridge: {response}")
        return response

    def close(self):
        self.stream.close()
        self.socket.close()


def remote_input(name, t):
    duration, forward, yaw, knob = CASES[name]
    request = dict(right_stick=[0., 0.], left_stick=[0., 0.],
                   left_switch=3, right_switch=3, rotary_knob=0., keyboard=0,
                   remote_fresh=True, feedback_fresh=True, dm_control_ready=True)
    if t < .005 or (name == "disable" and t >= 5.):
        request.update(left_switch=2, right_switch=2)
        return request
    if t < 4.:
        return request
    if name.endswith("_stop") and t >= 16.:
        forward = 0.
    request.update(right_stick=[forward, 0.], left_stick=[0., yaw], rotary_knob=knob)
    if name.startswith("spin_"):
        # Keep translation requested: production SPIN must suppress it itself.
        request["right_switch"] = 2
        if name == "spin_positive":
            # First edge enters negative; second exits; third enters positive.
            age = round((t - 4.) / .005)
            if age in (1, 3):
                request["right_switch"] = 3
    return request


def summarize(name, rows, completed):
    t = np.array([r["time_s"] for r in rows])
    steady_start = 12. if name.startswith("height_") else 6.
    mask = t >= steady_start
    if name.endswith("_stop"):
        mask &= t < 16.
    if name == "disable":
        # Rows describe the END of a physical step. DOWN first arrives at
        # step-start 5.000 s; the 5.000-s row still belongs to the prior input.
        mask = t >= 5.005 - 1e-9
    steady = [r for r, selected in zip(rows, mask) if selected]
    output = {"case": name, "completed": completed, "samples": len(rows),
              "steady_samples": len(steady), "rl_entered_at_s": next(
                  (r["time_s"] for r in rows if r["state"] == 3), None),
              "fault": next((r for r in rows if r.get("fault")), None)}
    early = [r for r in rows if r["time_s"] <= 2.]
    entry = next((i for i, r in enumerate(rows) if r["state"] == 3), None)
    output["takeover"] = {
        "tilt_peak_first_2s_deg": max((r["tilt_deg"] for r in early), default=None),
        "planar_speed_peak_first_2s_m_s": max((math.hypot(*r["velocity"][:2]) for r in early), default=None),
        "entry_effort_jump_max_nm": (float(np.max(abs(np.array(rows[entry]["torque_api"]) -
             np.array(rows[entry - 1]["torque_api"])))) if entry is not None and entry > 0 else None),
        "effort_step_peak_first_2s_nm": max((float(np.max(abs(np.array(b["torque_api"]) -
             np.array(a["torque_api"])))) for a, b in zip(early, early[1:])), default=None),
    }
    if not steady:
        output.update(passed=False, checks={"steady_window_completed": False})
        return output
    velocities = np.array([r["velocity"] for r in steady])
    commands = np.array([r["reference"] for r in steady])
    heights = np.array([r["height_m"] for r in steady])
    tilt = np.array([r["tilt_deg"] for r in steady])
    xy = np.array([r["position_m"][:2] for r in steady])
    command_height = np.array([r["height_reference_m"] for r in steady])
    output.update(vx_mae_m_s=float(np.mean(abs(velocities[:, 0] - commands[:, 0]))),
                  yaw_mae_rad_s=float(np.mean(abs(velocities[:, 2] - commands[:, 2]))),
                  height_mae_m=float(np.mean(abs(heights - command_height))),
                  planar_speed_mean_m_s=float(np.mean(np.linalg.norm(velocities[:, :2], axis=1))),
                  tilt_max_deg=float(tilt.max()),
                  drift_max_m=float(np.linalg.norm(xy - xy[0], axis=1).max()),
                  final_height_m=float(heights[-1]),
                  steady_command=commands[-1].tolist())
    expected_vx = .5 if name == "forward_stop" else -.5 if name == "backward_stop" else 0.
    expected_yaw = 1. if name in ("yaw_positive", "spin_positive") else (
        -1. if name in ("yaw_negative", "spin_negative") else 0.)
    expected_height = .23 if name == "height_low" else .43 if name == "height_high" else .305
    checks = dict(steady_window_completed=completed, controller_rl=all(r["state"] == 3 for r in steady),
                  expected_command=bool(np.max(abs(commands[:, 0] - expected_vx)) < 1e-6 and
                      np.max(abs(commands[:, 2] - expected_yaw)) < 1e-6 and
                      np.max(abs(command_height - expected_height)) < 1e-6),
                  upright=output["tilt_max_deg"] <= 20.,
                  velocity_tracking=output["vx_mae_m_s"] <= .1,
                  yaw_tracking=output["yaw_mae_rad_s"] <= .15,
                  height_tracking=output["height_mae_m"] <= (.005 if name in
                      ("stand", "height_low", "height_high") else .01),
                  mechanism=all(r["mechanism_valid"] for r in steady),
                  no_physics_failure=all(not r.get("physics_failure") for r in rows))
    if name in ("stand", "height_low", "height_high"):
        checks.update(stand_speed=output["planar_speed_mean_m_s"] <= .03,
                      stand_drift=output["drift_max_m"] <= .1)
    if name.startswith("spin_"):
        checks["pure_spin_command"] = all(abs(r["reference"][0]) < 1e-6 and
            r["command_velocity"][:2] == [0., 0.] and r["mode"] == 2 for r in steady)
    if name.endswith("_stop"):
        stopped = [r for r in rows if r["time_s"] >= 18.]
        output["stop_speed_mean_m_s"] = float(np.mean([
            math.hypot(r["velocity"][0], r["velocity"][1]) for r in stopped])) if stopped else None
        checks["stop_speed"] = bool(stopped) and output["stop_speed_mean_m_s"] <= .03
    if name == "disable":
        checks = dict(steady_window_completed=completed,
                      disabled_immediately=all(not r["enable_request"] and
                      max(abs(x) for x in r["torque_api"]) == 0 for r in steady))
    output.update(checks=checks, passed=all(checks.values()))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-repo", type=Path, required=True)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", choices=tuple(CASES), default=list(CASES))
    parser.add_argument("--initial-profiles", nargs="+", choices=tuple(INITIAL_PROFILES), default=["nominal"])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, help="Shortened smoke run; incomplete cases cannot pass")
    args = parser.parse_args()
    if args.seconds is not None and not 0 < args.seconds <= 32:
        parser.error("seconds must be in (0,32]")
    args.output.mkdir(parents=True, exist_ok=False)
    root = args.training_repo.resolve()
    sys.path[:0] = [str(root / "src"), str(root / "scripts")]
    from train_chassis import preflight
    contract_file = Path(str(POLICY) + ".contract.json")
    contract, manifest = preflight(contract_file)
    prepared_reference = None
    if "native_prepare" in args.initial_profiles:
        profiles = root / contract["recovery_training"]["profiles_file"]
        if digest(profiles) != contract["recovery_training"]["profiles_sha256"]:
            raise ValueError("V6 native prepare geometry identity differs")
        prepared_reference = json.loads(profiles.read_text())["prepare_reference"]
    metadata = json.loads(Path(str(POLICY) + ".json").read_text())
    if metadata["onnx_sha256"] != digest(POLICY) or metadata["contract_sha256"] != digest(contract_file):
        raise ValueError("Pinned policy/contract identity differs")
    from wheeled_tasks.chassis.evaluation import fixed_suite_contract
    config = fixed_suite_contract(contract)
    # This is a nominal deployment bench, separate from randomized qualification.
    # The independent native playback profile permits one flat CPU box plant.
    design = root / "contracts/v6_flat_keyboard_playback_expectations_v1.json"
    if json.loads(design.read_text())["source_contract_sha256"] != digest(contract_file):
        raise ValueError("Playback profile is not bound to this flat contract")
    config.update(scene_groups=[{"name": "live", "fraction": 1., "terrain": ["jump"]}],
                  record_diagnostics=True, auto_reset=False, monitor_applied_effort=True,
                  evaluation_long_corridors=True, playback_open_ground=False,
                  evaluation_exact_cases=True, flat_triangle_mesh=False, flat_half_length_m=44.,
                  skill_specs={"live": {"kind": "constant", "command": [0., 0., .305], "mode": 0}},
                  design_preflight=dict(file=str(design), sha256=digest(design), stage="flat"),
                  evaluation={**config["evaluation"], "protocol_id": "keyboard_playback_unscored",
                      "episodes_per_case": 1, "canonical_case_names": ["live"],
                      "promotion_case_names": [], "protected_case_names": [], "retention_case_names": [],
                      "quick_case_names": [], "cases": [{"name": "live", "terrain": "jump",
                          "command": [0., 0., .305], "skill": {"kind": "constant",
                          "command": [0., 0., .305], "mode": 0}}], "seed": 190619})
    report = dict(schema="rmcs-v6-cpp-isaac-v1", status="starting",
                  started_at=datetime.now(timezone.utc).isoformat(),
                  controller="production WheelLegChassisController + RlController + OnnxPolicy",
                  policy_sha256=digest(POLICY), contract_sha256=digest(contract_file),
                  asset_manifest_sha256=contract["asset_manifest_sha256"],
                  physics_dt_s=.005, executor_rate_hz=1000, policy_rate_hz=50, pd_rate_hz=200,
                  sensor_source="ideal base IMU and output-axis encoders",
                  reset_jitter=dict(roll_rad=[-.01, .01], pitch_rad=[-.02, .02], z_m=[0., .002]),
                  timing_scope="logical simulation ticks with real steady-clock freshness; paced socket loop",
                  fixed_base=False, external_guide=False, state_writes_between_resets=0,
                  hardware_gate_overrides="bridge only; installed YAML remains disarmed",
                  domain_qualified=None, cases=[])
    trials = [(name, profile, name if profile == "nominal" else f"{name}__{profile}")
              for profile in args.initial_profiles for name in args.cases]
    report["requested_cases"] = [key for _, _, key in trials]
    report["initial_profiles"] = {name: dict(pitch_rad=values[0], pitch_rate_rad_s=values[1], vx_m_s=values[2])
                                  for name, values in INITIAL_PROFILES.items() if name in args.initial_profiles}
    report["initial_profile_scope"] = "engineering end-state/release stress; no full fallen-pose trajectory executed"
    if prepared_reference:
        report["initial_profiles"]["native_prepare"] = {
            "source_sha256": contract["recovery_training"]["profiles_sha256"],
            "reference": prepared_reference,
            "scope": "geometry-derived V6 prepared endpoint, zero velocities; no preceding script executed",
        }
    source_paths = [
        ".script/simulation/play_v6_cpp_isaac.py",
        ".script/simulation/v6_component_sim_bridge.cpp",
        "rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml",
        "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg_chassis_controller.cpp",
        "rmcs_ws/src/rmcs_rl/src/rl_controller.cpp",
        "rmcs_ws/src/rmcs_rl/src/rl_controller.hpp",
        "rmcs_ws/src/rmcs_rl/src/controller_recovery.cpp",
        "rmcs_ws/src/rmcs_rl/src/observation.cpp",
        "rmcs_ws/src/rmcs_rl/src/action.cpp",
        "rmcs_ws/src/rmcs_rl/src/configuration.cpp",
        "rmcs_ws/src/rmcs_rl/src/policy.hpp",
    ]
    report["source_sha256"] = {path: digest(RMCS / path) for path in source_paths}
    write_json(args.output / "report.json", report)
    launcher = client = env = None
    gc_was_enabled = gc.isenabled()
    try:
        from isaaclab.app import AppLauncher
        launcher = AppLauncher({"headless": args.headless, "device": "cpu", "enable_cameras": False,
                              **({"visualizer": ["kit"]} if not args.headless else {})})
        import torch
        import warp as wp
        from wheeled_tasks.chassis.eval_env import FixedCaseEnv
        from wheeled_tasks.chassis.control_contract import load_chassis_control
        from wheeled_tasks.chassis.recovery_observer import SimulatedImu
        from wheeled_tasks.v40.core import motor_torque_limit
        torch.set_num_threads(4)
        env = FixedCaseEnv(config, manifest, load_chassis_control(root / config["control_math_source"]),
                           root, stage_name=config["enabled_stages"][0], num_envs=1, device="cpu",
                           seed=190619, level=0., run_role="playback")
        if not math.isclose(env.dt, .005) or not math.isclose(env.policy_dt, .02):
            raise ValueError("Plant timing differs from the frozen deployment profile")
        policy_ids = [env.robot.joint_names.index(n) for n in POLICY_NAMES]
        report["plant"] = env.startup_report
        report["policy_joint_ids"] = policy_ids
        client = ComponentClient(args.socket)
        report["bridge"] = client.request({"op": "hello"})
        env.sim.set_camera_view((1., 1.2, .85), (0., 0., .23))
        overlay = None
        if not args.headless:
            import omni.ui as ui
            from pxr import UsdLux
            UsdLux.DomeLight.Define(env.sim.stage, "/World/CppBenchLight").CreateIntensityAttr(1800.)
            overlay_window = ui.Window("RMCS C++ / V6 FLAT", width=410, height=180)
            with overlay_window.frame:
                overlay = ui.Label("Production C++ component simulation", word_wrap=True)
        report["status"] = "running"
        for name, initial_profile, trial_key in trials:
            # Kit retains a large Python scene graph. Collect outside the
            # control interval so cyclic GC cannot create artificial sensor
            # outages; the production steady-clock guard remains unchanged.
            gc.collect()
            gc.disable()
            client.request({"op": "reset"})
            env.reset_suite()
            initialize_endpoint(env, initial_profile, policy_ids, torch, prepared_reference)
            imu = SimulatedImu("cpu", 1, env.dt)
            imu.reset(env.robot.data.root_link_lin_vel_w.torch)
            rows, started = [], time.monotonic()
            duration = min(CASES[name][0], args.seconds or CASES[name][0])
            completed, last_render = False, 0.
            for step in range(round(duration / env.dt)):
                t = step * env.dt
                data = env.robot.data
                q = data.joint_pos.torch[0, policy_ids].numpy().copy()
                dq = data.joint_vel.torch[0, policy_ids].numpy().copy()
                xyzw = data.root_link_pose_w.torch[0, 3:].numpy().copy()
                acceleration = imu.sample(data.root_link_lin_vel_w.torch,
                                          data.root_link_pose_w.torch[:, 3:])[0].tolist()
                response = client.request(dict(op="step", q_api=((q - OFFSET) * SIGN).tolist(),
                    dq_api=(dq * SIGN).tolist(),
                    feedback_torque=(data.applied_torque.torch[0, policy_ids].numpy() * SIGN).tolist(),
                    orientation_wxyz=xyzw[[3, 0, 1, 2]].tolist(),
                    gyro=data.root_com_ang_vel_b.torch[0].tolist(), acceleration=acceleration,
                    **remote_input(name, t)))
                torque_p = np.asarray(response["torque_api"]) * SIGN
                torque_c = torch.tensor(torque_p[[0, 1, 4, 2, 3, 5]], dtype=torch.float32)[None, :]
                # Retain the physical drive envelope and the native identified
                # actuator response once, without Python target decoding/PD.
                bounds = torch.full_like(torque_c, 40.)
                bounds[:, [2, 5]] = motor_torque_limit(
                    data.joint_vel.torch[:, [env.ids[2], env.ids[5]]], env.v5.wheel_prior)
                torque_c.clamp_(min=-bounds, max=bounds)
                torque_c = env.actuator_response.apply_torque(torque_c)
                effort = torch.zeros_like(env.nominal)
                effort[:, env.ids] = torque_c
                effort[:, env.spring_ids] = env.v5.spring_efforts(data.joint_pos.torch[:, env.spring_ids])
                env.robot.set_joint_effort_target_index(target=effort)
                env.robot.write_data_to_sim()
                env.sim.step(render=False)
                env.robot.update(env.dt)
                for ids, contact_view in env.contact_views:
                    matrix = wp.to_torch(contact_view.get_contact_force_matrix(dt=env.dt))
                    env.contact_force[ids] = matrix.reshape(len(ids), env.body_count, -1, 3).sum(2)
                data = env.robot.data
                position = (data.root_link_pose_w.torch[0, :3] - env.origins[0]).tolist()
                gravity = data.projected_gravity_b.torch[0].tolist()
                tilt = math.degrees(math.acos(np.clip(-gravity[2], -1., 1.)))
                passive_q = data.joint_pos.torch[0, env.knee_ids]
                compression, _ = env.v5.spring_state(data.joint_pos.torch[:, env.spring_ids],
                                                   data.joint_vel.torch[:, env.spring_ids])
                mechanism = bool(((passive_q >= env.v5.knee_bounds[:, 0] - .03) &
                                  (passive_q <= env.v5.knee_bounds[:, 1] + .03)).all() and
                                 ((compression >= -.001) & (compression <= env.v5.stroke + .001)).all())
                gap = float(env.closure_gap()[0])
                nonwheel_force = float(env.contact_force[0, env.nonwheel_ids].norm(dim=-1).max())
                physical_failure = []
                if not mechanism:
                    physical_failure.append("mechanism")
                if gap > config["closure_gap_termination_m"]:
                    physical_failure.append("closure_gap")
                if t > .2 and nonwheel_force > 5.:
                    physical_failure.append("nonwheel_contact")
                if (abs(position[0]) > config["flat_half_length_m"] - 4. or
                        abs(position[1]) > config.get("flat_floor_width_m", 4.) / 2 - .3):
                    physical_failure.append("boundary")
                if tilt > 45.:
                    physical_failure.append("fall")
                obs = response["observation"]
                row = dict(time_s=t + env.dt, wall_s=time.monotonic() - started,
                    state=response["state"], enable_request=response["enable_request"], mode=response["mode"],
                    reference=[obs[0], 0., obs[2]] if response["state"] == 3 else [0., 0., 0.],
                    height_reference_m=obs[3] / 5. if response["state"] == 3 else .305,
                    velocity=[float(data.root_com_lin_vel_b.torch[0, 0]),
                              float(data.root_com_lin_vel_b.torch[0, 1]),
                              float(data.root_com_ang_vel_b.torch[0, 2])],
                    height_m=position[2], position_m=position, tilt_deg=tilt,
                    mechanism_valid=mechanism, closure_gap_m=gap,
                    nonwheel_force_n=nonwheel_force, physics_failure=physical_failure,
                    observation=obs, q_model=q.tolist(), dq_model=dq.tolist(),
                    command_velocity=response["command_velocity"], command_height_m=response["command_height"],
                    torque_api=response["torque_api"], action=response["actions"],
                    sensor_issue=response.get("sensor_issue"), sensor_mask=response.get("sensor_mask"),
                    inference_us=response.get("inference_us"), pd_us=response.get("pd_us"))
                row["takeover_blend_fraction"] = response.get("v6_takeover_blend_fraction")
                for key in ("wall_sample_interval_ms", "wall_step_duration_ms", "sensor_sequence",
                            "sensor_steady_ns", "update_count", "sensors_valid", "failure",
                            "motor_age_ms", "imu_age_ms", "acceleration_age_ms"):
                    row[key] = response.get(key)
                row["fault"] = bool(t > .02 and t < (5. if name == "disable" else duration) and
                                    response["state"] in (0, 1))
                rows.append(row)
                if not args.headless and time.monotonic() - last_render >= 1 / 30:
                    look_at = np.asarray(position) + [0., 0., -.08]
                    env.sim.set_camera_view(look_at + [1., 1.2, .6], look_at)
                    overlay.text = (f"{name} | simulation {t:.2f} s\n"
                        f"C++ state {row['state']} | mode {row['mode']} | enable {row['enable_request']}\n"
                        f"vx {row['velocity'][0]:+.3f} / {row['reference'][0]:+.2f} m/s\n"
                        f"yaw {row['velocity'][2]:+.3f} / {row['reference'][2]:+.2f} rad/s\n"
                        f"height {row['height_m']:.4f} / {row['height_reference_m']:.3f} m")
                    env.sim.render()
                    last_render = time.monotonic()
                    screenshot = args.output / f"{trial_key}_viewport.png"
                    if t >= 2. and not screenshot.exists():
                        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                        capture_viewport_to_file(get_active_viewport(), str(screenshot))
                if row["fault"] or (physical_failure and name != "disable"):
                    break
                if not launcher.app.is_running():
                    break
            else:
                completed = duration >= CASES[name][0]
            result = summarize(name, rows, completed)
            result.update(case=trial_key, command_case=name, initial_profile=initial_profile,
                          expected_duration_s=CASES[name][0], requested_duration_s=duration)
            report["cases"].append(result)
            write_json(args.output / f"{trial_key}.json", rows)
            write_json(args.output / "report.json", report)
            print(f"RMCS_CASE {trial_key}: {json.dumps(result, allow_nan=False)}", flush=True)
        report["status"] = "complete"
        report["passed_cases"] = sum(c["passed"] for c in report["cases"])
    except Exception as error:
        report.update(status="error", error=str(error), traceback=traceback.format_exc())
        traceback.print_exc()
        raise
    finally:
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        write_json(args.output / "report.json", report)
        if client:
            client.close()
        if env:
            env.close()
        if launcher:
            launcher.app.close()
        if gc_was_enabled:
            gc.enable()


if __name__ == "__main__":
    main()
