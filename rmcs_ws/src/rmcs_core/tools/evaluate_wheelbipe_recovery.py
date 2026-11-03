#!/usr/bin/env python3
"""Read-only native diagnostic of the RMCS script core -> saved WheelBipe actor.

Uses a frozen training source and its authenticated physical assets. This is
an isolated simulator, without an optimizer, learner control or hardware I/O.
The production executor/bridge is covered separately by the integration test.
"""
import argparse
import ctypes as ct
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import traceback

import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=1500)
    parser.add_argument("--project-fixed-gimbal", action="store_true",
        help="Explicitly project historical pose-bank gimbal coordinates to the frozen asset's zero")
    parser.add_argument("--starts", nargs="+", default=["stand"])
    args = parser.parse_args()
    if args.steps < 20:
        parser.error("Require at least 20 policy steps")
    args.output.mkdir(parents=True, exist_ok=False)
    source = args.source_root.resolve()
    sys.path.insert(0, str(source / "src"))
    from wheeled_tasks.wheelbipe_port.model import expose_source
    expose_source()
    from isaaclab.app import AppLauncher
    app = AppLauncher(dict(headless=True, device="cuda:0", enable_cameras=False)).app
    env = writer = None
    handles = []
    result = dict(status="preparing", hardware_tested=False, behavior_qualified=False, ppo_updates=0,
        live_training_modified=False, checkpoint_sha256=digest(args.checkpoint),
        library_sha256=digest(args.library), physics_dt=.005, policy_hz=50, pd_hz=200,
        solver_iterations=[64, 32], cases=[], scope="Core via C ABI; native port physics; separate executor test")
    result["gap_scope"] = "Excluded-anchor distance at policy observation boundaries; no per-physics-step maximum claim"
    result["source_files_sha256"] = {name: digest(source / name) for name in (
        "src/wheeled_tasks/wheelbipe_port/environment.py", "src/wheeled_tasks/wheelbipe_port/config.py",
        "src/wheeled_tasks/wheelbipe_port/model.py", "src/wheeled_tasks/wheelbipe_port/training_profile.py")}
    def save():
        (args.output / "receipt.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    try:
        import torch
        from torch.utils.tensorboard import SummaryWriter
        from wheeled_tasks.wheelbipe_port.config import WheelbipePortFlatCfg
        from wheeled_tasks.wheelbipe_port.environment import WheelbipePortEnv
        from wheeled_tasks.wheelbipe_port.training_profile import configure_profile
        torch.set_num_threads(4)
        cfg = WheelbipePortFlatCfg()
        configure_profile(cfg, "foundation-v2")
        cfg.scene.num_envs, cfg.seed = 1, 111
        cfg.sim.dt, cfg.decimation, cfg.sim.render_interval = .005, 4, 4
        cfg.episode_length_s = args.steps * .02 + 2.
        cfg.robot_cfg.spawn.articulation_props.solver_position_iteration_count = 64
        cfg.robot_cfg.spawn.articulation_props.solver_velocity_iteration_count = 32
        cfg.sim.physx.enable_external_forces_every_iteration = False
        cfg.robot_cfg.actuators["legs_act"].armature = .015795
        cfg.port_leg_kp, cfg.port_leg_kd = 60., 2.
        cfg.port_randomization_profile, cfg.port_randomization_strength = "nominal", 0.
        cfg.use_leg_random_start = False
        cfg.commands.special_modes = {}
        cfg.events.reset_base.params["pose_range"] = {}
        cfg.events.reset_base.params["velocity_range"] = {}
        class RecoveryEnv(WheelbipePortEnv):
            script_policy = False

            def _get_dones(self):
                terminated, truncated = super()._get_dones()
                # A fallen pose is the script's input, not a policy failure.
                # Script speed/timeout/finite guards remain in the C++ core.
                if not self.script_policy:
                    terminated.zero_()
                return terminated, truncated

        env = RecoveryEnv(cfg)
        result["policy_fall_termination_deferred_until_handoff"] = True
        env.eval_command = (0., 0., .305)
        bank = source / "model/recovery_v202_20261031/pose_bank.json"
        bank_data = json.loads(bank.read_text())
        bundle = source / "model/wheel_leg_v2.0.2/friction_v1"
        canonical = bundle / "training/model_spec.json"
        physics_spec = bundle / "isaac/model_spec.json"
        if bank_data["model_spec_sha256"] != digest(canonical):
            raise ValueError("Pose bank belongs to a different canonical vehicle")
        expected = deepcopy(json.loads(canonical.read_text()))
        for joint in expected["joints"]:
            if joint["name"] in ("M_joint1", "M_joint2"):
                joint.update(type="fixed", limit={})
        if expected != json.loads(physics_spec.read_text()):
            raise ValueError("Physics differs beyond the frozen zero-gimbal adaptation")
        result.update(canonical_model_spec_sha256=digest(canonical),
            physics_model_spec_sha256=digest(physics_spec), fixed_zero_gimbal_adaptation_verified=True)
        poses = {p["name"]: p for p in bank_data["poses"]}
        result["pose_bank_sha256"] = digest(bank)
        state = torch.load(args.checkpoint, map_location=env.device, weights_only=False)["model_state_dict"]
        layers = []
        for index, (ni, no) in enumerate(((35, 256), (256, 128), (128, 64), (64, 6))):
            layer = torch.nn.Linear(ni, no, device=env.device)
            layer.weight.data.copy_(state[f"actor.{index*2}.weight"])
            layer.bias.data.copy_(state[f"actor.{index*2}.bias"])
            layers.append(layer)
            if index != 3:
                layers.append(torch.nn.ELU())
        actor = torch.nn.Sequential(*layers).eval()
        lib = ct.CDLL(str(args.library.resolve()))
        pointer = ct.POINTER(ct.c_double)
        lib.wheelleg_recovery_create_v202.restype = ct.c_void_p
        lib.wheelleg_recovery_destroy.argtypes = [ct.c_void_p]
        lib.wheelleg_recovery_step.argtypes = [ct.c_void_p, pointer, ct.c_double, ct.c_int, ct.c_int, pointer]
        lib.wheelleg_recovery_compensation.argtypes = [pointer, pointer]
        result["status"] = "sampling"
        result["diagnostic_script_sha256"] = digest(__file__)
        save()
        ptr = lambda a: a.ctypes.data_as(pointer)
        native_effort = env.control.motor_efforts
        control = dict(policy=False, ready=False, fault=0, phase=0, calls=0, handle=None)
        def efforts(q, dq, legs, wheels, **kwargs):
            motor = native_effort(q, dq, legs, wheels, **kwargs)
            if control["policy"]:
                return motor
            if control["fault"]:
                return torch.zeros_like(motor)
            position = q[0, (0, 1, 3, 4)].cpu().double().numpy().copy()
            velocity = dq[0, (0, 1, 3, 4)].cpu().double().numpy().copy()
            compensation = np.zeros(4)
            assert lib.wheelleg_recovery_compensation(ptr(position), ptr(compensation)) == 0
            gravity = env.robot.data.projected_gravity_b[0].cpu().double().numpy()
            gyro = env.robot.data.root_ang_vel_b[0].cpu().double().numpy()
            wheel_velocity = dq[0, (2, 5)].cpu().double().numpy()
            feedback = np.concatenate((position, velocity, compensation, gravity, gyro, wheel_velocity))
            out = np.zeros(24)
            assert lib.wheelleg_recovery_step(control["handle"], ptr(feedback), .005, 1, 1, ptr(out)) == 0
            control.update(phase=int(out[14]), fault=int(out[15]), calls=control["calls"]+1)
            nominal = env.robot.data.default_joint_pos[0, [env.motor_ids[i] for i in (0, 1, 3, 4)]].cpu().numpy()
            # Pair turn changes do not change the CAD included angle.
            error = position - nominal
            for side in (0, 2):
                error[side:side+2] -= 2*np.pi*np.round(error[side]/(2*np.pi))
            control["ready"] = int(out[14]) == 5 and gravity[2] < -.9903 and np.linalg.norm(gyro[:2]) < .35 \
                and np.max(np.abs(velocity)) < .6 and np.max(np.abs(wheel_velocity)) < 2. \
                and np.max(np.abs(error)) < .25
            motor.zero_()
            motor[:, (0, 1, 3, 4)] = torch.as_tensor(out[8:12], device=env.device, dtype=motor.dtype)
            motor[:, (2, 5)] = torch.as_tensor(out[12:14], device=env.device, dtype=motor.dtype)
            return motor
        env.control.motor_efforts = efforts
        writer = SummaryWriter(str(args.output / "events"))
        for name in args.starts:
            pose = poses[name]
            if not args.project_fixed_gimbal and any(
                abs(pose["full_joint_positions"][joint]) > 1e-8 for joint in ("M_joint1", "M_joint2")):
                raise ValueError("Pose requires gimbal motion absent from the frozen physics asset")
            with torch.inference_mode():
                env.reset()
                # Only initial placement writes state; no writes during rollout.
                joint_q = env.robot.data.default_joint_pos.clone()
                joint_q[0] = torch.tensor([pose["full_joint_positions"][n] for n in env.robot.cad_joint_names], device=env.device)
                env.robot.write_joint_state_to_sim(joint_q, torch.zeros_like(joint_q))
                root = env.robot.data.default_root_state.clone()
                root[0, :3] = torch.tensor(pose["root_position_m"], device=env.device) + env.scene.env_origins[0]
                x, y, z, w = pose["root_quaternion_xyzw"]
                root[0, 3:7] = root.new_tensor([w, x, y, z])
                root[:, 7:] = 0.
                env.robot.write_root_state_to_sim(root)
                env.scene.update(0.)
                env.finish_init[:] = True
                obs = env._get_observations()
            handle = lib.wheelleg_recovery_create_v202()
            if not handle:
                raise ValueError("C++ rejected v2.0.2 recovery core")
            handles.append(handle)
            control.update(policy=False, ready=False, fault=0, phase=0, calls=0, handle=handle)
            env.script_policy = False
            row = dict(start=name, initial_condition="pose bank geometry placement; no passive settle proof",
                pose_bank_gimbal_q={joint: pose["full_joint_positions"][joint] for joint in ("M_joint1", "M_joint2")},
                actual_fixed_gimbal_q=[0., 0.], handoff_s=None, fault=0, resets=0, peak_gap_m=0., tail=[])
            result["cases"].append(row)
            for step in range(args.steps):
                with torch.inference_mode():
                    if control["ready"] and not control["policy"]:
                        control["policy"] = True
                        env.script_policy = True
                        row["handoff_s"] = step * .02
                    action = actor(obs["policy"]) if control["policy"] else torch.zeros(1, 6, device=env.device)
                    obs, reward, terminated, truncated, _ = env.step(action)
                    row["resets"] += int((terminated | truncated).sum())
                    gap = float(env.constraint_gaps().max())
                    row["peak_gap_m"] = max(row["peak_gap_m"], gap)
                    data = env.robot.data
                    sample = [float(data.root_lin_vel_b[0, 0]), float(data.root_pos_w[0, 2]),
                        float(torch.acos((-data.projected_gravity_b[0, 2]).clamp(-1., 1.))*180./torch.pi)]
                    if not np.isfinite(sample + [gap]).all():
                        raise ValueError("Nonfinite rollout")
                    for key, value in zip(("vx_m_s", "height_m", "tilt_deg"), sample):
                        writer.add_scalar(f"{name}/{key}", value, step+1)
                    writer.add_scalar(f"{name}/phase", control["phase"], step+1)
                    if step >= args.steps//2:
                        row["tail"].append(sample)
                    row.update(fault=control["fault"], final_phase=control["phase"], duration_s=(step+1)*.02)
                    if control["fault"] or row["resets"]:
                        break
            if row["tail"]:
                values = np.asarray(row.pop("tail"))
                row["tail_mean_vx_height_tilt"] = values.mean(0).tolist()
            else:
                row.pop("tail")
            print(json.dumps(row), flush=True)
            save()
            lib.wheelleg_recovery_destroy(handle)
            handles.remove(handle)
        result["status"] = "complete"
        save()
    except BaseException as error:
        result.update(status="failed", error=repr(error), traceback=traceback.format_exc())
        save()
        raise
    finally:
        if handles:
            for handle in handles:
                lib.wheelleg_recovery_destroy(handle)
        if writer is not None:
            writer.close()
        if env is not None:
            env.close()
        app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
