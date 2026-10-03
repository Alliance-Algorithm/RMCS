#!/usr/bin/env python3
"""Run deployment C++ recovery/observer inside the existing Isaac activation bench.

Use the activation bench's remaining CLI arguments unchanged. Contact force,
root height and root velocity are used only to score the result, never by C++.
"""
from __future__ import annotations

import argparse
import ctypes
import importlib.util
import json
from pathlib import Path
import subprocess
import sys


RMCS = Path(__file__).resolve().parents[2]
CPP = RMCS / "rmcs_ws/src/rmcs_rl/src"
P_FROM_CONTROL = [0, 1, 3, 4, 2, 5]
CONTROL_FROM_P = [0, 1, 4, 2, 3, 5]
FAILURES = [None, "invalid_feedback", "timeout", "no_contact", "no_reorientation",
            "orbit_exhausted", "lost_upright", "cancelled", "unsafe_request"]
SENSOR_COLUMNS = ["conditional_height", "wheel_height_difference", "geometry_valid",
                  "height_valid", "contact_candidate", "alignment_candidate", "settled",
                  "support_confirmed", "body_clear", "world_wheel_omega_valid"]
DIAGNOSTIC_COLUMNS = [*["q_" + str(i) for i in range(4)], *["dq_" + str(i) for i in range(4)],
                      *["desired_" + str(i) for i in range(4)], "gravity_x", "gravity_y", "gravity_z",
                      "omega_x", "omega_y", "omega_z", "velocity_x", "velocity_y", "velocity_z",
                      "wheel_force_L", "wheel_force_R", "nonwheel_force_max", "knee_L", "knee_R",
                      "wheel_dq_L", "wheel_dq_R"]


def finalize_report(path):
    """Isaac may terminate Python at app.close(); the parent finalizes metadata."""
    report_path = path / "report.json"
    report = json.loads(report_path.read_text())
    if (path / "cpp_metadata.json").exists():
        metadata = json.loads((path / "cpp_metadata.json").read_text())
        report.update(source_kind="rmcs_cpp_recovery_observer_controller_isaac_closed_loop",
                      feedback_hz=metadata["feedback_hz"], physics_hz=metadata["physics_hz"],
                      diagnostic_wheel_brake=metadata["diagnostic_wheel_brake"],
                      control_groups_are_ungated=False, sensor_columns=SENSOR_COLUMNS,
                      diagnostic_columns=DIAGNOSTIC_COLUMNS)
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--training-repo", type=Path, required=True)
    parser.add_argument("--rmcs-feedback-hz", type=int, choices=(200, 1000), default=200)
    parser.add_argument("--diagnostic-wheel-brake", action="store_true",
                        help="Ablation only: supply the old sensor-derived calibrated world wheel rates")
    parser.add_argument("--isaac-worker", action="store_true", help=argparse.SUPPRESS)
    options, remaining = parser.parse_known_args()
    if not options.isaac_worker:
        # Keep a process outside Kit so report finalization survives its shutdown.
        result = subprocess.run([sys.executable, "-B", str(Path(__file__).resolve()),
                                 *sys.argv[1:], "--isaac-worker"])
        output_path = Path(remaining[remaining.index("--output") + 1])
        if (output_path / "report.json").exists():
            finalize_report(output_path)
        return result.returncode
    source = options.training_repo.resolve() / "scripts/inspect_v5_activation.py"
    spec = importlib.util.spec_from_file_location("activation_reference", source)
    reference = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reference)
    sys.path.insert(0, str(source.parent))
    sys.argv = [str(source), *remaining]

    class CppBench(reference.ActivationBench):
        def __init__(self, args, contract, manifest, prior):
            contract = dict(contract, physics_dt=1.0 / options.rmcs_feedback_hz)
            if not args.sensor_only:
                raise ValueError("C++ evaluation requires --sensor-only")
            if args.device != "cpu":
                raise ValueError("The C++ adapter currently supports CPU PhysX only")
            if args.methods != ["recovery_auto"]:
                raise ValueError("The C++ route selector accepts recovery_auto only")
            super().__init__(args, contract, manifest, prior)
            library = args.output / "recovery_sim_bridge.so"
            bridge = Path(__file__).with_name("recovery_sim_bridge.cpp")
            subprocess.run(["c++", "-std=c++20", "-O2", "-shared", "-fPIC", "-I", str(CPP),
                            str(bridge), str(CPP / "recovery_controller.cpp"),
                            str(CPP / "recovery_observer.cpp"), "-o", str(library)], check=True)
            self.native = ctypes.CDLL(str(library))
            ptr = self.np.ctypeslib.ndpointer(dtype=self.np.float64, flags="C_CONTIGUOUS")
            self.native.rmcs_sim_create.argtypes = [ctypes.c_char_p]
            self.native.rmcs_sim_create.restype = ctypes.c_void_p
            self.native.rmcs_sim_error.restype = ctypes.c_char_p
            self.native.rmcs_sim_step.argtypes = [ctypes.c_void_p, ptr, ctypes.c_uint64,
                                                ctypes.c_uint64, ctypes.c_double, ctypes.c_int, ptr]
            self.native.rmcs_sim_constrain_goal.argtypes = [ctypes.c_void_p, ptr]
            self.native.rmcs_sim_apply.argtypes = [ctypes.c_void_p, ptr, ctypes.c_uint64]
            self.native.rmcs_sim_destroy.argtypes = [ctypes.c_void_p]
            self.events = [[] for _ in self.rows]
            self.feedback_traces = []
            self.tick_inputs, self.tick_outputs = [], []
            self.handles = []
            self.previous_native_phase = self.np.zeros(self.count, dtype=int)
            self.previous_applied = self.np.zeros((self.count, 2))
            params = self.parameters()
            (args.output / "simulation_profile.json").write_text(json.dumps(params, indent=2) + "\n")
            for _ in self.rows:
                handle = self.native.rmcs_sim_create(json.dumps(params).encode())
                if not handle:
                    raise RuntimeError(self.native.rmcs_sim_error().decode())
                self.handles.append(handle)
            self.cpp_source_hashes = {str(path.relative_to(RMCS)): reference.digest(path)
                                     for path in [bridge, Path(__file__), CPP / "recovery_controller.cpp",
                                                  CPP / "recovery_controller.hpp", CPP / "recovery_observer.cpp",
                                                  CPP / "recovery_observer.hpp"]}
            for path in [bridge, CPP / "recovery_controller.cpp", CPP / "recovery_controller.hpp",
                         CPP / "recovery_observer.cpp", CPP / "recovery_observer.hpp"]:
                (args.output / path.name).write_bytes(path.read_bytes())

        def parameters(self):
            data = json.loads((reference.ROOT / "docs/evidence/v5_self_righting_reference_20260926.json").read_text())
            sides = []
            for index, (side, name) in enumerate(zip(self.observer.sides, ("L_spring_slide", "R_spring_slide"))):
                row = {key: side[key].cpu().double().tolist()
                       for key in ("delta", "inner", "slider", "point", "hip_origin", "hip_axis",
                                   "knee_axis", "wheel_axis")}
                row["sign"] = side["sign"]
                knots, slope = self.spring_maps[index]
                row["slider_slope"] = self.np.interp(row["delta"], knots.numpy(), slope.numpy()).tolist()
                # CAD slider q has an arbitrary origin and can be negative.
                # Translate both q and s0; compression and dq/d(delta) stay identical.
                slider_origin = max(0., -min(row["slider"]))
                row["slider"] = [value + slider_origin for value in row["slider"]]
                row["spring_zero"] = (self.spec["spring_binding"][name]["compression_at_q_zero_m"]
                                      + slider_origin)
                row["slider_coordinate_offset_m"] = slider_origin
                sides.append(row)
            poses = {"fold": self.fold, "thrust": self.rollover_thrust,
                     "side_extended": self.thrust, "plant": self.nominal,
                     "stand": self.stand_target, "upright": self.upright_target,
                     "support_extended": self.support_target,
                     "upright_support_extended": self.upright_support_target,
                     "capture_extended": self.capture_support_target}
            points = self.base_hull_points
            if points is None:
                import trimesh
                bodies = {body["name"]: body for body in self.spec["bodies"]}
                meshes = {c["file"]: trimesh.load(self.bundle / c["file"], force="mesh", process=False).vertices
                          for c in bodies["base_link"]["collisions"] if c["type"] == "mesh"}
                points = self.np.concatenate([reference.collision_points(c, self.np.asarray(c["origin"]), meshes)
                                             for c in bodies["base_link"]["collisions"]])
            return {"sides": sides, "shell_points": points.tolist(),
                    "root_axis_y": self.axis_sign.double().tolist(),
                    "wheel_axis_y": self.wheel_axis_sign.double().tolist(),
                    "diagnostic_wheel_brake": options.diagnostic_wheel_brake,
                    "wheel_radius_m": float(self.wheel_radius[0]),
                    "spring_stroke_m": data["spring_stroke_m"],
                    "spring_force_n": data["spring_force_coefficients_ascending_N"],
                    "poses": {name: pose[self.legs].double().tolist() for name, pose in poses.items()}}

        def step(self):
            import math
            import warp as wp
            from wheeled_tasks.chassis.scut_observation import build_scut35, CONTROL_FROM_POLICY
            torch, np, env = self.torch, self.np, self.env
            data = env.robot.data
            gravity, omega = self.simulated_imu.measure_attitude(
                data.projected_gravity_b.torch, data.root_com_ang_vel_b.torch)
            q, dq, _ = self.encoder_alignment.sample(data.joint_pos.torch[:, env.ids],
                                                     data.joint_vel.torch[:, env.ids])
            q = self.update_motor_angles(q)
            acceleration = self.simulated_imu.sample(data.root_com_lin_vel_w.torch,
                                                      data.root_link_pose_w.torch[:, 3:7])
            world_wheel_rates = np.zeros((self.count, 2))
            if options.diagnostic_wheel_brake:
                observed = self.observer.update(q, dq, gravity, omega,
                                                data.root_link_pose_w.torch[:, 3:7], acceleration)
                world_wheel_rates = observed.wheel_world_omega_y.double().numpy()
            inputs = np.concatenate((q[:, P_FROM_CONTROL].double().numpy(),
                                     dq[:, P_FROM_CONTROL].double().numpy(), gravity.double().numpy(),
                                     omega.double().numpy(), acceleration.double().numpy(), self.previous_applied,
                                     world_wheel_rates, data.root_link_pose_w.torch[:, 3:7].double().numpy()), axis=1)
            output = np.zeros((self.count, 39), dtype=np.float64)
            now_ns = 1_000_000_000 + round(self.time * 1e9)
            old_phase = self.phase.clone()
            for i, handle in enumerate(self.handles):
                enabled = self.time + 1e-8 >= float(self.release[i])
                self.native.rmcs_sim_step(handle, inputs[i], now_ns, self.ticks + 1,
                                          self.dt, enabled, output[i])
            self.tick_inputs.append(inputs.copy())
            self.tick_outputs.append(output.copy())
            native_phase = output[:, 6].astype(int)
            mapped = np.where(native_phase == 10, 11, np.where(native_phase == 11, 10, native_phase))
            self.phase = torch.as_tensor(mapped, dtype=torch.long)
            self.phase[self.mech_failed] = 6
            entering_blend = (self.phase == 4) & (old_phase != 4)
            self.previous_action[entering_blend] = 0.
            self.handover_time[entering_blend] = self.time
            tilt = torch.acos((-gravity[:, 2]).clamp(-1., 1.))
            for i in np.flatnonzero(native_phase != self.previous_native_phase):
                self.events[i].append({"time_s": self.time, "phase": reference.PHASES[mapped[i]],
                                       "tilt_deg": math.degrees(float(tilt[i])), "height_m": output[i, 9],
                                       "height_valid": bool(output[i, 12]), "contact_candidate": bool(output[i, 13]),
                                       "support_confirmed": bool(output[i, 16]), "body_clear": bool(output[i, 17])})
            self.previous_native_phase = native_phase
            for i in np.flatnonzero(output[:, 7]):
                self.failure_reason[i] = FAILURES[int(output[i, 7])]
            self.desired = torch.as_tensor(output[:, 19:23], dtype=torch.float32)
            if self.ticks % self.policy_steps == 0:
                commands = q.new_tensor([0., 0., .305]).expand(self.count, -1)
                zeros = q.new_zeros(self.count)
                obs = build_scut35(omega, gravity, commands, q, dq, self.previous_action,
                                   self.nominal, zeros.bool(), zeros, zeros)
                action = torch.as_tensor(self.session.run(None, {"obs": obs.numpy()})[0])[:, CONTROL_FROM_POLICY]
                self.rl_legs, self.rl_wheels, clipped = env.v5.decode(action, q)
                goals = self.rl_legs.double().numpy()
                for i, handle in enumerate(self.handles):
                    if output[i, 28]:
                        self.native.rmcs_sim_constrain_goal(handle, goals[i])
                self.rl_legs = torch.as_tensor(goals, dtype=torch.float32)
                active = (self.phase == 4) | (self.phase == 5)
                self.previous_action[active] = clipped[active]
            tau_rl = env.v5.motor_efforts(q, dq, self.rl_legs, self.rl_wheels)
            tau_script = torch.as_tensor(output[:, CONTROL_FROM_P], dtype=torch.float32)
            tau = tau_script.clone()
            blending = self.phase == 4
            blend = torch.as_tensor(output[:, 8], dtype=torch.float32)
            tau[blending] = ((1 - blend[blending, None]) * tau_script[blending]
                             + blend[blending, None] * tau_rl[blending])
            tau[self.phase == 5] = tau_rl[self.phase == 5]
            tau[self.phase == 6] = 0.
            packed_tau = np.ascontiguousarray(tau[:, P_FROM_CONTROL].double().numpy())
            for i, handle in enumerate(self.handles):
                self.native.rmcs_sim_apply(handle, packed_tau[i], now_ns)
            tau = torch.as_tensor(packed_tau[:, CONTROL_FROM_P], dtype=torch.float32)
            if self.motor_envelope is not None:
                tau = self.motor_envelope.clamp(tau, dq)
            tau[self.mech_failed] = 0.
            self.peak_torque = torch.maximum(self.peak_torque, tau.abs())
            self.max_leg_speed = torch.maximum(self.max_leg_speed, dq[:, self.legs].abs().amax(-1))
            self.leg_above_20_nm_seconds += (tau[:, self.legs].abs() > 20.).float() * self.dt
            effort = torch.zeros_like(env.nominal)
            effort[:, env.ids] = tau
            mean_force = torch.zeros_like(env.contact_force)
            gap = torch.zeros(self.count)
            for _ in range(self.args.physics_substeps):
                effort[:, env.spring_ids] = env.v5.spring_efforts(data.joint_pos.torch[:, env.spring_ids])
                env.robot.set_joint_effort_target_index(target=effort)
                env.robot.write_data_to_sim()
                env.sim.step(render=False)
                env.robot.update(env.dt)
                for ids, view in env.contact_views:
                    matrix = wp.to_torch(view.get_contact_force_matrix(dt=env.dt))
                    mean_force[ids] += matrix.reshape(len(ids), env.body_count, -1, 3).sum(2)
                gap = torch.maximum(gap, env.closure_gap())
            env.contact_force.copy_(mean_force / self.args.physics_substeps)
            self.previous_applied = data.applied_torque.torch[:, [env.ids[2], env.ids[5]]].double().numpy().copy()
            compression, _ = env.v5.spring_state(data.joint_pos.torch[:, env.spring_ids], data.joint_vel.torch[:, env.spring_ids])
            self.max_gap = torch.maximum(self.max_gap, gap)
            knee, bounds = data.joint_pos.torch[:, env.knee_ids], env.v5.knee_bounds
            for reason, mask in (("closure_gap", gap > .003),
                                 ("knee_limit", ((knee < bounds[:, 0] - .03) | (knee > bounds[:, 1] + .03)).any(-1)),
                                 ("spring_limit", ((compression < -.001) | (compression > env.v5.stroke + .001)).any(-1))):
                for i in (mask & ~self.mech_failed).nonzero().flatten().tolist():
                    self.failure_reason[i] = reason
                self.mech_failed |= mask
            self.phase[self.mech_failed] = 6
            self.time += self.dt
            self.ticks += 1
            if self.ticks % options.rmcs_feedback_hz == 0:
                print(f"RMCS_SIM_PROGRESS time={self.time:.1f}s phases={self.phase.tolist()}", flush=True)
            if self.ticks % self.policy_steps == 0:
                velocity, omega_truth, gravity_truth, height, _, _, local = env.state()
                truth_tilt = torch.acos((-gravity_truth[:, 2]).clamp(-1., 1.))
                wheel_force = env.contact_force[:, env.wheel_ids].norm(dim=-1)
                body_force = env.contact_force[:, env.nonwheel_ids].norm(dim=-1).amax(-1)
                good = ((self.phase == 5) & ~self.mech_failed & (truth_tilt < math.radians(10.))
                        & ((height - .305).abs() < .02) & (velocity[:, :2].norm(dim=-1) < .2)
                        & (omega_truth.norm(dim=-1) < .5) & (wheel_force.amin(-1) > 2.) & (body_force < 5.))
                self.good_duration = torch.where(good, self.good_duration + env.policy_dt, 0.)
                self.success |= self.good_duration >= 1.
                self.last = torch.cat((height[:, None], truth_tilt[:, None], omega_truth.norm(dim=-1, keepdim=True),
                                       local, self.phase[:, None], tau, compression), -1).numpy()
                self.traces.append(self.last)
                self.sensor_traces.append(output[:, 9:19].copy())
                self.feedback_traces.append(output.copy())
                self.diagnostics.append(torch.cat((q[:, self.legs], dq[:, self.legs], self.desired,
                                                     gravity, omega, velocity, wheel_force, body_force[:, None],
                                                     knee, dq[:, self.wheels]), -1).numpy())
                self.writer.add_scalar("recovery/strict_success_fraction", float(good.float().mean()), self.ticks)

        def report(self):
            result = []
            for i, (pose, method, replica) in enumerate(self.rows):
                result.append({"pose": pose, "method": method, "replica": replica,
                               "success": bool(self.success[i] and self.good_duration[i] >= 1. and not self.mech_failed[i]),
                               "reached_stable_rl": bool(self.success[i]), "final_stable_seconds": float(self.good_duration[i]),
                               "phase": reference.PHASES[int(self.phase[i])], "handover_s": float(self.handover_time[i]),
                               "final_height_m": float(self.last[i, 0]), "final_tilt_deg": float(self.last[i, 1]) * 180 / self.np.pi,
                               "failure": self.failure_reason[i], "max_closure_gap_m": float(self.max_gap[i]),
                               "peak_torque_nm": self.peak_torque[i].tolist(), "max_leg_speed_rad_s": float(self.max_leg_speed[i]),
                               "leg_above_20_nm_seconds": self.leg_above_20_nm_seconds[i].tolist(), "transitions": self.events[i]})
            self.np.savez_compressed(self.args.output / "cpp_feedback.npz", values=self.np.asarray(self.feedback_traces))
            self.np.savez_compressed(self.args.output / "cpp_ticks.npz", inputs=self.np.asarray(self.tick_inputs),
                                    outputs=self.np.asarray(self.tick_outputs))
            (self.args.output / "cpp_metadata.json").write_text(json.dumps({
                "source_sha256": self.cpp_source_hashes, "feedback_hz": options.rmcs_feedback_hz,
                "physics_hz": options.rmcs_feedback_hz * self.args.physics_substeps,
                "sensor_columns": SENSOR_COLUMNS, "controller_inputs": "IMU, encoders, previous PhysX applied wheel torque",
                "contact_truth_used_by_controller": False, "full_rmcs_component_graph_tested": False,
                "hardware_can_usb_ekf_tested": False, "world_wheel_axes_calibrated": True,
                "diagnostic_wheel_brake": options.diagnostic_wheel_brake,
                "thermal_budget": "recorded exposure only, no measured hardware budget"}, indent=2) + "\n")
            return result

    reference.ActivationBench = CppBench
    return reference.main()


if __name__ == "__main__":
    raise SystemExit(main())
