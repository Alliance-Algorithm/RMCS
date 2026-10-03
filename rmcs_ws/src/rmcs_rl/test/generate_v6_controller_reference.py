#!/usr/bin/env python3
"""Freeze native Torch observations/actions/PD for synthetic RMCS API snapshots.

Run with the training environment's Python. This imports the sealed handoff's
scut_observation.py and V5Control methods, not copies of their equations. The
synthetic wheel signs and wide hinge limits test software coordinate transport;
they do not certify the installed hardware calibration or mechanism domain.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff-dir", type=Path, required=True)
    parser.add_argument("--training-repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    import numpy as np
    import onnxruntime as ort
    import torch

    bundle = args.handoff_dir.resolve()
    for line in (bundle / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split(maxsplit=1)
        name = name.lstrip("*")
        if hashlib.sha256((bundle / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"Handoff SHA mismatch: {name}")
    deployment = bundle / "deployment"
    sys.path.insert(0, str(args.training_repo.resolve() / "src"))
    observation = load_module("v6_handoff_observation", deployment / "scut_observation.py")
    native_control = load_module("v6_handoff_control", deployment / "v5_control.py")
    interface = json.loads((deployment / "interface.json").read_text())
    contract = json.loads((bundle / "policy.onnx.contract.json").read_text())
    prior = json.loads((deployment / "v6_wheel_response_control_200hz_v1.json").read_text())
    manifest = json.loads((deployment / "manifest.json").read_text())
    c_from_p = interface["output"]["control_from_policy"]
    p_from_c = interface["output"]["policy_from_control"]
    tensor = lambda values: torch.tensor([values], dtype=torch.float32)
    nominal_p = interface["output"]["nominal_position_exact_from_manifest_rad"]
    nominal_c = tensor([manifest["nominal_joint_pos"][name] for name in native_control.V5Control.ACTIVE])
    # decode/motor_efforts do not use spring geometry. Initialize only their
    # fields from the sealed contracts and call the original native methods.
    control = native_control.V5Control.__new__(native_control.V5Control)
    control.settings = contract["v5_control"]
    control.nominal = nominal_c[0]
    control.wheel_prior = prior["actuators"]["wheel"]
    control.action_bounds = tensor([3, 3, 9, 3, 3, 9])[0]
    reference = SimpleNamespace(elapsed=torch.zeros(1), terrain_mode=torch.zeros(1),
                                phase=torch.zeros(1), STEP_LOWER=4)
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(bundle / "policy.onnx"), options,
                                   providers=["CPUExecutionProvider"])
    offsets = interface["hardware_mapping_reference"]["leg_offset_rad"]
    jacobian = tensor([-1, -1, -1, -1, 1, 1])[0]
    previous_c = tensor([0] * 6)
    frames = []
    specs = [
        ("nominal_reset", 0, True, [0]*6, [0]*6, [0, 0, 0], 0.0, [0, 0, 0]),
        ("fresh_pd_feedback_with_held_target", 5, False, [.02, -.03, .04, -.01, 2, -3],
         [.3, -.5, .7, -.9, 3, -4], [.1, -.2, .3], .12, [0, 0, 0]),
        ("imu_mapping_and_previous_action", 15, True, [.02, -.03, .04, -.01, 2, -3],
         [.3, -.5, .7, -.9, 3, -4], [.1, -.2, .3], .12, [.03, 0, .08]),
        ("common_encoder_turns", 20, True,
         [2*math.pi, 2*math.pi, -2*math.pi, -2*math.pi, 12, -14],
         [0]*6, [0, 0, 0], .0, [0, 0, 0]),
        ("observation_and_action_clipping", 20, True, [.03, -.02, -.04, .05, 12, -14],
         [1000, -2000, 3000, -4000, 6000, -7000], [500, -450, 600], .0, [0, 0, 0]),
        ("previous_clipped_action_then_quiet", 20, True, [0]*6, [0]*6, [0, 0, 0], .0, [-.03, 0, -.08]),
        ("held_target_zero_error_probe", 5, False, [0]*6, [0]*6, [0, 0, 0], .0, [0, 0, 0]),
    ]
    for name, advance, policy_step, displacement, dq_p, gyro_base, roll, command in specs:
        if name == "common_encoder_turns":
            # This case starts a separate controller session at continuous
            # encoder coordinates; it is not a spurious 2pi feedback jump.
            saved = previous_c, legs, wheels, expected_obs, raw_p
            previous_c = tensor([0]*6)
        q_p = [a+b for a, b in zip(nominal_p, displacement)]
        if name == "held_target_zero_error_probe":
            q_p = legs[0].tolist() + [0., 0.]
            dq_p = [0., 0., 0., 0.] + wheels[0].tolist()
        q_api = [offsets[i] - q_p[i] for i in range(4)] + q_p[4:]
        dq_api = [-v for v in dq_p[:4]] + dq_p[4:]
        # Synthetic +90deg base<-IMU mount. q_world_imu=q_world_base*q_base_imu.
        s, c, half = math.sin(roll/2), math.cos(roll/2), math.sqrt(.5)
        orientation_imu = [c*half, s*half, -s*half, c*half]
        gyro_imu = [gyro_base[1], -gyro_base[0], gyro_base[2]]
        gravity = [0, -math.sin(roll), -math.cos(roll)]
        q_c, dq_c = tensor(q_p)[:, c_from_p], tensor(dq_p)[:, c_from_p]
        if policy_step:
            obs = observation.build_manual35(
                tensor(gyro_base), tensor(gravity), tensor([command[0], command[2], .305]),
                q_c, dq_c, previous_c, nominal_c, torch.zeros(1, dtype=torch.bool),
                torch.zeros(1), reference)
            raw_p = session.run(["actions"], {"obs": obs.numpy().astype(np.float32)})[0]
            legs, wheels, previous_c = control.decode(torch.from_numpy(raw_p)[:, c_from_p], q_c)
            expected_obs = obs[0].tolist()
        tau_c = control.motor_efforts(q_c, dq_c, legs, wheels)
        targets_p = torch.cat((legs, wheels), -1)[0].tolist()
        # legs are already [LH,LK,RH,RK] and wheels [LW,RW], i.e. P order.
        torque_p = tau_c[:, p_from_c]
        frames.append(dict(name=name, advance_ms=advance, policy_step=policy_step,
                           q_api=q_api, dq_api=dq_api, orientation_imu_wxyz=orientation_imu,
                           gyro_imu=gyro_imu, command=command, obs=expected_obs,
                           raw_actions=raw_p[0].tolist(),
                           clipped_actions=previous_c[:, p_from_c][0].tolist(),
                           targets=targets_p, torque_model=torque_p[0].tolist(),
                           torque_api=(torque_p*jacobian)[0].tolist()))
        if name == "common_encoder_turns":
            previous_c, legs, wheels, expected_obs, raw_p = saved
    sources = [deployment / "scut_observation.py", deployment / "v5_control.py",
               deployment / "interface.json", deployment / "v6_wheel_response_control_200hz_v1.json",
               args.training_repo.resolve() / "src/wheeled_tasks/v40/core.py"]
    result = dict(schema="rmcs-v6-native-controller-fixture-v1",
                  onnx_sha256=interface["identity"]["onnx_sha256"],
                  reference_sources={str(p.relative_to(bundle)) if p.is_relative_to(bundle)
                                     else "training/src/wheeled_tasks/v40/core.py":
                                     hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  scope="synthetic API feedback; J=-I, supplied offsets; wheel signs +1/+1; "
                        "wide synthetic hinge limits; not hardware or V6 mechanism qualification",
                  imu_to_base=[0, -1, 0, 1, 0, 0, 0, 0, 1], cases=frames)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
