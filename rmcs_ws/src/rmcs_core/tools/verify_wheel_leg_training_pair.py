#!/usr/bin/env python3
"""Replay the new profiles through production C++ without hardware or Isaac Sim."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

import numpy as np
import onnxruntime as ort
import torch
import yaml

from verify_prone_motion_parity import sensor_records, python_frames, sha, require

ROOT = Path(__file__).resolve().parents[4]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--container", default="rmcs_devcontainer-rmcs-develop-1")
    parser.add_argument("--output", type=Path, default=ROOT / "reports/wheel_leg_training_pair_20261103")
    args = parser.parse_args()
    torch.set_num_threads(1)
    args.output.mkdir(parents=True, exist_ok=True)
    report = dict(status="incomplete", hardware_io=False, behavior_qualified=False)
    receipt = args.output / "parity.json"
    receipt.write_text(json.dumps(report, indent=2) + "\n")
    cfg_dir = ROOT / "rmcs_ws/src/rmcs_bringup/config"
    pair = json.loads((cfg_dir / "contracts/wheel_leg_training_pair_20261103.json").read_text())
    sys.path.insert(0, str(args.training_root / "src"))
    from wheeled_tasks.chassis.standing_local_odometry import LocalStandingOdometry
    observer = LocalStandingOdometry(1, "cpu", .01, pair["prone"]["observation"],
        geometry_directory=args.training_root / "model/wheel_leg_v2.0.2/friction_v1/training")
    records, nominal = sensor_records(args.training_root, observer)
    expected = python_frames(records, observer, nominal)
    data = "\n".join(" ".join(map(str, [r["tick"], int(r["reset"]), int(r["enabled"]),
        *r["q"], *r["dq"], *r["gyro"], *r["gravity"], *r["command"], r["height"], *r["applied"]]))
        for r in records) + "\n"

    def run(binary, arguments, payload, reject=False):
        result = subprocess.run(["docker", "exec", "-i", args.container, "bash", "-lc",
            "source /opt/ros/jazzy/setup.bash && source /workspaces/RMCS/rmcs_ws/install/setup.bash && "
            + shlex.join(["/workspaces/RMCS/rmcs_ws/build/" + binary, *arguments])],
            input=payload, text=True, capture_output=True)
        if reject:
            require(result.returncode != 0 and "does not match" in result.stderr,
                    "Wrong-profile model was not rejected by its contract: " + result.stderr)
            return result.stderr
        require(result.returncode == 0, result.stderr)
        return result.stdout

    lines = run("rmcs_core/wheel_leg_observation_parity", ["--prone-motion-v4-odom-v2"], data).splitlines()
    require(len(lines) == len(records), "Prone frame count differs")
    worst, frames = 0., []
    for row, line, value in zip(records, lines, expected):
        result = np.fromstring(line, sep=" ")
        require(bool(result[0]) == (value is not None), "Prone finite-input rejection differs: " + row["label"])
        if value is not None:
            got = result[1:36].astype(np.float32)
            require(np.array_equal(got[28:], value[28:]), "Prone context/capture differs: " + row["label"])
            worst = max(worst, float(np.max(np.abs(got - value))))
            frames.append(got)
    require(worst < 3e-5, f"Prone Python/C++ frame error: {worst}")
    report.update(prone_frames=len(records), prone_frame_max_absolute_error=worst)
    # V14's raw positions, clip-before-scale and different action order need
    # their own frame check; shared dimensions cannot establish compatibility.
    rng = np.random.default_rng(119)
    order = [0, 2, 1, 3, 4, 5]
    nominal6 = np.r_[nominal.numpy(), 0., 0.]
    wb_rows, wb_expected = [], []
    for i in range(256):
        q = nominal6 + rng.uniform(-10., 10., 6)
        dq = rng.uniform(-250., 250., 6)
        gyro = rng.uniform(-150., 150., 3)
        gravity = rng.uniform(-1., 1., 3)
        command = rng.uniform(-3., 3., 3)
        height = float(rng.uniform(-.5, 1.5))
        applied = rng.uniform(-120., 120., 6)
        wb_rows.append(" ".join(map(str, [i, 0, 1, *q, *dq, *gyro, *gravity, *command, height, *applied])))
        frame = torch.zeros(35)
        frame[:3] = torch.tensor(command, dtype=torch.float32).clamp(-100., 100.)
        frame[3] = torch.tensor(height, dtype=torch.float32).clamp(0., 1.) * 5.
        frame[4:7] = torch.tensor(gyro, dtype=torch.float32).clamp(-100., 100.) * .5
        frame[7:10] = torch.tensor(gravity, dtype=torch.float32).clamp(-100., 100.)
        delta = torch.tensor(q, dtype=torch.float32) - torch.tensor(nominal6, dtype=torch.float32)
        frame[10:14] = delta[order[:4]].clamp(-100., 100.)
        frame[16:22] = torch.tensor(dq[order], dtype=torch.float32).clamp(-200., 200.) * .1
        frame[22:28] = torch.tensor(applied[order], dtype=torch.float32).clamp(-100., 100.)
        frame[28] = 1.
        wb_expected.append(frame.numpy())
    wb_result = run("rmcs_core/wheel_leg_observation_parity", ["--wheelbipe-v202"], "\n".join(wb_rows) + "\n")
    wb_actual = np.array([np.fromstring(line, sep=" ")[1:36] for line in wb_result.splitlines()])
    wb_error = float(np.abs(wb_actual - np.array(wb_expected)).max())
    require(wb_error < 1e-6, f"Wheelbipe frame arithmetic differs: {wb_error}")
    report.update(wheelbipe_frames=len(wb_expected), wheelbipe_frame_max_absolute_error=wb_error)
    from wheeled_tasks.chassis.gas_spring_compensation import GasSpringCompensation
    gas = GasSpringCompensation(pair["prone"]["gas_spring_feedforward"], root=args.training_root,
                               device="cpu", dtype=torch.float64)
    delta = np.linspace(-.1408, 1.4282, 1201)
    gas_q = np.c_[delta*0., delta, delta*0., -delta]
    gas_data = "\n".join(" ".join(map(str, row)) for row in gas_q) + "\n"
    gas_result = run("rmcs_core/wheel_leg_gas_spring_parity", ["--full-vehicle-v202"], gas_data)
    cpp_gas = np.loadtxt(gas_result.splitlines())
    python_gas, _ = gas.evaluate(torch.from_numpy(gas_q))
    gas_error = float(np.abs(cpp_gas[:, :4] - python_gas.numpy()).max())
    require(gas_error < 1e-6, f"Measured spring compensation differs: {gas_error}")
    report.update(gas_samples=len(gas_q), gas_max_absolute_error_Nm=gas_error)
    report["profiles"] = {}
    for name, filename, model, mode in (
        ("prone", "wheel-leg-infantry-prone-v4.yaml", "prone_motion_v4_100hz.onnx", "--prone-200hz"),
        ("wheelbipe", "wheel-leg-infantry-wheelbipe.yaml", "wheelbipe_v202_50hz.onnx", "--wheelbipe")):
        config_file = cfg_dir / filename
        cfg = yaml.safe_load(config_file.read_text())
        bridge, control, hardware = [cfg[key]["ros__parameters"] for key in ("rl_bridge", "wheel_leg_controller", "wheel_leg")]
        require(bridge["policy_rate"] == pair[name]["policy_hz"] and control["pd_frequency"] == 200., "Profile clocks differ")
        require(control["gas_spring_compensation_enabled"] == (name == "prone"), "Spring feedforward differs")
        require(hardware["joint_control_mode"] == "torque" and not hardware["allow_set_zero"], "Drive mode differs")
        require((hardware["joint_position_kp"], hardware["joint_position_kd"])
                == ((160., 2.5) if name == "prone" else (60., 2.)), "Leg gains differ")
        require(bridge["expected_observation_contract_sha256"] == pair[name]["observation_contract_sha256"], "Observation identity differs")
        path = ROOT / "rmcs_ws/src/rmcs_rl/models/wheel_leg" / model
        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        sample = np.array(frames[::20], dtype=np.float32) if name == "prone" else np.random.default_rng(119).uniform(-5, 5, (256, 35)).astype(np.float32)
        reference = session.run(None, {"obs": sample})[0]
        payload = "\n".join(" ".join(map(str, row)) for row in sample) + "\n"
        cpp = np.loadtxt(run("rmcs_rl/prone_policy_parity", [str(path), mode], payload).splitlines())
        error = float(np.abs(cpp - reference).max())
        require(error < 2e-5, f"C++ policy engine differs for {name}: {error}")
        other = "--wheelbipe" if name == "prone" else "--prone-200hz"
        run("rmcs_rl/prone_policy_parity", [str(path), other], payload, reject=True)
        report["profiles"][name] = dict(config_sha256=sha(config_file), model_sha256=sha(path),
            policy_samples=len(sample), policy_max_absolute_error=error,
            wrong_profile_model_rejected=True)
    report["status"] = "passed"
    receipt.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
