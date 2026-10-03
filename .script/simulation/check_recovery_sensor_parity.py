#!/usr/bin/env python3
"""Replay identical Isaac sensor samples through the original Python observer.

The controller's actual C++ outputs are taken from cpp_ticks.npz. This checks
the calibrated kinematics independently of closed-loop trajectory divergence.
No simulator, contact truth or hardware is needed for this replay.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-repo", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    options = parser.parse_args()
    training = options.training_repo.resolve()
    report = json.loads((options.run / "report.json").read_text())
    source = training / "src/wheeled_tasks/chassis/recovery_observer.py"
    sys.path.insert(0, str(training / "tools"))
    spec = importlib.util.spec_from_file_location("recovery_reference", source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    bundle = Path(report["arguments"]["bundle"])
    if not bundle.is_absolute():
        bundle = training / bundle
    model = json.loads((bundle / "model_spec.json").read_text())
    calibration = json.loads((training / "docs/evidence/v5_self_righting_reference_20260926.json").read_text())
    traces = np.load(options.run / "cpp_ticks.npz")
    inputs, outputs = traces["inputs"], traces["outputs"]
    dt = 1.0 / report["feedback_hz"]
    observer = module.RecoveryObserver(model, calibration, "cpu", inputs.shape[1], dt)
    maximum = {"conditional_height_m": 0., "wheel_height_difference_m": 0.,
               "world_wheel_rate_rad_s": 0., "inner_knee_deg": 0.}
    checked = 0
    p_to_control = [0, 1, 4, 2, 3, 5]
    torch.set_num_threads(1)
    for sample, native in zip(inputs, outputs):
        value = torch.as_tensor(sample, dtype=torch.float32)
        observed = observer.update(value[:, :6][:, p_to_control], value[:, 6:12][:, p_to_control],
                                   value[:, 12:15], value[:, 15:18], value[:, 25:29], value[:, 18:21])
        # The native bridge stops updating after a latched fault; skip those
        # sentinel rows and the disabled release, while keeping them in replay.
        valid = native[:, 11].astype(bool) & (native[:, 6] != 0) & (native[:, 6] != 6)
        checked += int(valid.sum())
        pairs = {
            "conditional_height_m": (native[:, 9], observed.height_if_wheels_grounded.numpy()),
            "wheel_height_difference_m": (native[:, 10], observed.wheel_height_difference.numpy()),
            "world_wheel_rate_rad_s": (native[:, 34:36], observed.wheel_world_omega_y.numpy()),
            "inner_knee_deg": (native[:, 23:25], observed.knee_inner_deg.numpy()),
        }
        for name, (actual, expected) in pairs.items():
            if valid.any():
                maximum[name] = max(maximum[name], float(np.abs(actual[valid] - expected[valid]).max()))
        # Give both observers the same issued pulse, rather than letting their
        # different CAN corroboration requirements change the replay stimulus.
        observer.probe.previous_pulse = torch.as_tensor(native[:, 25:27], dtype=torch.float32)
    tolerances = {"conditional_height_m": 1e-6, "wheel_height_difference_m": 1e-6,
                  "world_wheel_rate_rad_s": 1e-3, "inner_knee_deg": 1e-4}
    passed = checked > 0 and all(maximum[key] <= limit for key, limit in tolerances.items())
    result = {"passed": passed, "checked_sensor_rows": checked,
              "python_source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "maximum_absolute_error": maximum, "tolerances": tolerances,
              "scope": "calibrated kinematics on identical inputs; no assertion of identical CAN proof or physics"}
    (options.run / "sensor_parity.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
