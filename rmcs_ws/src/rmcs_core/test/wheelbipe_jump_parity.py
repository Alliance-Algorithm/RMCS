#!/usr/bin/env python3
"""Compare compiled deployment control with the actual pinned Python training code."""

import argparse
import hashlib
import io
import json
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[4]


def trace(binary, option, rows):
    stream = io.StringIO()
    np.savetxt(stream, rows, fmt="%.17g")
    result = subprocess.run(
        [str(binary), option],
        input=stream.getvalue(),
        text=True,
        capture_output=True,
        check=True,
    )
    return np.loadtxt(io.StringIO(result.stdout), ndmin=2)


def compare(training_repo, revision, binary):
    contract = json.loads(
        (
            ROOT / "rmcs_ws/src/rmcs_bringup/config/contracts/wheelbipe_jump_v4.json"
        ).read_text()
    )
    commit = subprocess.check_output(
        ["git", "-C", str(training_repo), "rev-parse", revision + "^{commit}"],
        text=True,
    ).strip()
    assert commit == contract["training_source_commit"], "Wrong training source"
    paths = [
        "src/wheeled_tasks",
        "contracts",
        "model/wheel_leg_v2.0.2/friction_v1/isaac/gas_spring_model.json",
        "model/gas_spring_identified_symmetric_20261028/model.json",
    ]
    archive = subprocess.check_output(
        ["git", "-C", str(training_repo), "archive", commit, *paths]
    )
    with tempfile.TemporaryDirectory(prefix="wheelbipe-jump-parity-") as directory:
        root = Path(directory)
        with tarfile.open(fileobj=io.BytesIO(archive)) as bundle:
            bundle.extractall(root, filter="data")
        sys.path.insert(0, str(root / "src"))
        from wheeled_tasks.chassis.gas_spring_compensation import GasSpringCompensation
        from wheeled_tasks.wheelbipe_port.control import tuck_spring_effort
        from wheeled_tasks.wheelbipe_port.mixed_curriculum import TASKS, TaskSchedule
        from wheeled_tasks.wheelbipe_port.model import PairedTargets

        gas_path = "model/wheel_leg_v2.0.2/friction_v1/isaac/gas_spring_model.json"
        gas = json.loads((root / gas_path).read_text())
        assert (
            hashlib.sha256((root / gas_path).read_bytes()).hexdigest()
            == contract["gas_model_sha256"]
        )
        settings = {
            "enabled": True,
            "model_file": gas_path,
            "model_sha256": contract["gas_model_sha256"],
            "gain": 1.0,
            "force_extrapolation": "positive_c1_endpoint_trend_v1",
            "physical_curve_outside_geometry": "hold_geometry_endpoint",
            "relative_soft_limits": {
                "minimum": contract["real_state_soft_limits_rad"][0],
                "maximum": contract["real_state_soft_limits_rad"][1],
                "margin": 0.03,
                "projection": "outward_relative_torque_v1",
            },
        }
        spring = GasSpringCompensation(
            settings, root=root, device="cpu", dtype=torch.float64
        )
        paired = PairedTargets(gas, "cpu", overtravel_rad=0.2)
        generator = np.random.default_rng(119)
        n = 4096
        requested = torch.tensor(generator.integers(0, 2, n), dtype=torch.bool)
        flight = torch.tensor(generator.integers(0, 2, n), dtype=torch.bool) & requested
        desired = torch.tensor(generator.uniform(-14.0, 14.0, (n, 4)))
        q = torch.tensor(generator.uniform(-7.0, 7.0, (n, 4)))
        relative = torch.tensor(generator.uniform(-0.35, 1.65, (n, 2)))
        q[:, 1] = q[:, 0] + relative[:, 0]
        q[:, 3] = q[:, 2] - relative[:, 1]
        dq = torch.tensor(generator.uniform(-18.0, 18.0, (n, 4)))
        target = paired(desired, q, extended_mask=requested)
        pd = 80.0 * (target - q) - 1.5 * dq
        holding, _ = spring.evaluate(q)
        extra, gate = tuck_spring_effort(
            q,
            target,
            pd,
            flight,
            holding,
            contract["control"]["tuck_spring_compensation"],
        )
        limited = (pd + extra).clamp(-40.0, 40.0).unflatten(-1, (2, 2))
        coefficients = q.new_tensor([[-1.0, 1.0], [1.0, -1.0]])
        actual_relative = (q.unflatten(-1, (2, 2)) * coefficients).sum(-1)
        outward = (limited * coefficients).sum(-1)
        low, high = contract["real_state_soft_limits_rad"]
        blocked = ((actual_relative >= high - 0.03) & (outward > 0.0)) | (
            (actual_relative <= low + 0.03) & (outward < 0.0)
        )
        final = (
            limited - coefficients * torch.where(blocked, outward / 2.0, 0.0)[..., None]
        ).flatten(-2)
        expected = torch.cat((target, pd, holding, extra, gate, final), -1).numpy()
        rows = np.column_stack((requested, flight, desired, q, dq))
        actual = trace(binary, "--trace", rows)
        error = np.max(np.abs(actual - expected), axis=0)
        np.testing.assert_allclose(actual, expected, atol=1e-8, rtol=1e-10)
        assert gate.max() == 1.0 and (gate > 0).sum() > 100
        assert np.max(np.abs(final.numpy())) <= 40.0
        spec = json.loads(
            (root / "contracts/wheelbipe_foundation_jump_v4.json").read_text()
        )
        count = 1024
        schedule = TaskSchedule(count, "cpu", 119, spec)
        schedule.role.fill_(TASKS.index("jump"))
        schedule.apex = torch.linspace(0.42, 0.54, count, dtype=torch.float64)
        schedule.delay = torch.full((count,), 0.8, dtype=torch.float64)
        elapsed = torch.linspace(
            0.0,
            contract["request"]["outcome_seconds"]
            - contract["request"]["nominal_request_delay_s"]
            - 0.001,
            count,
            dtype=torch.float64,
        )
        reference = schedule.reference(elapsed + schedule.delay)
        context = schedule.context(
            torch.tensor(
                [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=torch.float64
            ).expand(count, -1),
            elapsed + schedule.delay,
        )
        states = torch.where(
            reference["crouch"],
            1,
            torch.where(reference["push"], 2, torch.where(reference["flight"], 3, 4)),
        )
        expected_clock = torch.cat(
            (reference["height"][:, None], states[:, None], context), -1
        ).numpy()
        actual_clock = trace(
            binary, "--reference", np.column_stack((schedule.apex, elapsed))
        )
        np.testing.assert_allclose(actual_clock, expected_clock, atol=1e-12, rtol=1e-12)
        return {
            "training_commit": commit,
            "control_samples": n,
            "reference_samples": count,
            "max_absolute_error": float(error.max()),
            "max_reference_error": float(np.max(np.abs(actual_clock - expected_clock))),
            "compensation_positive_cases": int((gate > 0).sum()),
            "actual_limit_blocked_cases": int(blocked.sum()),
            "physical_robot_tested": False,
            "learned_behavior_tested": False,
        }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-repo", type=Path, required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--test-binary", type=Path, required=True)
    args = parser.parse_args()
    print(
        json.dumps(
            compare(args.training_repo, args.revision, args.test_binary.resolve()),
            indent=2,
        )
    )
