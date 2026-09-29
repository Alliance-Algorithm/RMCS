#!/usr/bin/env python3
"""Audit collected-controller identity against the intended RL PD contract.

This checks recorded algebra and source/config provenance. It does not simulate
counterfactual robot motion or estimate physical plant parameters.
"""
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
RMCS = ROOT.parents[4]
TRAIN = RMCS.parent / "robot_rl/isaac_wheeled_rl_train"
FROZEN = TRAIN / (
    "reports/v6_p0_diag_probe_20260928/train_control_verified/"
    "checkpoints/update_00000010/contract.json"
)


def main():
    contract = json.loads(FROZEN.read_text())
    profile = json.loads((ROOT / "response_1khz.json").read_text())["controller"]
    bag = np.load(ROOT / "response_1khz.npz")
    active = bag["phase"] == 2
    ep = bag["position_error_model"][active]
    ev = bag["speed_error_model"][active]
    dq = bag["dq_api"][active]
    dq_ref = bag["dq_ref_model"][active]
    v_target = bag["velocity_target_model"][active]
    integral_torque = bag["torque_integral_model"][active]
    preclip = bag["tau_preclip_model"][active]

    assert profile["side"] == "left"
    assert profile["model_sign"][:2] == [1.0, 1.0]
    assert profile["control_frequency_hz"] == 1000.0
    assert profile["kp_position"][:2] == [6.0, 6.0]
    assert profile["kp_velocity"][:2] == [10.0, 10.0]
    assert profile["ki_velocity"][:2] == [50.0, 50.0]
    assert profile["gravity_feedforward_model"][:2] == [0.0, 0.0]
    assert contract["v5_control"]["leg_kp"] == 60.0
    assert contract["v5_control"]["leg_kd"] == 2.0
    assert contract["physics_dt"] == 0.005
    assert contract["policy_dt"] == 0.02

    checks = {
        "velocity_target_rad_s": np.max(abs(v_target - np.clip(dq_ref + 6 * ep, -4, 4))),
        "velocity_error_rad_s": np.max(abs(ev - (v_target - dq))),
        "preclip_torque_nm": np.max(abs(preclip - (10 * ev + integral_torque))),
        "limited_torque_nm": np.max(abs(bag["tau_cmd_api"][active] - np.clip(preclip, -20, 20))),
    }
    assert all(error < 1e-10 for error in checks.values()), checks
    files = {
        "rmcs_action": RMCS / "rmcs_ws/src/rmcs_rl/src/action.cpp",
        "rmcs_timing": RMCS / "rmcs_ws/src/rmcs_rl/src/rl_controller.cpp",
        "training_control": TRAIN / "src/wheeled_tasks/chassis/v5_control.py",
        "training_step": TRAIN / "src/wheeled_tasks/chassis/env.py",
        "frozen_training_contract": FROZEN,
    }
    action = files["rmcs_action"].read_text()
    timing = files["rmcs_timing"].read_text()
    assert "60.0 * (policy_targets_[i] - q_[i]) - 2.0 * dq_[i]" in action
    assert "rate / 200.0" in timing
    report = {
        "status": "controller_contract_audited_not_plant_identified",
        "source_run": "left-pair-multiband-20260928T040116Z",
        "mcap_sha256": "a052c14e7070c7ad6171cc64bc6fe9c993d1c7521a20ff00a5018bae656d1232",
        "active_samples": int(active.sum()),
        "recorded_controller_algebra_max_absolute_error": {k: float(v) for k, v in checks.items()},
        "rl_leg_pd": {
            "formula": "tau_request_model = 60 * (q_target_model - q_feedback_model) - 2 * dq_feedback_model",
            "kp_nm_per_rad": 60.0,
            "kd_nm_s_per_rad": 2.0,
            "integral": False,
            "reference_velocity_feedforward": False,
            "gravity_feedforward": False,
            "policy_period_s": 0.02,
            "pd_period_s": 0.005,
            "mit_transmit_period_s": 0.001,
            "inter_update_behavior": "hold computed torque; refresh feedback on next PD tick",
            "mit_kp": 0.0,
            "mit_kd": 0.0,
            "model_torque_clip_nm": 40.0,
            "downstream": "soft-limit projection, optional recovery envelope/budget, J-transpose mapping, hardware torque clip, MIT quantization",
            "scope": "leg drives only; wheel controller is velocity P",
        },
        "collected_controller": {
            "kind": "position_P_velocity_PI_cascade",
            "period_s": 0.001,
            "kp_position": 6.0,
            "kp_velocity": 10.0,
            "ki_velocity": 50.0,
            "velocity_goal_clip_rad_s": 4.0,
            "integral_limit": 0.4,
            "torque_clip_nm": 20.0,
            "reference_velocity_feedforward": True,
        },
        "manufacturer_basis": {
            "local_manual": "/home/yukikaze/Downloads/DM-J8009P-2EC减速电机说明书V1.0.pdf",
            "parameter_table_page": 8,
            "rated_torque_nm": 20.0,
            "peak_torque_nm": 40.0,
            "configured_mit_torque_encoding_max_nm": 54.0,
            "drive_registers_read_this_analysis": False,
            "peak_duration_and_measured_torque_speed_curve": "not identified",
        },
        "excluded_from_training_parameters": [
            "1 kHz cascade gain recommendations and 1 kHz pure-PD screening results",
            "5-90 Hz surrogate effective mass/damping/stiffness as per-joint physical parameters",
            "velocity reporting phase lag relabelled as USB delay",
        ],
        "next_identification": {
            "known_fixed": ["PD law and gains", "feedback/command cadence", "axis mapping after verification", "existing gas-spring curve", "validated geometry and rigid-body inertias"],
            "unknown_candidates": ["joint losses", "unmodelled reflected inertia", "effective torque response/gain", "feedback reporting dynamics", "identifiable aggregate delay"],
            "old_bag_replay": "retain archived 1 kHz cascade and +/-20 Nm; never relabel it as RL PD data",
            "rl_acceptance": "new predefined paired trajectory with RL PD 200 Hz and 50 Hz target hold, simulated using its own feedback in the full fixed-base closed chain",
            "training_parameter_release": False,
        },
        "source_files": {
            name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            for name, path in files.items()
        },
    }
    (ROOT / "rl_pd_contract_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"active_samples": int(active.sum()), "checks": report["recorded_controller_algebra_max_absolute_error"], "status": report["status"]}, indent=2))


if __name__ == "__main__":
    main()
