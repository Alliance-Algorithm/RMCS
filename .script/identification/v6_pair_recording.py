"""Versioned V6 single-side profiles; no robot, ROS or simulator side effects."""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
CALIBRATION = Path(__file__).parent / "calibration/v6_pair_from_dev_rmcs_rl.json"
BASE_PROFILE = ROOT / "rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-pair-pd-loaded-identification.yaml"
REVISION = "pair_v6_recording_v1"
RUNS = ("L01", "L02", "L03", "LJ01", "LJ02", "R01", "R02", "R03", "RJ01", "RJ02")
ASSET_SHA = "875fa71e89d4d669af0a5471b2334f878d302e06f1b354fc245b95cb5d6f29ba"


def generate_profile(run: str, calibration: Path = CALIBRATION, *, beta_min: float = 65.,
                     beta_max: float = 100.) -> dict:
    if run not in RUNS:
        raise ValueError(f"single-side recording run must be one of {RUNS}")
    binding_text = calibration.read_text(encoding="utf-8")
    binding = json.loads(binding_text)
    side = "left" if run.startswith("L") else "right"
    geometry = binding["sides"][side]
    profile = yaml.safe_load(BASE_PROFILE.read_text())
    p = profile["wheel_leg_pair_identification_controller"]["ros__parameters"]
    for key in list(p):
        if key.startswith("multiband_"):
            del p[key]
    p.update(side=side, probe_pattern="calibrated_recording", trajectory_revision=REVISION,
             model_sign=binding["model_sign"], model_offset=binding["model_offset_rad"],
             probe_measured_delta_policy="stop", ready_timeout_s=20.,
             spring_delta_min=[-.474, -1.625], spring_delta_max=[1.625, .474],
             spring_margin=0.00001, recording_run=run,
             recording_hip_sign=geometry["hip_sign"], recording_hip_zero=geometry["hip_zero_rad"],
             recording_beta_degrees=geometry["beta_degrees"], recording_delta_rad=geometry["delta_rad"],
             recording_beta_min_deg=float(beta_min), recording_beta_max_deg=float(beta_max),
             recording_beta_tolerance_deg=2., recording_arrival_speed=.08,
             recording_arrival_stable_s=.5, recording_arrival_timeout_s=10.,
             recording_center_step_rad=.01, recording_center_max_rad=.20,
             recording_move_s=2., recording_dwell_s=4., recording_baseline_s=30.,
             calibration_sha256=hashlib.sha256(binding_text.encode()).hexdigest(),
             calibration_binding_json=binding_text, asset_manifest_sha256=binding["asset_manifest_sha256"])
    recorder = profile["wheel_leg_identification_recorder"]["ros__parameters"]
    recorder.update(side=side, trajectory_revision=REVISION, recording_run=run,
                    asset_manifest_sha256=binding["asset_manifest_sha256"], asset_path=binding["asset_path"],
                    calibration_sha256=p["calibration_sha256"], calibration_binding_json=binding_text,
                    reference_frequency_hz=50.,
                    initial_pose_provenance="fresh feedback at arm; V6 API map and continuous pair branch",
                    beta_source="calibrated FK estimate; independent physical verification recorded separately")
    broadcaster = profile["value_broadcaster"]["ros__parameters"]["forward_list"]
    broadcaster += ["/wheel_leg/identification/recording/" + name for name in (
        "beta_reference_deg", "theta_measured_deg", "center_delta_rad", "segment_time_s",
        "admission", "skipped_segment", "qualified_segment", "cycle", "jump_phase")]
    errors = check_recording_profile(profile)
    if errors:
        raise ValueError("\n".join(errors))
    return copy.deepcopy(profile)


def check_recording_profile(profile: dict) -> list[str]:
    errors: list[str] = []
    p = profile.get("wheel_leg_pair_identification_controller", {}).get("ros__parameters", {})
    r = profile.get("wheel_leg_identification_recorder", {}).get("ros__parameters", {})
    h = profile.get("wheel_leg", {}).get("ros__parameters", {})
    graph = profile.get("rmcs_executor", {}).get("ros__parameters", {})
    finite = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
    run = p.get("recording_run")
    side = "left" if isinstance(run, str) and run.startswith("L") else "right"
    if run not in RUNS or p.get("side") != side or r.get("side") != side:
        errors.append("V6 recording run must identify exactly the selected left/right side")
    components = graph.get("components", [])
    if len(components) != len(set(components)) or sum("hardware::WheelLeg ->" in str(c) for c in components) != 1:
        errors.append("V6 recording requires exactly one hardware owner and unique components")
    for params, expected in ((p, {"control_law": "rl_pd", "experiment_stage": "probe",
                                 "probe_pattern": "calibrated_recording", "phase_api_angle": True,
                                 "reference_frequency_hz": 50., "control_frequency_hz": 1000.,
                                 "trajectory_revision": REVISION, "probe_measured_delta_policy": "stop",
                                 "pd_kp": [160.]*4, "pd_kd": [2.5]*4, "max_torque": [40.]*4,
                                 "ki_velocity": [0.]*4, "integral_limit": [0.]*4,
                                 "gravity_feedforward_model": [0.]*4,
                                 "dm_motor_family": "DM-J8009", "dm_rated_torque_nm": 20., "dm_peak_torque_nm": 40.}),
                             (r, {"recording_run": run, "trajectory_revision": REVISION,
                                  "experiment_kind": "pair", "record_on_double_middle": True,
                                  "sample_frequency_hz": 1000., "control_frequency_hz": 1000.,
                                  "reference_frequency_hz": 50., "encoder_zero_inner_knee_deg": 105.,
                                  "gas_springs_installed": True, "wheels_off_ground": True,
                                  "support_fixture": "rigid_fixed_frame"}),
                             (h, {"dm_mit_position_max": [12.5]*4, "dm_mit_velocity_max": [45.]*4,
                                  "dm_mit_torque_max": [54.]*4, "dm_control_torque_max": [40.]*4})):
        for key, value in expected.items():
            if params.get(key) != value:
                errors.append(f"V6 {key} must be {value!r}")
    try:
        text = p["calibration_binding_json"]
        binding = json.loads(text)
        if hashlib.sha256(text.encode()).hexdigest() != p["calibration_sha256"]:
            errors.append("calibration text/hash mismatch")
        for key in ("calibration_binding_json", "calibration_sha256", "asset_manifest_sha256"):
            if p.get(key) != r.get(key):
                errors.append(f"controller/recorder {key} mismatch")
        if binding["asset_manifest_sha256"] != ASSET_SHA or p["asset_manifest_sha256"] != ASSET_SHA:
            errors.append("V6 stop105 asset hash mismatch")
        if r.get("asset_path") != binding["asset_path"]:
            errors.append("asset path differs from calibration")
        for key, source in (("model_sign", "model_sign"), ("model_offset", "model_offset_rad")):
            if p.get(key) != binding[source]:
                errors.append(f"{key} differs from versioned calibration")
        if binding["model_sign"] != [-1.]*4 or binding["old_model_to_v6_sign"] != -1:
            errors.append("V6 must apply the old-model sign reversal exactly once")
        g = binding["sides"][side]
        for key, source in (("recording_beta_degrees", "beta_degrees"), ("recording_delta_rad", "delta_rad"),
                            ("recording_hip_sign", "hip_sign"), ("recording_hip_zero", "hip_zero_rad")):
            if p.get(key) != g[source]:
                errors.append(f"{key} differs from bound {side} geometry")
        beta, delta = g["beta_degrees"], g["delta_rad"]
        if (len(beta) < 3 or len(beta) != len(delta) or not all(map(finite, beta+delta))
                or not all(30 <= v <= 120 for v in beta)
                or not all(a < b for a, b in zip(beta, beta[1:]))
                or not (all(a < b for a, b in zip(delta, delta[1:])) or all(a > b for a, b in zip(delta, delta[1:])))):
            errors.append("geometry requires a finite, monotone beta/delta branch without extrapolation")
    except (KeyError, TypeError, ValueError, AttributeError):
        errors.append("missing or invalid versioned calibration binding")
    for key, lower, upper in (("recording_beta_min_deg", 45., 65.), ("recording_beta_max_deg", 100., 102.),
                              ("recording_beta_tolerance_deg", .001, 5.), ("recording_arrival_speed", .001, .08),
                              ("recording_arrival_stable_s", .1, 2.), ("recording_arrival_timeout_s", 4., 60.),
                              ("recording_center_step_rad", .0001, .01), ("recording_center_max_rad", .0001, .20),
                              ("recording_move_s", 1., 10.), ("recording_dwell_s", 4., 6.),
                              ("recording_baseline_s", 30., 30.), ("ready_timeout_s", 20., 60.),
                              ("feedback_timeout_s", .001, .05), ("max_tick_gap_s", .001001, .02),
                              ("other_side_speed_limit", .001, 1.)):
        if not finite(p.get(key)) or not lower <= p[key] <= upper:
            errors.append(f"{key} must be finite in [{lower},{upper}]")
    for key in ("root_min", "root_max", "max_speed", "max_acceleration", "braking_acceleration",
                "max_position_error", "max_velocity_error", "max_measured_acceleration"):
        values = p.get(key)
        if not isinstance(values, list) or len(values) != 4 or not all(map(finite, values)):
            errors.append(f"{key} requires four finite values")
        elif key not in ("root_min", "root_max") and not all(v > 0 for v in values):
            errors.append(f"{key} requires positive values")
    for key in ("spring_delta_min", "spring_delta_max"):
        values = p.get(key)
        if not isinstance(values, list) or len(values) != 2 or not all(map(finite, values)):
            errors.append(f"{key} requires two finite values")
    for key in ("joint_margin", "spring_margin"):
        if not finite(p.get(key)) or p[key] < 0:
            errors.append(f"{key} must be finite and nonnegative")
    if all(finite(p.get(k)) for k in ("recording_arrival_timeout_s", "recording_dwell_s", "recording_arrival_stable_s")):
        if p["recording_arrival_timeout_s"] < p["recording_dwell_s"] or p["recording_arrival_stable_s"] > p["recording_dwell_s"]:
            errors.append("arrival timeout/stability must cover dwell")
    return errors


def preview_input(p: dict) -> str:
    first = 0 if p["side"] == "left" else 2
    rad = math.pi / 180
    values = [p["recording_run"], p["recording_hip_sign"], p["recording_hip_zero"],
              p["recording_beta_min_deg"]*rad, p["recording_beta_max_deg"]*rad,
              *p["max_speed"][first:first+2], *p["max_acceleration"][first:first+2],
              p["recording_beta_tolerance_deg"]*rad, p["recording_arrival_speed"],
              p["recording_arrival_stable_s"], p["recording_arrival_timeout_s"],
              p["recording_center_step_rad"], p["recording_center_max_rad"],
              p["recording_move_s"], p["recording_dwell_s"], p["recording_baseline_s"],
              len(p["recording_beta_degrees"])]
    for beta, delta in zip(p["recording_beta_degrees"], p["recording_delta_rad"]):
        values += [beta*rad, delta]
    return " ".join(map(str, values)) + "\n"
