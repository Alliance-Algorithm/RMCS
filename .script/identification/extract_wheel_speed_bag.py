#!/usr/bin/env python3
"""Stream a sealed wheel MCAP into full-rate arrays and audit the applied P/current contract.

Run with ROS Jazzy and the matching rmcs_msgs installation sourced. This export
retains timestamp/sequence provenance; it does not qualify a fixed-carrier model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def audit(arrays, params, manifest, wheel_motor_reversed=(True, True)):
    ratio = float(params["wheel_reduction_ratio"])
    if not np.isfinite(ratio) or ratio <= 0:
        raise ValueError("Archived wheel reduction ratio must be positive and finite")
    if len(wheel_motor_reversed) != 2 or any(type(v) is not bool for v in wheel_motor_reversed):
        raise ValueError("Archived wheel directions must contain two booleans")
    signs = np.where(wheel_motor_reversed, -1., 1.)
    active = arrays["phase"] == 2
    ids = arrays["segment_id"][active]
    expected_ids = list(range(len(manifest["segments"])))
    found = np.unique(ids).tolist()
    reasons = []
    if not np.any(active):
        return {"status": "no_running_data", "reasons": ["No phase 2 samples"]}
    if found != expected_ids:
        reasons.append("trajectory segment coverage incomplete")
    if not np.any(arrays["phase"] == 3):
        reasons.append("completion event missing")
    if np.any(arrays["failure_reason"] != 0):
        reasons.append("controller stop/failure present")
    if np.max(arrays["dropped_samples"]) != 0:
        reasons.append("recorder reported dropped samples")
    if np.unique(arrays["repetition_id"][active]).size != 1:
        reasons.append("multiple attempts in one bag; split before fitting")
    if np.any(arrays["dm_status"][active] != 0):
        reasons.append("DM drive did not remain disabled")
    if np.any(arrays["actuation_scope"][active] != 3):
        reasons.append("wheel command gate was not continuously active")
    if np.any(arrays["tx_can_bus"][active, 4:6] != 0) or np.any(arrays["tx_can_id"][active, 4:6] != 0x200):
        reasons.append("wheel frame bus or CAN ID mismatch")
    valid = (ids >= 0) & (ids < len(expected_ids))
    if not np.all(valid):
        return {"status": "invalid_segment_ids", "reasons": reasons + ["unknown segment id"]}
    servo = np.array([[not s["coast"] and v != 0 for v in s["axis_scale"]]
                      for s in manifest["segments"]], dtype=bool)[ids]
    target = arrays["wheel_velocity_target_api"][active]
    speed = arrays["dq_api"][active, 4:6]
    preclip = arrays["tau_preclip_api"][active, 4:6]
    requested = arrays["tau_cmd_api"][active, 4:6]
    expected_preclip = np.where(servo, params["wheel_velocity_kp"] * (target-speed), 0.)
    expected_request = np.clip(expected_preclip, -params["wheel_torque_cap"], params["wheel_torque_cap"])
    finite = np.isfinite(np.stack([target, speed, preclip, requested])).all()
    p_error = float(np.max(np.abs(preclip-expected_preclip))) if finite else None
    limit_error = float(np.max(np.abs(requested-expected_request))) if finite else None
    if not finite or p_error > 1e-9 or limit_error > 1e-9:
        reasons.append("P/current limit algebra mismatch")
    raw = arrays["tx_frame_bytes"][active, 32:40].astype(np.int32)
    counts = np.stack([raw[:, 0]*256+raw[:, 1], raw[:, 2]*256+raw[:, 3]], axis=1)
    counts = np.where(counts >= 32768, counts-65536, counts)
    full_scale = 20*ratio*.3*187/3591
    encoded = signs*counts*full_scale/16384
    quantum = full_scale/16384
    encoding_error = float(np.max(np.abs(encoded-requested)))
    frame_error = float(np.max(np.abs(encoded-arrays["tau_frame_api"][active, 4:6])))
    if not np.isfinite(encoding_error) or encoding_error > quantum*.501 or frame_error > 1e-9:
        reasons.append("raw C620 current packet differs from requested/recorded effort")
    feedback = arrays["feedback_frame_bytes"][active, 32:48].astype(np.int32).reshape(-1, 2, 8)
    rpm = feedback[:, :, 2]*256+feedback[:, :, 3]
    rpm = np.where(rpm >= 32768, rpm-65536, rpm)
    decoded_speed = signs*rpm/ratio/60*2*np.pi
    speed_conversion_error = float(np.max(np.abs(decoded_speed-speed)))
    if not np.isfinite(speed_conversion_error) or speed_conversion_error > 1e-9:
        reasons.append("raw M3508 RPM differs from output-shaft speed for the archived reduction")
    t_ns = arrays["control_steady_ns"][active].astype(np.int64)
    intervals = np.diff(t_ns)*1e-9
    tick_gaps = int(np.count_nonzero(np.diff(arrays["tick"][active].astype(np.int64)) != 1))
    if tick_gaps or (intervals.size and (intervals.max() > .002 or intervals.min() <= 0)):
        reasons.append("execution/sample clock gaps: use contiguous windows for dynamics")
    ages = (t_ns[:, None]-arrays["feedback_steady_ns"][active].astype(np.int64))*1e-6
    segments = []
    for ident in found:
        mask = ids == ident
        desc = manifest["segments"][ident]
        local = speed[mask]
        error = target[mask]-local
        segments.append({"id": ident, "label": desc["label"], "samples": int(mask.sum()),
            "validation": desc["validation"], "axis_scale": desc["axis_scale"],
            "mode": "zero_current" if desc["coast"] else "velocity_P",
            "speed_start": local[0].tolist(), "speed_end": local[-1].tolist(),
            "speed_rmse": np.sqrt(np.mean(error**2, axis=0)).tolist(),
            "peak_abs_leg_velocity": np.max(np.abs(arrays["dq_api"][active, :4][mask]), axis=0).tolist()})
    return {"status": "complete_for_analysis" if not reasons else "inspect_before_fitting", "reasons": reasons,
        "wheel_reduction_ratio": ratio, "wheel_motor_reversed": list(wheel_motor_reversed),
        "wheel_output_nm_per_amp": full_scale/20,
        "raw_rpm_conversion_max_error_rad_s": speed_conversion_error,
        "samples": int(len(active)), "running_samples": int(active.sum()), "segments": segments,
        "p_algebra_max_error_nm": p_error, "limit_algebra_max_error_nm": limit_error,
        "raw_current_encoding_max_error_nm": encoding_error, "frame_telemetry_max_error_nm": frame_error,
        "tick_gap_count": tick_gaps, "sample_interval_max_ms": float(intervals.max()*1000) if intervals.size else None,
        "feedback_age_max_ms": ages.max(axis=0).tolist(),
        "wheel_speed_range_rad_s": [speed.min(axis=0).tolist(), speed.max(axis=0).tolist()],
        "wheel_command_peak_nm": np.max(np.abs(requested), axis=0).tolist(),
        "wheel_current_command_peak_a": (np.max(np.abs(counts), axis=0)*20/16384).tolist(),
        "leg_velocity_peak_rad_s": np.max(np.abs(arrays["dq_api"][active, :4]), axis=0).tolist(),
        "note": "DM axes are passive, not clamped. Carrier motion and feedback age must be qualified before fitting wheel dynamics. CAN bytes are host submissions, not receipt acknowledgements."}


def main():
    import rosbag2_py
    from ament_index_python.packages import get_package_share_directory
    from rclpy.serialization import deserialize_message
    from rmcs_msgs.msg import WheelLegIdentificationSample
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    definition = Path(get_package_share_directory("rmcs_msgs"))/"msg/WheelLegIdentificationSample.msg"
    if definition.read_bytes() != (args.run/definition.name).read_bytes():
        raise SystemExit("Recorded message schema differs from installed type; use the matching installation")
    profile = yaml.safe_load((args.run/"profile.yaml").read_text())
    params = profile["wheel_leg_wheel_identification_controller"]["ros__parameters"]
    recorder = profile["wheel_leg_identification_recorder"]["ros__parameters"]
    if params["wheel_reduction_ratio"] != recorder["wheel_reduction_ratio"]:
        raise SystemExit("Controller and recorder reduction ratios disagree")
    manifest = json.loads((args.run/"trajectory-plan.json").read_text())
    if params["trajectory_revision"] != manifest["revision"]:
        raise SystemExit("Trajectory manifest/profile mismatch")
    metadata = yaml.safe_load((args.run/"bag/metadata.yaml").read_text())
    n = metadata["rosbag2_bagfile_information"]["message_count"]
    specs = {k: (np.float64, 6) for k in ("q_api", "dq_api", "torque_fb_api", "tau_preclip_api",
        "tau_cmd_api", "tau_frame_api", "feedback_current_a", "temperature_c")}
    specs.update(wheel_velocity_target_api=(np.float64, 2), imu_gyro=(np.float64, 3), imu_quaternion_xyzw=(np.float64, 4))
    specs.update({k: (np.uint64, 0) for k in ("control_steady_ns", "tick", "dropped_samples", "repetition_id")})
    specs.update({k: (np.uint64, 6) for k in ("feedback_steady_ns", "feedback_sequence", "tx_queued_steady_ns")})
    specs.update({k: (np.int32, 0) for k in ("phase", "segment_id", "segment_role", "segment_waveform",
        "wheel_mode", "failure_reason", "actuation_scope", "feedback_fresh", "enable_requested")})
    specs.update(dm_status=(np.int32, 4), dm_fault=(np.int32, 4), tx_frame_bytes=(np.uint8, 48),
        feedback_frame_bytes=(np.uint8, 48), tx_can_bus=(np.uint8, 6), tx_can_id=(np.uint32, 6), tx_kind=(np.uint8, 6))
    data = {k: np.empty((n, size) if size else n, dtype=dtype) for k, (dtype, size) in specs.items()}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.run/"bag"), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    i = 0
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic != "/wheel_leg/identification/sample":
            raise SystemExit("Unexpected topic in wheel-only telemetry bag")
        msg = deserialize_message(payload, WheelLegIdentificationSample)
        for key, values in data.items(): values[i] = getattr(msg, key)
        i += 1
        if i % 100000 == 0: print(f"Extracted {i}/{n}", flush=True)
    if i != n: raise SystemExit("Bag count differs from sealed metadata")
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output/"wheel_1khz.npz", **data)
    report = audit(data, params, manifest, recorder["wheel_motor_reversed"])
    report.update(source_run=str(args.run), profile_sha256=hashlib.sha256((args.run/"profile.yaml").read_bytes()).hexdigest())
    (args.output/"wheel_quality.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k != "segments"}, indent=2), flush=True)


if __name__ == "__main__":
    main()
