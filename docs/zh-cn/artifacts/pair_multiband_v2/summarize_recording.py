#!/usr/bin/env python3
"""Stream a sealed bag into an observation report; no physics-fit acceptance."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rmcs_msgs.msg import WheelLegIdentificationSample
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    installed = Path(get_package_share_directory("rmcs_msgs")) / "msg/WheelLegIdentificationSample.msg"
    if installed.read_bytes() != (args.run / "WheelLegIdentificationSample.msg").read_bytes():
        raise ValueError("Installed and archived message definitions differ")
    profile = yaml.safe_load((args.run / "profile.yaml").read_text())
    config = profile["wheel_leg_pair_identification_controller"]["ros__parameters"]
    side = 0 if config["side"] == "left" else 1
    axes = (2 * side, 2 * side + 1)
    offsets = np.asarray(config["model_offset"])[list(axes)]
    signs = np.asarray(config["model_sign"])[list(axes)]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.run / "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    phases, failures, segments = Counter(), Counter(), {}
    n = dropped = gaps = negative_age = opposite_enabled = 0
    first_ns = previous = last = None
    max_gap_ms = 0.0
    max_step = np.zeros(2)
    max_raw_step = np.zeros(2, dtype=int)
    max_command = np.zeros(2)
    max_frame = np.zeros(2)
    q_min, q_max = np.full(2, np.inf), np.full(2, -np.inf)
    temp_min, temp_max = np.full(2, np.inf), np.full(2, -np.inf)
    max_age = np.zeros(2)
    preview = []
    running_start = running_end = None
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic != "/wheel_leg/identification/sample":
            continue
        m = deserialize_message(payload, WheelLegIdentificationSample)
        n += 1
        if first_ns is None:
            first_ns = m.control_steady_ns
        phases[m.phase] += 1
        if m.failure_reason:
            failures[m.failure_reason] += 1
        dropped = max(dropped, m.dropped_samples)
        q = np.asarray(m.q_api)[list(axes)]
        raw = np.asarray(m.feedback_frame_bytes).reshape(6, 8)[list(axes)].astype(int)
        raw_u = (raw[:, 1] << 8) | raw[:, 2]
        if previous is not None:
            prev_tick, prev_ns, prev_q, prev_raw = previous
            gaps += int(m.tick != prev_tick + 1)
            max_gap_ms = max(max_gap_ms, (m.control_steady_ns - prev_ns) / 1e6)
            if m.tick == prev_tick + 1:
                max_step = np.maximum(max_step, abs(q - prev_q))
                max_raw_step = np.maximum(max_raw_step, abs(raw_u - prev_raw))
        previous = (m.tick, m.control_steady_ns, q, raw_u)
        last = m
        if m.phase != 2:
            continue
        if running_start is None:
            running_start = m.control_steady_ns
        running_end = m.control_steady_ns
        age = np.array([(m.control_steady_ns - m.feedback_steady_ns[j]) / 1e6 for j in axes])
        negative_age += int(any(age < 0))
        max_age = np.maximum(max_age, age)
        opposite_enabled += int(any(m.dm_status[j] != 0 for j in range(4) if j not in axes))
        q_min, q_max = np.minimum(q_min, q), np.maximum(q_max, q)
        temp = np.asarray(m.temperature_c)[list(axes)]
        temp_min, temp_max = np.fmin(temp_min, temp), np.fmax(temp_max, temp)
        cmd = np.asarray(m.tau_cmd_api)[list(axes)]
        frame = np.asarray(m.tau_frame_api)[list(axes)]
        max_command, max_frame = np.fmax(max_command, abs(cmd)), np.fmax(max_frame, abs(frame))
        error = np.asarray(m.position_error_model)[list(axes)]
        delta = float(np.diff(q * signs + offsets)[0])
        seg = segments.setdefault(m.segment_id, {"samples": 0, "role": m.segment_role,
            "waveform": m.segment_waveform, "saturated": np.zeros(2, dtype=int),
            "error_square_sum": np.zeros(2), "error_max_rad": np.zeros(2),
            "delta_min_rad": delta, "delta_max_rad": delta})
        seg["samples"] += 1
        seg["saturated"] += np.asarray(m.torque_limited)[list(axes)]
        seg["error_square_sum"] += error ** 2
        seg["error_max_rad"] = np.maximum(seg["error_max_rad"], abs(error))
        seg["delta_min_rad"] = min(seg["delta_min_rad"], delta)
        seg["delta_max_rad"] = max(seg["delta_max_rad"], delta)
        if n % 10 == 0:
            ref = np.asarray(m.q_ref_model)[list(axes)]
            preview.append([(m.control_steady_ns - first_ns) / 1e9, m.segment_id,
                            *q, *ref, *error, *cmd, *frame, delta])
    for seg in segments.values():
        seg["saturation_fraction"] = (seg.pop("saturated") / seg["samples"]).tolist()
        seg["position_rmse_rad"] = np.sqrt(seg.pop("error_square_sum") / seg["samples"]).tolist()
        seg["error_max_rad"] = seg["error_max_rad"].tolist()
    report = {"run": args.run.name, "samples": n, "phase_counts": dict(phases),
              "terminal_phase": last.phase, "failure_counts": dict(failures),
              "duration_s": (last.control_steady_ns - first_ns) / 1e9,
              "running_sample_span_s": (running_end - running_start) / 1e9,
              "segment_count": len(segments), "segments": segments,
              "validation_samples": sum(s["samples"] for s in segments.values() if s["role"] == 1),
              "max_sample_gap_ms": max_gap_ms, "tick_gap_count": gaps,
              "recorder_dropped_samples": dropped, "negative_selected_feedback_age_rows": negative_age,
              "max_selected_feedback_age_ms": max_age.tolist(),
              "opposite_pair_enabled_rows": opposite_enabled,
              "max_adjacent_q_step_rad": max_step.tolist(),
              "max_adjacent_raw_step_counts": max_raw_step.tolist(),
              "q_api_min_rad": q_min.tolist(), "q_api_max_rad": q_max.tolist(),
              "max_command_nm": max_command.tolist(), "max_submitted_frame_nm": max_frame.tolist(),
              "temperature_min_c": temp_min.tolist(), "temperature_max_c": temp_max.tolist(),
              "profile_sha256": hashlib.sha256((args.run / "profile.yaml").read_bytes()).hexdigest(),
              "note": "Recording/response observations only; not an identified-parameter acceptance."}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "recording_summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(args.output / "response_preview_100hz.npz", data=np.asarray(preview),
                        columns=np.array(["time_s", "segment", "q_hip_api", "q_aux_api", "ref_hip", "ref_aux",
                                          "error_hip", "error_aux", "cmd_hip", "cmd_aux", "frame_hip", "frame_aux", "delta"]))
    print(json.dumps({k: v for k, v in report.items() if k != "segments"}, indent=2))


if __name__ == "__main__":
    main()
