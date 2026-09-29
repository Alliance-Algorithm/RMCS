#!/usr/bin/env python3
"""Export typed wheel-leg MCAP to lossless-timestamp NPZ + QC/split JSON.

Run under /opt/ros/jazzy with the RMCS workspace sourced. No ROS imports are
needed when using the pure-Python QC functions in tests. Never unwraps encoders,
resamples feedback, or equates a submitted CAN frame with motor execution.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


TOPIC = "/wheel_leg/identification/sample"
TYPE = "rmcs_msgs/msg/WheelLegIdentificationSample"
AXES = ("left_hip", "left_aux", "right_hip", "right_aux", "left_wheel", "right_wheel")
FLOAT_FIELDS = {
    "q_api": 6, "dq_api": 6, "torque_fb_api": 6, "max_torque_api": 6,
    "temperature_c": 6, "dm_rotor_temperature_c": 4,
    "supply_voltage_v": None,
    "feedback_current_a": 6, "wheel_velocity_target_api": 2,
    "tau_preclip_api": 6,
    "tau_cmd_api": 6, "tau_frame_api": 6, "q_ref_model": 4,
    "dq_ref_model": 4, "velocity_target_model": 4,
    "position_error_model": 4, "speed_error_model": 4,
    "tau_preclip_model": 4, "torque_integral_model": 4,
    "tau_gravity_ff_model": 4,
    "q_ref_api": 4, "velocity_target_api": 4,
    "imu_quaternion_xyzw": 4, "imu_gyro": 3, "imu_acceleration_mps2": 3,
}
U64_FIELDS = {
    "control_steady_ns": None, "tick": None, "feedback_steady_ns": 6,
    "feedback_sequence": 6, "tx_queued_steady_ns": 6,
    "dropped_samples": None, "imu_last_ns": None,
    "imu_acceleration_steady_ns": None, "bag_timestamp_ns": None,
}
U32_FIELDS = {"repetition_id": None, "tx_can_id": 6,
              "imu_board_timestamp_quarter_us": 2}
I32_FIELDS = {"dm_fault": 4, "dm_status": 4, "phase": None, "segment_id": None,
              "failure_reason": None}
U8_FIELDS = {"tx_kind": 6, "feedback_fresh": None, "enable_requested": None,
             "selected_side": None, "actuation_scope": None,
             "experiment_kind": None, "feedback_frame_bytes": 48,
             "tx_frame_bytes": 48, "feedback_torque_source": 6,
             "tx_can_bus": 6, "torque_limited": 6}
U8_FIELDS.update({"segment_role": None, "segment_waveform": None, "wheel_mode": None})
OPTIONAL_DEFAULTS = {
    "feedback_current_a": float("nan"), "wheel_velocity_target_api": float("nan"),
    "tau_preclip_api": float("nan"), "repetition_id": 0,
    "tau_gravity_ff_model": float("nan"),
    "tx_can_id": 0, "imu_board_timestamp_quarter_us": 0,
    "experiment_kind": 0, "feedback_frame_bytes": 0, "tx_frame_bytes": 0,
    "feedback_torque_source": 0, "tx_can_bus": 255, "torque_limited": 0,
    "segment_role": 2, "segment_waveform": 0, "wheel_mode": 0,
    "failure_reason": 0,
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_matching_message_definition(saved: Path, installed: Path) -> None:
    if not saved.is_file():
        raise ValueError(f"bag message-definition snapshot missing: {saved}")
    if sha(saved) != sha(installed):
        raise ValueError(
            "bag message definition differs from installed rmcs_msgs; "
            "do not deserialize old CDR with a new type layout")


def profile_side(profile: object) -> str:
    """Look for a unique ROS parameter named side, including under ros__parameters."""
    found = set()

    def visit(value: object) -> None:
        if isinstance(value, dict):
            if "side" in value:
                found.add(value["side"])
            for sub in value.values():
                visit(sub)
        elif isinstance(value, list):
            for sub in value:
                visit(sub)

    visit(profile)
    if len(found) != 1 or next(iter(found)) not in ("left", "right"):
        raise ValueError("profile YAML needs one unambiguous side: left or right")
    return next(iter(found))


def load_identity(calibration: Path, profile: Path) -> dict:
    raw = json.loads(calibration.read_text())
    params = raw.get("ros__parameters", raw)
    profile_data = yaml.safe_load(profile.read_text())
    side = profile_side(profile_data)
    controller = profile_data.get("wheel_leg_pair_identification_controller", {}).get("ros__parameters", {})
    phase_api_angle = controller.get("phase_api_angle", False)
    if not isinstance(phase_api_angle, bool):
        raise ValueError("phase_api_angle must be a boolean")
    if "side" in params and params["side"] != side:
        raise ValueError("calibration and profile side disagree")
    sign = np.asarray(params["model_sign"], dtype=np.float64)
    if "model_offset_rad" in params and "model_offset" in params \
            and params["model_offset_rad"] != params["model_offset"]:
        raise ValueError("calibration model_offset and model_offset_rad disagree")
    offset = np.asarray(params.get("model_offset", params.get("model_offset_rad")), dtype=np.float64)
    if sign.shape != (4,) or not np.isin(sign, [-1., 1.]).all():
        raise ValueError("calibration model_sign must be four measured +/-1 API-to-model signs")
    if offset.shape != (4,) or not np.isfinite(offset).all():
        raise ValueError("calibration model_offset_rad must be four finite measured offsets")
    # If the run profile carries the controller's mapping, it must be the one
    # used by this conversion. Never silently apply a different JSON zero.
    def visit(value: object) -> None:
        if isinstance(value, dict):
            if "model_sign" in value and not np.array_equal(np.asarray(value["model_sign"]), sign):
                raise ValueError("run profile model_sign differs from calibration JSON")
            if "model_offset_rad" in value and not np.array_equal(np.asarray(value["model_offset_rad"]), offset):
                raise ValueError("run profile model_offset_rad differs from calibration JSON")
            if "model_offset" in value and not np.array_equal(np.asarray(value["model_offset"]), offset):
                raise ValueError("run profile model_offset differs from calibration JSON")
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)
    visit(profile_data)
    spring_min = controller.get("spring_delta_min")
    spring_max = controller.get("spring_delta_max")
    return {"side": side, "model_sign": sign.tolist(), "model_offset_rad": offset.tolist(),
            "phase_api_angle": phase_api_angle,
            "spring_delta_min": spring_min, "spring_delta_max": spring_max,
            "calibration_sha256": sha(calibration), "profile_sha256": sha(profile),
            "calibration_source": str(calibration.resolve()), "profile_source": str(profile.resolve())}


def read_bag(uri: Path) -> list[dict]:
    from ament_index_python.packages import get_package_share_directory
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rmcs_msgs.msg import WheelLegIdentificationSample

    installed = Path(get_package_share_directory("rmcs_msgs")) / "msg/WheelLegIdentificationSample.msg"
    require_matching_message_definition(
        uri.parent / "WheelLegIdentificationSample.msg", installed)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    matches = [t for t in reader.get_all_topics_and_types() if t.name == TOPIC]
    if len(matches) != 1 or matches[0].type != TYPE:
        raise ValueError(f"expected exactly one {TOPIC} of type {TYPE}: {matches}")
    rows = []
    while reader.has_next():
        name, payload, timestamp_ns = reader.read_next()
        if name != TOPIC:
            continue
        msg = deserialize_message(payload, WheelLegIdentificationSample)
        row = {key: getattr(msg, key) for key in
               (*FLOAT_FIELDS, *U64_FIELDS, *U32_FIELDS, *I32_FIELDS, *U8_FIELDS)
               if key != "bag_timestamp_ns"}
        row["bag_timestamp_ns"] = timestamp_ns
        row["stamp_sec"] = msg.stamp.sec
        row["stamp_nanosec"] = msg.stamp.nanosec
        rows.append(row)
    if not rows:
        raise ValueError(f"no {TOPIC} messages in {uri}")
    return rows


def _arrays(rows: list[dict]) -> dict[str, np.ndarray]:
    if not rows:
        raise ValueError("no identification samples")
    result = {}
    for fields, dtype in ((FLOAT_FIELDS, np.float64), (U64_FIELDS, np.uint64),
                          (U32_FIELDS, np.uint32), (I32_FIELDS, np.int32), (U8_FIELDS, np.uint8)):
        for key, width in fields.items():
            default = OPTIONAL_DEFAULTS.get(key)
            values = [row.get(key, [default] * width if width is not None else default)
                      if key in OPTIONAL_DEFAULTS else row[key] for row in rows]
            if width is not None and any(len(v) != width for v in values):
                raise ValueError(f"wrong width for {key}")
            result[key] = np.asarray(values, dtype=dtype)
    for name, dtype in (("stamp_sec", np.int32), ("stamp_nanosec", np.uint32)):
        result[name] = np.asarray([r[name] for r in rows], dtype=dtype)
    return result


def write_csv(path: Path, arrays: dict[str, np.ndarray]) -> None:
    """Optional offline inspection table; never cast uint64 nanoseconds to float."""
    count = len(arrays["tick"])
    columns = []
    for name in ("control_steady_ns", "tick", "selected_side", "phase", "segment_id",
                 "actuation_scope", *sorted(arrays.keys())):
        if name not in arrays or name in {column[0] for column in columns}:
            continue
        values = arrays[name]
        if values.shape[0] != count or values.ndim not in (1, 2):
            raise ValueError(f"cannot flatten irregular CSV field {name}")
        for index in range(values.shape[1] if values.ndim == 2 else 1):
            columns.append((name, index if values.ndim == 2 else None))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([f"{name}_{index}" if index is not None else name for name, index in columns])
        for row in range(count):
            writer.writerow([arrays[name][row, index].item() if index is not None
                             else arrays[name][row].item() for name, index in columns])


def split_segments(segment: np.ndarray, seed: int, holdout_fraction: float) -> dict[int, int]:
    """Split on segment ID, not frames. A single segment cannot have a holdout."""
    if not (0 < holdout_fraction < 1):
        raise ValueError("holdout fraction must be between zero and one")
    ids = sorted(set(map(int, segment)) - {-1})
    if len(ids) < 2:
        return {i: 1 for i in ids}
    ranked = sorted(ids, key=lambda i: (hashlib.sha256(f"{seed}:{i}".encode()).digest(), i))
    n_holdout = min(len(ids) - 1, max(1, round(len(ids) * holdout_fraction)))
    return {i: 2 if i in ranked[:n_holdout] else 1 for i in ids}


def convert(rows: list[dict], identity: dict, *, seed: int = 0,
             holdout_fraction: float = .25, max_encoder_step_rad: float = .35,
            saturation_tolerance_nm: float = .05, max_skew_ms: float = 3.,
            max_age_ms: float = 12.) -> tuple[dict[str, np.ndarray], dict]:
    if (not all(math.isfinite(x) for x in (max_encoder_step_rad, saturation_tolerance_nm,
                                           max_skew_ms, max_age_ms))
            or max_encoder_step_rad <= 0 or saturation_tolerance_nm < 0
            or max_skew_ms <= 0 or max_age_ms <= 0):
        raise ValueError("QC thresholds must be positive/nonnegative")
    skew_ns, age_ns = round(max_skew_ms * 1_000_000), round(max_age_ms * 1_000_000)
    if not skew_ns or not age_ns:
        raise ValueError("CAN pairing thresholds round to zero nanoseconds")
    a = _arrays(rows)
    n = len(rows)
    side = identity["side"]
    if side not in ("left", "right"):
        raise ValueError("invalid side")
    expected_side = 0 if side == "left" else 1
    if np.any(a["selected_side"] != expected_side):
        raise ValueError("bag-selected side and run profile/calibration disagree")
    if np.any(a["actuation_scope"] > 3):
        raise ValueError("unknown actuation_scope code in bag")
    if np.any(a["experiment_kind"] != a["experiment_kind"][0]) or a["experiment_kind"][0] > 1:
        raise ValueError("a bag cannot mix pair and wheel experiment kinds")
    sign = np.asarray(identity["model_sign"], dtype=np.float64)
    offset = np.asarray(identity["model_offset_rad"], dtype=np.float64)
    if sign.shape != (4,) or offset.shape != (4,) or not np.isin(sign, [-1, 1]).all() or not np.isfinite(offset).all():
        raise ValueError("invalid calibration mapping")
    model_angle = a["q_api"][:, :4] * sign + offset
    if identity.get("phase_api_angle", False):
        # Reproduce the controller's session-local continuous lift. Do not
        # bridge separate arm epochs or gaps in finite feedback.
        model_angle = np.remainder(model_angle + np.pi, 2 * np.pi) - np.pi
        for repetition in np.unique(a["repetition_id"]):
            indices = np.flatnonzero(a["repetition_id"] == repetition)
            for axis in range(4):
                valid = np.isfinite(model_angle[indices, axis])
                for run in np.split(indices, np.flatnonzero(np.diff(valid.astype(int)) != 0) + 1):
                    if len(run) and np.isfinite(model_angle[run[0], axis]):
                        model_angle[run, axis] = np.unwrap(model_angle[run, axis])
            minimum, maximum = identity.get("spring_delta_min"), identity.get("spring_delta_max")
            if (isinstance(minimum, list) and isinstance(maximum, list)
                    and len(minimum) == len(maximum) == 2):
                for side_index in range(2):
                    hip, knee = 2 * side_index, 2 * side_index + 1
                    valid_pair = indices[
                        np.isfinite(model_angle[indices, hip])
                        & np.isfinite(model_angle[indices, knee])]
                    if len(valid_pair):
                        first = valid_pair[0]
                        midpoint = (minimum[side_index] + maximum[side_index]) * .5
                        shift = round((model_angle[first, hip] + midpoint
                                       - model_angle[first, knee]) / (2 * np.pi))
                        model_angle[indices, knee] += shift * 2 * np.pi
    a["q_model"] = model_angle
    a["dq_model"] = a["dq_api"][:, :4] * sign
    a["torque_fb_model"] = a["torque_fb_api"][:, :4] * sign
    a["tau_cmd_model"] = a["tau_cmd_api"][:, :4] * sign
    a["tau_frame_model"] = a["tau_frame_api"][:, :4] * sign
    sequence, timestamp = a["feedback_sequence"], a["feedback_steady_ns"]
    repeat = np.zeros((n, 6), bool)
    gap = np.zeros((n, 6), bool)
    ambiguous = np.zeros((n, 6), bool)
    fresh_axis = np.zeros((n, 6), bool)
    time_bad = np.zeros(n, bool)
    tick_gap = np.zeros(n, bool)
    drop_delta = np.zeros(n, np.uint64)
    for i in range(n):
        ctl = int(a["control_steady_ns"][i])
        time_bad[i] = ctl == 0 or any(
            int(t) > ctl for t in timestamp[i] if t != 0)
        if i:
            last_ctl = int(a["control_steady_ns"][i - 1])
            time_bad[i] |= ctl <= last_ctl
            tick_gap[i] = int(a["tick"][i]) != int(a["tick"][i-1]) + 1
            dropped = int(a["dropped_samples"][i]) - int(a["dropped_samples"][i-1])
            if dropped < 0:
                time_bad[i] = True
            else:
                drop_delta[i] = dropped
        for j in range(6):
            seq, fb = int(sequence[i, j]), int(timestamp[i, j])
            if not seq or not fb or fb > ctl:
                ambiguous[i, j] = True
                continue
            if i:
                prev_seq, prev_ns = int(sequence[i-1, j]), int(timestamp[i-1, j])
                repeat[i, j] = seq == prev_seq
                gap[i, j] = seq > prev_seq + 1
                ambiguous[i, j] = (seq < prev_seq or fb < prev_ns
                                   or (repeat[i, j] and fb != prev_ns)
                                   or (seq > prev_seq and fb <= prev_ns)
                                   or (seq > prev_seq and prev_seq != 0
                                       and (not np.isfinite(a["q_api"][i-1, j])
                                            or abs(a["q_api"][i, j] - a["q_api"][i-1, j])
                                            > max_encoder_step_rad)))
                # A status value that changes without a new CAN sequence can
                # indicate the recorder sampled across a status callback.
                # Quarantine BOTH snapshots; the previous one may have paired
                # a new sequence with old decoded values.
                if repeat[i, j] and (fb != prev_ns or any(
                        a[field][i, j] != a[field][i-1, j]
                        for field in ("q_api", "dq_api", "torque_fb_api"))):
                    ambiguous[i, j] = True
                    ambiguous[i-1, j] = True
                    fresh_axis[i-1, j] = False
            fresh_axis[i, j] = not repeat[i, j] and not ambiguous[i, j] and not gap[i, j]
    selected = (0, 1) if side == "left" else (2, 3)
    opposite = (2, 3) if side == "left" else (0, 1)
    # A PC-side cascaded controller must expose the outer-loop speed target;
    # a position reference and sent torque alone do not identify both loops.
    a["double_loop_observed"] = (
        np.isfinite(a["q_ref_model"][:, selected]).all(axis=1)
        & np.isfinite(a["velocity_target_model"][:, selected]).all(axis=1)
        & np.isfinite(a["tau_preclip_model"][:, selected]).all(axis=1)
        & np.isfinite(a["tau_cmd_api"][:, selected]).all(axis=1)
        & np.all(a["tx_kind"][:, selected] == 1, axis=1)
        & (a["actuation_scope"] == 1)
    )
    preclip = a["tau_preclip_model"]
    saturated = np.zeros((n, 6), bool)
    saturated[:, :4] = np.isfinite(preclip) & np.isfinite(a["tau_cmd_model"]) & (
        np.abs(preclip) > np.abs(a["tau_cmd_model"]) + saturation_tolerance_nm)
    saturated |= np.isfinite(a["tau_cmd_api"]) & np.isfinite(a["max_torque_api"]) & (
        np.abs(a["tau_cmd_api"]) > a["max_torque_api"] + saturation_tolerance_nm)
    tx_missing = (a["tx_kind"] != 1) | (a["tx_queued_steady_ns"] == 0) | ~np.isfinite(a["tau_frame_api"])
    disabled = np.broadcast_to(~a["enable_requested"].astype(bool)[:, None], (n, 6)).copy()
    disabled[:, :4] |= (a["dm_status"] != 1) | (a["dm_fault"] != 0)
    axis_invalid = (ambiguous | gap | repeat | tx_missing | disabled | saturated
                    | ~np.isfinite(a["q_api"]) | ~np.isfinite(a["dq_api"])
                    | ~np.isfinite(a["torque_fb_api"]))
    a.update(axis_repeated_feedback=repeat, axis_sequence_gap=gap,
             axis_encoder_ambiguous=ambiguous, axis_new_feedback=fresh_axis,
             axis_saturated=saturated, axis_tx_missing=tx_missing, axis_disabled=disabled,
             time_invalid=time_bad, tick_gap=tick_gap, drop_delta=drop_delta)
    a["axis_valid"] = ~axis_invalid & a["feedback_fresh"].astype(bool)[:, None] & ~time_bad[:, None]
    # axis_valid refers to a NEW frame on this executor tick. A cached, repeated
    # frame is not a new observation, but it can complete a two-axis paired
    # observation when the other axis advances on a later tick.
    tx_bad_kind = ((a["tx_kind"] == 2) | (a["tx_kind"] == 3) | (a["tx_kind"] > 4)
                   | ((a["tx_kind"] == 1) & tx_missing))
    cache_good = ~(ambiguous | gap | disabled | saturated | tx_bad_kind
                   | ~np.isfinite(a["q_api"]) | ~np.isfinite(a["dq_api"])
                   | ~np.isfinite(a["torque_fb_api"]))
    cache_good &= (sequence != 0) & (timestamp != 0)
    pair_age_fail = np.zeros(n, bool)
    opposite_valid = np.zeros(n, bool)
    for i in range(n):
        now = int(a["control_steady_ns"][i])
        pair_age_fail[i] = any(now - int(timestamp[i, j]) > age_ns for j in selected)
        opposite_valid[i] = (all(cache_good[i, j] and 0 <= now - int(timestamp[i, j]) <= age_ns
                                 for j in opposite)
                             and abs(int(timestamp[i, opposite[0]]) - int(timestamp[i, opposite[1]])) <= skew_ns
                             and not time_bad[i])
    a["side_valid"] = (np.all(cache_good[:, selected], axis=1) & ~time_bad & ~tick_gap
                       & (drop_delta == 0) & ~pair_age_fail
                       & a["feedback_fresh"].astype(bool)
                       & (a["phase"] == 2) & (a["segment_id"] >= 0)
                       & np.isfinite(a["q_ref_model"][:, selected]).all(axis=1)
                       & np.isfinite(a["dq_ref_model"][:, selected]).all(axis=1))
    a["opposite_valid"] = opposite_valid
    # First sample of any contiguous segment is an initial condition, not a derivative.
    a["segment_instance"] = np.cumsum(np.r_[True, (a["segment_id"][1:] != a["segment_id"][:-1]) |
                                                   (a["tick"][1:] <= a["tick"][:-1])]).astype(np.int32) - 1
    pair_valid = np.zeros(n, bool)
    pair_skew_fail = np.zeros(n, bool)
    pair_waiting = np.zeros(n, bool)
    pair_epoch = np.zeros(n, np.int32)
    last_accepted = [0, 0]
    epoch, broken = 0, False
    for i in range(n):
        changed_segment = i != 0 and a["segment_instance"][i] != a["segment_instance"][i-1]
        if changed_segment:
            broken = True
            last_accepted = [0, 0]
        if not a["side_valid"][i]:
            broken = True
            pair_epoch[i] = epoch
            continue
        seq = [int(sequence[i, j]) for j in selected]
        advanced = all(seq[k] > last_accepted[k] for k in (0, 1))
        if not advanced:
            pair_waiting[i] = True
        else:
            pair_skew_fail[i] = (abs(int(timestamp[i, selected[0]])
                                     - int(timestamp[i, selected[1]])) > skew_ns)
            if pair_skew_fail[i]:
                broken = True
            else:
                if broken:
                    epoch += 1
                    broken = False
                pair_valid[i] = True
                last_accepted = seq
        pair_epoch[i] = epoch
    a.update(pair_valid=pair_valid, pair_epoch=pair_epoch, pair_age_fail=pair_age_fail,
             pair_skew_fail=pair_skew_fail, pair_waiting_for_both=pair_waiting)
    split = split_segments(a["segment_id"][pair_valid], seed, holdout_fraction)
    if a["experiment_kind"][0] == 1:
        split = {int(segment): 2 if np.any((a["segment_id"] == segment)
                     & (a["segment_role"] == 1)) else 1
                 for segment in np.unique(a["segment_id"][a["segment_id"] >= 0])}
    elif np.any((a["segment_role"] == 1) & pair_valid):
        # Explicit held-out waveform: never train on this segment's samples.
        heldout = set(map(int, a["segment_id"][(a["segment_role"] == 1) & pair_valid]))
        split = {segment: 2 if segment in heldout else 1 for segment in split}
    a["split"] = np.asarray([split.get(int(seg), 0) if ok else 0 for seg, ok in
                             zip(a["segment_id"], pair_valid)], dtype=np.uint8)
    metadata = {
        "schema_version": 4, "topic": TOPIC, "message_type": TYPE,
        "axis_order": list(AXES), "model_order": list(AXES[:4]),
        "side": side, "side_source": "bag_and_profile_checked",
        "experiment_kind": "wheel" if a["experiment_kind"][0] == 1 else "pair",
        "selected_axes": list(selected), "opposite_axes": list(opposite),
        "model_sign": sign.tolist(), "model_offset_rad": offset.tolist(),
        "phase_api_angle": bool(identity.get("phase_api_angle", False)),
        "profile_sha256": identity["profile_sha256"],
        "calibration_sha256": identity["calibration_sha256"],
        "profile_source": identity.get("profile_source"),
        "calibration_source": identity.get("calibration_source"),
        "split_seed": seed, "holdout_fraction": holdout_fraction,
        "segment_split": {str(k): "holdout" if v == 2 else "train" for k, v in sorted(split.items())},
        "insufficient_holdout": len(split) < 2,
        "qc_thresholds": {"max_encoder_step_rad": max_encoder_step_rad,
                          "saturation_tolerance_nm": saturation_tolerance_nm,
                          "max_skew_ms": max_skew_ms, "max_age_ms": max_age_ms},
        "qc_definitions": {"axis_saturated": "preclip versus command or command versus torque cap; also flags intentional ramp/slew/dwell limiting, not measured motor saturation",
                           "axis_encoder_ambiguous": "missing/out-of-order CAN timing or raw position step above threshold; no P_MAX unwrap inferred",
                           "axis_valid": "new CAN frame on this raw tick only; cached repeats are separately flagged",
                           "side_valid": "raw tick has eligible latest side feedback/status; one or both CAN values may be held",
                           "pair_valid": "both selected CAN sequences advanced since previous accepted pair, latest receive stamps meet age/skew thresholds; no intervening status/limit/sequence/clock failure",
                           "pair_epoch": "incremented at rejected intervals/segment boundaries; the fitter cannot bridge epochs"},
        "counts": {key: int(np.count_nonzero(a[key])) for key in (
            "side_valid", "pair_valid", "pair_age_fail", "pair_skew_fail",
            "opposite_valid", "axis_repeated_feedback", "axis_sequence_gap",
            "axis_encoder_ambiguous", "axis_saturated", "axis_tx_missing", "axis_disabled",
             "tick_gap", "time_invalid", "double_loop_observed")},
        "actuation_scope_counts": {"unconfirmed": int(np.count_nonzero(a["actuation_scope"] == 0)),
                                   "selected_command_only": int(np.count_nonzero(a["actuation_scope"] == 1)),
                                   "opposite_commanded": int(np.count_nonzero(a["actuation_scope"] == 2))},
        "dropped_samples_last": int(a["dropped_samples"][-1]),
        "limitations": ["q/dq/torque feedback and CAN sequence/time may straddle a callback boundary",
                        "tx_queued_steady_ns is host submission, not acknowledged CAN or motor execution",
                        "q_model uses supplied API-to-model calibration without encoder unwrap or interpolation"],
    }
    return a, metadata


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", required=True, type=Path, help="rosbag2 MCAP directory")
    p.add_argument("--output", required=True, type=Path, help="output .npz; adjacent .json metadata")
    p.add_argument("--csv-output", type=Path, help="optional offline flattened .csv; MCAP stays authoritative")
    p.add_argument("--calibration", required=True, type=Path, help="measured JSON: model_sign[4], model_offset_rad[4]")
    p.add_argument("--profile", required=True, type=Path, help="actual run YAML with side: left/right")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--holdout-fraction", type=float, default=.25)
    p.add_argument("--max-encoder-step-rad", type=float, default=.35)
    p.add_argument("--saturation-tolerance-nm", type=float, default=.05)
    p.add_argument("--max-skew-ms", type=float, default=3., help="latest selected CAN receive stamp difference")
    p.add_argument("--max-age-ms", type=float, default=12., help="latest selected CAN age at pairing tick")
    args = p.parse_args()
    invalid_run = args.bag.resolve().parent / "RUN_INVALID.json"
    if invalid_run.exists():
        p.error(f"run has invalid experiment provenance; inspect {invalid_run} before export")
    if args.output.suffix != ".npz":
        p.error("--output must end in .npz")
    if args.csv_output is not None and args.csv_output.suffix != ".csv":
        p.error("--csv-output must end in .csv")
    identity = load_identity(args.calibration, args.profile)
    arrays, meta = convert(read_bag(args.bag), identity, seed=args.seed,
                           holdout_fraction=args.holdout_fraction,
                           max_encoder_step_rad=args.max_encoder_step_rad,
                           saturation_tolerance_nm=args.saturation_tolerance_nm,
                           max_skew_ms=args.max_skew_ms, max_age_ms=args.max_age_ms)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **arrays)
    if args.csv_output is not None:
        write_csv(args.csv_output, arrays)
    args.output.with_suffix(".json").write_text(json.dumps(meta, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output), "csv": str(args.csv_output) if args.csv_output else None,
                       "samples": len(arrays["tick"]),
                      "train_pairs": int(np.sum(arrays["split"] == 1)),
                      "holdout_pairs": int(np.sum(arrays["split"] == 2)),
                      "counts": meta["counts"]}, indent=2))


if __name__ == "__main__":
    main()
