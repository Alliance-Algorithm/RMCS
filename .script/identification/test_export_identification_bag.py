"""Typed-bag round trip when run under ROS; pure QC tests also work on host."""
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

from export_identification_bag import (convert, load_identity, read_bag,
                                      require_matching_message_definition, split_segments, write_csv)


IDENTITY = {"side": "right", "model_sign": [1, 1, -1, 1],
            "model_offset_rad": [0, 0, .42, -.14], "profile_sha256": "profile",
            "calibration_sha256": "calibration"}


def row(i, segment=0):
    now = 2**60 + i * 5_000_000
    return dict(control_steady_ns=now, bag_timestamp_ns=now+50, tick=i+1,
                feedback_steady_ns=[now-1_000_000]*6, feedback_sequence=[i+1]*6,
                q_api=[.1, .2, -.3+ i*.001, .4+i*.002, 0, 0],
                dq_api=[0, 0, -.1, .2, 0, 0], torque_fb_api=[0, 0, -.3, .4, 0, 0],
                 max_torque_api=[5.]*6, dm_fault=[0]*4, dm_status=[1]*4,
                 temperature_c=[32., 33., 34., 35., float("nan"), float("nan")],
                 dm_rotor_temperature_c=[34., 35., 36., 37.],
                 supply_voltage_v=float("nan"),
                 q_ref_model=[.1, .2, .7, .5], dq_ref_model=[0.]*4,
                 velocity_target_model=[0., 0., .25, .35],
                 position_error_model=[0., 0., .1, .2],
                 speed_error_model=[0., 0., .35, .15],
                 torque_integral_model=[0.]*4,
                 q_ref_api=[float("nan")]*4,
                 velocity_target_api=[float("nan")]*4,
                 tau_preclip_model=[0, 0, .3, .4], tau_cmd_api=[0, 0, -.3, .4, 0, 0],
                tau_frame_api=[0, 0, -.3, .4, 0, 0],
                tx_queued_steady_ns=[now-100_000]*6, tx_kind=[1]*6,
                 phase=2, segment_id=segment, feedback_fresh=True, enable_requested=True,
                 selected_side=1, actuation_scope=1,
                 dropped_samples=0, imu_quaternion_xyzw=[0., 0., 0., 1.],
                 imu_gyro=[0., 0., 0.], imu_last_ns=now-1_000_000,
                 imu_acceleration_mps2=[0., 0., 9.81],
                 imu_acceleration_steady_ns=now-500_000,
                stamp_sec=1, stamp_nanosec=i)


def test_side_mapping_exact_integer_clock_and_flags():
    rows = [row(i) for i in range(8)]
    rows[1]["feedback_sequence"][2] = rows[0]["feedback_sequence"][2]
    rows[1]["feedback_steady_ns"][2] = rows[0]["feedback_steady_ns"][2]
    rows[1]["q_api"][2] = rows[0]["q_api"][2]
    rows[2]["feedback_sequence"][2] = 4  # missed CAN frame
    rows[3]["q_api"][3] += 1.0  # unmeasured unwrap/jump
    rows[4]["tau_preclip_model"][2] = 4.
    rows[5]["enable_requested"] = False
    rows[6]["dropped_samples"] = 1
    rows[7]["tick"] += 1
    a, m = convert(rows, IDENTITY)
    assert a["control_steady_ns"].dtype == np.uint64
    assert int(a["control_steady_ns"][0]) == 2**60
    np.testing.assert_allclose(a["q_model"][0, 2:4], [.72, .26])
    np.testing.assert_allclose(a["dq_model"][0, 2:4], [.1, .2])
    np.testing.assert_allclose(a["torque_fb_model"][0, 2:4], [.3, .4])
    np.testing.assert_allclose(a["velocity_target_model"][0, 2:4], [.25, .35])
    assert a["side_valid"][0] and a["pair_valid"][0]
    assert a["double_loop_observed"][0]
    assert a["axis_repeated_feedback"][1, 2] and a["side_valid"][1] and not a["pair_valid"][1]
    assert a["axis_sequence_gap"][2, 2] and not a["pair_valid"][2]
    assert a["axis_encoder_ambiguous"][3, 3]
    assert a["axis_saturated"][4, 2]
    assert a["axis_disabled"][5, 2]
    assert a["drop_delta"][6] == 1
    assert a["tick_gap"][7]
    assert m["insufficient_holdout"]


def test_bag_side_must_match_recorded_run_identity():
    with pytest.raises(ValueError, match="bag-selected side"):
        convert([row(0)], {**IDENTITY, "side": "left"})


def test_old_cdr_layout_is_not_silently_reinterpreted(tmp_path):
    saved = tmp_path / "WheelLegIdentificationSample.msg"
    installed = tmp_path / "installed.msg"
    saved.write_text("int32 phase\n", encoding="utf-8")
    installed.write_text("int32 phase\nuint8 segment_waveform\n", encoding="utf-8")
    with pytest.raises(ValueError, match="differs from installed"):
        require_matching_message_definition(saved, installed)
    installed.write_text(saved.read_text(encoding="utf-8"), encoding="utf-8")
    require_matching_message_definition(saved, installed)


def test_old_encoder_phase_is_lifted_before_model_offset():
    sample = row(0)
    sample["q_api"][2] = -4 * np.pi - .3
    sample["q_api"][3] = 4 * np.pi + .4
    arrays, meta = convert([sample], {**IDENTITY, "phase_api_angle": True})
    np.testing.assert_allclose(arrays["q_model"][0, 2:4], [.72, .26], atol=1e-12)
    assert meta["phase_api_angle"]


def test_model_angle_continues_through_phase_seam_without_raw_encoder_jump():
    samples = [row(0), row(1)]
    samples[0]["q_api"][2] = -(3.13 - .42)
    samples[1]["q_api"][2] = -(2 * np.pi - 3.13 - .42)
    arrays, _ = convert(samples, {**IDENTITY, "phase_api_angle": True})
    np.testing.assert_allclose(
        arrays["q_model"][:, 2], [3.13, 2 * np.pi - 3.13], atol=1e-10)


def test_raw_can_bytes_current_and_device_imu_clock_are_exported_without_casting():
    sample = row(0)
    sample["feedback_frame_bytes"] = list(range(48))
    sample["tx_frame_bytes"] = list(range(48, 96))
    sample["feedback_current_a"] = [float("nan")] * 4 + [1.25, -1.25]
    sample["feedback_torque_source"] = [1, 1, 1, 1, 2, 2]
    sample["tx_can_id"] = [1, 1, 2, 2, 0x200, 0x200]
    sample["tx_can_bus"] = [1, 2, 1, 2, 0, 0]
    sample["imu_board_timestamp_quarter_us"] = [2**32 - 2, 2**32 - 1]
    sample["wheel_velocity_target_api"] = [0.4, -0.4]
    sample["tau_preclip_api"] = [0.1] * 6
    sample["torque_limited"] = [False, False, False, True, False, True]
    arrays, meta = convert([sample], IDENTITY)
    assert meta["schema_version"] == 4
    assert arrays["feedback_frame_bytes"][0, 47] == 47
    assert arrays["tx_frame_bytes"][0, 0] == 48
    assert arrays["tx_can_id"][0, 4] == 0x200
    assert arrays["imu_board_timestamp_quarter_us"].dtype == np.uint32
    assert arrays["imu_board_timestamp_quarter_us"][0, 1] == 2**32 - 1
    np.testing.assert_allclose(arrays["feedback_current_a"][0, 4:], [1.25, -1.25])
    assert arrays["torque_limited"][0, 3] == 1


def test_double_down_terminal_sample_has_no_identification_effort():
    sample = row(0)
    sample["phase"] = 3
    sample["failure_reason"] = 63
    sample["segment_id"] = -1
    sample["enable_requested"] = False
    sample["actuation_scope"] = 0
    sample["dm_status"] = [0] * 4
    sample["tau_cmd_api"] = [float("nan")] * 6
    sample["tau_frame_api"] = [float("nan")] * 4 + [0, 0]
    sample["tx_kind"] = [2] * 4 + [1, 1]
    arrays, _ = convert([sample], IDENTITY)
    assert np.isnan(arrays["tau_cmd_api"][0]).all()
    assert arrays["failure_reason"][0] == 63
    assert not arrays["pair_valid"][0]
    assert not arrays["double_loop_observed"][0]


def test_csv_export_keeps_u64_nanosecond_timestamp_exact(tmp_path):
    arrays, _ = convert([row(0), row(1)], IDENTITY)
    output = tmp_path / "inspect.csv"
    write_csv(output, arrays)
    with output.open(newline="", encoding="utf-8") as stream:
        records = list(csv.DictReader(stream))
    assert int(records[0]["control_steady_ns"]) == 2**60
    assert int(records[1]["feedback_steady_ns_2"]) == 2**60 + 4_000_000


def test_missing_outer_velocity_target_is_not_inferred_from_position_reference():
    sample = row(0)
    sample["velocity_target_model"] = [float("nan")] * 4
    arrays, metadata = convert([sample], IDENTITY)
    assert not arrays["double_loop_observed"][0]
    assert metadata["counts"]["double_loop_observed"] == 0


def test_other_side_actuation_never_counts_as_single_side_double_loop():
    sample = row(0)
    sample["actuation_scope"] = 2
    arrays, metadata = convert([sample], IDENTITY)
    assert not arrays["double_loop_observed"][0]
    assert metadata["actuation_scope_counts"]["opposite_commanded"] == 1


def async_rows(count=54):
    rows = []
    sequence = [0, 0]
    stamps = [0, 0]
    angles = [float("nan"), float("nan")]
    for k in range(count):
        r = row(k, 1 if k >= 45 else 0)
        now = 2**60 + k*1_000_000
        r["control_steady_ns"] = now
        r["tick"] = k + 1
        r["bag_timestamp_ns"] = now + 50
        r["feedback_steady_ns"] = [now-1_000_000]*6
        r["tx_queued_steady_ns"] = [now-100_000]*6
        r["imu_last_ns"] = now-1_000_000
        for j in range(2):
            if k % 5 == j:
                sequence[j] += 1
                stamps[j] = now - (500_000 if j == 0 else 200_000)
                angles[j] = .02*k + j*.1
            r["feedback_sequence"][j+2] = sequence[j]
            r["feedback_steady_ns"][j+2] = stamps[j]
            r["q_api"][j+2] = angles[j]
        rows.append(r)
    return rows


def test_asynchronous_200hz_pair_accepts_held_axis_and_preserves_raw_ticks():
    rows = async_rows()
    a, meta = convert(rows, IDENTITY, max_skew_ms=2., max_age_ms=8.)
    assert len(a["control_steady_ns"]) == 54
    pairs = np.flatnonzero(a["pair_valid"])
    np.testing.assert_array_equal(pairs[:4], [1, 6, 11, 16])
    assert not np.any(a["axis_new_feedback"][:, 2] & a["axis_new_feedback"][:, 3])
    assert a["axis_repeated_feedback"][6, 2]
    assert not a["axis_valid"][6, 2] and a["pair_valid"][6]
    assert int(a["feedback_sequence"][6, 2]) == 2
    assert a["control_steady_ns"].dtype == np.uint64
    assert int(a["control_steady_ns"][6]) == 2**60 + 6_000_000
    assert meta["qc_thresholds"]["max_skew_ms"] == 2.
    assert set(meta["segment_split"].values()) == {"train", "holdout"}


def test_pair_boundaries_skew_age_seq_gap_gate_and_segment():
    rows = async_rows()
    # A missing selected CAN frame at k=15 invalidates this pairing interval.
    for k in range(15, len(rows)):
        rows[k]["feedback_sequence"][2] += 1
    for k in range(26, 31):
        rows[k]["feedback_steady_ns"][3] -= 3_100_000  # same held frame, >1.5 ms skew
    rows[40]["enable_requested"] = False
    a, _ = convert(rows, IDENTITY, max_skew_ms=1.5, max_age_ms=8.)
    assert a["axis_sequence_gap"][15, 2]
    assert not a["pair_valid"][15]
    assert a["pair_valid"][16] and a["pair_epoch"][16] > a["pair_epoch"][11]
    assert a["pair_skew_fail"][26] and not a["pair_valid"][26]
    assert a["pair_valid"][31] and a["pair_epoch"][31] > a["pair_epoch"][21]
    assert not a["side_valid"][40] and not a["pair_valid"][40]
    assert a["pair_valid"][41] and a["pair_epoch"][41] > a["pair_epoch"][36]
    assert a["segment_id"][46] != a["segment_id"][41]
    assert a["pair_epoch"][46] > a["pair_epoch"][41]
    younger, _ = convert(async_rows(), IDENTITY, max_skew_ms=2., max_age_ms=3.)
    assert np.any(younger["pair_age_fail"]) and not younger["pair_valid"][20]


def test_callback_straddle_quarantines_prior_pair_without_seq_advance():
    rows = async_rows(20)
    rows[2]["q_api"][2] += .02  # hip sequence/timestamp unchanged after tick 1 pair
    a, _ = convert(rows, IDENTITY)
    assert a["axis_encoder_ambiguous"][1, 2]
    assert a["axis_encoder_ambiguous"][2, 2]
    assert not a["pair_valid"][1]


def test_reproducible_segment_grouping_and_profile_guard(tmp_path):
    ids = np.array([1, 1, 2, 2, 3, 3, 4, 4])
    split = split_segments(ids, 93, .25)
    assert split == split_segments(ids[::-1], 93, .25)
    assert set(split.values()) == {1, 2}
    assert len({key for key in split if split[key] == 2}) == 1
    assert set(split_segments(np.array([3]*10), 1, .5).values()) == {1}
    calibration = tmp_path / "measured.json"
    profile = tmp_path / "run.yaml"
    calibration.write_text(json.dumps(IDENTITY))
    profile.write_text("controller:\n  ros__parameters:\n    side: right\n    model_sign: [1, 1, -1, 1]\n    model_offset_rad: [0, 0, 0.42, -0.14]\n")
    assert load_identity(calibration, profile)["side"] == "right"
    profile.write_text(profile.read_text().replace("side: right", "side: left"))
    with pytest.raises(ValueError, match="disagree"):
        load_identity(calibration, profile)


def test_pair_controller_model_offset_must_match_export_calibration(tmp_path):
    calibration = tmp_path / "measured.json"
    profile = tmp_path / "run.yaml"
    calibration.write_text(json.dumps({"side": "left", "model_sign": [1, -1, 1, -1],
                                       "model_offset": [.1, .2, -.1, -.2]}))
    profile.write_text("wheel_leg_pair_identification_controller:\n  ros__parameters:\n"
                       "    side: left\n    model_sign: [1, -1, 1, -1]\n"
                       "    model_offset: [0.1, 0.2, -0.1, -0.2]\n")
    assert load_identity(calibration, profile)["model_offset_rad"] == [.1, .2, -.1, -.2]
    profile.write_text(profile.read_text().replace("model_offset: [0.1, 0.2", "model_offset: [0.3, 0.2"))
    with pytest.raises(ValueError, match="model_offset differs"):
        load_identity(calibration, profile)


@pytest.mark.parametrize("recording_run", [None, "RJ02"])
def test_actual_rosbag2_mcap_round_trip(tmp_path, monkeypatch, recording_run):
    rosbag2_py = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from rmcs_msgs.msg import WheelLegIdentificationSample
    from export_identification_bag import TOPIC, TYPE

    uri = tmp_path / "typed_mcap"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=TOPIC, type=TYPE, serialization_format="cdr"))
    for i in range(3):
        sample = row(i)
        if recording_run:
            sample.update(recording_protocol_version=1, segment_role=1,
                          q_unwrapped_model=[1.6, 2.93, -8., -9.],
                          inner_knee_fk_deg=75., inner_knee_requested_deg=75.,
                          inner_knee_reference_deg=74., thigh_orientation_fk_deg=15.,
                          reference_update_tick=2**60+i, segment_elapsed_s=.02*i,
                          reference_center_delta_rad=.01, configuration_admission=1,
                          skipped_arrival_segment=7, qualified_arrival_segment=9,
                          jump_cycle_id=2, jump_phase=3)
        msg = WheelLegIdentificationSample()
        for key, value in sample.items():
            if key == "bag_timestamp_ns":
                continue
            if key == "stamp_sec":
                msg.stamp.sec = value
            elif key == "stamp_nanosec":
                msg.stamp.nanosec = value
            else:
                setattr(msg, key, value)
        writer.write(TOPIC, serialize_message(msg), sample["bag_timestamp_ns"])
    del writer  # flush metadata/mcap before reader opens
    from ament_index_python.packages import get_package_share_directory
    installed = Path(get_package_share_directory("rmcs_msgs")) / "msg/WheelLegIdentificationSample.msg"
    (uri.parent / "WheelLegIdentificationSample.msg").write_bytes(installed.read_bytes())
    calibration = tmp_path / "measured_calibration.json"
    profile = tmp_path / "run.yaml"
    if recording_run:
        from prepare_v6_pair_recording import prepare
        from v6_pair_recording import CALIBRATION
        prepare(recording_run, profile)
        calibration.write_bytes(CALIBRATION.read_bytes())
    else:
        calibration.write_text(json.dumps(IDENTITY))
        profile.write_text("controller:\n  ros__parameters:\n    side: right\n")
    identity = load_identity(calibration, profile)
    exported = read_bag(uri)
    a, _ = convert(exported, identity)
    assert len(exported) == 3
    assert a["feedback_sequence"].dtype == np.uint64
    assert int(a["control_steady_ns"][2]) == 2**60 + 10_000_000
    assert int(a["bag_timestamp_ns"][2]) == 2**60 + 10_000_050
    assert int(a["stamp_nanosec"][2]) == 2
    from export_identification_bag import main
    output = tmp_path / "export.npz"
    monkeypatch.setattr(sys, "argv", ["export_identification_bag.py", "--bag", str(uri),
                                  "--calibration", str(calibration), "--profile", str(profile),
                                  "--output", str(output)])
    main()
    metadata = json.loads(output.with_suffix(".json").read_text())
    assert metadata["side"] == "right"
    assert metadata["profile_sha256"] == hashlib.sha256(profile.read_bytes()).hexdigest()
    with np.load(output, allow_pickle=False) as npz:
        assert int(npz["control_steady_ns"][2]) == 2**60 + 10_000_000
        if recording_run:
            assert metadata["schema_version"] == 5
            assert metadata["controller"]["recording_run"] == recording_run
            assert npz["reference_update_tick"].dtype == np.uint64
            assert int(npz["reference_update_tick"][2]) == 2**60+2
            np.testing.assert_array_equal(npz["q_model"][:, 2:4], [[-8., -9.]]*3)
            assert npz["qualified_arrival_segment"].tolist() == [9]*3
            assert npz["skipped_arrival_segment"].tolist() == [7]*3
            assert npz["jump_phase"].tolist() == [3]*3
            assert set(npz["split"][npz["pair_valid"]]) == {2}


def test_invalid_run_marker_blocks_export_before_reading_bag(tmp_path, monkeypatch, capsys):
    from export_identification_bag import main
    run = tmp_path / "invalid-run"
    run.mkdir()
    (run / "RUN_INVALID.json").write_text('{"status":"invalid_profile_provenance"}')
    monkeypatch.setattr("sys.argv", ["export_identification_bag.py", "--bag", str(run / "bag"),
        "--output", str(tmp_path / "export.npz"), "--calibration", "unused.json", "--profile", "unused.yaml"])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
    assert "invalid experiment provenance" in capsys.readouterr().err
    assert not (tmp_path / "export.npz").exists()
