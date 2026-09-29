"""Synthetic coupled closed-chain pair with asynchronous 200 Hz CAN on a 1 kHz host."""
import hashlib
import json

import numpy as np
import pytest

from export_identification_bag import convert
from fit_wheel_leg_pair_bag import checked_run, evaluate, load_run


MASS = np.array([[.075, .019], [.019, .11]])
DAMPING = np.array([[.13, .035], [.035, .20]])
FC = np.array([.17, .13])
LOAD = np.array([[.60, .32, -.21], [-.40, .14, .35]])
CENTER = np.array([.25, -.20])
BASE = 2**60


def motion(time, segment):
    phases = np.array([.17 * segment, .43 + .21 * segment])
    q = np.empty(2)
    v = np.empty(2)
    acc = np.empty(2)
    for j in range(2):
        freq = (1.1 + .08 * segment + .4 * j, 2.6 + .14 * j)
        amps = (.18 + .02 * j, .048)
        origin = (.25, -.20)[j]
        x = 2 * np.pi * (freq[0] * time + phases[j])
        y = 2 * np.pi * (freq[1] * time + .25 * segment + .37 * j)
        q[j] = origin + amps[0] * np.sin(x) + amps[1] * np.sin(y)
        v[j] = amps[0] * (2 * np.pi * freq[0]) * np.cos(x) + amps[1] * (2 * np.pi * freq[1]) * np.cos(y)
        acc[j] = -amps[0] * (2 * np.pi * freq[0])**2 * np.sin(x) - amps[1] * (2 * np.pi * freq[1])**2 * np.sin(y)
    tau = MASS @ acc + DAMPING @ v + FC * np.tanh(30 * v) + LOAD @ np.r_[1., q - CENTER]
    return q, v, tau


def fixture(tmp_path, side="right", n_segment=4, ticks_per_segment=1150, missed_at=None):
    selected = (0, 1) if side == "left" else (2, 3)
    sign = np.array([1, -1, -1, 1])
    profile = tmp_path / "profile.yaml"
    yaml_lines = ["wheel_leg_identification_recorder:", "  ros__parameters:", f"    side: {side}",
                  "wheel_leg_pair_identification_controller:", "  ros__parameters:", f"    side: {side}",
                  "    kp_position: [8.0, 9.0, 10.0, 11.0]",
                  "    kp_velocity: [2.0, 2.5, 3.0, 3.5]",
                  "    ki_velocity: [0.0, 0.0, 0.0, 0.0]",
                  "    max_speed: [9.0, 9.0, 9.0, 9.0]",
                  "    max_torque: [80.0, 80.0, 80.0, 80.0]",
                  "    integral_limit: [0.0, 0.0, 0.0, 0.0]"]
    profile.write_text("\n".join(yaml_lines) + "\n")
    identity = {"side": side, "model_sign": sign.tolist(), "model_offset_rad": [0.] * 4,
                "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
                "calibration_sha256": "synthetic-calibration"}
    rows = []
    seq = [0, 0]
    stamps = [0, 0]
    held_q = np.zeros(2)
    held_v = np.zeros(2)
    held_tau = np.zeros(2)
    for k in range(n_segment * ticks_per_segment):
        segment = k // ticks_per_segment
        local_t = (k % ticks_per_segment) * .001
        now = BASE + k * 1_000_000
        for j in range(2):
            if k % 5 == j:
                seq[j] += 1
                stamps[j] = now - 400_000
                q, v, tau = motion(local_t - .0004, segment)
                held_q[j], held_v[j], held_tau[j] = q[j], v[j], tau[j]
        frame_q, frame_v, frame_tau = motion(local_t + .0029, segment)
        q_api = np.zeros(6)
        dq_api = np.zeros(6)
        fb_api = np.zeros(6)
        cmd_api = np.zeros(6)
        frame_api = np.zeros(6)
        ref = np.zeros(4)
        dq_ref = np.zeros(4)
        velocity_target = np.zeros(4)
        position_error = np.zeros(4)
        speed_error = np.zeros(4)
        preclip = np.zeros(4)
        for j, axis in enumerate(selected):
            q_api[axis] = sign[axis] * held_q[j]
            dq_api[axis] = sign[axis] * held_v[j]
            fb_api[axis] = sign[axis] * held_tau[j]
            cmd_api[axis] = sign[axis] * held_tau[j]
            frame_api[axis] = sign[axis] * frame_tau[j]
            kp_p, kp_v = (8. + axis, 2. + .5 * axis)
            dq_ref[axis] = held_v[j]
            velocity_target[axis] = held_v[j] + held_tau[j] / kp_v
            ref[axis] = held_q[j] + held_tau[j] / (kp_v * kp_p)
            position_error[axis] = ref[axis] - held_q[j]
            speed_error[axis] = velocity_target[axis] - held_v[j]
            preclip[axis] = held_tau[j]
        r = dict(control_steady_ns=now, bag_timestamp_ns=now + 20, tick=k+1,
                 feedback_steady_ns=[now - 1_000_000] * 6, feedback_sequence=[k+1] * 6,
                 q_api=q_api.tolist(), dq_api=dq_api.tolist(), torque_fb_api=fb_api.tolist(),
                 max_torque_api=[80.] * 6, temperature_c=[33., 34., 35., 36., np.nan, np.nan],
                 dm_rotor_temperature_c=[34.] * 4, supply_voltage_v=24.0,
                 dm_status=[int(i in selected) for i in range(4)], dm_fault=[0]*4,
                 q_ref_model=ref.tolist(), dq_ref_model=dq_ref.tolist(),
                 velocity_target_model=velocity_target.tolist(), position_error_model=position_error.tolist(),
                 speed_error_model=speed_error.tolist(), torque_integral_model=[0.] * 4,
                 q_ref_api=[np.nan] * 4, velocity_target_api=[np.nan] * 4,
                 tau_preclip_model=preclip.tolist(), tau_cmd_api=cmd_api.tolist(),
                 tau_frame_api=frame_api.tolist(),
                 tx_queued_steady_ns=[now - 100_000] * 6, tx_kind=[int(i in selected) for i in range(6)],
                 phase=2, segment_id=segment, selected_side=int(side == "right"),
                 actuation_scope=1, feedback_fresh=True, enable_requested=True,
                 dropped_samples=0, imu_quaternion_xyzw=[0., 0., 0., 1.],
                 imu_gyro=[0.] * 3, imu_last_ns=now - 1_000_000,
                 imu_acceleration_mps2=[0., 0., 9.81], imu_acceleration_steady_ns=now - 500_000,
                 stamp_sec=1, stamp_nanosec=k)
        for j, axis in enumerate(selected):
            r["feedback_steady_ns"][axis] = stamps[j]
            r["feedback_sequence"][axis] = seq[j] + int(j == 0 and missed_at is not None and k >= missed_at)
        rows.append(r)
    arrays, metadata = convert(rows, identity)
    npz = tmp_path / "export.npz"
    np.savez_compressed(npz, **arrays)
    npz.with_suffix(".json").write_text(json.dumps(metadata))
    return arrays, metadata, npz, profile


@pytest.mark.parametrize("side", ["left", "right"])
def test_coupled_model_and_heldout_replay(tmp_path, side):
    a, meta, npz, profile = fixture(tmp_path, side)
    assert a["control_steady_ns"].dtype == np.uint64
    assert int(a["control_steady_ns"][100]) == BASE + 100_000_000
    assert not np.any(a["axis_new_feedback"][:, meta["selected_axes"]].all(axis=1))
    assert np.any(a["pair_valid"] & a["axis_repeated_feedback"][:, meta["selected_axes"][0]])
    result = evaluate(npz, profile, tmp_path / "fit.json")
    np.testing.assert_allclose(result["model"]["mass_matrix_kg_m2_effective"], MASS, atol=.013)
    np.testing.assert_allclose(result["model"]["coulomb_nm_effective"], FC, atol=.10)
    assert result["model"]["damping_observable"]
    assert result["candidate_simulator_only"] is not None, result["qualification_failures"]
    assert result["candidate_simulator_only"]["auto_update_training"] is False
    assert result["controller_from_saved_run_yaml"]["kp_position"] == [8., 9., 10., 11.]
    assert max(result["logged_pc_cascade_consistency"]["velocity_target_rmse_rad_s"]) < 1e-10
    assert result["hardware_scope_and_conditions"]["supply_voltage_v_selected_pairs"]["min"] == 24.
    assert all(1 <= item["effective_lag_ms"] <= 4
               for item in result["command_to_feedback_alignment"]["per_axis"])
    assert result["torque_feedback_residuals"]["train"]["segments"]
    assert result["torque_feedback_residuals"]["holdout"]["segments"]
    assert not (set(result["torque_feedback_residuals"]["train"]["segments"])
                & set(result["torque_feedback_residuals"]["holdout"]["segments"]))
    with np.load(tmp_path / "fit.replay.npz", allow_pickle=False) as curves:
        assert curves["feedback_steady_ns"].dtype == np.uint64
        assert int(curves["control_steady_ns"][0]) >= BASE
        assert curves["q_residual"].shape[1] == 2
        assert curves["torque_residual_nm"].shape[1] == 2


def test_gaps_repeats_and_epoch_do_not_bridge(tmp_path):
    a, meta, _, _ = fixture(tmp_path, n_segment=2, ticks_per_segment=600)
    before, selected = checked_run(a, meta, .012)
    assert len(before) == 2
    assert np.all(np.diff(a["feedback_sequence"][before[0], selected[0]]) > 0)
    pairs = np.flatnonzero(a["pair_valid"] & (a["segment_id"] == 0))
    break_at = int(pairs[len(pairs)//2])
    a["side_valid"][break_at-1] = False  # poisoned raw waiting tick: no bridge
    after, _ = checked_run(a, meta, .012)
    assert len(after) == len(before) + 1
    assert all(not (min(run) < break_at < max(run)) for run in after)
    a["side_valid"][break_at-1] = True
    a["pair_epoch"][break_at:] += 1
    epochs, _ = checked_run(a, meta, .012)
    assert len(epochs) == len(before) + 1
    a["pair_epoch"][break_at:] -= 1
    a["axis_sequence_gap"][break_at-1, selected[0]] = True
    gaps, _ = checked_run(a, meta, .012)
    assert len(gaps) == len(before) + 1

    corrupted, meta, gap_npz, gap_profile = fixture(tmp_path, n_segment=4, ticks_per_segment=600, missed_at=200)
    assert corrupted["axis_sequence_gap"][200, selected[0]]
    assert not corrupted["pair_valid"][200]
    clean_runs, _ = checked_run(corrupted, meta, .012)
    assert len(clean_runs) == 5
    assert max(clean_runs[0]) < 200 < min(clean_runs[1])
    report = evaluate(gap_npz, gap_profile, tmp_path / "gap.json")
    assert report["candidate_simulator_only"] is None
    assert report["interval_qc"]["excluded_fault_ticks"]["sequence_gap"] == 1


def test_missing_outer_loop_profile_and_holdout_failure(tmp_path):
    a, meta, npz, profile = fixture(tmp_path)
    selected = meta["selected_axes"]
    saved_velocity_target = a["velocity_target_model"].copy()
    a["velocity_target_model"][a["pair_valid"], selected[0]] = np.nan
    a["double_loop_observed"][a["pair_valid"]] = False
    np.savez_compressed(npz, **a)
    with pytest.raises(ValueError, match="two-loop"):
        evaluate(npz, profile, tmp_path / "fit.json")
    a["velocity_target_model"] = saved_velocity_target
    a["double_loop_observed"][a["pair_valid"]] = True
    hold = np.flatnonzero((a["split"] == 2) & a["pair_valid"])
    a["torque_fb_model"][hold, selected[0]] += 7.
    np.savez_compressed(npz, **a)
    result = evaluate(npz, profile, tmp_path / "fit.json")
    assert result["candidate_simulator_only"] is None
    assert any("holdout axis 0: torque" in failure for failure in result["qualification_failures"])
    profile.write_text(profile.read_text().replace("kp_position: [8.0", "kp_position: [9.0"))
    with pytest.raises(ValueError, match="SHA256"):
        evaluate(npz, profile, tmp_path / "fit.json")


def test_rank_rejection_and_split_integrity(tmp_path):
    a, meta, npz, profile = fixture(tmp_path, n_segment=3, ticks_per_segment=600)
    result = evaluate(npz, profile, tmp_path / "ill_conditioned.json", max_condition=1.01)
    assert result["candidate_simulator_only"] is None
    assert result["observability"]["with_damping"]["condition"] > 1.01
    assert (tmp_path / "ill_conditioned.json").is_file()
    zero = a.copy()
    zero["dq_model"] = np.zeros_like(a["dq_model"])
    zero["q_model"] = np.zeros_like(a["q_model"])
    np.savez_compressed(npz, **zero)
    deficient = evaluate(npz, profile, tmp_path / "rank_deficient.json")
    assert deficient["candidate_simulator_only"] is None
    assert deficient["observability"]["without_damping"]["rank"] == 0
    a["split"][np.flatnonzero(a["split"] == 2)[0]] = 1
    np.savez_compressed(npz, **a)
    with pytest.raises(ValueError, match="complete-segment labels"):
        evaluate(npz, profile, tmp_path / "fit.json")


def test_each_heldout_segment_is_qualified_independently(tmp_path):
    a, _, npz, profile = fixture(tmp_path, n_segment=6, ticks_per_segment=600)
    holdout_segments = np.unique(a["segment_id"][a["split"] == 2])
    assert len(holdout_segments) == 2
    mask = (a["segment_id"] == holdout_segments[0]) & a["pair_valid"]
    a["torque_fb_model"][mask, 2] += 6.
    np.savez_compressed(npz, **a)
    result = evaluate(npz, profile, tmp_path / "fit.json")
    assert result["candidate_simulator_only"] is None
    assert len(result["torque_feedback_residuals"]["holdout"]["by_segment"]) == 2
    assert any(f"holdout segment {holdout_segments[0]} axis 0" in text
               for text in result["qualification_failures"])


@pytest.mark.parametrize("pattern", ["rotation_chirp", "multiband_chirp"])
def test_full_turn_run_cannot_use_small_angle_affine_gravity_fit(tmp_path, pattern):
    _, meta, npz, profile = fixture(tmp_path, n_segment=2, ticks_per_segment=20)
    profile.write_text(profile.read_text() + f"    probe_pattern: {pattern}\n")
    meta["profile_sha256"] = hashlib.sha256(profile.read_bytes()).hexdigest()
    npz.with_suffix(".json").write_text(json.dumps(meta))
    with pytest.raises(ValueError, match="fixed-base nonlinear closed-chain"):
        load_run(npz, profile)
