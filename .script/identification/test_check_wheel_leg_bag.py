from check_wheel_leg_bag import inspect


def sample(tick=1, scope=1):
    return {"tick": tick, "phase": 2, "segment_id": 0, "actuation_scope": scope,
            "selected_side": 0, "q_ref_model": [.1, .2, float("nan"), float("nan")],
            "q_api": [.1, .2, float("nan"), float("nan"), 0., 0.],
            "torque_fb_api": [.1, .2, float("nan"), float("nan"), 0., 0.],
            "tau_frame_api": [.1, .2, float("nan"), float("nan"), 0., 0.],
            "velocity_target_model": [.3, .4, float("nan"), float("nan")],
            "tau_preclip_model": [1., 2., float("nan"), float("nan")],
            "feedback_steady_ns": [tick * 1_000_000 - 200_000] * 6,
            "imu_last_ns": tick * 1_000_000 - 200_000,
            "control_steady_ns": tick * 1_000_000,
            "dm_status": [1, 1, 0, 0], "dropped_samples": 0}


def test_pair_data_complete():
    assert inspect([sample(1), sample(2)], "left")["status"] == "recording_complete"


def test_any_starting_pose_can_be_recorded_while_all_motors_disabled():
    rows = [sample(1), sample(2)]
    for row in rows:
        row["phase"] = -1
        row["actuation_scope"] = 0
        row["enable_requested"] = False
        row["dm_status"] = [0, 0, 0, 0]
        row["q_ref_model"] = [float("nan")] * 4
    result = inspect(rows, "left")
    assert result["status"] == "passive_capture"
    assert result["complete_two_loop_samples"] == 0


def test_pid_or_missing_outer_loop_is_flagged():
    rows = [sample(1), sample(2, 2)]
    rows[-1]["velocity_target_model"][0] = float("nan")
    rows[-1]["dm_status"][2] = 1
    report = inspect(rows, "left")
    assert report["status"] == "inspect_before_fitting"
    assert any("torque-mode" in reason for reason in report["reasons"])
    assert any("inactive DM" in reason for reason in report["reasons"])


def test_wheel_audit_distinguishes_servo_coast_and_validation():
    rows = [sample(i, 3) for i in range(1, 5)]
    for row, target in zip(rows, (.6, 0., -.6, .4)):
        row.update(experiment_kind=1, enable_requested=True, dm_status=[0] * 4,
                   wheel_mode=2 if target == 0 else 1,
                   segment_role=1 if target == .4 else 0,
                   wheel_velocity_target_api=[target, target],
                   dq_api=[0., 0., 0., 0., target, target],
                   tau_preclip_api=[float("nan")] * 4 + [0., 0.],
                   tau_frame_api=[float("nan")] * 4 + [0., 0.])
    assert inspect(rows, "left", "wheel")["status"] == "wheel_recording_complete"
    rows[1]["tau_frame_api"][4] = .2
    assert any("not neutral" in reason for reason in inspect(rows, "left", "wheel")["reasons"])


def test_invalid_run_marker_prevents_recording_complete(tmp_path, monkeypatch, capsys):
    from check_wheel_leg_bag import main
    (tmp_path / "RUN_INVALID.json").write_text('{"status":"invalid_profile_provenance"}')
    monkeypatch.setattr("sys.argv", ["check_wheel_leg_bag.py", "--bag", str(tmp_path / "bag"),
                                    "--profile", "unused.yaml"])
    assert main() == 2
    assert "invalid_profile_provenance" in capsys.readouterr().out
