"""Static bringup gate for the real, 105-degree motor-zero pair rig."""

from pathlib import Path

import yaml

from check_wheel_leg_profile import check
from check_wheel_leg_profile import main


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "rmcs_ws/src/rmcs_bringup/config"
LEGACY = ROOT / "docs/zh-cn/artifacts/pair_multiband_v2/legacy_profiles"
# Legacy checks are offline archive validation, not launchable profiles.
PROFILE = LEGACY / "wheel-leg-infantry-pair-identification.yaml"
PASSIVE_PROFILE = CONFIG / "wheel-leg-infantry-pair-observe.yaml"
WHEEL_PROFILE = CONFIG / "wheel-leg-infantry-wheel-identification.yaml"
PD_PROFILE = LEGACY / "wheel-leg-infantry-pair-pd-identification.yaml"
LOADED_PD_PROFILE = CONFIG / "wheel-leg-infantry-pair-pd-loaded-identification.yaml"


def test_loaded_pd_profile_requires_explicit_loading_and_matching_revision():
    run = yaml.safe_load(LOADED_PD_PROFILE.read_text())
    assert check(run) == []
    params = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    del params["multiband_load_offset"]
    assert any("explicit loading" in e for e in check(run))
    params["multiband_load_offset"] = .2
    run["wheel_leg_identification_recorder"]["ros__parameters"]["trajectory_revision"] = "pair_multiband_v3_pd_1khz"
    assert any("requires loaded PD" in e for e in check(run))


def test_pd_profile_has_explicit_multirate_and_manufacturer_contract():
    run = yaml.safe_load(PD_PROFILE.read_text())
    assert check(run) == []
    run["wheel_leg_pair_identification_controller"]["ros__parameters"]["ki_velocity"][0] = .01
    assert any("four zeros" in e for e in check(run))


def test_pd_profile_rejects_wrong_clock_and_encoding_scale_as_peak_torque():
    run = yaml.safe_load(PD_PROFILE.read_text())
    params = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    params["reference_frequency_hz"] = 1000.0
    params["dm_peak_torque_nm"] = 54.0
    run["wheel_leg"]["ros__parameters"]["dm_control_torque_max"][0] = 54.0
    errors = check(run)
    assert any("reference_frequency_hz" in e for e in errors)
    assert any("rated20/peak40" in e for e in errors)
    assert any("peak 40" in e for e in errors)


def profile():
    return yaml.safe_load(PROFILE.read_text(encoding="utf-8"))


def test_template_fails_closed_without_real_calibration():
    run = profile()
    controller = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    controller["experiment_stage"] = "trajectory"
    issues = check(run)
    assert any("left_beta_degrees" in issue for issue in issues)
    assert any("right_beta_degrees" in issue for issue in issues)
    assert any("inertia_bound" in issue for issue in issues)


def test_initial_motor_coordinate_probe_uses_no_invented_knee_lut():
    run = profile()
    assert check(run) == []
    controller = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    assert "left_beta_degrees" not in controller
    controller["probe_pattern"] = "bounded"
    controller["probe_knee_amplitude"] = .12
    assert any("probe_knee_amplitude" in issue for issue in check(run))
    controller["probe_knee_amplitude"] = .08
    run["wheel_leg"]["ros__parameters"]["dm_control_torque_max"][0] = 20.01
    assert any("PROBE hardware torque" in issue for issue in check(run))


def test_pair_direct_middle_enable_needs_no_down_dwell_parameter():
    run = profile()
    controller = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    controller.pop("arm_dwell_s", None)
    assert check(run) == []
    # Wheel identification retains its separate dwell contract.
    wheel = yaml.safe_load(WHEEL_PROFILE.read_text(encoding="utf-8"))
    del wheel["wheel_leg_wheel_identification_controller"]["ros__parameters"]["arm_dwell_s"]
    assert any("arm_dwell_s" in issue for issue in check(wheel))


def test_ros_parameter_yaml_alias_is_rejected_before_launch(tmp_path, monkeypatch, capsys):
    profile_path = tmp_path / "aliased.yaml"
    profile_path.write_text("side: &side left\ncopy: *side\n", encoding="utf-8")
    monkeypatch.setattr("sys.argv", ["check_wheel_leg_profile.py", str(profile_path)])
    assert main() == 2
    assert "does not support anchors" in capsys.readouterr().out


def test_wheels_have_separate_esc_zero_current_profile():
    run = yaml.safe_load(WHEEL_PROFILE.read_text(encoding="utf-8"))
    assert check(run) == []
    run["wheel_leg_wheel_identification_controller"]["ros__parameters"]["wheel_torque_cap"] = 5.0
    assert any("wheel_torque_cap" in issue for issue in check(run))
    run["rmcs_executor"]["ros__parameters"]["components"].append(
        "rmcs_core::controller::identification::WheelLegPairIdentificationController -> wheel_leg_pair_identification_controller")
    assert any("cannot share torque outputs" in issue for issue in check(run))


def test_passive_capture_loads_without_model_calibration_and_refuses_torque_controller():
    run = yaml.safe_load(PASSIVE_PROFILE.read_text(encoding="utf-8"))
    assert check(run) == []
    run["wheel_leg_identification_recorder"]["ros__parameters"]["record_on_double_middle"] = False
    assert any("record_on_double_middle" in issue for issue in check(run))
    run["wheel_leg_identification_recorder"]["ros__parameters"]["record_on_double_middle"] = True
    run["rmcs_executor"]["ros__parameters"]["components"].append(
        "rmcs_core::controller::identification::WheelLegPairIdentificationController -> wheel_leg_pair_identification_controller")
    assert any("passive recorder cannot include" in issue for issue in check(run))


def test_profile_requires_one_matching_side_and_105_degree_motor_zero():
    run = profile()
    recorder = run["wheel_leg_identification_recorder"]["ros__parameters"]
    recorder["side"] = "right"
    assert any("same left/right side" in issue for issue in check(run))
    recorder["side"] = "left"
    recorder["encoder_zero_inner_knee_deg"] = 110.0
    assert any("105-degree" in issue for issue in check(run))


def test_profile_rejects_internal_pd_or_duplicate_rl_torque_owner():
    run = profile()
    run["wheel_leg"]["ros__parameters"]["joint_control_mode"] = "position_pd"
    assert any("joint_control_mode" in issue for issue in check(run))
    run["rmcs_executor"]["ros__parameters"]["components"].append("rmcs::rl::RlController -> rl_controller")
    assert any("share the six torque outputs" in issue for issue in check(run))


def test_trajectory_rejects_training_110_degree_lut():
    run = profile()
    controller = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    controller["experiment_stage"] = "trajectory"
    controller["left_beta_degrees"] = [75.0, 90.0, 105.0]
    controller["left_delta_rad"] = [.2, .4, .6]
    controller["right_beta_degrees"] = [75.0, 90.0, 105.0]
    controller["right_delta_rad"] = [-.6, -.4, -.2]
    assert any("pose_hip" in issue or "inertia_bound" in issue for issue in check(run))
    controller["left_beta_degrees"][-1] = 110.0
    assert any("real 40..105 degree" in issue for issue in check(run))


def test_rotation_sweep_fields_and_fixed_rig_are_explicit():
    run = profile()
    params = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    assert params["probe_pattern"] == "rotation_chirp"
    assert check(run) == []
    del params["sweep_common_hz"]
    assert any("sweep_common_hz" in error for error in check(run))
    params["sweep_common_hz"] = [.05, .5]
    params["sweep_relative_amplitude"] = .8
    assert any("two linkage shapes" in error for error in check(run))
    params["sweep_relative_amplitude"] = .2
    params["phase_api_angle"] = False
    assert any("phase_api_angle" in error for error in check(run))
    params["phase_api_angle"] = True
    run["wheel_leg_identification_recorder"]["ros__parameters"]["wheels_off_ground"] = False
    assert any("fixed chassis" in error for error in check(run))


def test_missing_pattern_cannot_silently_fall_back_to_tiny_probe():
    run = profile()
    del run["wheel_leg_pair_identification_controller"]["ros__parameters"]["probe_pattern"]
    assert any("probe_pattern" in error for error in check(run))


def test_pair_loop_frequency_and_recorded_contract_cannot_drift():
    run = profile()
    run["wheel_leg_identification_recorder"]["ros__parameters"]["control_frequency_hz"] = 200.0
    assert any("control_frequency_hz" in error for error in check(run))
    run = profile()
    del run["wheel_leg_pair_identification_controller"]["ros__parameters"]["control_frequency_hz"]
    assert any("control_frequency_hz" in error for error in check(run))


def multiband_profile():
    return yaml.safe_load(PROFILE.with_name(
        "wheel-leg-infantry-pair-multiband-identification.yaml").read_text())


def test_multiband_profile_requires_all_fields_without_legacy_fallback():
    run = multiband_profile()
    assert check(run) == []
    p = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    assert "sweep_common_hz" not in p
    assert "probe_common_amplitude" not in p
    del p["multiband_relative_hz"]
    assert any("multiband_relative_hz" in e for e in check(run))
    p["multiband_relative_hz"] = [.1, .65, 1.8, 4.]
    p["multiband_common_hz"] = [.08, .5, .5, 4.]
    assert any("strictly ordered" in e for e in check(run))


def test_multiband_budget_validation_and_recorded_revision_match():
    run = multiband_profile()
    p = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    p["ready_timeout_s"] = 4.
    assert any("preflight" in e for e in check(run))
    p["ready_timeout_s"] = 12.
    p["multiband_ramp_s"] = 6.
    assert any("full-amplitude" in e for e in check(run))
    p["multiband_ramp_s"] = 2.
    run["wheel_leg_identification_recorder"]["ros__parameters"]["trajectory_revision"] = "old"
    assert any("trajectory_revision" in e for e in check(run))


def test_multiband_requires_fixed_chassis_and_preserves_20_nm_cap():
    run = multiband_profile()
    run["wheel_leg_identification_recorder"]["ros__parameters"]["wheels_off_ground"] = False
    assert any("fixed chassis" in e for e in check(run))
    run["wheel_leg_identification_recorder"]["ros__parameters"]["wheels_off_ground"] = True
    run["wheel_leg"]["ros__parameters"]["dm_control_torque_max"][2] = 25.
    assert any("PROBE hardware torque" in e for e in check(run))


def test_multiband_tick_budget_matches_controller_admission():
    run = multiband_profile()
    p = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    assert p["max_tick_gap_s"] == .02
    assert check(run) == []
    for gap in [.001, .020001]:
        p["max_tick_gap_s"] = gap
        assert any("max_tick_gap_s" in e for e in check(run))


def test_multiband_measured_delta_policy_is_explicit_and_scoped():
    run = multiband_profile()
    p = run["wheel_leg_pair_identification_controller"]["ros__parameters"]
    assert p["probe_measured_delta_policy"] == "diagnostic"
    assert check(run) == []
    p["probe_measured_delta_policy"] = "stop"
    assert check(run) == []
    p["probe_measured_delta_policy"] = "ignore_everything"
    assert any("probe_measured_delta_policy" in e for e in check(run))
    del p["probe_measured_delta_policy"]
    assert any("probe_measured_delta_policy" in e for e in check(run))
    p["probe_measured_delta_policy"] = "diagnostic"
    p["experiment_stage"] = "hold"
    assert any("probe_measured_delta_policy" in e for e in check(run))


def test_wheel_contract_rejects_missing_frequency_and_wrong_current_conversion():
    run = yaml.safe_load(WHEEL_PROFILE.read_text())
    params = run["wheel_leg_wheel_identification_controller"]["ros__parameters"]
    del params["reference_frequency_hz"]
    assert any("reference_frequency_hz" in e for e in check(run))
    params["reference_frequency_hz"] = 50.
    params["wheel_current_limit_a"] = 21.
    assert any("wheel_current_limit_a" in e for e in check(run))
    params["wheel_current_limit_a"] = 20.
    params["wheel_velocity_ki"] = .1
    assert any("wheel_velocity_ki" in e for e in check(run))
    params["wheel_velocity_ki"] = 0.
    run["wheel_leg_identification_recorder"]["ros__parameters"]["sample_frequency_hz"] = 100.
    assert any("sample_frequency_hz" in e for e in check(run))
