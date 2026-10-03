"""Calibration, acquisition identities and independent real2sim holdout contracts."""
import json

import numpy as np
import pytest
import yaml

from check_wheel_leg_profile import check
from export_identification_bag import convert, load_identity
from prepare_v6_pair_recording import prepare
from test_export_identification_bag import IDENTITY, row
from v6_pair_recording import CALIBRATION, ROOT, RUNS, generate_profile


@pytest.mark.parametrize("run", RUNS)
def test_all_runs_reuse_upstream_calibration_and_freeze_pd(run):
    profile = generate_profile(run)
    assert check(profile) == []
    p = profile["wheel_leg_pair_identification_controller"]["ros__parameters"]
    binding = json.loads(p["calibration_binding_json"])
    assert binding["upstream_commit"] == "8149afda1de0f0e79dac07a6444fc3741759d5c2"
    assert p["model_sign"] == [-1.]*4
    assert p["model_offset"] == [1.6, 2.93, -1.6, -2.93]
    assert p["pd_kp"] == [160.]*4 and p["pd_kd"] == [2.5]*4
    assert binding["hardware_multipoint_g0_verified"] is False
    assert np.array(binding["sides"]["left"]["delta_rad"]).tolist() == (-np.array(binding["sides"]["right"]["delta_rad"])).tolist()
    assert binding["sides"]["left"]["hip_sign"] == -1
    assert binding["sides"]["right"]["hip_sign"] == 1


@pytest.mark.parametrize("key,value", [("pd_kp", [170.]*4), ("model_sign", [1.]*4),
    ("recording_beta_min_deg", 40.), ("recording_beta_max_deg", 105.),
    ("recording_hip_zero", 0.), ("recording_arrival_timeout_s", float("nan")),
    ("calibration_sha256", "unbound"), ("recording_run", "B01"),
    ("recording_delta_rad", [0., 1., 2.]), ("reference_frequency_hz", 1000.)])
def test_invalid_control_or_mapping_is_rejected(key, value):
    profile = generate_profile("L01")
    profile["wheel_leg_pair_identification_controller"]["ros__parameters"][key] = value
    assert check(profile)


def test_side_and_recorder_asset_identity_must_match():
    profile = generate_profile("RJ01")
    profile["wheel_leg_identification_recorder"]["ros__parameters"]["side"] = "left"
    assert check(profile)
    profile = generate_profile("RJ01")
    profile["wheel_leg_identification_recorder"]["ros__parameters"]["asset_manifest_sha256"] = "v5"
    assert check(profile)


def test_saved_profile_and_calibration_load_for_export(tmp_path):
    profile = tmp_path / "profile.yaml"
    prepare("L02", profile)
    identity = load_identity(CALIBRATION, profile)
    assert identity["recording"]["recording_run"] == "L02"
    assert identity["model_sign"] == [-1.]*4
    changed = tmp_path / "calibration.json"
    changed.write_text(CALIBRATION.read_text() + "\n")
    with pytest.raises(ValueError, match="differs from the run binding"):
        load_identity(changed, profile)


@pytest.mark.parametrize("run,split", [("R02", 1), ("R03", 2), ("RJ01", 1), ("RJ02", 2)])
def test_run_splits_and_actual_coverage_do_not_use_requested_beta(run, split):
    p = generate_profile(run)["wheel_leg_pair_identification_controller"]["ros__parameters"]
    samples = [row(i, i//5) for i in range(30)]
    for sample in samples:
        sample.update(recording_protocol_version=1, inner_knee_fk_deg=104.,
                      inner_knee_requested_deg=65., inner_knee_reference_deg=64.,
                      thigh_orientation_fk_deg=0., reference_update_tick=2**60,
                      segment_elapsed_s=.2, reference_center_delta_rad=.01,
                      configuration_admission=0, skipped_arrival_segment=7,
                      jump_cycle_id=0, jump_phase=3,
                      q_unwrapped_model=[0., 1., -8., -9.], segment_role=split-1)
    identity = {**IDENTITY, "recording": p}
    arrays, metadata = convert(samples, identity)
    assert set(arrays["split"][arrays["pair_valid"]]) == {split}
    assert arrays["reference_update_tick"].dtype == np.uint64
    assert int(arrays["reference_update_tick"][0]) == 2**60
    assert arrays["q_model"][0, 2] == -8
    qc = metadata["recording"]
    assert qc["coverage"]["65"]["actual_fk_seconds"] == 0
    assert qc["coverage"]["65"]["requested_seconds"] > 0
    assert qc["uncovered_arrival_segments"] == [7]
    assert qc["jump_phases"][0]["covered"] is False
    assert metadata["controller"]["recording_run"] == run
    assert metadata["insufficient_holdout"] == (split != 2)


def test_v6_bag_rejects_missing_protocol_version():
    p = generate_profile("R02")["wheel_leg_pair_identification_controller"]["ros__parameters"]
    with pytest.raises(ValueError, match="protocol version"):
        convert([row(0)], {**IDENTITY, "recording": p})


def test_cpp_preview_matches_protocol_phases_and_serialized_profile(tmp_path):
    import os
    from pathlib import Path
    binary = Path(os.environ.get("RMCS_RECORDING_PREVIEW_BIN", ROOT / "rmcs_ws/install/lib/rmcs_core/wheel_leg_recording_preview"))
    if not binary.is_file():
        pytest.skip("build rmcs_core preview tool first")
    output = tmp_path / "profile.yaml"
    manifest = prepare("LJ01", output, preview_dir=tmp_path, preview_bin=binary)
    assert 360 <= manifest["duration_s"] <= 480
    assert {s["jump_phase"] for s in manifest["segments"] if s["cycle"] >= 0} == set(range(1, 9))
    assert all(s["role"] == 0 for s in manifest["segments"])
    assert all(abs(s["duration_s"]*50-round(s["duration_s"]*50)) < 1e-8 for s in manifest["segments"])
    assert check(yaml.safe_load(output.read_text())) == []
    assert (tmp_path / "reference-50hz.csv").is_file()
    assert (tmp_path / "calibration.json").read_bytes() == CALIBRATION.read_bytes()
