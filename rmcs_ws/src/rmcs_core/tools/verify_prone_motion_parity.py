#!/usr/bin/env python3
"""Replay identical sensor fixtures through training Python and production C++.

The normalized inference fixture is synthetic and never installed as a policy.
This checks arithmetic and contracts, without starting Isaac Sim or hardware.
"""
import argparse
from copy import deepcopy
import hashlib
import io
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import tempfile

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto
import onnxruntime as ort
import torch
import yaml

ROOT = Path(__file__).resolve().parents[4]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_config():
    file = ROOT / "rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml"
    cfg = yaml.safe_load(file.read_text())
    contract_file = file.parent / "contracts/wheel_leg_prone_motion_v4.json"
    contract = json.loads(contract_file.read_text())
    bridge, controller, hardware, chassis = [cfg[name]["ros__parameters"] for name in
        ("rl_bridge", "wheel_leg_controller", "wheel_leg", "wheel_leg_chassis_controller")]
    require(bridge["policy_rate"] == 100. and controller["pd_frequency"] == 1000.
            and cfg["rmcs_executor"]["ros__parameters"]["update_rate"] == 1000., "Timing differs")
    require(controller["unified_full_rl"] and not controller["heading_hold_enabled"]
            and hardware["joint_control_mode"] == "torque", "Need direct full RL and host PD")
    require(bridge["expected_observation_contract_sha256"] == contract["observation_contract_sha256"]
            and bridge["expected_observation_version"] == controller["observation_version"]
            == contract["training_code_version"], "Observation identity differs")
    require(bridge["expected_obs_normalization"] == "embedded"
            and not bridge["normalization_from_metadata"], "Training normalization must be embedded once")
    require(bridge["history_length"] == 1 and bridge["rl_obs_size"] == bridge["rl_feature_size"] == 35
            and bridge["rl_action_size"] == 6, "Actor ABI differs")
    require(bridge["observation_terms"] == ["index=0 path=/wheel_leg/rl/input/frame take=array type=double_array dim=35 name=frame"]
            and bridge["observation_frame_sequence_interface"] == "/wheel_leg/rl/input/frame_sequence",
            "Need one coherent frame")
    require(bridge["expected_model_spec_sha256"] == contract["model_spec_sha256"]
            and bridge["leg_winding_version"] == controller["leg_winding_version"] == "paired_branch107_v1",
            "Mechanical contract differs")
    require(controller["wheel_velocity_kp"] == .5 and hardware["joint_position_kp"] == 160.
            and hardware["joint_position_kd"] == 2.5
            and controller["gas_spring_compensation_enabled"] and controller["gas_spring_compensation_gain"] == 1.,
            "Actuator parameters differ")
    require(bridge["model_type"] == "wheel_leg_v202"
            and controller["control_profile"] == contract["control_profile"] == "full_vehicle_v202"
            and controller["nominal_model_pos"] == contract["nominal_policy_position"],
            "Full-vehicle model, profile or nominal posture differs")
    for params in (controller, chassis):
        require(params["vx_max"] == 2. and abs(params["yaw_rate_max"] - 100 * math.tau / 60) < 1e-9,
                "Motion command limits differ")
    require(controller["height_min"] == chassis["command_height_min"] == .23
            and controller["height_max"] == chassis["command_height_max"] == .42, "Height bounds differ")
    for i, name in enumerate(contract["action_order"]):
        scale, bound = (.25, 4) if i < 4 else (10., 90)
        require(bridge["action_terms"][i] == f"index={i} output=/wheel_leg/rl/target/{name} scale={scale} clip=-{bound}:{bound}",
                "Action scale/bounds differ")
    return dict(config_sha256=sha(file), contract_sha256=sha(contract_file))


def sensor_records(training, observer):
    records = []
    bank = json.loads((training / "model/wheel_leg_v2.0.2/training/pose_bank.json").read_text())
    geometry = observer.coordinate
    nominal32 = geometry.geometry.nominal
    nominal = nominal32.numpy().astype(np.float64)
    q = np.r_[nominal, 0., 0.]
    tick = 0

    def emit(label, *, dq=None, gyro=(0., 0., 0.), gravity=(0., 0., -1.), command=(0., 0., 0.),
             height=.305, enabled=True, reset=False, duplicate=False):
        nonlocal tick
        if not duplicate:
            tick += 1
        records.append(dict(label=label, tick=tick, reset=reset, enabled=enabled, q=q.copy(),
            dq=np.zeros(6) if dq is None else np.asarray(dq), gyro=np.asarray(gyro), gravity=np.asarray(gravity),
            command=np.asarray(command), height=height, applied=.2*np.sin(np.arange(6)+tick*.03)))

    def repeat(label, count, **kwargs):
        for _ in range(count):
            emit(label, **kwargs)

    def coord(legs, gravity=(0., 0., -1.)):
        position = torch.tensor(np.r_[legs, 0., 0.], dtype=torch.float32)[None]
        return geometry(position, torch.tensor([gravity], dtype=torch.float32))[0][0].numpy()

    for offset in (0., 1e8, -1e8):
        q = np.r_[nominal, offset, -offset]
        emit("origin", reset=True)
        repeat("capture", 24)
        for _ in range(80):
            q[4:] += [.03, -.03]
            emit("drift", dq=[0., 0., 0., 0., 3., -3.])
        repeat("hold", 80)
        emit("duplicate", duplicate=True)
        base_coordinate = coord(nominal)
        origin = np.array([offset+2.4, -offset-2.4])
        for name in ("stand_low", "stand_high", "stand"):
            row = bank["poses"][bank["diagnostic_height_starts"]["mapping"][name]]["full_joint_positions"]
            end = np.array([row[k] for k in ("L_joint1", "LL_joint1", "R_joint1", "RR_joint1")])
            start = q[:4].copy()
            for i in range(1, 101):
                previous = q.copy()
                q[:4] = start + (end-start)*i/100
                q[4:] = origin + (base_coordinate-coord(q[:4]))/np.array([.06, -.06])
                emit("height_geometry", dq=(q-previous)/.01, height=.23 if name == "stand_low" else .42)
            repeat("height_hold", 40)
        pitch = 0.
        for endpoint in (-.2, .2, 0.):
            start_pitch = pitch
            for i in range(1, 41):
                previous, previous_pitch = q.copy(), pitch
                pitch = start_pitch+(endpoint-start_pitch)*i/40
                gravity = (math.sin(pitch), 0., -math.cos(pitch))
                q[4:] = origin+(base_coordinate-coord(q[:4], gravity))/np.array([.06, -.06])
                emit("pitch_geometry", dq=(q-previous)/.01, gyro=(0., (pitch-previous_pitch)/.01, 0.),
                     gravity=gravity)
        repeat("invalid_tilt", 20, gravity=(math.sin(.3), 0., -math.cos(.3)))
        repeat("restored_tilt", 30)
        emit("inverted", gravity=(0., 0., 1.))
        repeat("recapture", 25)
        for direction in (1, -1):
            for _ in range(60):
                q[4:] += np.array([.5, -.5])*direction
                emit("saturation_and_return", dq=np.r_[np.zeros(4), np.array([50., -50.])*direction])
            repeat("settled", 100)
        q[4] += math.tau/15.62
        repeat("jump_latch", 30)
        emit("end_request", command=(.001, 0., 0.))
        repeat("recapture", 25)
        q[5] = float("nan")
        emit("nonfinite_wheel")
        q[5] = origin[1]
        repeat("nonfinite_latch", 25)
        emit("disable", enabled=False)
        repeat("recapture", 25)
        for command in ((2., 0., 0.), (-2., 0., 0.), (0., .1, 0.), (0., 0., 100*math.tau/60),
                        (0., 0., -100*math.tau/60), (.2, 0., .6)):
            repeat("motion", 3, command=command)
            repeat("recapture", 25)
        emit("invalid_gyro", gyro=(float("nan"), 0., 0.))
        repeat("restored_gyro", 5)
        emit("disable", enabled=False)
        repeat("fast_wheel_capture_denied", 25, dq=(0., 0., 0., 0., 2.5, -2.5))
        repeat("fast_leg_capture_denied", 25, dq=(3., 0., 0., 0., 0., 0.))
        repeat("capture", 25)
    # Physical endpoint / bounded sensor projection and a whole-turn leg chart.
    for side in (0, 1):
        knots = getattr(geometry.geometry, f"side{side}_delta_rad")
        for delta in (float(knots[0]), float(knots[-1]), float(knots[0])-.0005, float(knots[-1])+.0005,
                      float(knots[-1])+.002, float(knots[-1])+.02):
            q = np.r_[nominal, 0., 0.]
            q[side*2+1] = q[side*2]+delta
            emit("endpoint", reset=True)
            repeat("endpoint", 25)
    q = np.r_[nominal+math.tau, 0., 0.]
    emit("whole_turn_legs", reset=True)
    repeat("whole_turn_legs", 25)
    return records, nominal32


def python_frames(records, observer, nominal):
    results = []
    for row in records:
        if row["reset"]:
            observer.reset(torch.tensor([0]))
        frame = torch.zeros(1, 35)
        frame[0, :3] = torch.tensor(row["command"])
        frame[0, 3] = torch.tensor(row["height"], dtype=torch.float32)*5.
        frame[0, 4:7] = torch.tensor(row["gyro"], dtype=torch.float32)*.5
        frame[0, 7:10] = torch.tensor(row["gravity"], dtype=torch.float32)
        delta = torch.tensor(row["q"][:4], dtype=torch.float32)-nominal
        frame[0, 10:14] = torch.atan2(delta.sin(), delta.cos())
        frame[0, 16:22] = torch.tensor(row["dq"], dtype=torch.float32)*.1
        frame[0, 22:28] = torch.tensor(row["applied"], dtype=torch.float32)
        frame[0, 28] = 1.
        enabled = torch.tensor([row["enabled"]])
        frame = frame.clamp(-100., 100.)
        frame[:, 14:16] = observer.local_wheel_angles(frame,
            torch.tensor(row["q"][4:], dtype=torch.float64)[None], enabled, row["tick"])
        frame = observer.update(frame, nominal, enabled, row["tick"])
        results.append(frame[0].numpy() if torch.isfinite(frame).all() else None)
    return results


def fixture_model(output, contract):
    rng = np.random.default_rng(261030)
    tensors = dict(mean=rng.normal(0., .3, (1, 35)), denominator=rng.uniform(.2, 2., (1, 35))+.01,
                   w1=rng.normal(0., .03, (35, 12)), b1=rng.normal(0., .05, (1, 12)),
                   w2=rng.normal(0., .1, (12, 6)), b2=rng.normal(0., .05, (1, 6)))
    tensors = {k: np.asarray(v, dtype=np.float32) for k, v in tensors.items()}
    nodes = [helper.make_node(op, inputs, [name]) for op, inputs, name in (
        ("Sub", ["obs", "mean"], "centered"), ("Div", ["centered", "denominator"], "normalized"),
        ("MatMul", ["normalized", "w1"], "hidden_raw"), ("Add", ["hidden_raw", "b1"], "hidden_bias"),
        ("Elu", ["hidden_bias"], "hidden"), ("MatMul", ["hidden", "w2"], "output_raw"),
        ("Add", ["output_raw", "b2"], "actions"))]
    graph = helper.make_graph(nodes, "synthetic_normalization_parity_only",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 35])],
        [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 6])],
        initializer=[numpy_helper.from_array(value, name) for name, value in tensors.items()])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=9)
    metadata = dict(policy_hz="100", rmcs_observation_contract_sha256=contract["observation_contract_sha256"],
        rmcs_observation_version=contract["training_code_version"], rmcs_obs_normalization="embedded",
        leg_winding_version="paired_branch107_v1", leg_target_model_spec_sha256=contract["model_spec_sha256"],
        scope="SYNTHETIC TEST FIXTURE; no training or hardware qualification")
    helper.set_model_props(model, metadata)
    path = output / "synthetic_normalization_fixture.onnx"
    onnx.checker.check_model(model)
    onnx.save(model, path)
    return path, model, tensors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--training-revision", help="Replay exact committed Python sources, preserving the live checkout")
    parser.add_argument("--container", default="rmcs_devcontainer-rmcs-develop-1")
    parser.add_argument("--output", type=Path, default=ROOT / "reports/wheel_leg_prone_motion_v4")
    args = parser.parse_args()
    training, output = args.training_root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # A failed rerun must not leave a previous successful receipt in place.
    (output / "parity.json").write_text(json.dumps(dict(status="incomplete",
        training_revision=args.training_revision, scope="offline synthetic parity"), indent=2)+"\n")
    # A concurrent training-asset edit must never silently change this parity baseline.
    snapshot = None
    source_root = training
    if args.training_revision:
        snapshot = tempfile.TemporaryDirectory(prefix="rmcs-prone-training-")
        source_root = Path(snapshot.name)
        archive = subprocess.check_output(["git", "archive", args.training_revision, "src",
            "contracts/standing_geometry107_v1.json",
            "model/wheel_leg_v2.0.2/training", "model/gas_spring_identified_symmetric_20261028/model.json",
            "reports/flat_base_continuation_20261019/low_friction_candidate001/height_lookup.json"], cwd=training)
        with tarfile.open(fileobj=io.BytesIO(archive)) as source:
            source.extractall(source_root, filter="data")
    sys.path.insert(0, str(source_root / "src"))
    from wheeled_tasks.chassis.standing_local_odometry import LocalStandingOdometry
    from wheeled_algo.paired_recovery import paired_offsets
    from wheeled_tasks.chassis.gas_spring_compensation import GasSpringCompensation
    from sync_prone_motion_contract import generate
    for path, expected in generate(training, args.training_revision).items():
        require(path.read_text() == expected, "Frozen training contract changed: " + str(path))
    config = check_config()
    contract = json.loads((ROOT / "rmcs_ws/src/rmcs_bringup/config/contracts/wheel_leg_prone_motion_v4.json").read_text())

    def mapped(path):
        return "/workspaces/RMCS/" + str(path.relative_to(ROOT))

    def run(binary, arguments, data="", environment=None):
        command = ["docker", "exec", "-i"]
        for key, value in (environment or {}).items():
            command += ["-e", key+"="+value]
        command += [args.container, "bash", "-lc",
            'source /opt/ros/jazzy/setup.bash; source /workspaces/RMCS/rmcs_ws/install/setup.bash; exec "$@"',
            "parity", "/workspaces/RMCS/rmcs_ws/build/" + binary] + list(arguments)
        return subprocess.run(command, input=data, capture_output=True, text=True, timeout=60)

    observer = LocalStandingOdometry(1, "cpu", .01, contract["observation"],
        geometry_directory=source_root / "model/wheel_leg_v2.0.2/training")
    records, nominal = sensor_records(source_root, observer)
    expected = python_frames(records, observer, nominal)
    input_text = "\n".join(" ".join(map(str, [r["tick"], int(r["reset"]), int(r["enabled"]),
        *r["q"], *r["dq"], *r["gyro"], *r["gravity"], *r["command"], r["height"], *r["applied"]])) for r in records)+"\n"
    sensors = output / "synthetic_sensor_records.txt"
    sensors.write_text(input_text)
    result = run("rmcs_core/wheel_leg_observation_parity", ["--prone-motion-v4"], input_text)
    require(result.returncode == 0, result.stderr)
    actual, worst, worst_row, history_worst = [], 0., None, 0.
    require(len(result.stdout.splitlines()) == len(records), "Replay row count differs")
    for i, (line, exp) in enumerate(zip(result.stdout.splitlines(), expected)):
        values = np.fromstring(line, sep=" ")
        require(bool(values[0]) == (exp is not None), f"Invalid-frame rejection differs at {i}: {records[i]['label']}")
        if exp is None:
            continue
        got = values[1:36].astype(np.float32)
        error = float(np.abs(got-exp).max())
        if error > worst:
            worst, worst_row = error, dict(row=i, label=records[i]["label"], slot=int(np.abs(got-exp).argmax()))
        require(np.array_equal(got[[28, 29, 30, 31, 32, 33, 34]], exp[[28, 29, 30, 31, 32, 33, 34]]),
                f"Context/capture decision differs at {i}: {records[i]['label']}")
        history = np.zeros(6, dtype=np.float32)
        delta = torch.tensor(records[i]["q"][:4], dtype=torch.float32)-nominal
        history[:4] = (paired_offsets(delta[None])/.25)[0].numpy()
        history_worst = max(history_worst, float(np.abs(values[36:42]-history).max()))
        actual.append(got)
    require(worst <= 1e-5, f"Observation error {worst}: {worst_row}")
    require(history_worst <= 1e-5, f"Measured entry action history differs: {history_worst}")
    frames = np.stack(actual)
    expected_frames = np.stack([frame for frame in expected if frame is not None])
    require((frames[:, 31] == 1).sum() > 1000 and np.abs(frames[:, 14:16]).max() > 19.9,
            "Replay failed to exercise active references and saturation")
    fixture, model, tensors = fixture_model(output, contract)
    actions = run("rmcs_rl/prone_policy_parity", [mapped(fixture)],
        "\n".join(" ".join(map(str, row)) for row in frames)+"\n")
    require(actions.returncode == 0, actions.stderr)
    cpp_actions = np.loadtxt(actions.stdout.splitlines(), dtype=np.float32)
    session = ort.InferenceSession(str(fixture), providers=["CPUExecutionProvider"])
    python_actions = np.concatenate([session.run(None, {"obs": frame[None]})[0] for frame in expected_frames])
    same_input_actions = np.concatenate([session.run(None, {"obs": frame[None]})[0] for frame in frames])
    torch_frames = torch.from_numpy(expected_frames)
    with torch.no_grad():
        normalized = (torch_frames-torch.from_numpy(tensors["mean"]))/torch.from_numpy(tensors["denominator"])
        hidden = torch.nn.functional.elu(normalized @ torch.from_numpy(tensors["w1"])+torch.from_numpy(tensors["b1"]))
        torch_actions = (hidden @ torch.from_numpy(tensors["w2"])+torch.from_numpy(tensors["b2"])).numpy()
    action_error = float(np.abs(cpp_actions-python_actions).max())
    engine_error = float(np.abs(cpp_actions-same_input_actions).max())
    torch_error = float(np.abs(cpp_actions-torch_actions).max())
    require(max(action_error, torch_error) <= 1e-5, "Normalized inference differs")
    # The full CAD model changed both gas-spring geometry and Jacobians.
    # Compare each side against the exact training implementation in double,
    # including both unmeasured force tails and out-of-domain zero effort.
    gas = GasSpringCompensation(contract["gas_spring"], root=source_root, device="cpu", dtype=torch.float64)
    deltas = np.linspace(-.16, 1.41, 1201)
    angles = np.linspace(-6., 6., len(deltas))
    gas_q = np.c_[angles, angles+deltas, -angles, -angles-deltas]
    gas_input = "\n".join(" ".join(map(str, row)) for row in gas_q)+"\n"
    gas_rows = output / "synthetic_gas_records.txt"
    gas_rows.write_text(gas_input)
    gas_result = run("rmcs_core/wheel_leg_gas_spring_parity", ["--full-vehicle-v202"], gas_input)
    require(gas_result.returncode == 0, gas_result.stderr)
    cpp_gas = np.loadtxt(gas_result.stdout.splitlines())
    python_gas, gas_diagnostics = gas.evaluate(torch.from_numpy(gas_q))
    gas_error = float(np.abs(cpp_gas[:, :4]-python_gas.numpy()).max())
    beta = gas_diagnostics["beta_deg"].numpy()
    require(np.array_equal(np.isnan(beta), np.isnan(cpp_gas[:, 4:])), "Gas geometry validity differs")
    beta_error = float(np.nanmax(np.abs(beta-cpp_gas[:, 4:])))
    require(gas_error < 1e-9 and beta_error < 1e-9, "Full-vehicle spring compensation differs")
    negative = {}
    for key, value in (("policy_hz", "200"), ("rmcs_observation_version", "wheel-leg-prone-motion-v3"),
                       ("rmcs_observation_contract_sha256", "0"*64), ("rmcs_obs_normalization", ""),
                       ("leg_target_model_spec_sha256", "0"*64), ("leg_winding_version", "independent_nearest_v1"),
                       ("rmcs_obs_mean", ",".join(["0"]*35))):
        bad = deepcopy(model)
        metadata = {p.key: p.value for p in bad.metadata_props}
        metadata[key] = value
        helper.set_model_props(bad, metadata)
        path = output / ("reject_"+key+".onnx")
        onnx.save(bad, path)
        checked = run("rmcs_rl/prone_policy_parity", [mapped(path)])
        negative[key] = checked.returncode == 1 and bool(checked.stderr.strip())
        require(negative[key], "Invalid contract accepted: " + key)
    # Reuse the production controller/bridge/executor test with this 100Hz
    # config and the synthetic normalization graph. Memory-only sensors.
    profile = (ROOT / "rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml").read_text()
    profile, replacements = re.subn(r"^    rl_model_path: .+$",
        lambda _: "    rl_model_path: "+json.dumps(mapped(fixture)), profile, flags=re.MULTILINE)
    require(replacements == 1, "Need exactly one deployment model path")
    profile_path = output / "synthetic_integration_profile.yaml"
    # Preserve the production YAML's explicit scalar types; ROS and PyYAML
    # resolve a few plain scalars (notably `nan`) differently.
    profile_path.write_text(profile)
    integration = run("rmcs_core/wheel_leg_unified_policy_integration",
        ["--gtest_filter=UnifiedPolicyIntegration.NeutralPrepareDirectInvertedAndSideEntryResetAndStaleFeedback"],
        environment={"RMCS_UNIFIED_TEST_PROFILE": mapped(profile_path)})
    (output / "integration.log").write_text(integration.stdout+integration.stderr)
    require(integration.returncode == 0, "100Hz controller/bridge integration failed; see integration.log")
    report = dict(schema="rmcs-prone-motion-v4-offline-parity-v1", status="passed", **config,
        training_commit=contract["training_commit"],
        policy_hz=100, host_pd_hz=1000, records=len(records), accepted_frames=len(actual),
        rejected_frames=len(records)-len(actual), observation_max_abs_error=worst, worst_observation=worst_row,
        context_and_validity_exact=True, normalized_action_max_abs_error=action_error,
        onnx_runtime_same_input_max_abs_error=engine_error,
        torch_normalized_action_max_abs_error=torch_error, negative_contracts=negative,
        measured_entry_history_max_abs_error=history_worst,
        gas_records=len(gas_q), gas_torque_max_abs_error_nm=gas_error,
        gas_beta_max_abs_error_deg=beta_error, gas_records_sha256=sha(gas_rows),
        production_controller_bridge_executor_100hz="passed_with_synthetic_normalized_graph",
        integration_log_sha256=sha(output / "integration.log"),
        sensor_records_sha256=sha(sensors), inference_fixture_sha256=sha(fixture),
        scenarios=sorted(set(row["label"] for row in records)),
        trained_model_tested=False, physical_sensor_recording_tested=False, behavior_qualified=False,
        hardware_tested=False, source="synthetic_sensor_records_with_selected_training_python_observer")
    (output / "parity.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
