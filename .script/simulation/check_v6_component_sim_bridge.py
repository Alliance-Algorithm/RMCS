#!/usr/bin/env python3
"""Check the production-component socket with sensor frames, without a plant.

Online mode records actual public controller outputs. Offline mode compares
captured 35-element observations with the frozen ONNX actor's clipped actions.
No Python motor controller or motor-effort calculation is implemented here.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import socket
import sys


AXES = ["left_hip_joint", "left_knee_joint", "right_hip_joint",
        "right_knee_joint", "left_wheel", "right_wheel"]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def model_sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


class Client:
    def __init__(self, path, records):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(10.)
        self.socket.connect(str(path))
        self.stream = self.socket.makefile("rwb")
        self.records = records

    def request(self, request, case, expect_error=False):
        line = request if isinstance(request, str) else json.dumps(request, allow_nan=False)
        self.stream.write((line + "\n").encode())
        self.stream.flush()
        response = json.loads(self.stream.readline())
        self.records.append({"case": case, "request": copy.deepcopy(request), "response": response})
        require(response.get("ok", False) != expect_error,
                f"{case}: unexpected bridge result: {response}")
        return response

    def close(self):
        self.stream.close()
        self.socket.close()


def check_peer(peer, args, sha256):
    require(peer["protocol"] == "rmcs_v6_component_sim_v1", "Unexpected socket protocol")
    require(peer["simulation_only"] is True, "The peer is not a simulation bridge")
    require(peer["policy_profile"] == args.expected_policy_profile, "Wrong policy profile")
    require(peer["axis_order"] == AXES, "Wrong policy axis order")
    require(peer["model_sha256"] == sha256, "Peer model SHA256 differs from the supplied ONNX")
    for field, expected in (("profile_path", args.expected_peer_profile),
                            ("model_path", args.expected_peer_model)):
        require(Path(peer[field]).is_absolute(), f"Peer {field} is not absolute")
        if expected:
            require(peer[field] == expected, f"Unexpected peer {field}: {peer[field]}")
    for field, expected in (("executor_hz", 1000), ("physics_hz", 200),
                            ("inference_hz", 50), ("pd_hz", 200)):
        require(peer[field] == expected, f"Unexpected declared {field}")
    rl = peer["parameters"]["rl_controller"]
    if args.expected_blend_seconds is not None:
        actual_blend = rl.get("v6_takeover_blend_seconds")
        require(actual_blend is not None and math.isfinite(actual_blend)
                and math.isclose(actual_blend, args.expected_blend_seconds, abs_tol=1e-12),
                "Peer takeover blend parameter differs from the requested value")
        if args.expected_blend_seconds > 0:
            require(peer.get("v6_takeover_blend_supported") is True,
                    "The peer does not expose production V6 takeover blending")
    for name in ("calibration_ready", "soft_limits_ready", "imu_alignment_ready"):
        require(rl[name] is True, f"Simulation readiness flag is false: {name}")
    require(rl["recovery_enabled"] is False, "Recovery must be disabled")
    require(rl["policy_profile"] == args.expected_policy_profile, "ROS profile disagrees with peer")
    require(rl["rl_model_path"] == peer["model_path"], "ROS model path disagrees with peer")
    expected_jacobian = [-1. if row == column else 0. for row in range(4) for column in range(4)]
    require(rl["leg_motor_to_model"] == expected_jacobian, "This sensor check requires q_api=-q_model+offset")
    require(rl["wheel_model_scale"] == [1., 1.], "This sensor check requires unit wheel scales")
    return rl, peer["parameters"]["chassis"]


def check_active(response, case):
    require(response["state"] == 3 and response["enable_request"],
            f"{case}: RL did not remain active: {response}")
    require(response["sensors_valid"] and response["sensor_issue"] == 0
            and response["sensor_mask"] == 0, f"{case}: invalid sensor diagnostics")
    require(response["effort_within_limits"], f"{case}: motor effort exceeds reported limits")
    require(len(response["observation"]) == 35 and len(response["actions"]) == 6,
            f"{case}: wrong actor tensor dimensions")
    if "v6_takeover_blend_fraction" in response:
        fraction = response["v6_takeover_blend_fraction"]
        require(math.isfinite(fraction) and 0. <= fraction <= 1.,
                f"{case}: invalid takeover blend fraction")


def check_off(response, case):
    require(not response["enable_request"] and response["state"] in (0, 1),
            f"{case}: still enabled: {response}")
    require(all(abs(value) < 1e-12 for value in response["torque_api"]),
            f"{case}: nonzero disabled effort: {response['torque_api']}")


def check_close(actual, expected, case):
    require(len(actual) == len(expected) and all(
        math.isclose(a, e, rel_tol=1e-10, abs_tol=1e-10) for a, e in zip(actual, expected)),
        f"{case}: {actual} != {expected}")


def run_online(args, records, sha256):
    client = Client(args.socket, records)
    passed = []
    try:
        peer = client.request({"op": "hello"}, "peer")
        rl, chassis = check_peer(peer, args, sha256)
        client.request({"op": "reset"}, "initial_reset")
        nominal = rl["nominal_model_pos"]
        offsets = rl["leg_model_offsets"]
        nominal_api = [offsets[i] - nominal[i] for i in range(4)] + nominal[4:]
        frame = dict(op="step", q_api=nominal_api, dq_api=[0.] * 6,
                     orientation_wxyz=[1., 0., 0., 0.], gyro=[0., 0., 0.],
                     acceleration=[0., 0., 9.81], right_stick=[0., 0.], left_stick=[0., 0.],
                     left_switch=2, right_switch=2, rotary_knob=0., keyboard=0,
                     remote_fresh=True, feedback_fresh=True, dm_control_ready=True)

        def step(case, **updates):
            frame.update(updates)
            return client.request(frame, case)

        def arm(case):
            frame.update(q_api=nominal_api.copy(), dq_api=[0.] * 6,
                         orientation_wxyz=[1., 0., 0., 0.], gyro=[0., 0., 0.],
                         right_stick=[0., 0.], left_stick=[0., 0.], rotary_knob=0., keyboard=0,
                         remote_fresh=True, feedback_fresh=True, dm_control_ready=True)
            check_off(step(case + "_down", left_switch=2, right_switch=2), case)
            response = step(case + "_middle", left_switch=3, right_switch=3)
            for _ in range(args.max_prepare_steps):
                if response["state"] == 3:
                    check_active(response, case)
                    return response
                response = step(case + "_prepare")
            raise AssertionError(f"{case}: PREPARE did not reach RL: {response}")

        check_off(step("unarmed_spin", left_switch=3, right_switch=2), "unarmed_spin")
        arm("arming")
        passed.append("DOWN/MIDDLE arming and unarmed SPIN rejection")
        vx = chassis["vx_max"]
        yaw = chassis["yaw_rate_max"] * (-1. if chassis["angular_z_invert"] else 1.)
        default_height = chassis["default_command_height"]
        response = step("sticks", right_stick=[1., 0.], left_stick=[1., 1.])
        check_active(response, "sticks")
        check_close(response["command_velocity"], [vx, 0., yaw], "sticks")
        check_close([response["command_height"]], [default_height], "left_x_height_isolation")
        response = step("negative_sticks", right_stick=[-1., 0.], left_stick=[-1., -1.])
        check_close(response["command_velocity"], [-vx, 0., -yaw], "negative_sticks")
        response = step("unsupported_lateral", right_stick=[0., 1.], left_stick=[1., 0.])
        check_close(response["command_velocity"], [0., 0., 0.], "unsupported_lateral")
        passed.append("translation/yaw mapping and left-x height isolation")

        for knob, height in ((-1., chassis["command_height_min"]),
                             (0., default_height),
                             (chassis["deadzone"] * .5, default_height),
                             (1., chassis["command_height_max"])):
            if chassis["height_invert"] and abs(knob) == 1.:
                height = chassis["command_height_max"] if knob < 0 else chassis["command_height_min"]
            response = step("rotary", rotary_knob=knob)
            check_close([response["command_height"]], [height], "rotary")
            check_active(response, "rotary")
        step("rotary_center", rotary_knob=0.)
        response = step("height_R", keyboard=1 << 8)
        check_close([response["command_height"]],
                    [min(default_height + chassis["height_step"], chassis["command_height_max"])], "height_R")
        step("height_key_release", keyboard=0)
        response = step("height_F", keyboard=1 << 9)
        check_close([response["command_height"]], [default_height], "height_F")
        step("height_key_release", keyboard=0)
        passed.append("rotary endpoints/center/deadzone and R/F offset")

        first = step("spin_negative", right_stick=[1., 0.], left_stick=[1., 1.], right_switch=2)
        check_active(first, "spin_negative")
        check_close(first["command_velocity"], [0., 0., -chassis["spin_yaw_rate"]], "spin_negative")
        for _ in range(4):
            response = step("spin_hold")
            check_close(response["command_velocity"], first["command_velocity"], "spin_hold")
        step("spin_switch_release", right_switch=3)
        response = step("spin_exit", right_switch=2)
        check_close(response["command_velocity"], [vx, 0., yaw], "spin_exit")
        step("spin_switch_release", right_switch=3)
        response = step("spin_positive", right_switch=2)
        check_active(response, "spin_positive")
        check_close(response["command_velocity"], [0., 0., chassis["spin_yaw_rate"]], "spin_positive")
        passed.append("pure SPIN both signs, edge toggle and held switch")
        check_off(step("disable", left_switch=2, right_switch=2), "disable")
        passed.append("double-DOWN zero effort")

        arm("dynamic")
        for index in range(args.dynamic_steps):
            time_s = index * .005
            q = nominal_api.copy()
            dq = [0.] * 6
            for axis in range(4):
                phase = 4. * time_s + .3 * axis
                q[axis] -= .002 * math.sin(phase)
                dq[axis] = -.008 * math.cos(phase)
            roll = .015 * math.sin(3. * time_s)
            response = step("dynamic", q_api=q, dq_api=dq,
                            orientation_wxyz=[math.cos(roll / 2.), math.sin(roll / 2.), 0., 0.],
                            gyro=[.045 * math.cos(3. * time_s), 0., .01],
                            right_stick=[.4 * math.cos(time_s), 0.],
                            left_stick=[.8, .5 * math.sin(time_s)],
                            rotary_knob=.4 * math.sin(time_s))
            check_active(response, "dynamic")
        passed.append("dynamic sensor-frame capture with valid bounded effort")

        arm("remote_loss")
        check_off(step("remote_loss", remote_fresh=False), "remote_loss")
        check_off(step("remote_return_unarmed", remote_fresh=True), "remote_return_unarmed")
        passed.append("remote freshness loss disarms the session")
        arm("feedback_loss")
        check_off(step("feedback_loss", feedback_fresh=False), "feedback_loss")
        check_off(step("feedback_return_latched", feedback_fresh=True), "feedback_return_latched")
        passed.append("strict feedback freshness rejection stays latched")
        check_off(step("malformed_disabled", left_switch=2, right_switch=2), "malformed_disabled")
        bad_frame = copy.deepcopy(frame)
        bad_frame["q_api"] = [0.]
        client.request(bad_frame, "malformed_shape", expect_error=True)
        check_off(client.request({"op": "hello"}, "after_malformed_shape"), "after_malformed_shape")
        client.request('{"op":"step",bad}', "malformed_json", expect_error=True)
        check_off(client.request({"op": "hello"}, "after_malformed_json"), "after_malformed_json")
        passed.append("malformed shape/JSON rejected while disabled output stays zero")
        reset = client.request({"op": "reset"}, "final_reset")
        check_off(reset, "final_reset")
        require(reset["update_count"] == 0 and reset["sensor_sequence"] == 0, "RESET did not recreate session")
        passed.append("session reset")
        client.request({"op": "shutdown" if args.shutdown_bridge else "close"}, "close")
        return {"peer": peer, "passed_cases": passed}
    finally:
        client.close()


def capture_rows(path):
    if path.is_dir():
        records = []
        for file in sorted(list(path.glob("*.json")) + list(path.glob("*.jsonl"))):
            for record in capture_rows(file):
                if isinstance(record, dict):
                    records.append({"source_capture": str(file), **record})
        return records
    text = path.read_text()
    if text.lstrip().startswith("["):
        return json.loads(text)
    try:
        value = json.loads(text)
        return [value] if isinstance(value, dict) else value
    except json.JSONDecodeError:
        return [json.loads(line) for line in text.splitlines() if line.strip()]


def compare_actor(records, model, atol, rtol):
    # Loading the independent oracle happens only after the socket is closed.
    import numpy as np
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(str(model), sess_options=options, providers=["CPUExecutionProvider"])
    require(session.get_inputs()[0].name == "obs" and session.get_outputs()[0].name == "actions",
            "Unexpected ONNX actor IO names")
    require(session.get_inputs()[0].shape[-1] == 35 and session.get_outputs()[0].shape[-1] == 6,
            "Unexpected ONNX actor dimensions")
    # Frozen output ABI only; this is not a Python PD or effort generator.
    action_limits = np.asarray([3., 3., 3., 3., 9., 9.], dtype=np.float32)
    compared = 0
    maximum_error = 0.
    mismatches = []
    for index, record in enumerate(records):
        response = record.get("response", record)
        if response.get("state") != 3 or "observation" not in response:
            continue
        actions = response.get("actions", response.get("action"))
        if actions is None:
            continue
        obs = np.asarray(response["observation"], dtype=np.float32)
        actual = np.asarray(actions, dtype=np.float32)
        require(obs.shape == (35,) and actual.shape == (6,) and np.isfinite(obs).all()
                and np.isfinite(actual).all(), f"Malformed captured actor sample: {index}")
        raw = session.run(["actions"], {"obs": obs.reshape(1, 35)})[0].reshape(6)
        expected = np.clip(raw, -action_limits, action_limits)
        error = float(np.max(np.abs(actual - expected)))
        maximum_error = max(maximum_error, error)
        compared += 1
        if not np.allclose(actual, expected, atol=atol, rtol=rtol):
            mismatches.append({"record": index, "case": record.get("case"),
                               "source_capture": record.get("source_capture"), "maximum_error": error,
                               "actual": actual.tolist(), "expected": expected.tolist()})
    require(compared > 0, "Capture has no RL-state actor observations to compare")
    return {"passed": not mismatches, "compared_samples": compared,
            "maximum_absolute_action_error": maximum_error, "atol": atol, "rtol": rtol,
            "onnxruntime_version": ort.__version__, "mismatches": mismatches[:10]}


def write_json(path, value):
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--expected-policy-profile", default="v6_flat_14020")
    parser.add_argument("--expected-peer-profile", help="Exact YAML path as seen inside the bridge container")
    parser.add_argument("--expected-peer-model", help="Exact ONNX path as seen inside the bridge container")
    parser.add_argument("--expected-blend-seconds", type=float,
                        help="Optionally verify the peer's V6 takeover blend parameter")
    parser.add_argument("--offline-capture", type=Path,
                        help="Compare JSON/JSONL file or directory without connecting to a socket")
    parser.add_argument("--capture", type=Path, help="Write online requests/responses to JSONL after the run")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--max-prepare-steps", type=int, default=200)
    parser.add_argument("--dynamic-steps", type=int, default=48)
    parser.add_argument("--shutdown-bridge", action="store_true")
    parser.add_argument("--atol", type=float, default=1e-5)
    parser.add_argument("--rtol", type=float, default=1e-5)
    args = parser.parse_args()
    if not args.offline_capture and not args.socket:
        parser.error("--socket is required for online checks")
    if args.expected_blend_seconds is not None and not (
            math.isfinite(args.expected_blend_seconds) and 0 <= args.expected_blend_seconds <= .3):
        parser.error("--expected-blend-seconds must be finite and in [0,0.3]")
    records = []
    report = {"schema": "rmcs-v6-component-sensor-check-v1", "passed": False,
              "scope": "sensor frames and public actor outputs; no physical dynamics/cadence proof",
              "model_path": str(args.model.resolve())}
    try:
        report["model_sha256"] = model_sha256(args.model)
        if args.offline_capture:
            records = capture_rows(args.offline_capture)
            report["capture_path"] = str(args.offline_capture.resolve())
        else:
            report.update(run_online(args, records, report["model_sha256"]))
        report["actor_comparison"] = compare_actor(records, args.model, args.atol, args.rtol)
        require(report["actor_comparison"]["passed"], "Captured C++ actions differ from independent ONNX output")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
    finally:
        if args.capture and not args.offline_capture:
            args.capture.parent.mkdir(parents=True, exist_ok=True)
            args.capture.write_text("".join(json.dumps(row, allow_nan=False) + "\n" for row in records))
        report["record_count"] = len(records)
        write_json(args.report, report)
    summary = {key: report[key] for key in ("passed", "record_count", "model_sha256", "error") if key in report}
    if "actor_comparison" in report:
        summary["actor_comparison"] = report["actor_comparison"]
    if "passed_cases" in report:
        summary["passed_case_count"] = len(report["passed_cases"])
    print(json.dumps(summary, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
