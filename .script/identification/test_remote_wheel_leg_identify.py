"""Exercise the launcher's actual ROS readiness contract without hardware."""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest
import yaml


@pytest.mark.parametrize("pd", [False, True])
@pytest.mark.parametrize("mismatch", [None, "probe_pattern", "control_frequency_hz", "side", "max_tick_gap_s", "probe_measured_delta_policy", "pd_kp", "reference_frequency_hz", "ki_velocity", "max_torque", "multiband_load_offset"])
def test_recorder_ready_requires_actual_controller_contract(tmp_path, monkeypatch, mismatch, pd):
    if not pd and mismatch in ("pd_kp", "reference_frequency_hz", "ki_velocity", "max_torque", "multiband_load_offset"):
        pytest.skip("PD-only parameter")
    expected = {"probe_pattern": "multiband_chirp", "control_frequency_hz": 1000.0,
                "side": "left", "max_tick_gap_s": .02, "probe_measured_delta_policy": "diagnostic"}
    if pd:
        expected.update(control_law="rl_pd", control_frequency_hz=1000.0,
                        reference_frequency_hz=50.0, pd_kp=[60.0]*4, pd_kd=[2.0]*4,
                        ki_velocity=[0.0]*4, integral_limit=[0.0]*4,
                        gravity_feedforward_model=[0.0]*4, max_torque=[40.0]*4,
                        multiband_load_offset=.2, multiband_validation_relative_scale=3.0)
    actual = dict(expected)
    if mismatch:
        actual[mismatch] = {"probe_pattern": "rotation_chirp", "control_frequency_hz": 200.0,
                            "side": "right", "max_tick_gap_s": .01, "probe_measured_delta_policy": "stop",
                            "pd_kp": [80.0]*4, "reference_frequency_hz": 1000.0,
                            "ki_velocity": [50.0]*4, "max_torque": [20.0]*4,
                            "multiband_load_offset": 0.0}[mismatch]
    (tmp_path / "profile.yaml").write_text(yaml.safe_dump({
        "wheel_leg_pair_identification_controller": {"ros__parameters": expected}}))
    values = [SimpleNamespace(string_value=actual["probe_pattern"]),
              SimpleNamespace(double_value=actual["control_frequency_hz"]),
              SimpleNamespace(string_value=actual["side"]),
              SimpleNamespace(double_value=actual["max_tick_gap_s"]),
              SimpleNamespace(string_value=actual["probe_measured_delta_policy"])]
    if pd:
        values.extend([SimpleNamespace(string_value=actual["control_law"]),
                       SimpleNamespace(double_value=actual["reference_frequency_hz"])])
        values.extend(SimpleNamespace(double_array_value=actual[key]) for key in
                      ("pd_kp", "pd_kd", "ki_velocity", "integral_limit", "gravity_feedforward_model", "max_torque"))
        values.extend(SimpleNamespace(double_value=actual[key]) for key in
                      ("multiband_load_offset", "multiband_validation_relative_scale"))

    class Client:
        def __init__(self, path):
            self.path = path

        def service_is_ready(self):
            return True

        def call_async(self, request):
            result = SimpleNamespace(values=values) if self.path.endswith("get_parameters") \
                else SimpleNamespace(paused=False)
            return SimpleNamespace(done=lambda: True, result=lambda: result)

    node = SimpleNamespace(
        create_client=lambda service, path: Client(path),
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(node_name="wheel_leg_identification_recorder")],
        get_subscriptions_info_by_topic=lambda topic: [SimpleNamespace(node_name="rosbag2_recorder")],
        destroy_node=lambda: None)
    ros = ModuleType("rclpy")
    ros.init = lambda: None
    ros.shutdown = lambda: None
    ros.create_node = lambda name: node
    ros.spin_once = lambda *args, **kwargs: None
    ros.spin_until_future_complete = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    for package, name in (("rosbag2_interfaces", "IsPaused"), ("rcl_interfaces", "GetParameters")):
        parent = ModuleType(package)
        service = ModuleType(package + ".srv")
        setattr(service, name, SimpleNamespace(Request=SimpleNamespace))
        parent.srv = service
        monkeypatch.setitem(sys.modules, package, parent)
        monkeypatch.setitem(sys.modules, package + ".srv", service)
    monkeypatch.setattr(sys, "argv", ["readiness", str(tmp_path)])
    launcher = Path(__file__).with_name("remote-wheel-leg-identify").read_text()
    source = launcher.split("<<'PY_READY'\n", 1)[1].split("\nPY_READY", 1)[0]
    if mismatch:
        with pytest.raises(SystemExit, match="disagree with archived profile"):
            exec(compile(source, "launcher-readiness", "exec"), {})
        assert not (tmp_path / "recording-ready.json").exists()
    else:
        exec(compile(source, "launcher-readiness", "exec"), {})
        assert (tmp_path / "recording-ready.json").exists()


@pytest.mark.parametrize("mismatch", [None, "wheel_velocity_kp", "reference_frequency_hz",
    "wheel_current_limit_a", "wheel_speeds", "wheel_step_repetitions", "undeclared_zero_ki",
    "recorder_frequency", "recorder_gate", "manifest_revision"])
def test_wheel_readiness_checks_full_controller_and_recorder_contract(tmp_path, monkeypatch, mismatch):
    import copy
    import json
    profile_path = Path(__file__).resolve().parents[2] / "rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-wheel-identification.yaml"
    profile = yaml.safe_load(profile_path.read_text())
    controller = copy.deepcopy(profile["wheel_leg_wheel_identification_controller"]["ros__parameters"])
    recorder = copy.deepcopy(profile["wheel_leg_identification_recorder"]["ros__parameters"])
    manifest = {"revision": controller["trajectory_revision"], "reference_frequency_hz": 50.,
                "segments": [{"id": 0}]}
    if mismatch == "wheel_speeds":
        controller[mismatch] = [.6, 1.2, 2.]
    elif mismatch in ("wheel_velocity_kp", "reference_frequency_hz", "wheel_current_limit_a", "wheel_step_repetitions"):
        controller[mismatch] *= 2
    elif mismatch == "recorder_frequency":
        recorder["sample_frequency_hz"] = 100.
    elif mismatch == "recorder_gate":
        recorder["record_on_double_middle"] = False
    elif mismatch == "manifest_revision":
        manifest["revision"] = "old_probe"
    controller["wheel_trajectory_manifest_json"] = json.dumps(manifest)
    (tmp_path / "profile.yaml").write_text(yaml.safe_dump(profile, sort_keys=False))

    def parameter_value(key, value):
        field, kind = ("bool_value", 1) if isinstance(value, bool) else ("string_value", 4) if isinstance(value, str) \
            else ("double_array_value", 8) if isinstance(value, list) else ("integer_value", 2) if isinstance(value, int) \
            else ("double_value", 3)
        if mismatch == "undeclared_zero_ki" and key == "wheel_velocity_ki":
            kind = 0
        return SimpleNamespace(type=kind, **{field: value})

    class Client:
        def __init__(self, path): self.path = path
        def service_is_ready(self): return True
        def call_async(self, request):
            if self.path.endswith("get_parameters"):
                data = recorder if "recorder/" in self.path else controller
                result = SimpleNamespace(values=[parameter_value(k, data[k]) for k in request.names])
            else:
                result = SimpleNamespace(paused=False)
            return SimpleNamespace(done=lambda: True, result=lambda: result)

    node = SimpleNamespace(create_client=lambda service, path: Client(path),
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(node_name="wheel_leg_identification_recorder")],
        get_subscriptions_info_by_topic=lambda topic: [SimpleNamespace(node_name="rosbag2_recorder")],
        destroy_node=lambda: None)
    ros = ModuleType("rclpy")
    ros.init = lambda: None
    ros.shutdown = lambda: None
    ros.create_node = lambda name: node
    ros.spin_once = lambda *a, **kw: None
    ros.spin_until_future_complete = lambda *a, **kw: None
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    for package, name in (("rosbag2_interfaces", "IsPaused"), ("rcl_interfaces", "GetParameters")):
        parent = ModuleType(package)
        service = ModuleType(package + ".srv")
        setattr(service, name, SimpleNamespace(Request=SimpleNamespace))
        parent.srv = service
        monkeypatch.setitem(sys.modules, package, parent)
        monkeypatch.setitem(sys.modules, package + ".srv", service)
    monkeypatch.setattr(sys, "argv", ["readiness", str(tmp_path)])
    launcher = Path(__file__).with_name("remote-wheel-leg-identify").read_text()
    source = launcher.split("<<'PY_READY'\n", 1)[1].split("\nPY_READY", 1)[0]
    if mismatch:
        with pytest.raises(SystemExit, match="disagree"):
            exec(compile(source, "wheel-readiness", "exec"), {})
        assert not (tmp_path / "recording-ready.json").exists()
    else:
        exec(compile(source, "wheel-readiness", "exec"), {})
        saved = json.loads((tmp_path / "recording-ready.json").read_text())
        assert saved["runtime_parameters"] == profile["wheel_leg_wheel_identification_controller"]["ros__parameters"]
        assert saved["runtime_recorder_parameters"]["sample_frequency_hz"] == 1000.
        assert json.loads((tmp_path / "trajectory-plan.json").read_text())["revision"] == "wheel_multiband_v2"
