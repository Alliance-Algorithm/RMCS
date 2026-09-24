"""Verify and run the historical V5 policy on an offline fixture; no hardware IO."""
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort

from v5_policy_io import decode_actions, reference_joint_torques, verify_bundle


def main():
    root = Path(__file__).resolve().parent
    contract, _, prior, _ = verify_bundle(
        root / "policy.onnx", root / "policy.onnx.contract.json",
        root / "manifest.json", root / "own_v40_v2.json")
    fixture = json.loads((root / "io_fixture.json").read_text())
    session = ort.InferenceSession(str(root / "policy.onnx"), providers=["CPUExecutionProvider"])
    observation = np.asarray(fixture["observation"], dtype=np.float32)
    raw = session.run(["actions"], {"obs": observation})[0][0]
    legs, wheels, clipped = decode_actions(
        raw, fixture["q_control"], fixture["nominal_control"], contract["v5_control"])
    torque = reference_joint_torques(
        fixture["q_control"], fixture["dq_control"], legs, wheels,
        contract["v5_control"], prior["actuators"]["wheel"])
    outputs = {"raw_actions_policy": raw, "clipped_actions_policy": clipped,
               "leg_targets_policy_rad": legs, "wheel_targets_policy_rad_s": wheels,
               "joint_torques_control_nm": torque}
    for name, actual in outputs.items():
        np.testing.assert_allclose(actual, fixture[name], atol=1e-5, rtol=1e-5)
    print(json.dumps({"fixture_verified": True, "input_shape": list(observation.shape),
                      "input_dtype": str(observation.dtype),
                      **{name: value.tolist() for name, value in outputs.items()}}, indent=2))


if __name__ == "__main__":
    main()
