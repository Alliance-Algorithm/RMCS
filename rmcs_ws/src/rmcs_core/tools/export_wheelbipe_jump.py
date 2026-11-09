#!/usr/bin/env python3
"""Export a saved v4 actor with the exact RMCS control contract; never controls a learner."""

import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
CONTRACT = ROOT / "rmcs_ws/src/rmcs_bringup/config/contracts/wheelbipe_jump_v4.json"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_receipt(receipt, contract):
    require(
        receipt["arguments"]["training_profile"] == contract["training_profile"]
        and receipt["deployment"]["training_profile"] == contract["training_profile"],
        "Wrong training profile",
    )
    require(
        receipt["deployment"]["source_commit"] == contract["training_source_commit"],
        "Training source commit differs",
    )
    native = receipt["native"]
    control = contract["control"]
    require(
        native["leg_kp_Nm_per_rad"] == control["leg_kp_nm_per_rad"]
        and native["leg_kd_Nm_per_rad_s"] == control["leg_kd_nm_s_per_rad"],
        "PD differs",
    )
    require(
        native["virtual_pair_target_overtravel_rad"]
        == control["virtual_pair_overtravel_rad"]
        and native["virtual_pair_target_overtravel_scope"]
        == control["virtual_pair_overtravel_scope"],
        "Target authority differs",
    )
    require(
        native["tuck_spring_compensation"] == control["tuck_spring_compensation"],
        "Tuck compensation differs",
    )
    require(
        native["spring_model_sha256"] == contract["gas_model_sha256"],
        "Spring model differs",
    )
    require(
        native["physical_measured_gas_spring"]
        and not native["gas_cancellation_feedforward"],
        "Physical spring or global cancellation differs",
    )
    require(
        native["policy_dt_s"] == 1.0 / contract["policy_hz"]
        and native["physics_dt_s"] == 1.0 / contract["feedback_pd_hz"],
        "Policy or feedback cadence differs",
    )
    require(
        receipt["arguments"]["iterations"] == receipt["arguments"]["course_updates"],
        "Course and budget differ",
    )
    require(
        receipt["initial_optimizer_state_count"] == 0
        and not receipt["cold_resume"]
        and not receipt["cross_course_continuation"],
        "This profile requires a fresh v4 experiment",
    )
    model_path = "model/wheel_leg_v2.0.2/friction_v1/isaac/model_spec.json"
    require(
        receipt["deployment"]["files_sha256"][model_path]
        == contract["model_spec_sha256"],
        "Physical model differs",
    )


def export(checkpoint, receipt_path, output, *, allow_pilot=False):
    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch
    from torch import nn

    contract = json.loads(CONTRACT.read_text())
    receipt = json.loads(Path(receipt_path).read_text())
    validate_receipt(receipt, contract)
    checkpoint = Path(checkpoint).resolve()
    checksum = digest(checkpoint)
    data = torch.load(checkpoint, map_location="cpu", weights_only=False)
    if data.get("format") == "wheelbipe-rsl-durable-v1":
        metadata = json.loads(Path(str(checkpoint) + ".json").read_text())
        pins = metadata["checkpoint_pins"]
        require(
            len(pins) == 1
            and pins[0]["sha256"] == checksum
            and pins[0]["bytes"] == checkpoint.stat().st_size,
            "Checkpoint sidecar does not pin the payload",
        )
        require(
            data["session_sha256"] == metadata["session_sha256"]
            and Path(pins[0]["path"]).parent
            == Path(receipt["arguments"]["output"]) / "checkpoints",
            "Checkpoint and receipt belong to different sessions",
        )
        updates = data["completed_updates"]
        require(
            updates == metadata["completed_updates"]
            and data["total_updates"] == receipt["arguments"]["iterations"],
            "Checkpoint budget differs",
        )
        require(
            data["curriculum_state"]["profile_sha256"]
            == receipt["effective_course_sha256"],
            "Checkpoint course differs",
        )
    else:
        require(
            checksum == receipt.get("checkpoint_sha256"),
            "RSL checkpoint is not pinned by the completed receipt",
        )
        updates = int(data["iter"]) + 1
    budget = receipt["arguments"]["iterations"]
    require(0 < updates <= budget, "Checkpoint update count is outside its budget")
    complete = receipt["status"] == "training_complete" and updates == budget
    require(
        allow_pilot or complete,
        "Final export requires completion of the declared course budget; pilot needs --allow-pilot",
    )
    state = data["model_state_dict"]
    require(
        not data.get("normalizers"), "Unexpected external observation normalization"
    )
    require(
        not any("obs_normalizer" in k for k in state),
        "Unexpected external observation normalization",
    )
    weights = sorted(
        (int(k.split(".")[1]), k, v)
        for k, v in state.items()
        if k.startswith("actor.") and k.endswith(".weight")
    )
    require(
        [(v.shape[1], v.shape[0]) for _, _, v in weights]
        == [(35, 256), (256, 128), (128, 64), (64, 6)],
        "Unexpected actor topology",
    )
    layers = []
    for i, (_, key, weight) in enumerate(weights):
        layer = nn.Linear(weight.shape[1], weight.shape[0])
        layer.weight.data.copy_(weight)
        layer.bias.data.copy_(state[key.removesuffix("weight") + "bias"])
        layers.append(layer)
        if i < 3:
            layers.append(nn.ELU())
    actor = nn.Sequential(*layers).eval()
    require(
        all(torch.isfinite(v).all() for v in actor.state_dict().values()),
        "Actor contains nonfinite parameters",
    )
    output = Path(output).resolve()
    require(
        not output.exists() and not output.with_suffix(".deployment.json").exists(),
        "Refuse to overwrite an existing artifact",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".pending.onnx")
    require(not temporary.exists(), "A pending export already exists")
    torch.onnx.export(
        actor,
        torch.zeros(1, 35),
        str(temporary),
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=17,
        dynamo=False,
    )
    model = onnx.load(str(temporary))
    props = {
        "rmcs_model_spec_sha256": contract["model_spec_sha256"],
        "leg_target_model_spec_sha256": contract["model_spec_sha256"],
        "leg_winding_version": "paired_full_vehicle110_jump_v4",
        "rmcs_observation_version": contract["observation_version"],
        "rmcs_observation_contract_sha256": contract["observation_contract_sha256"],
        "rmcs_obs_normalization": "none",
        "policy_hz": "50",
        "feedback_pd_hz": "200",
        "wheelbipe_deployment_contract_sha256": contract["deployment_contract_sha256"],
        "training_source_commit": contract["training_source_commit"],
        "gas_spring_compensation_enabled": "false",
        "tuck_spring_compensation": "model",
        "leg_kp": "80",
        "leg_kd": "1.5",
        "leg_target_overtravel_rad": "0.2",
        "leg_target_overtravel_scope": "jump-request",
        "gas_model_sha256": contract["gas_model_sha256"],
        "rmcs_checkpoint_sha256": checksum,
        "completed_updates": str(updates),
        "training_budget": str(budget),
        "effective_course_sha256": receipt["effective_course_sha256"],
        "behavior_qualified": "false",
        "hardware_tested": "false",
        "pilot": str(not complete).lower(),
    }
    onnx.helper.set_model_props(model, props)
    onnx.checker.check_model(model)
    onnx.save(model, str(temporary))
    sample = (
        np.random.default_rng(119).uniform(-5.0, 5.0, (1024, 35)).astype(np.float32)
    )
    with torch.inference_mode():
        expected = actor(torch.from_numpy(sample)).numpy()
    actual = ort.InferenceSession(
        str(temporary), providers=["CPUExecutionProvider"]
    ).run(None, {"obs": sample})[0]
    error = float(np.max(np.abs(actual - expected)))
    require(
        np.allclose(actual, expected, atol=2e-5, rtol=2e-5),
        f"ONNX parity failed: {error}",
    )
    temporary.replace(output)
    result = {
        "schema": "wheelbipe-jump-v4-onnx-export-v1",
        "checkpoint_sha256": checksum,
        "onnx_sha256": digest(output),
        "completed_updates": updates,
        "budget_updates": budget,
        "training_complete": complete,
        "metadata": props,
        "parity_samples": 1024,
        "max_absolute_error": error,
        "behavior_qualified": False,
        "hardware_tested": False,
        "learner_modified": False,
    }
    output.with_suffix(".deployment.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-pilot", action="store_true")
    args = parser.parse_args()
    print(
        json.dumps(
            export(
                args.checkpoint, args.receipt, args.output, allow_pilot=args.allow_pilot
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
