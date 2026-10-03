#!/usr/bin/env python3
"""Generate CPU reference snapshots from the immutable V6 training source.

Imports only Torch and the frozen sensor FSM; it starts no Isaac application,
training worker, GUI or hardware process. The source repository stays read-only.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace

import torch


SOURCE = "2778206b5905c2760ccc381f2592b08e1cad611a"
ORDER = [0, 1, 3, 4, 2, 5]  # CONTROL -> RMCS policy


def frozen(root, name):
    return subprocess.check_output(
        ["git", "show", f"{SOURCE}:src/wheeled_tasks/chassis/{name}.py"], cwd=root
    ).decode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("training_root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    bundle = args.training_root / "models/v6_flat_candidate_14020_20261003"
    contract = json.loads((bundle / "policy.onnx.contract.json").read_text())
    settings = contract["recovery_training"]
    raw = (args.training_root / settings["profiles_file"]).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == settings["profiles_sha256"]
    profile = json.loads(raw)
    fsm_source = frozen(args.training_root, "recovery")
    training_source = frozen(args.training_root, "recovery_training")
    namespace = {"__name__": "frozen_v6_recovery_reference"}
    exec(compile(fsm_source, f"{SOURCE}/recovery.py", "exec"), namespace)
    # The selected class methods do not construct an environment or observers.
    stripped = "\n".join(line for line in training_source.splitlines()
                         if not line.startswith("from ."))
    exec(compile(stripped, f"{SOURCE}/recovery_training.py", "exec"), namespace)
    FSM, Training = namespace["RecoveryFSM"], namespace["RecoveryTraining"]
    reference = profile["prepare_reference"]["pd_control6_rad"]
    nominal = profile["targets"]["nominal"]["control6_rad"]
    fold = profile["targets"]["fold"]["control6_rad"]
    thrust = profile["targets"]["thrust"]["control6_rad"]
    cfg = {key: settings[key] for key in FSM.DEFAULTS if key in settings}
    bases = {"q": reference, "dq": [0., 0., 0., 0., .04, -.03],
             "gravity": [0., 0., -1.], "gyro": [.01, .02, -.01],
             "height": .305, "support": True, "body_clear": True,
             "probe": [.18, -.18], "actor_torque": [2., -3., 4., -5., .7, -.9],
             "actor_action": [.2, -.3, .4, -.5, .07, -.09]}

    def pitch(deg):
        angle = math.radians(deg)
        return [math.sin(angle), 0., -math.cos(angle)]

    def segment(tick, **values):
        return {"tick": tick, **values}

    # Input schedules are fixed sensor sequences, not physics success claims.
    scenarios = [
        ("upright", 290, {}, []),
        ("plant", 600, {"q": fold, "gravity": pitch(30), "height": .19,
                         "support": False, "body_clear": False},
         [segment(140, gravity=pitch(0), height=.305, support=True,
                  body_clear=True, q=fold), segment(260, q=reference)]),
        ("front_thrust_capture", 700, {"q": fold, "gravity": pitch(90), "height": .15,
                                       "support": False, "body_clear": False},
         [segment(255, gravity=pitch(100), height=.18, support=True),
          segment(270, gravity=pitch(40), height=.27, q=thrust),
          segment(300, gravity=pitch(10), height=.305, q=reference, body_clear=True),
          segment(400, gravity=pitch(0))]),
        ("back_thrust_capture", 700, {"q": fold, "gravity": pitch(-90), "height": .15,
                                      "support": False, "body_clear": False},
         [segment(255, gravity=pitch(-100), height=.18, support=True),
          segment(270, gravity=pitch(-40), height=.27, q=thrust),
          segment(300, gravity=pitch(-10), height=.305, q=reference, body_clear=True),
          segment(400, gravity=pitch(0))]),
        ("orbit_early_capture", 490, {"q": fold, "gravity": pitch(90), "height": .15,
                                     "support": False, "body_clear": False},
         [segment(90, gravity=pitch(25), height=.25, support=True),
          segment(100, gravity=pitch(10), height=.305, q=reference, body_clear=True),
          segment(200, gravity=pitch(0))]),
        ("side_convert", 700, {"q": fold, "gravity": [0., 1., 0.], "height": .15,
                               "support": False, "body_clear": False},
         [segment(80, gravity=pitch(30)),
          segment(260, gravity=pitch(0), height=.305, support=True, body_clear=True,
                  q=fold), segment(390, q=reference)]),
        ("side_exhausted", 255, {"q": fold, "gravity": [0., -1., 0.], "height": .15,
                                 "support": False, "body_clear": False}, []),
        ("orbit_exhausted", 420, {"q": fold, "gravity": pitch(90), "height": .15,
                                  "support": False, "body_clear": False}, []),
        ("plant_timeout", 650, {"q": fold, "gravity": pitch(30), "height": .19,
                                "support": False, "body_clear": False}, []),
        ("prepare_timeout", 1610, {"body_clear": False}, []),
        ("one_reroute_total_budget", 1610, {"body_clear": False},
         [segment(4, gravity=pitch(100), height=.18, support=False),
          segment(40, q=fold),
          segment(190, gravity=pitch(10), height=.305, support=True, q=reference),
          segment(230, gravity=pitch(100), height=.18, support=False)]),
        ("takeover_lost_upright", 80, {},
         [segment(35, gravity=pitch(50), support=False)]),
        ("paired_winding", 340,
         {"q": [value + (math.tau if index < 2 else -math.tau if 3 <= index < 5 else 0.)
                for index, value in enumerate(reference)], "gyro": [0., 0., 0.]}, []),
    ]

    def c_vector(p_values):
        values = [p_values[ORDER.index(index)] for index in range(6)]
        return torch.tensor([values], dtype=torch.float32)

    # Pose arrays above are CONTROL order; all stored test feedback is P order.
    base_p = dict(bases)
    base_p["q"] = [reference[i] for i in ORDER]
    results = []
    for name, count, overrides, changes in scenarios:
        current = dict(base_p)
        overrides = dict(overrides)
        if "q" in overrides:
            overrides["q"] = [overrides["q"][i] for i in ORDER]
        converted_changes = []
        for change in changes:
            change = dict(change)
            if "q" in change:
                change["q"] = [change["q"][i] for i in ORDER]
            converted_changes.append(change)
        current.update(overrides)
        initial = c_vector(current["q"])
        fsm = FSM(1, "cpu", cfg, reference, fold, thrust, reference, profile["axis_signs"])
        fsm.reset([0], initial, True)
        training = Training.__new__(Training)
        bounds = torch.tensor([[40., 40., 4.5, 40., 40., 4.5]])
        training.env = SimpleNamespace(
            dt=.005, actions=c_vector(current["actor_action"]),
            v5=SimpleNamespace(LEGS=(0, 1, 3, 4), WHEELS=(2, 5), settings=contract["v5_control"],
                wheel_prior={"kd": .6}, current_motor_bounds=bounds,
                requested_motor_effort=torch.zeros(1, 6), nominal=torch.tensor(nominal)))
        training.settings = settings
        training.fsm = fsm
        training.enabled = fsm.enabled
        training.release_finished = torch.ones(1, dtype=torch.bool)
        training.premature_release = torch.zeros(1, dtype=torch.bool)
        training.elapsed = torch.zeros(1)
        training.learning_mask = torch.ones(1, dtype=torch.bool)
        training.wheel_balance = settings["wheel_balance"]
        training.wheel_axis_signs = torch.tensor(profile["wheel_axis_signs"])
        training.wheel_balance_active = torch.zeros(1, dtype=torch.bool)
        # q schedules are already the external provider's canonical output,
        # passed to the FSM. Native PD reads that provider's bitwise projection,
        # never the FSM's separate trigonometric angle accumulation.
        training.project = lambda q: q
        rows, old_phase = [], -1
        for tick in range(count):
            for change in converted_changes:
                if change["tick"] == tick:
                    current.update({key: value for key, value in change.items() if key != "tick"})
            q, dq = c_vector(current["q"]), c_vector(current["dq"])
            g, gyro = (torch.tensor([current[key]], dtype=torch.float32) for key in ("gravity", "gyro"))
            height = torch.tensor([current["height"]])
            support, clear = (torch.tensor([current[key]]) for key in ("support", "body_clear"))
            previous = fsm.phase.clone()
            fsm.update(q, dq, g, gyro, height, support, clear)
            training._update_wheel_targets(g, gyro, dq, support, previous)
            training.wheel_probe_torque = torch.tensor([current["probe"]])
            torque = training.torque(q, dq, c_vector(current["actor_torque"]))
            training.env.actions = c_vector(current["actor_action"])
            training.effective_action_history()
            phase = int(fsm.phase[0])
            if tick % 10 == 0 or phase != old_phase or tick == count - 1:
                rows.append({"tick": tick, "phase": phase, "route": int(fsm.route[0]),
                    "failure": int(fsm.failure_code[0]), "age": int(fsm.age_ticks[0]),
                    "phase_ticks": int(fsm.phase_ticks[0]), "stable_ticks": int(fsm.stable_ticks[0]),
                    "reroutes": int(fsm.reroute_count[0]), "motion_released": bool(fsm.motion_released[0]),
                    "wheel_balance": bool(training.wheel_balance_active[0]),
                    "blend": float(fsm.blend[0]), "targets": fsm.script_targets[0, ORDER].tolist(),
                    "continuous_q": fsm.continuous_q[0, ORDER].tolist(),
                    "torque": torque[0, ORDER].tolist(), "history": training.env.actions[0, ORDER].tolist()})
            old_phase = phase
        results.append({"name": name, "ticks": count, "initial": overrides,
                        "changes": converted_changes, "expected": rows})
    result = {"schema": "rmcs-v6-recovery-frozen-reference-v1", "source_commit": SOURCE,
              "source_sha256": {"recovery.py": hashlib.sha256(fsm_source.encode()).hexdigest(),
                                 "recovery_training.py": hashlib.sha256(training_source.encode()).hexdigest()},
              "profile_sha256": settings["profiles_sha256"], "order": "LH LA RH RA LW RW",
              "scope": "synthetic_sensor_numerical_parity_not_dynamic_success_rate",
              "config": {"nominal": [reference[i] for i in ORDER[:4]],
                         "fold": [fold[i] for i in ORDER[:4]],
                         "thrust": [thrust[i] for i in ORDER[:4]],
                         "support": [reference[i] for i in ORDER[:4]],
                         "rl_nominal": [nominal[i] for i in ORDER],
                         "root_axis_signs": profile["axis_signs"],
                         "wheel_axis_signs": profile["wheel_axis_signs"]},
              "feedback_default": base_p, "cases": results}
    args.output.write_text(json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n")
    print(json.dumps({"cases": len(results), "ticks": sum(item["ticks"] for item in results),
                      "snapshots": sum(len(item["expected"]) for item in results),
                      "bytes": args.output.stat().st_size}))


if __name__ == "__main__":
    main()
