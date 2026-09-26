#!/usr/bin/env python3
"""Replay disabled right-motor feedback through the real DM/component bridge."""
import argparse
import json
from pathlib import Path

import numpy as np

from wheel_leg_dm_feedback import DmFeedbackBridge

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--container", default="rmcs_devcontainer-rmcs-develop-1")
args = parser.parse_args()
args.output.parent.mkdir(parents=True, exist_ok=True)

# Exact angles reported by the user; desired URDF zero matches left-down/right-mid.
q = np.tile(np.deg2rad([9.28, -65.81, -35.84, 36.06]), (3, 1))
zero = np.zeros_like(q)
bridge = DmFeedbackBridge(3, [-1.6, -2.93, 1.6, 2.93], args.container,
                          args.output.with_suffix(".bridge.log"))
bridge.wrap_signed[:] = False
bridge.feedback_delay[:] = 0
bridge.command_delay[:] = 0
first_active = [None] * 3
nonzero_before_ready = 0
active_after_running_drop = 0
try:
    for tick in range(1000):
        # Lose all FC deliveries to RH, RK, or both until 550 ms.
        if tick < 550:
            bridge.status[0, 2] = 0
            bridge.status[1, 3] = 0
            bridge.status[2, 2:] = 0
        result = bridge.step(1. + tick * .001, q, zero, zero, zero, True, 0)
        command, active, healthy = result[2], result[4], result[5]
        assert healthy.all(), "The supplied angles must be accepted by the production controller"
        for env in range(3):
            if active[env] and first_active[env] is None:
                first_active[env] = tick * .001
        if tick < 550:
            nonzero_before_ready += int(np.any(command != 0.) or np.any(active))
    assert all(t is not None and .550 < t < 1. for t in first_active)
    # A fresh status=0 after active must latch a stop, even if status later recovers.
    bridge.status[:, 2] = 0
    for tick in range(1000, 1300):
        result = bridge.step(1. + tick * .001, q, zero, zero, zero, True, 0)
        active_after_running_drop += int(np.any(result[4]) or np.any(result[2] != 0.))
        bridge.status[:, 2] = 1
    report = {
        "method": "Injected CAN feedback through production DmMotor, controller, enable sequence, VEL bytes; no plant motion or real CAN transport",
        "reported_motor_angles_deg": [9.28, -65.81, -35.84, 36.06],
        "cases": ["right hip missing FC", "right knee missing FC", "both right motors missing FC"],
        "dropped_until_s": .550, "first_active_s": first_active,
        "nonzero_or_active_before_ready": nonzero_before_ready,
        "nonzero_or_active_after_running_disable": active_after_running_drop,
        "first_controller_fault": bridge.first_fault,
        "passed": nonzero_before_ready == 0 and active_after_running_drop == 0,
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)
    assert report["passed"]
finally:
    bridge.close()
