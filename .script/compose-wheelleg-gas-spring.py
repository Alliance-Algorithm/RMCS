#!/usr/bin/env python3
"""Merge the authenticated v202 encoder gas-load overlay without launching RMCS."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    fragment = ROOT/"rmcs_ws/src/rmcs_bringup/config/wheelleg-gas-spring-v202.yaml"
    base = yaml.safe_load(args.base.read_text())
    components = base["rmcs_executor"]["ros__parameters"]["components"]
    if sum("WheelLegDriveBridge -> wheelleg_drive_bridge" in c for c in components) != 1:
        raise ValueError("Expected exactly one traditional WheelLegDriveBridge")
    if any("WheelLegSimulation" in c for c in components):
        raise ValueError("The legacy MuJoCo model uses different geometry; this is a v202 hardware overlay")
    if not any("hardware::WheelLeg ->" in c for c in components):
        raise ValueError("Expected the WheelLeg hardware graph with API encoder feedback")
    merged = copy.deepcopy(base)
    bridge = merged["wheelleg_drive_bridge"]["ros__parameters"]
    if bridge.get("gas_spring_enabled"):
        raise ValueError("Gas-spring compensation is already enabled")
    bridge.update(yaml.safe_load(fragment.read_text())["wheelleg_drive_bridge"]["ros__parameters"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as file:
        yaml.safe_dump(merged, file, allow_unicode=True, sort_keys=False)
    print(json.dumps(dict(output=str(args.output.resolve()), launched=False,
        base_sha256=hashlib.sha256(args.base.read_bytes()).hexdigest(),
        overlay_sha256=hashlib.sha256(fragment.read_bytes()).hexdigest(),
        gas_spring_gain=bridge["gas_spring_gain"], calibration_reference_inner_deg=110.)))


if __name__ == "__main__":
    main()
