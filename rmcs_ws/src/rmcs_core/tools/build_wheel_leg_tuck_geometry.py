#!/usr/bin/env python3
"""Export wheel and mass-moment FK from the sealed paired-107 closed chain."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    core = Path(__file__).resolve().parents[1]
    lookup_path = core / "config/wheel_leg/v7_height_lookup_107.json"
    lookup = json.loads(lookup_path.read_text())
    bundle = args.simulation_root / "model/纯底盘_v7_flat_base107/urdf"
    source = args.simulation_root / "scripts/build_v6_height_lookup.py"
    spec = importlib.util.spec_from_file_location("tuck_lookup_builder", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    builder = module.LookupBuilder(bundle, digest(bundle / "manifest.json"))
    if (digest(bundle / "model_spec.json") != lookup["model_spec_sha256"]
            or digest(builder.solver_path) != lookup["solver_sha256"]
            or digest(bundle / "tools/v6_mechanism.py") != lookup["fk_sha256"]):
        raise ValueError("Tuck and deployment model identities differ")
    base = builder.bodies["base_link"]
    result = {
        "schema": "wheel_leg_tuck_geometry_v1",
        "asset_manifest_sha256": lookup["asset_manifest_sha256"],
        "source_asset_manifest_sha256": builder.manifest_sha256,
        "model_spec_sha256": lookup["model_spec_sha256"],
        "height_lookup_sha256": digest(lookup_path),
        "generator_sha256": digest(Path(__file__)),
        "solver_sha256": digest(builder.solver_path),
        "base_mass_kg": base["mass"],
        "base_moment_kg_m": (base["mass"] * np.asarray(base["com"])).tolist(),
        "scope": "sealed CAD mass and geometry; real COM and encoder calibration remain external",
        "sides": [],
    }
    max_error = 0.
    for side, key in (("L", "left"), ("R", "right")):
        table = lookup["sides"][key]
        bodies = [body for body in builder.spec["bodies"] if body["name"].startswith(side)]
        row = {key: table[key] for key in (
            "hip_origin_b_m", "hip_axis_b", "wheel_axis_b", "hip_reference_rad", "wheel_radius_m")}
        row["mass_kg"] = sum(body["mass"] for body in bodies)
        row["knots"] = []
        # Continue independently away from nominal, preserving the assembly branch.
        states = [None] * len(table["delta_rad"])
        nominal_delta = builder.nominal[side * 2 + "_joint1"] - builder.nominal[side + "_joint1"]
        delta = np.asarray(table["delta_rad"])
        for indices in (np.flatnonzero(delta < nominal_delta)[::-1],
                        np.flatnonzero(delta >= nominal_delta)):
            q = dict(builder.nominal)
            for index in indices:
                fixed = {**builder.active, side * 2 + "_joint1": table["hip_reference_rad"] + delta[index]}
                q = builder.solve(fixed, q)
                states[index] = q
        for index, q in enumerate(states):
            frames = builder.fk(builder.spec, q)
            moment = sum(body["mass"] * (frames[body["name"]][:3, :3] @ body["com"]
                                         + frames[body["name"]][:3, 3]) for body in bodies)
            wheel = builder.wheel(side, q)[0]
            max_error = max(max_error, float(np.linalg.norm(wheel - table["wheel_center_b_m"][index])))
            knee = np.degrees((1. if side == "L" else -1.) * q[builder.passive_name(side)]
                              + np.pi - 2.3573)
            row["knots"].append({"delta_rad": float(delta[index]), "knee_deg": float(knee),
                                 "wheel_b_m": wheel.tolist(), "moment_kg_m": moment.tolist()})
        result["sides"].append(row)
    if max_error > 1e-6:
        raise ValueError(f"Independent closed-chain wheel FK mismatch: {max_error}")
    result["max_wheel_lookup_error_m"] = max_error
    result["max_closure_gap_m"] = builder.max_gap_m
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output), "sha256": digest(args.output),
                      "max_wheel_error_m": max_error, "max_closure_gap_m": builder.max_gap_m}))


if __name__ == "__main__":
    main()
