#!/usr/bin/env python3
"""Derive the user-specified 110-degree encoder zero and 40--110 stops, offline.

Uses the frozen training CAD solver without changing its asset or policy domain.
Requires the same numpy/scipy/trimesh environment as that solver.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
CONFIG = ROOT / "rmcs_ws/src/rmcs_bringup/config"


def generate(training):
    deployment = json.loads((CONFIG / "contracts/wheel_leg_prone_motion_v4.json").read_text())
    revision = deployment["training_commit"]

    def read(name):
        return subprocess.check_output(["git", "show", revision + ":" + name], cwd=training)

    names = ("tools/full_robot_v202.py", "model/wheel_leg_v2.0.2/training/model_spec.json",
             "model/wheel_leg_v2.0.2/training/gas_spring_model.json")
    contents = {name: read(name) for name in names}
    digests = {name: hashlib.sha256(value).hexdigest() for name, value in contents.items()}
    if digests[names[1]] != deployment["model_spec_sha256"]:
        raise ValueError("Calibration CAD model differs from the deployed policy")
    if digests[names[2]] != deployment["gas_spring"]["model_sha256"]:
        raise ValueError("Calibration gas-spring source differs")
    spec, gas = [json.loads(contents[name]) for name in names[1:]]
    with tempfile.TemporaryDirectory(prefix="rmcs-calibration110-") as temporary:
        script = Path(temporary) / "tools/full_robot_v202.py"
        script.parent.mkdir()
        script.write_bytes(contents[names[0]])
        loader = importlib.util.spec_from_file_location("calibration_cad", script)
        cad = importlib.util.module_from_spec(loader)
        loader.loader.exec_module(cad)
        active = [j["name"] for j in spec["joints"] if j["type"] != "fixed"]
        drive_indices = [active.index(name) for name in cad.ACTIVE]
        passive = [i for i in range(len(active)) if i not in drive_indices]
        q, max_gap, max_mapping_error = spec["nominal_joint_pos"], 0., 0.
        extension = {side: [] for side in ("left", "right")}
        # The inherited table ends at 108.5. Solve a matching overlap and eight
        # new quarter-degree points, retaining half a degree beyond the stop.
        for beta in np.arange(108.5, 110.5001, .25):
            prescribed = {"L_joint1": .42, "R_joint1": -.42, "L_joint3": 0., "R_joint3": 0.,
                "L_joint2": np.deg2rad(beta) - spec["passive_knee_zero_inner_rad"]["L_joint2"],
                "R_jonit2": -(np.deg2rad(beta) - spec["passive_knee_zero_inner_rad"]["R_jonit2"])}
            q = cad.solve_pose(spec, prescribed, q)
            columns = []
            for name in active:
                plus, minus = dict(q), dict(q)
                plus[name] += 1e-6
                minus[name] -= 1e-6
                columns.append((cad.closure_error(spec, plus) - cad.closure_error(spec, minus)) / 2e-6)
            jac = np.column_stack(columns)
            mapping = np.zeros((len(active), 6))
            mapping[drive_indices] = np.eye(6)
            mapping[passive] = np.linalg.lstsq(jac[:, passive], -jac[:, drive_indices], rcond=1e-8)[0]
            max_mapping_error = max(max_mapping_error, float(np.abs(jac @ mapping).max()))
            max_gap = max(max_gap, float(np.linalg.norm(cad.closure_error(spec, q).reshape(-1, 3), axis=1).max()))
            for side, prefix, indices in (("left", "L", slice(0, 2)), ("right", "R", slice(3, 5))):
                hip, aux, slide = prefix + "_joint1", prefix * 2 + "_joint1", prefix + "_spring_slide"
                delta = (q[aux] - q[hip]) * (1. if prefix == "L" else -1.)
                row = [delta, float(beta), spec["spring_binding"][slide]["compression_at_q_zero_m"] - q[slide],
                       *mapping[active.index(slide), indices].tolist()]
                if beta == 108.5:
                    source = gas["sides"][side]
                    expected = [source["delta_rad"][-1], source["beta_deg"][-1], source["compression_m"][-1],
                                *source["length_jacobian_m_per_rad"][-1]]
                    if np.max(np.abs(np.asarray(row) - expected)) > 1e-8:
                        raise ValueError("Extended geometry changed the inherited assembly branch")
                else:
                    extension[side].append(row)
        if max_gap > 2e-7 or max_mapping_error > 1e-7:
            raise ValueError("Calibration closure or velocity mapping failed")
    minimum = [gas["sides"][side]["delta_rad"][2] for side in ("left", "right")]
    if any(gas["sides"][side]["beta_deg"][2] != 40. for side in ("left", "right")):
        raise ValueError("Expected an exact 40-degree geometry knot")
    maximum = [next(row[0] for row in extension[side] if row[1] == 110.) for side in ("left", "right")]
    offsets = [1.6, 1.6 + maximum[0], -1.6, -1.6 - maximum[1]]
    receipt = dict(schema="rmcs-wheel-leg-calibration110-v1", authorization="User correction 2026-10-30",
        training_commit=revision, source_sha256=digests, model_spec_sha256=deployment["model_spec_sha256"],
        encoder_zero_inner_knee_deg=110., mechanical_inner_knee_domain_deg=[40., 110.],
        coordinate="q_model = -q_api + offset; left delta=q_LK-q_LH, right delta=q_RH-q_RK",
        model_offset_rad=offsets, retained_hip_zero_model_rad=[1.6, -1.6],
        hinge_min=minimum, hinge_max=maximum, hinge_margin_rad=.03,
        calibration_scope="User-specified knee zero; existing absolute hip reference retained",
        gas_geometry_extension=extension, gas_geometry_beta_domain_deg=[39.5, 110.5],
        gas_force_curve="Inherited measured curve and declared endpoint trend; no refit",
        max_closure_gap_m=max_gap, max_velocity_mapping_residual=max_mapping_error,
        policy_target_domain_changed=False, standing_observation_domain_changed=False,
        hardware_zero_command_sent=False, hardware_tested=False)
    content = json.dumps(receipt, indent=2) + "\n"
    header = ["#pragma once", "", '#include "full_vehicle_v202.hpp"', "",
        "// Generated by tools/sync_wheel_leg_calibration.py; deployment calibration only.",
        "// Calibration JSON SHA256: " + hashlib.sha256(content.encode()).hexdigest(),
        "namespace rmcs_core::controller::chassis::full_vehicle_calibration110 {",
        f'inline constexpr std::string_view kModelSpecSha256 = "{deployment["model_spec_sha256"]}";',
        f'inline constexpr std::string_view kCalibrationSha256 = "{hashlib.sha256(content.encode()).hexdigest()}";',
        "static_assert(kModelSpecSha256 == full_vehicle_v202::kModelSpecSha256);",
        "inline constexpr double kEncoderZeroInnerDeg = 110.0;"]
    for name, values in (("LegModelOffsets", offsets), ("HingeMin", minimum), ("HingeMax", maximum)):
        header.append(f"inline constexpr std::array<double, {len(values)}> k{name}{{"
                      + ", ".join(f"{v:.17g}" for v in values) + "};")
    header += ["using GeometryKnot = gas_spring_model::GeometryKnot;"]
    for side in ("left", "right"):
        title = side.capitalize()
        header.append(f"inline constexpr auto k{title} = [] {{")
        header.append(f"    std::array<GeometryKnot, 285> result{{}};")
        header.append(f"    for (std::size_t i = 0; i < full_vehicle_v202::k{title}.size(); ++i)")
        header.append(f"        result[i] = full_vehicle_v202::k{title}[i];")
        for i, row in enumerate(extension[side], 277):
            header.append(f"    result[{i}] = {{" + ", ".join(f"{v:.17g}" for v in row) + "};")
        header += ["    return result;", "}();"]
    header += ["} // namespace rmcs_core::controller::chassis::full_vehicle_calibration110", ""]
    return {CONFIG / "contracts/wheel_leg_calibration110.json": content,
            ROOT / "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/full_vehicle_calibration110.hpp": "\n".join(header)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for path, content in generate(args.training_root.resolve()).items():
        if args.check:
            if path.read_text() != content:
                raise ValueError("Calibration differs: " + str(path))
        else:
            path.write_text(content)
        print(path.relative_to(ROOT))


if __name__ == "__main__":
    main()
