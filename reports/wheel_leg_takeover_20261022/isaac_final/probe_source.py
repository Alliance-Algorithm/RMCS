#!/usr/bin/env python3
"""Bounded Isaac Sim leg-only closed-chain check of the production C++ tuck.

Fixtures hold body attitude to isolate leg geometry. This does not qualify a
free-body self-righting policy or a real vehicle. No actor or training runs.
"""
import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-root", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=6.)
    parser.add_argument("--cases", nargs="+")
    parser.add_argument("--trajectory", action="store_true",
                        help="Run production two-stage FSM until handover; fixture PD holds afterward")
    args = parser.parse_args()
    if not 3. <= args.seconds <= 10.:
        parser.error("Probe duration must be 3..10 seconds")
    core = Path(__file__).resolve().parents[1]
    geometry = core / "config/wheel_leg/tuck_geometry_107_v1.json"
    bundle = args.simulation_root / "model/纯底盘_v7_flat_base107/urdf"
    sys.path.insert(0, str(args.simulation_root / "scripts"))
    from recovery_static_targets import RecoveryStaticTargetBuilder
    builder = RecoveryStaticTargetBuilder(bundle, digest(bundle / "manifest.json"))
    spec, manifest = builder.spec, builder.manifest
    fit = builder.fit
    dt = .001
    args.output.mkdir(parents=True, exist_ok=True)
    library = ctypes.CDLL(str(args.library.resolve()))
    pointer = ctypes.POINTER(ctypes.c_double)
    library.wheel_leg_tuck_create.argtypes = [ctypes.c_char_p, ctypes.c_double]
    library.wheel_leg_tuck_create.restype = ctypes.c_void_p
    library.wheel_leg_tuck_step.argtypes = [ctypes.c_void_p, pointer, pointer,
                                           ctypes.c_double, ctypes.c_int, pointer]
    library.wheel_leg_tuck_error.restype = ctypes.c_char_p
    if args.trajectory:
        library.wheel_leg_trajectory_create.argtypes = [ctypes.c_char_p, ctypes.c_double]
        library.wheel_leg_trajectory_create.restype = ctypes.c_void_p
        library.wheel_leg_trajectory_step.argtypes = [ctypes.c_void_p, pointer, pointer, pointer,
                                                     ctypes.c_double, pointer]
    baseline = builder.balanced_pose(50.)
    cases = [
        ("front", 0., (1.2, 1.2), False, 100.),
        ("back", 0., (-1.2, -1.2), False, 100.),
        ("asymmetric", 0., (1.2, -1.2), False, 100.),
        ("pitch_forward", 20., (-.8, -.8), False, 100.),
        ("pitch_backward", -20., (.8, .8), False, 100.),
        ("ground_supported", 0., (0., 0.), True, 100.),
        ("ground_no_integral", 0., (0., 0.), True, 0.),
    ]
    if args.cases:
        cases = [case for case in cases if case[0] in args.cases]
        if len(cases) != len(args.cases):
            parser.error("Unknown or duplicate case name")
    prepared = []
    for name, pitch, headings, grounded, ki in cases:
        q = builder.common_rotate(baseline["q"], headings[0], -headings[1])
        rotation = Rotation.from_euler("yx", [math.radians(pitch), baseline["root_roll_rad"]])
        prepared.append((name, rotation, q, grounded, ki))
    from isaaclab.app import AppLauncher
    launcher = AppLauncher({"headless": True, "device": "cpu"})
    try:
        import carb.settings
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.actuators import IdealPDActuatorCfg
        from isaaclab.assets import Articulation, ArticulationCfg
        from isaaclab_physx.physics import PhysxCfg, PhysxManager
        from pxr import Gf, UsdPhysics, PhysxSchema, UsdGeom
        import warp as wp
        torch.set_num_threads(4)
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            device="cpu", dt=dt, render_interval=100,
            physics=PhysxCfg(solver_type=1, enable_external_forces_every_iteration=True)))
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        floor = sim_utils.GroundPlaneCfg(size=(20., 20.),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=.8, dynamic_friction=.8, restitution=0.))
        floor.func("/World/Ground", floor, translation=(0., 0., 0.))
        floor_paths = [str(prim.GetPath()) for prim in sim.stage.Traverse()
                       if str(prim.GetPath()).startswith("/World/Ground/")
                       and prim.HasAPI(UsdPhysics.CollisionAPI)]
        rigs = []
        for index, (name, rotation, q, grounded, ki) in enumerate(prepared):
            path = "/World/" + name
            active = manifest["control_joint_names"]
            springs = manifest["spring_joint_names"]
            passive = [n for n in manifest["tree_joint_names"] if n not in active + springs]
            cfg = ArticulationCfg(prim_path=path,
                spawn=sim_utils.UsdFileCfg(usd_path=str(bundle / "robot.usda"), activate_contact_sensors=True,
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.),
                    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                        fix_root_link=False, enabled_self_collisions=False,
                        solver_position_iteration_count=32, solver_velocity_iteration_count=8)),
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=(index * 1.2, 0., .7), joint_pos=q, joint_vel={".*": 0.}),
                actuators={
                    "active": IdealPDActuatorCfg(joint_names_expr=active, stiffness=0., damping=0.,
                        effort_limit=40., effort_limit_sim=40., velocity_limit_sim=1000.),
                    "passive": IdealPDActuatorCfg(joint_names_expr=passive, stiffness=0., damping=.002,
                        effort_limit=100., effort_limit_sim=100., velocity_limit_sim=1000.),
                    "gas": IdealPDActuatorCfg(joint_names_expr=springs, stiffness=0., damping=.002,
                        effort_limit=1000., effort_limit_sim=1000., velocity_limit_sim=100.)},
                soft_joint_pos_limit_factor=1.)
            robot = Articulation(cfg)
            for prim in sim.stage.Traverse():
                if str(prim.GetPath()).startswith(path + "/") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
            guide_type = UsdPhysics.PrismaticJoint if grounded else UsdPhysics.FixedJoint
            guide = guide_type.Define(sim.stage, path + "_fixture")
            guide.CreateBody1Rel().SetTargets([path + "/base_link"])
            quat = rotation.as_quat()
            guide.CreateLocalPos0Attr(Gf.Vec3f(index * 1.2, 0., 0. if grounded else .7))
            guide.CreateLocalPos1Attr(Gf.Vec3f(0., 0., 0.))
            if grounded:
                guide.CreateAxisAttr("Z")
                guide.CreateLocalRot0Attr(Gf.Quatf(1.))
                guide.CreateLocalRot1Attr(Gf.Quatf(quat[3], Gf.Vec3f(*(-quat[:3]))))
                guide.CreateLowerLimitAttr(-10.)
                guide.CreateUpperLimitAttr(10.)
            else:
                guide.CreateLocalRot0Attr(Gf.Quatf(quat[3], Gf.Vec3f(*quat[:3])))
                guide.CreateLocalRot1Attr(Gf.Quatf(1.))
            guide.CreateExcludeFromArticulationAttr(True)
            context = (library.wheel_leg_trajectory_create(str(geometry.parent).encode(), ki)
                       if args.trajectory else library.wheel_leg_tuck_create(str(geometry).encode(), ki))
            if not context:
                raise ValueError(library.wheel_leg_tuck_error().decode())
            rigs.append(dict(name=name, rotation=rotation, initial=q, grounded=grounded,
                             ki=ki, robot=robot, context=context, index=index, samples=[], handover=None))
        sim.reset()
        bounds = UsdGeom.BBoxCache(0., ["default", "render", "proxy"]).ComputeWorldBound(
            sim.stage.GetPrimAtPath("/World/Ground")).ComputeAlignedRange()
        print("FLOOR_BOUNDS", list(bounds.GetMin()), list(bounds.GetMax()), flush=True)
        for rig in rigs:
            robot = rig["robot"]
            rig["ids"] = [robot.joint_names.index(n) for n in ("L_joint1", "LL_joint1", "R_joint1", "RR_joint1")]
            rig["wheel_ids"] = [robot.joint_names.index(n) for n in ("L_joint3", "R_joint3")]
            rig["spring_ids"] = [robot.joint_names.index(n) for n in springs]
            rig["knee_ids"] = [robot.joint_names.index(n) for n in ("L_joint2", "R_jonit2")]
            body_paths = ["/World/" + rig["name"] + "/" + n for n in robot.body_names]
            rig["contact"] = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                body_paths, filter_patterns=[floor_paths for _ in body_paths],
                max_contact_data_count=4096)
            rig["contact_paths"] = list(rig["contact"].sensor_paths)
            print("CONTACT_PATHS", rig["name"], rig["contact_paths"], flush=True)
            joint = torch.tensor([[rig["initial"][n] for n in robot.joint_names]], dtype=torch.float32)
            root = torch.tensor([[rig["index"] * 1.2, 0., baseline["height_m"] + .002 if rig["grounded"] else .7,
                                  *rig["rotation"].as_quat()]], dtype=torch.float32)
            robot.write_root_link_pose_to_sim_index(root_pose=root)
            robot.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros(1, 6))
            robot.write_joint_position_to_sim_index(position=joint)
            robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joint))
            robot.reset()
            robot.update(dt)
            rig["target"] = joint[0, rig["ids"]].numpy().astype(np.float64)
            rig["output"] = np.zeros(31 if args.trajectory else 15, dtype=np.float64)
            rig["gravity"] = np.ascontiguousarray(rig["rotation"].inv().apply([0., 0., -1.]))
            rig["effort"] = torch.zeros_like(joint)
            rig["max_clamp_mm"] = 0.
        started = time.monotonic()
        for step in range(round(args.seconds / dt)):
            for rig in rigs:
                robot = rig["robot"]
                q = robot.data.joint_pos.torch[0]
                dq = robot.data.joint_vel.torch[0]
                measured = np.ascontiguousarray(q[rig["ids"]].numpy(), dtype=np.float64)
                out = rig["output"]
                if args.trajectory:
                    active_ids = rig["ids"] + rig["wheel_ids"]
                    q6 = np.ascontiguousarray(q[active_ids].numpy(), dtype=np.float64)
                    dq6 = np.ascontiguousarray(dq[active_ids].numpy(), dtype=np.float64)
                    code = library.wheel_leg_trajectory_step(rig["context"], q6.ctypes.data_as(pointer),
                        dq6.ctypes.data_as(pointer), rig["gravity"].ctypes.data_as(pointer), dt,
                        out.ctypes.data_as(pointer))
                    if out[25] and rig["handover"] is None:
                        rig["handover"] = dict(time_s=step * dt, phase=int(out[21]),
                            knee_deg=[math.degrees(float(q[rig["knee_ids"][0]]) + builder.knee_zero_rad),
                                      math.degrees(-float(q[rig["knee_ids"][1]]) + builder.knee_zero_rad)],
                            measured_fk_knee_deg=out[8:10].tolist(),
                            wheel_com_error_mm=(out[10:12] * 1000.).tolist(),
                            tracking_error_rad=float(out[23]), remaining_rad=float(out[22]),
                            integral_nm=out[4:8].tolist())
                        print("TRAJECTORY_HANDOVER", rig["name"], rig["handover"], flush=True)
                    rig["target"] = out[:4].copy()
                    motor = out[15:19]
                else:
                    code = library.wheel_leg_tuck_step(rig["context"], measured.ctypes.data_as(pointer),
                        rig["gravity"].ctypes.data_as(pointer), dt, int(step > 20), out.ctypes.data_as(pointer))
                    delta = out[:4] - rig["target"]
                    fraction = min(1. * dt / max(float(np.abs(delta).max()), 1e-6), 1.)
                    rig["target"] += delta * fraction
                    motor = 160. * (rig["target"] - measured) - 2.5 * dq[rig["ids"]].numpy() + out[4:8]
                if code <= 0:
                    raise RuntimeError(f"Invalid C++ tuck/trajectory: {rig['name']}, {code}, {measured}, {out}")
                effort = rig["effort"]
                effort.zero_()
                effort[0, rig["ids"]] = torch.tensor(np.clip(motor, -40., 40.), dtype=torch.float32)
                # Same bounded catalogue prior as the existing runtime, with clamp exposure recorded.
                compression = np.array([spec["spring_binding"][n]["compression_at_q_zero_m"]
                                         for n in springs]) - q[rig["spring_ids"]].numpy()
                if not np.isfinite(compression).all() or np.any(compression < -.001) or np.any(compression > .08):
                    raise RuntimeError("Spring stroke left the physical travel")
                rig["max_clamp_mm"] = max(rig["max_clamp_mm"],
                                          float(np.maximum(0., compression - .072).max() * 1000))
                force = np.polynomial.polynomial.polyval(np.clip(compression, 0., .072) / .08,
                                                         fit["monomial_coefficients_n"])
                effort[0, rig["spring_ids"]] = torch.tensor(force, dtype=torch.float32)
                effort[0, rig["wheel_ids"]] = (torch.tensor(out[19:21], dtype=torch.float32)
                    if args.trajectory else -.6 * dq[rig["wheel_ids"]])
                robot.set_joint_effort_target_index(target=effort)
                robot.write_data_to_sim()
            sim.step(render=False)
            for rig in rigs:
                rig["robot"].update(dt)
            if step % 20 == 0 or step == round(args.seconds / dt) - 1:
                for rig in rigs:
                    robot = rig["robot"]
                    poses = robot.data.body_link_pose_w.torch[0].numpy().astype(float)
                    q = robot.data.joint_pos.torch[0].numpy().astype(float)
                    def point(body, local):
                        pose = poses[robot.body_names.index(body)]
                        return pose[:3] + Rotation.from_quat(pose[3:]).apply(local)
                    com = sum(body["mass"] * point(body["name"], body["com"])
                              for body in spec["bodies"]) / builder.mass
                    wheels = [point(side + "_link3", np.asarray(builder.bodies[side + "_link3"]["collisions"][0]["origin"])[:3, 3])
                              for side in ("L", "R")]
                    knees = [math.degrees(q[rig["knee_ids"][0]] + builder.knee_zero_rad),
                             math.degrees(-q[rig["knee_ids"][1]] + builder.knee_zero_rad)]
                    def constraint_gap(c):
                        return np.linalg.norm(point(c["body0"], c["local_pos0_m"])
                                              - point(c["body1"], c["local_pos1_m"]))
                    forces = wp.to_torch(rig["contact"].get_contact_force_matrix(dt=dt)).numpy().sum(axis=1)
                    wheel_bodies = [rig["contact_paths"].index("/World/" + rig["name"] + "/" + side + "_link3")
                                    for side in ("L", "R")]
                    others = [i for i in range(len(forces)) if i not in wheel_bodies]
                    rig["samples"].append(dict(time_s=(step + 1) * dt, knee_deg=knees,
                        wheel_com_error_m=[float(w[0] - com[0]) for w in wheels],
                        wheel_z_m=[float(w[2]) for w in wheels],
                        wheel_axis_w=[Rotation.from_quat(poses[robot.body_names.index(side + "_link3"), 3:]).apply([0., 0., 1.]).tolist()
                                      for side in ("L", "R")],
                        wheel_support_n=forces[wheel_bodies, 2].tolist(),
                        nonwheel_support_n=float(np.linalg.norm(forces[others], axis=1).sum()),
                        contacts_n={name: force.tolist() for name, force in zip(rig["contact_paths"], forces)
                                    if np.linalg.norm(force) > .1},
                        active_q_rad=q[rig["ids"]].tolist(),
                        max_closure_gap_m=max(constraint_gap(c) for c in spec["constraints"]),
                        native_phase=int(rig["output"][21]) if args.trajectory else None,
                        integral_nm=rig["output"][4:8].tolist(),
                        max_leg_torque_nm=float(rig["effort"][0, rig["ids"]].abs().max()),
                        base_z_m=float(poses[robot.body_names.index("base_link"), 2])))
                    if step == 0:
                        print("FIRST_SAMPLE", rig["name"], rig["samples"][-1], "bodies", robot.body_names, flush=True)
            if step % 1000 == 0:
                print("TUCK_STEP", step, "wall_s", round(time.monotonic() - started, 1), flush=True)
        report = dict(isaac_sim_version="6.0.0.1", fixture="fixed attitude; five suspended, two vertically guided wheel-supported rigs",
            scope=("production trajectory to RL phase on fixed-attitude closed-chain fixtures; "
                   "fixture PD holds legs after transfer, no actor or free-body recovery validation"
                   if args.trajectory else
                   "leg geometry and load tracking only; no free-body recovery or actor handover validation"),
            production_trajectory=args.trajectory,
            asset_manifest_sha256=digest(bundle / "manifest.json"), model_spec_sha256=digest(bundle / "model_spec.json"),
            geometry_sha256=digest(geometry), library_sha256=digest(args.library),
            script_sha256=digest(Path(__file__)), dt=dt, duration_s=args.seconds, results=[])
        for rig in rigs:
            samples = rig["samples"]
            settled = samples[-26:]
            knees = np.asarray([s["knee_deg"] for s in settled])
            error = np.asarray([s["wheel_com_error_m"] for s in settled])
            support = np.asarray([s["wheel_support_n"] for s in settled])
            contact_ok = not rig["grounded"] or bool(
                support.min() > 10. and abs(support.sum(axis=1).mean() / (builder.mass * 9.81) - 1.) < .02
                and max(s["nonwheel_support_n"] for s in settled) < .1)
            row = dict(case=rig["name"], integral_gain=rig["ki"], final_knee_deg=samples[-1]["knee_deg"],
                settled_knee_range_deg=[float(knees.min()), float(knees.max())],
                max_settled_wheel_com_error_mm=float(np.abs(error).max() * 1000),
                max_closure_gap_mm=max(s["max_closure_gap_m"] for s in samples) * 1000,
                gas_curve_max_clamp_exposure_mm=rig["max_clamp_mm"],
                final_wheel_z_m=samples[-1]["wheel_z_m"],
                mean_wheel_support_n=np.asarray([s["wheel_support_n"] for s in settled]).mean(axis=0).tolist(),
                max_settled_nonwheel_support_n=max(s["nonwheel_support_n"] for s in settled),
                final_wheel_axis_w=samples[-1]["wheel_axis_w"],
                contact_paths=rig["contact_paths"],
                wheel_support_check=contact_ok,
                passed=bool(knees.min() >= 39.9 and knees.max() <= 41.0 and np.abs(error).max() <= .005
                            and max(s["max_closure_gap_m"] for s in settled) < .002 and contact_ok))
            if args.trajectory:
                row["handover"] = rig["handover"]
                row["settled_geometry_passed"] = row["passed"]
                row["passed"] = row["passed"] and rig["handover"] is not None
            report["results"].append(row)
            (args.output / (rig["name"] + "_samples.json")).write_text(json.dumps(samples, indent=2) + "\n")
        report["passed"] = all(r["passed"] for r in report["results"] if r["integral_gain"] > 0.)
        (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print("TUCK_REPORT", json.dumps(report), flush=True)
    except BaseException as exc:
        traceback.print_exc()
        (args.output / "failure.json").write_text(json.dumps({"status": "failed", "error": str(exc)}, indent=2))
        sys.stderr.flush()
        raise
    finally:
        launcher.app.close()


if __name__ == "__main__":
    main()
