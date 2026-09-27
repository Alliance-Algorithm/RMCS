#!/usr/bin/env python3
"""Record actual Isaac ArticulationData IMU outputs at known physical poses.

The CSV is consumed by the production C++ component test. The asset is spawned
with gravity disabled on its bodies, while world gravity remains -Z. No model
USD, training configuration, hardware interface, or policy weight is changed.
"""
import argparse
import csv
import hashlib
import importlib.metadata
import inspect
import json
import os
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--reference-csv", type=Path, required=True)
parser.add_argument("--device", default="cpu")
args = parser.parse_args()

import numpy as np
from scipy.spatial.transform import Rotation
from isaaclab.app import AppLauncher

launcher = AppLauncher({"headless": True, "device": args.device, "enable_cameras": False})
app = launcher.app
try:
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import IdealPDActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.assets.articulation.articulation_data import ArticulationData

    torch.set_num_threads(4)
    dt = 0.001
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=dt))
    robot = Articulation(ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(args.usd.resolve()),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=False, enabled_self_collisions=False)),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0., 0., 10.),
                                                  joint_pos={".*": 0.}, joint_vel={".*": 0.}),
        actuators={"all": IdealPDActuatorCfg(joint_names_expr=[".*"], stiffness=0., damping=0.)}))
    sim.reset()

    # Columns are the training frame axes expressed in the real chassis frame.
    # This defines the simulation pose independently of the RMCS converter.
    body_from_rl = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    poses = [
        ("level", 0., 0., 0.),
        ("body_roll_pos30", 30., 0., 0.),
        ("body_roll_neg30", -30., 0., 0.),
        ("body_pitch_pos30", 0., 30., 0.),
        ("body_pitch_neg30", 0., -30., 0.),
        ("roll30_yaw90", 30., 0., 90.),
        ("combined_25_neg20_70", 25., -20., 70.),
        ("combined_neg35_40_neg135", -35., 40., -135.),
        ("body_roll90", 90., 0., 0.),
        ("body_pitch90", 0., 90., 0.),
        ("inverted", 180., 0., 0.),
        ("near_yaw_wrap", 20., 30., 179.5),
    ]
    records = []
    reference_rows = []
    zero_vel = torch.zeros((1, 6), device=args.device)
    omega_body = np.array([.7, -1.2, 2.3])
    for name, roll, pitch, yaw in poses:
        # scipy lowercase xyz = extrinsic XYZ = Rz(yaw) Ry(pitch) Rx(roll).
        rotation_wb = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
        rotation_wr = Rotation.from_matrix(rotation_wb.as_matrix() @ body_from_rl)
        q_wb = rotation_wb.as_quat()[[3, 0, 1, 2]]
        q_wr = rotation_wr.as_quat()[[3, 0, 1, 2]]
        pose = torch.tensor([[0., 0., 10., *q_wr]], device=args.device, dtype=torch.float32)
        robot.write_root_pose_to_sim(pose)
        robot.write_root_velocity_to_sim(zero_vel)
        robot.write_joint_state_to_sim(robot.data.default_joint_pos,
                                      torch.zeros_like(robot.data.default_joint_vel))
        robot.reset()
        sim.step(render=False)
        robot.update(dt)

        # Read the pose back directly from PhysX, not from the write-side cache.
        physx_pose = robot.root_physx_view.get_root_transforms()[0].cpu().numpy()
        rotation_read = Rotation.from_quat(physx_pose[3:7])
        pose_error_rad = float((rotation_wr.inv() * rotation_read).magnitude())
        assert pose_error_rad < 2e-6, (name, "pose readback", pose_error_rad)
        gravity_rl = robot.data.projected_gravity_b[0].cpu().numpy().astype(float)

        omega_world = rotation_wb.apply(omega_body)
        velocity = torch.tensor([[0., 0., 0., *omega_world]], device=args.device, dtype=torch.float32)
        robot.write_root_velocity_to_sim(velocity)
        robot.update(dt)  # Advance data timestamp so the property reads the PhysX velocity.
        omega_rl = robot.data.root_ang_vel_b[0].cpu().numpy().astype(float)

        r, p = np.radians([roll, pitch])
        # Independent static accelerometer sample for the production EKF test.
        accel_body = np.array([-np.sin(p), np.sin(r) * np.cos(p), np.cos(r) * np.cos(p)])
        reference_rows.append([name, *q_wb, *omega_body, *accel_body, *gravity_rl, *omega_rl])
        records.append({
            "name": name, "physical_body_rpy_deg": [roll, pitch, yaw],
            "q_world_from_body_wxyz": q_wb.tolist(),
            "q_world_from_rl_wxyz": q_wr.tolist(),
            "physx_root_quat_xyzw": physx_pose[3:7].tolist(),
            "physx_pose_error_rad": pose_error_rad,
            "body_static_accel_g": accel_body.tolist(), "body_gyro_rad_s": omega_body.tolist(),
            "isaac_projected_gravity_b": gravity_rl.tolist(),
            "isaac_root_ang_vel_b": omega_rl.tolist(),
            "old_bridge_q_body_times_world_down": rotation_wb.apply([0., 0., -1.]).tolist(),
            "wrong_forward_q_body_times_q_mount": rotation_wr.apply([0., 0., -1.]).tolist(),
        })

    source_path = Path(inspect.getfile(ArticulationData))
    report = {
        "asset": str(args.usd.resolve()), "asset_sha256": hashlib.sha256(args.usd.read_bytes()).hexdigest(),
        "isaaclab_version": importlib.metadata.version("isaaclab"),
        "isaacsim_version": importlib.metadata.version("isaacsim"),
        "root_body": robot.body_names[0], "body_names": robot.body_names,
        "data_source": str(source_path), "data_source_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
        "projected_gravity_source": inspect.getsource(ArticulationData.projected_gravity_b.fget),
        "world_gravity_direction": robot.data.GRAVITY_VEC_W[0].cpu().tolist(),
        "body_from_rl_matrix": body_from_rl.tolist(),
        "pose_count": len(records),
        "max_physx_pose_error_rad": max(x["physx_pose_error_rad"] for x in records),
        "cases": records,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.reference_csv.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    with args.reference_csv.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["case", "qw", "qx", "qy", "qz", "wx_body", "wy_body", "wz_body",
                         "ax_body", "ay_body", "az_body", "gx_isaac", "gy_isaac", "gz_isaac",
                         "wx_isaac", "wy_isaac", "wz_isaac"])
        writer.writerows(reference_rows)
    print(json.dumps({"pose_count": report["pose_count"], "root_body": report["root_body"],
                      "max_physx_pose_error_rad": report["max_physx_pose_error_rad"],
                      "report": str(args.output), "reference_csv": str(args.reference_csv)}), flush=True)
finally:
    # This offline run has no render/writer work to drain. Full Kit cleanup
    # can hang on this headless Isaac Sim 5.1 installation after data is saved.
    app.close(wait_for_replicator=False, skip_cleanup=True)
