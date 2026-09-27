"""V5 closed-chain pose reconstruction from measured active joint phases."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

MOTOR_JOINTS = ("L_joint1", "LL_joint1", "R_joint1", "RR_joint1", "L_joint3", "R_joint3")


def wrap(value):
    return (value + np.pi) % (2. * np.pi) - np.pi


class PoseModel:
    def __init__(self, bundle, root_height=.65):
        self.bundle = Path(bundle).resolve()
        manifest = json.loads((self.bundle / "manifest.json").read_text())
        if manifest.get("control_frame") != "Xforward_Yleft_Zup":
            raise ValueError("expected the normalized V5 bundle with control_frame=Xforward_Yleft_Zup")
        self.model = mujoco.MjModel.from_xml_path(str(self.bundle / "robot.xml"))
        self.model.vis.headlight.ambient[:] = [.5, .5, .5]
        self.model.vis.headlight.diffuse[:] = [.8, .8, .8]
        self.data = mujoco.MjData(self.model)
        self.root_height = root_height
        self.base_id = self.model.body("base_link").id
        self.root_address = int(self.model.jnt_qposadr[self.model.joint("floating_base").id])
        self.motor_addresses = np.array([int(self.model.jnt_qposadr[self.model.joint(n).id])
                                         for n in MOTOR_JOINTS])
        self.passive_addresses = np.array([
            int(self.model.jnt_qposadr[self.model.joint(n).id])
            for n in manifest["tree_joint_names"] if n not in MOTOR_JOINTS])
        for name, value in manifest["nominal_joint_pos"].items():
            self.data.qpos[self.model.jnt_qposadr[self.model.joint(name).id]] = value
        root = ET.parse(self.bundle / "robot.xml").getroot()
        self.site_pairs = [(self.model.site(c.attrib["site1"]).id, self.model.site(c.attrib["site2"]).id)
                           for c in root.findall("equality/connect")]
        if len(self.site_pairs) != 6:
            raise ValueError("expected the V5 model with six closed-loop connect constraints")
        self.data.qpos[self.root_address:self.root_address + 3] = [0., 0., root_height]
        self.data.ctrl[:] = 0.
        self.last_positions = self.data.qpos[self.motor_addresses].copy()
        self.last_snapshot = None
        self.metrics = {}
        self.solve_count = 0
        mujoco.mj_forward(self.model, self.data)

    def loop_residual(self):
        mujoco.mj_kinematics(self.model, self.data)
        return np.concatenate([self.data.site_xpos[a] - self.data.site_xpos[b] for a, b in self.site_pairs])

    def lift_motor_phases(self, measured):
        """Lift an entire pair; do not choose the knee's shortest arc separately."""
        target = np.array(measured, dtype=float)
        for start, sign in ((0, 1.), (2, -1.)):
            hip, knee = target[start:start + 2]
            difference = wrap(sign * (hip - knee))
            heading = wrap(sign * hip - .5 * difference)
            previous = .5 * sign * sum(self.last_positions[start:start + 2])
            heading = previous + wrap(heading - previous)
            target[start:start + 2] = sign * np.array([heading + .5 * difference, heading - .5 * difference])
        return target

    def set_snapshot(self, snapshot):
        snapshot.validate()
        backup = self.data.qpos.copy()
        target = self.lift_motor_phases(snapshot.positions)
        try:
            # Continuation preserves the assembly branch from the nominal pose.
            # Native MuJoCo forward kinematics is used inside the closure solve.
            steps = max(1, math.ceil(np.max(np.abs(target[:4] - self.last_positions[:4])) / .12))
            for progress in np.linspace(0., 1., steps + 1)[1:]:
                self.data.qpos[self.motor_addresses] = self.last_positions + progress * (target - self.last_positions)

                def residual(values):
                    self.data.qpos[self.passive_addresses] = values
                    return self.loop_residual()

                result = least_squares(residual, self.data.qpos[self.passive_addresses].copy(),
                                       xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=80)
                error = float(np.max(np.abs(residual(result.x))))
                if error > 2e-6:
                    raise ValueError(f"closed-chain solve failed (gap {error:.3g} m); retaining last valid pose")
                self.solve_count += 1
            q = np.asarray(snapshot.quaternion_wxyz)
            world_from_body = Rotation.from_quat(q[[1, 2, 3, 0]])
            self.data.qpos[self.root_address:self.root_address + 3] = [0., 0., self.root_height]
            # The V5 exporter already rotates the source CAD base, root joint
            # origins and meshes into x-forward/y-left/z-up (physical Body).
            # Its base_link and training IMU observations both follow Body.
            self.data.qpos[self.root_address + 3:self.root_address + 7] = world_from_body.as_quat()[[3, 0, 1, 2]]
            self.data.qvel[:] = 0.
            self.data.ctrl[:] = 0.
            # This updates geometry only. The viewer never advances motor dynamics.
            mujoco.mj_forward(self.model, self.data)
            gravity_expected = world_from_body.inv().apply([0., 0., -1.])
            cosine = np.dot(gravity_expected, snapshot.gravity_rl) / np.linalg.norm(snapshot.gravity_rl)
            gyro_expected = np.asarray(snapshot.gyro_body)
            self.metrics = {
                "loop_gap_m": float(np.max(np.abs(self.loop_residual()))),
                "motor_phase_error_rad": float(np.max(np.abs(wrap(target - snapshot.positions)))),
                "gravity_error_deg": math.degrees(math.acos(float(np.clip(cosine, -1., 1.)))),
                "gravity_component_error": float(np.max(np.abs(gravity_expected - snapshot.gravity_rl))),
                "gyro_error_rad_s": float(np.linalg.norm(gyro_expected - snapshot.gyro_rl)),
                "body_rpy_deg": world_from_body.as_euler("xyz", degrees=True).tolist(),
                "rl_rpy_deg": world_from_body.as_euler("xyz", degrees=True).tolist(),
                "motor_deg": np.degrees(snapshot.positions).tolist(),
            }
            self.last_positions, self.last_snapshot = target, snapshot
            return self.metrics
        except Exception:
            self.data.qpos[:] = backup
            mujoco.mj_forward(self.model, self.data)
            raise

    def draw_axes(self, scene):
        if self.last_snapshot is None:
            return
        origin = self.data.xpos[self.base_id] + [0., 0., .20]
        q = np.asarray(self.last_snapshot.quaternion_wxyz)
        world_from_body = Rotation.from_quat(q[[1, 2, 3, 0]])

        def arrow(start, end, rgba, label):
            if scene.ngeom + 2 > scene.maxgeom:
                return
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                               np.eye(3).ravel(), np.asarray(rgba, dtype=np.float32))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, .007, start, end)
            # MuJoCo 3.5's OpenGL arrow tip is at local z=size[2]/2.
            # Match the visible tip to `end`, where the label is anchored.
            geom.size[2] *= 2.
            geom.label = ""
            scene.ngeom += 1
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3, .003), end,
                               np.eye(3).ravel(), np.asarray(rgba, dtype=np.float32))
            geom.label = label
            scene.ngeom += 1

        for rotation, start, prefix in ((world_from_body, origin, "RL IMU"),
                                         (world_from_body, origin + [-.60, 0., 0.], "Body")):
            for i, color in enumerate(((1., .15, .15, 1.), (.15, 1., .15, 1.), (.15, .4, 1., 1.))):
                arrow(start, start + .18 * rotation.as_matrix()[:, i], color, f"{prefix} {'xyz'[i]}")
            if scene.ngeom < scene.maxgeom:
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3, .003),
                                   start + [0., 0., -.10], np.eye(3).ravel(), np.ones(4, dtype=np.float32))
                geom.label = f"{prefix} origin"
                scene.ngeom += 1
        start = origin + [.45, 0., 0.]
        arrow(start, start + .25 * world_from_body.apply(self.last_snapshot.gravity_rl),
              (0., 1., 1., 1.), "RL gravity")
        arrow(start + [.1, 0., 0.], start + [.1, 0., -.25], (.8, .8, .8, 1.), "World down")

    def text(self, transport, age, error="", demo=False):
        prefix = "DEMO (synthetic)" if demo else "LIVE TELEMETRY"
        state = "STALE" if age > .5 else "RECEIVING"
        lines = [f"{prefix} | {state} | receive age {age:.2f}s", transport,
                 "Root position fixed for display; passive joints inferred", "RGB axes: x / y / z",
                 "RL IMU xyz = Body xyz (no axis remap)"]
        if error:
            lines.append("POSE REJECTED: " + error)
        if self.metrics:
            m = self.metrics
            lines.extend([
                "LH LK RH RK LW RW deg: " + " ".join(f"{v:7.2f}" for v in m["motor_deg"]),
                "Body roll/pitch/yaw: " + " ".join(f"{v:.2f}" for v in m["body_rpy_deg"]),
                "RL IMU roll/pitch/yaw: " + " ".join(f"{v:.2f}" for v in m["rl_rpy_deg"]),
                f"RL gravity error {m['gravity_error_deg']:.4f} deg | gyro error {m['gyro_error_rad_s']:.5f} rad/s",
                f"Closed-loop gap {m['loop_gap_m']:.3g} m | no dynamics/commands",
            ])
        return "\n".join(lines)
