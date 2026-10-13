#!/usr/bin/env python3
"""Close the v5.0 wheel-leg kinematic loops and write the MuJoCo MJCF.

The v5.0 leg is a Stephenson six-bar: two coaxial hip motors drive the thigh
(``L_joint1``) and the crank (``L_crank``); the crank drives the knee through
``LL_joint2 -> LL_joint3 -> LL_link4``. The SolidWorks tree export leaves two
revolute pins per leg missing, so we recover them from the mesh geometry at the
CAD assembly pose and add ``connect`` equality constraints:

    P: rocker ``LL_link3`` <-> thigh ``L_link1``
    E: coupler2 ``LL_link4`` <-> shank ``L_link2``

The pin centres are found by fitting a circle to the hole vertices; the local
coordinates on both mating links are then derived from the body transforms at
``qpos = 0`` so the two sites coincide exactly at assembly.

Run from this directory after ``convert_urdf.py``:

    ~/.venvs/mujoco/bin/python build_closed_chain.py

It writes ``wheelleg.xml``, the model the bridge loads.
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
SERIAL = HERE / "wheelleg_serial.xml"
OUTPUT = HERE / "wheelleg.xml"

CONNECT_SOLREF = (0.001, 1.0)
CONNECT_SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)

# Prior location of each pin in the sagittal (y, z) plane, plus the expected
# hole radius and fit tolerance. The hole is fitted on ``hole_body`` and the
# matching point is transferred to ``mate_body``.
#   name, hole_body, mate_body, guess (y, z), radius, tol
CLOSURES = (
    ("L_P", "LL_link3", "L_link1", (0.1049, 0.0053), 0.013, 0.004),
    ("L_E", "LL_link4", "L_link2", (0.2460, 0.0568), 0.009, 0.003),
    ("R_P", "RR_link3", "R_link1", (0.1049, 0.0053), 0.013, 0.004),
    ("R_E", "RR_link4", "R_link2", (0.2460, 0.0568), 0.009, 0.003),
)


def body_world_vertices(
    model: mujoco.MjModel, data: mujoco.MjData, body_name: str, plane_x: float
) -> np.ndarray:
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    points = []
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != body:
            continue
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[geom]
        start = model.mesh_vertadr[mesh]
        count = model.mesh_vertnum[mesh]
        verts = model.mesh_vert[start : start + count]
        rotation = data.geom_xmat[geom].reshape(3, 3)
        points.append(verts @ rotation.T + data.geom_xpos[geom])
    vertices = np.vstack(points)
    # The CAD meshes carry a stray triangle far outside the link; keep the
    # sagittal-plane slab only.
    return vertices[np.abs(vertices[:, 0] - plane_x) < 0.06]


def kasa_fit(points2d: np.ndarray) -> tuple[float, float, float]:
    x, y = points2d[:, 0], points2d[:, 1]
    matrix = np.column_stack([2.0 * x, 2.0 * y, np.ones(len(x))])
    rhs = x * x + y * y
    solution, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
    cx, cy = solution[0], solution[1]
    radius = np.sqrt(solution[2] + cx * cx + cy * cy)
    return cx, cy, radius


def fit_hole(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    plane_x: float,
    guess: tuple[float, float],
    radius0: float,
    tolerance: float,
) -> np.ndarray:
    vertices = body_world_vertices(model, data, body_name, plane_x)
    plane = vertices[:, 1:3]
    selected = np.abs(np.hypot(plane[:, 0] - guess[0], plane[:, 1] - guess[1]) - radius0) < tolerance
    cx, cy, radius = kasa_fit(plane[selected])
    for _ in range(6):
        selected = np.abs(np.hypot(plane[:, 0] - cx, plane[:, 1] - cy) - radius) < tolerance
        if selected.sum() < 8:
            break
        cx, cy, radius = kasa_fit(plane[selected])
    # Full 3D centre: the fitted (y, z) plus the mean x of the hole vertices.
    return np.array([vertices[selected, 0].mean(), cx, cy])


def local_point(model: mujoco.MjModel, data: mujoco.MjData, body_name: str, world: np.ndarray) -> np.ndarray:
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    rotation = data.xmat[body].reshape(3, 3)
    return rotation.T @ (world - data.xpos[body])


def main() -> None:
    spec = mujoco.MjSpec.from_file(str(SERIAL))

    # Work out the pin world positions and both mating local coordinates on a
    # copy of the model at the assembly pose (qpos = 0).
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)

    for name, hole_body, mate_body, guess, radius, tolerance in CLOSURES:
        plane_x = -0.19 if name.startswith("R") else 0.19
        world = fit_hole(model, data, hole_body, plane_x, guess, radius, tolerance)
        hole_local = local_point(model, data, hole_body, world)
        mate_local = local_point(model, data, mate_body, world)

        hole_site = next(b for b in spec.bodies if b.name == hole_body).add_site()
        hole_site.name = f"{name}_hole"
        hole_site.pos = tuple(hole_local)
        hole_site.size = (0.001, 0.0, 0.0)

        mate_site = next(b for b in spec.bodies if b.name == mate_body).add_site()
        mate_site.name = f"{name}_mate"
        mate_site.pos = tuple(mate_local)
        mate_site.size = (0.001, 0.0, 0.0)

        equality = spec.add_equality()
        equality.name = name
        equality.type = mujoco.mjtEq.mjEQ_CONNECT
        equality.objtype = mujoco.mjtObj.mjOBJ_SITE
        equality.name1 = hole_site.name
        equality.name2 = mate_site.name
        equality.solref = CONNECT_SOLREF
        equality.solimp = CONNECT_SOLIMP

        print(
            f"{name}: world {np.round(world, 5)}  "
            f"{hole_body} local {np.round(hole_local, 5)}  "
            f"{mate_body} local {np.round(mate_local, 5)}"
        )

    add_commissioning_scaffold(spec)

    spec.compile()
    spec.to_file(str(OUTPUT))
    print(f"wrote {OUTPUT}")


# Joints the bridge locks to turn the leg into a rigid link (commissioning).
LOCK_JOINTS = (
    "L_joint1", "L_crank", "L_joint2", "LL_joint2", "LL_joint3", "LL_link4",
    "R_joint1", "R_crank", "R_jonit2", "RR_joint2", "RR_joint3", "RR_joint4",
)


def add_carrier(spec: mujoco.MjSpec, name: str, axes) -> None:
    """A massless guide body whose joints leave exactly the wanted base DOFs."""
    body = spec.worldbody.add_body()
    body.name = name
    body.explicitinertial = True
    body.mass = 1e-6
    body.inertia = (1e-9, 1e-9, 1e-9)
    for joint_name, joint_type, axis in axes:
        joint = body.add_joint()
        joint.name = joint_name
        joint.type = joint_type
        joint.axis = axis


def add_weld(spec: mujoco.MjSpec, name: str, body2: str) -> None:
    """Weld ``base_link`` to ``body2`` (a carrier or the world), default off."""
    equality = spec.add_equality()
    equality.name = name
    equality.type = mujoco.mjtEq.mjEQ_WELD
    equality.objtype = mujoco.mjtObj.mjOBJ_BODY
    equality.name1 = "base_link"
    equality.name2 = body2
    equality.active = False


def add_commissioning_scaffold(spec: mujoco.MjSpec) -> None:
    """Real constraints the bridge toggles instead of clamping qpos/qvel.

    Clamping state after ``mj_step`` is *not* a constraint: the free DOFs
    evolve unconstrained within each step, so commissioning experiments are not
    physically faithful.  These equalities let the bridge pin the base and lock
    the leg the way MuJoCo's solver would for a genuine rigid connection.
    """
    # z-gantry: only vertical motion is free.
    add_carrier(spec, "zgantry", [("zgantry_slide_z", mujoco.mjtJoint.mjJNT_SLIDE, (0, 0, 1))])
    # planar carrier: forward y, vertical z and pitch free; lateral, roll, yaw
    # are held by the weld to the base.
    add_carrier(
        spec, "planar_carrier",
        [
            ("planar_slide_y", mujoco.mjtJoint.mjJNT_SLIDE, (0, 1, 0)),
            ("planar_slide_z", mujoco.mjtJoint.mjJNT_SLIDE, (0, 0, 1)),
            ("planar_pitch", mujoco.mjtJoint.mjJNT_HINGE, (1, 0, 0)),
        ],
    )

    add_weld(spec, "fix_base_z", "zgantry")
    add_weld(spec, "fix_base_planar", "planar_carrier")

    # Locking the z-gantry turns "vertical only" into "fully fixed".
    zgantry_lock = spec.add_equality()
    zgantry_lock.name = "lock_zgantry_slide_z"
    zgantry_lock.type = mujoco.mjtEq.mjEQ_JOINT
    zgantry_lock.objtype = mujoco.mjtObj.mjOBJ_JOINT
    zgantry_lock.name1 = "zgantry_slide_z"
    zgantry_lock.active = False

    for joint_name in LOCK_JOINTS:
        equality = spec.add_equality()
        equality.name = f"lock_{joint_name}"
        equality.type = mujoco.mjtEq.mjEQ_JOINT
        equality.objtype = mujoco.mjtObj.mjOBJ_JOINT
        equality.name1 = joint_name
        equality.active = False


if __name__ == "__main__":
    main()
