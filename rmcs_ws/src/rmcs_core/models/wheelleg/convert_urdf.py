#!/usr/bin/env python3
"""Convert the SolidWorks v5.0 wheel-leg URDF into an MJCF.

The v5.0 URDF is a *tree* (SolidWorks breaks every closed loop); it contains the
full Stephenson six-bar linkage (LL/RR) and the nitrogen spring (LLL/RRR) as
extra links, plus the thigh/shank/wheel. This script adds everything MuJoCo
needs before ``build_closed_chain.py`` closes the two kinematic loops per leg:

  * a free joint on ``base_link`` so the robot floats,
  * a ground plane,
  * smooth cylinder collision on the two wheels (mesh collision rolls badly),
  * collision disabled on the linkage / spring links (they self-intersect),
  * the gas-spring mounts welded to their parent (they carry inertia/visuals but
    must not add a DOF; the spring force is not modelled here),
  * the crank joints renamed to ``L_crank`` / ``R_crank`` to match the bridge,
  * one torque actuator per leg motor: hip / crank / wheel.

Run from this directory:

    ~/.venvs/mujoco/bin/python convert_urdf.py

It writes ``wheelleg_serial.xml``; run ``build_closed_chain.py`` afterwards.
"""

from __future__ import annotations

import pathlib

import mujoco

HERE = pathlib.Path(__file__).resolve().parent
SOURCE_URDF = HERE / "wheelleg_source.urdf"
OUTPUT_MJCF = HERE / "wheelleg_serial.xml"

# Joints that stay articulated before loop closure. The knee L_joint2/R_jonit2
# and the linkage LL_*/RR_* joints are passive.
CRANK_JOINT_RENAME = {"LL_joint1": "L_crank", "RR_joint1": "R_crank"}

# Actuated joints in the order the bridge expects, and their actuator names.
LEG_ACTUATORS = (
    ("L_joint1", "L_joint1_motor"),
    ("L_crank", "L_crank_motor"),
    ("L_joint3", "L_joint3_motor"),
    ("R_joint1", "R_joint1_motor"),
    ("R_crank", "R_crank_motor"),
    ("R_joint3", "R_joint3_motor"),
)

WHEEL_BODIES = ("L_link3", "R_link3")
# Keep collision only on the body, thighs, shanks and the wheels; the linkage
# links are visual-only.
COLLISION_BODIES = ("base_link", "L_link1", "L_link2", "R_link1", "R_link2")

WHEEL_RADIUS = 0.06
WHEEL_HALF_WIDTH = 0.0125

MOTOR_CTRL_RANGE = 100.0


def disable_collision(body: mujoco.MjsBody) -> None:
    for geom in body.geoms:
        geom.contype = 0
        geom.conaffinity = 0


def main() -> None:
    urdf = SOURCE_URDF.read_text()
    urdf = urdf.replace("package://urdf_v5.0/meshes/", "meshes/")

    spec = mujoco.MjSpec.from_string(urdf)
    spec.modelname = "wheelleg"

    spec.compiler.meshdir = "."
    spec.compiler.fusestatic = False
    spec.compiler.discardvisual = False
    # Some linkage links have a zero principal inertia in the CAD export.
    spec.compiler.balanceinertia = True

    spec.option.timestep = 0.001
    spec.option.gravity = (0.0, 0.0, -9.81)
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.iterations = 100
    spec.option.ls_iterations = 50

    spec.default.geom.friction = (1.0, 0.005, 0.0001)

    # Free joint so base_link is a floating body.
    base = next(body for body in spec.bodies if body.name == "base_link")
    base.add_freejoint(name="base_free")

    # Ground plane.
    ground = spec.worldbody.add_geom()
    ground.name = "ground"
    ground.type = mujoco.mjtGeom.mjGEOM_PLANE
    ground.size = (0.0, 0.0, 0.05)
    ground.rgba = (0.35, 0.35, 0.4, 1.0)
    ground.contype = 1
    ground.conaffinity = 1

    # Rename the crank joints.
    for old, new in CRANK_JOINT_RENAME.items():
        joint = next(j for j in spec.joints if j.name == old)
        joint.name = new

    # The SolidWorks gas-spring links (rod on the thigh, body on the shank) are
    # separate branches; welding both mounts leaves the rod sticking out, and
    # closing them with a prismatic over-constrains the six-bar (a real gas
    # spring is a passive force element, not a kinematic loop). Drop the rigid
    # links here; the spring force can be re-added later as a tendon.
    for name in ("LLL_link1", "LLL_link2", "RRR_link1", "RRR_link2"):
        body = next((b for b in spec.bodies if b.name == name), None)
        if body is not None:
            spec.delete(body)

    # Collision: keep only base/thigh/shank meshes; the wheels get a smooth
    # cylinder instead of the faceted mesh. Skip the world body so the ground
    # plane keeps colliding.
    for body in spec.bodies:
        if body.name == "world":
            continue
        if body.name not in COLLISION_BODIES and body.name not in WHEEL_BODIES:
            disable_collision(body)

    for body in spec.bodies:
        if body.name not in WHEEL_BODIES:
            continue
        disable_collision(body)
        collision = body.add_geom()
        collision.name = f"{body.name}_collision"
        collision.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        collision.size = (WHEEL_RADIUS, WHEEL_HALF_WIDTH, 0.0)
        collision.friction = (2.0, 0.01, 0.0005)

    # MuJoCo only auto-excludes parent-child body pairs, so the base would also
    # collide with its *grandchild* links (e.g. base_link <-> shank), producing
    # spurious internal contact forces through the closed chain. Put every active
    # robot geom in collision class 2 so it only interacts with the ground
    # (contype=1): class 2 never collides with class 2.
    for body in spec.bodies:
        if body.name not in COLLISION_BODIES and body.name not in WHEEL_BODIES:
            continue
        for geom in body.geoms:
            if geom.contype != 0 or geom.conaffinity != 0:
                geom.contype = 2
                geom.conaffinity = 1

    # Planarise the leg: the CAD joints carry ~0.26 deg axis tilts which make
    # the two equality constraints per leg independent instead of redundant,
    # over-constraining the six-bar (leg mobility drops from 3 to 1). Forcing
    # every hinge axis to the sagittal normal (world x) restores the intended
    # planar mobility while leaving the assembly pose unchanged.
    probe = spec.compile()
    probe_data = mujoco.MjData(probe)
    probe_data.qpos[:] = 0.0
    mujoco.mj_forward(probe, probe_data)
    body_rotation = {
        mujoco.mj_id2name(probe, mujoco.mjtObj.mjOBJ_BODY, i): probe_data.xmat[i].reshape(3, 3)
        for i in range(probe.nbody)
    }
    for joint in spec.joints:
        if joint.type != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        owner = next(
            (b for b in spec.bodies if any(j.name == joint.name for j in b.joints)), None
        )
        if owner is None:
            continue
        joint.axis = tuple(body_rotation[owner.name].T @ (1.0, 0.0, 0.0))

    # The wheels must roll without bound. The CAD URDF marks L_joint3/R_joint3
    # as ``continuous`` yet still carries +/-pi hard limits, which cap the
    # vehicle to ~0.19 m of travel before the wheel jams and the reaction torque
    # tips the body. Force them unlimited regardless of the URDF limit element.
    for joint in spec.joints:
        if joint.name in ("L_joint3", "R_joint3"):
            joint.limited = False

    # One torque motor per leg joint.
    for joint_name, actuator_name in LEG_ACTUATORS:
        actuator = spec.add_actuator()
        actuator.name = actuator_name
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
        actuator.target = joint_name
        actuator.set_to_motor()
        actuator.gear = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        actuator.ctrllimited = True
        actuator.ctrlrange = (-MOTOR_CTRL_RANGE, MOTOR_CTRL_RANGE)

    spec.compile()
    OUTPUT_MJCF.write_text(spec.to_xml())
    print(f"wrote {OUTPUT_MJCF}")


if __name__ == "__main__":
    main()
