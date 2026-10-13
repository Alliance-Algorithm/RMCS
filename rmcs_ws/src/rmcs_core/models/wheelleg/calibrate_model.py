#!/usr/bin/env python3
"""Calibrate closed-chain constraint stiffness and joint damping.

Loads the built closed-chain model, overrides eq_solref/eq_solimp and
dof_damping/dof_armature in memory, and probes the mechanism:

  * constraint probe: base welded, gravity off, kick the passive joints and
    watch (a) the loop-closure residual and (b) whether the kinetic energy
    decays. A too-stiff constraint (timeconst << dt) rings; a too-soft one
    leaves a steady gap.
  * damping probe: same kick, sweep dof_damping and measure energy decay
    (target: monotone, no ringing, no growth).

Run:  ~/.venvs/mujoco/bin/python calibrate_model.py
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
MODEL = HERE / "wheelleg.xml"

DT = 0.001
HIP_JOINTS = ("L_joint1", "L_crank", "R_joint1", "R_crank")
WHEEL_JOINTS = ("L_joint3", "R_joint3")
# Reflected rotor inertia estimate for the hip motors (J_rotor * N^2), see notes.
HIP_ARMATURE = 5.0e-3


def load() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    return model, data


def weld_base(model: mujoco.MjModel, data: mujoco.MjData) -> int:
    """Return the free-joint dof address; held by zeroing qvel each step."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    return model.jnt_dofadr[jid]


def closure_gap(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    worst = 0.0
    for side in ("L", "R"):
        for a, b in (("D_lower", "D_rocker"), ("E_upper", "E_shank")):
            pa = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_{a}")]
            pb = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_{b}")]
            worst = max(worst, float(np.linalg.norm(pa - pb)))
    return worst


def probe(
    timeconst: float, dampratio: float, damping: float, armature: float, kick: float = 2.0
) -> tuple[float, float, float]:
    model, data = load()
    model.opt.gravity[:] = 0.0
    # visual-only linkage geoms already have no collision; drop all contacts
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    model.eq_solref[:, 0] = timeconst
    model.eq_solref[:, 1] = dampratio
    base = weld_base(model, data)

    for name in HIP_JOINTS:
        ad = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        model.dof_armature[ad] = armature
    for name in HIP_JOINTS + WHEEL_JOINTS + (
        "L_joint2", "R_jonit2", "L_lower_coupler", "R_lower_coupler",
        "L_rocker", "R_rocker", "L_upper_coupler", "R_upper_coupler",
    ):
        ad = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        model.dof_damping[ad] = damping

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    for ad in range(model.nv):
        if ad not in (base + i for i in range(6)):
            data.qvel[ad] = kick
    energies = []
    max_gap = 0.0
    for _ in range(1500):
        data.qvel[base : base + 6] = 0.0
        mujoco.mj_step(model, data)
        max_gap = max(max_gap, closure_gap(model, data))
        energies.append(float(np.sum(data.qvel**2)))
    energies = np.array(energies)
    early = energies[200:400].mean()
    late = energies[1200:1400].mean()
    ratio = late / early if early > 0 else 0.0
    return max_gap, ratio, float(energies.max())


def main() -> None:
    print("== constraint probe (damping=0.05, armature=5e-3) ==")
    print("timeconst  dampratio  max_gap[m]  E_late/E_early")
    for tc in (0.0005, 0.001, 0.002, 0.004, 0.008, 0.016, 0.02):
        for dr in (0.7, 1.0):
            gap, ratio, _ = probe(tc, dr, 0.05, HIP_ARMATURE)
            print(f"  {tc:8.4f}  {dr:8.2f}  {gap:10.2e}  {ratio:8.3f}")

    print("\n== damping probe (timeconst=0.004, dampratio=1.0) ==")
    print("damping   E_late/E_early")
    for damping in (0.0, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0):
        _, ratio, _ = probe(0.004, 1.0, damping, HIP_ARMATURE)
        print(f"  {damping:6.3f}   {ratio:8.3f}")


if __name__ == "__main__":
    main()
