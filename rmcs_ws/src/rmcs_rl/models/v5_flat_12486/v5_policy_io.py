"""NumPy reference for V5 SCUT35 integration; no robot driver or simulator required."""
import hashlib
import json
from pathlib import Path

import numpy as np


CONTROL_ORDER = ("L_joint1", "LL_joint1", "L_joint3", "R_joint1", "RR_joint1", "R_joint3")
POLICY_ORDER = ("L_joint1", "LL_joint1", "R_joint1", "RR_joint1", "L_joint3", "R_joint3")
POLICY_FROM_CONTROL = np.array([0, 1, 3, 4, 2, 5])
CONTROL_FROM_POLICY = np.array([0, 1, 4, 2, 3, 5])
LEGS = np.array([0, 1, 3, 4])
WHEELS = np.array([2, 5])


def _vector(value, size, name):
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must be a finite vector of length {size}")
    return value


def build_observation(q_control, dq_control, omega_body, gravity_body,
                      command_vx_yaw_height, nominal_control, previous_policy_action,
                      *, lateral_command=0., jump_request=False, jump_apex_m=0., jump_elapsed_s=0.):
    """Return float32 [1,35]; encoder values are already calibrated joint-output SI units."""
    q = _vector(q_control, 6, "q_control")
    dq = _vector(dq_control, 6, "dq_control")
    nominal = _vector(nominal_control, 6, "nominal_control")
    omega = _vector(omega_body, 3, "omega_body")
    gravity = _vector(gravity_body, 3, "gravity_body")
    command = _vector(command_vx_yaw_height, 3, "command_vx_yaw_height")
    previous = _vector(previous_policy_action, 6, "previous_policy_action")
    if not np.isclose(np.linalg.norm(gravity), 1., atol=.02):
        raise ValueError("gravity_body is a unit gravity direction, not raw acceleration")
    scalar_commands = _vector([lateral_command, jump_apex_m, jump_elapsed_s], 3, "context commands")
    delta = q - nominal
    delta = np.arctan2(np.sin(delta), np.cos(delta))[POLICY_FROM_CONTROL]
    delta[4:] = 0.
    context = np.zeros(7, dtype=np.float32)
    context[0] = float(not jump_request)
    context[4] = float(jump_request)
    if jump_request:
        context[5] = scalar_commands[1] * 5.
        context[6] = np.clip(scalar_commands[2], 0., 5.)
    observation = np.concatenate((
        [command[0], scalar_commands[0], command[1], command[2] * 5.],
        omega * .5, gravity, delta, dq[POLICY_FROM_CONTROL] * .1, previous, context))
    return np.clip(observation, -100., 100.).astype(np.float32)[None, :]


def decode_actions(raw_policy_action, q_control, nominal_control, settings):
    """Return four leg position targets, two wheel speed targets, and clipped policy history."""
    raw = _vector(raw_policy_action, 6, "raw_policy_action")
    q = _vector(q_control, 6, "q_control")
    nominal = _vector(nominal_control, 6, "nominal_control")
    bounds = np.array([settings["action_clip"]] * 4
                      + [settings.get("wheel_action_clip", settings["action_clip"])] * 2, dtype=np.float32)
    clipped_policy = np.clip(raw, -bounds, bounds)
    control_action = clipped_policy[CONTROL_FROM_POLICY]
    desired = nominal[LEGS] + settings["leg_position_scale"] * control_action[LEGS]
    delta = desired - q[LEGS]
    leg_targets = q[LEGS] + np.arctan2(np.sin(delta), np.cos(delta))
    wheel_targets = settings["wheel_velocity_scale"] * control_action[WHEELS]
    return leg_targets, wheel_targets, clipped_policy


def reference_joint_torques(q_control, dq_control, leg_targets, wheel_targets, settings, wheel_prior):
    """Research joint-output controller, not identified real motor/current commands."""
    q = _vector(q_control, 6, "q_control")
    dq = _vector(dq_control, 6, "dq_control")
    legs = _vector(leg_targets, 4, "leg_targets")
    wheels = _vector(wheel_targets, 2, "wheel_targets")
    torque = np.zeros(6, dtype=np.float32)
    torque[LEGS] = np.clip(settings["leg_kp"] * (legs - q[LEGS]) - settings["leg_kd"] * dq[LEGS], -40., 40.)
    motor_speed = np.abs(dq[WHEELS]) * wheel_prior["gear_ratio"]
    bound = np.interp(motor_speed, wheel_prior["motor_speed_rad_s"], wheel_prior["motor_torque_nm"], right=0.)
    bound = np.minimum(bound * wheel_prior["gear_ratio"] * wheel_prior["gearbox_efficiency"], wheel_prior["effort_limit"])
    torque[WHEELS] = np.clip(wheel_prior["kd"] * (wheels - dq[WHEELS]), -bound, bound)
    return torque


def joint_to_drive_torques(torque_control, jacobian_control_from_drive):
    """Map C-order effort to drive order using an explicitly supplied calibration.

    dq_C = J_CH @ dtheta_H. Drive torque units must be power-conjugate to
    theta_H; gearbox efficiency and device current encoding are adapter duties.
    No identity mapping or transmission ratio is assumed.
    """
    torque = _vector(torque_control, 6, "torque_control")
    jacobian = np.asarray(jacobian_control_from_drive, dtype=np.float32)
    if (jacobian.shape != (6, 6) or not np.isfinite(jacobian).all()
            or np.linalg.matrix_rank(jacobian) != 6):
        raise ValueError("A finite full-rank 6x6 calibrated drive-to-control Jacobian is required")
    return jacobian.T @ torque


def verify_bundle(onnx_path, contract_path, manifest_path, control_prior_path):
    """Check artifact integrity and ABI; skill acceptance is read from the run's selection receipt."""
    paths = list(map(Path, (onnx_path, contract_path, manifest_path, control_prior_path)))
    onnx_path, contract_path, manifest_path, control_prior_path = paths
    metadata = json.loads(Path(str(onnx_path) + ".json").read_text())
    contract = json.loads(contract_path.read_text())
    manifest = json.loads(manifest_path.read_text())
    control_prior = json.loads(control_prior_path.read_text())
    for path, key in zip(paths, ("onnx_sha256", "contract_sha256", "asset_manifest_sha256", "control_math_sha256")):
        if hashlib.sha256(path.read_bytes()).hexdigest() != metadata[key]:
            raise ValueError(f"Bundle checksum mismatch: {path}")
    if (not metadata.get("verified") or contract["actor_dim"] != 35 or contract["action_dim"] != 6
            or contract["history_length"] != 1
            or contract["actor_observation_source"] != "scut35_encoders_imu_commands"
            or tuple(contract["policy_action_order"]) != POLICY_ORDER
            or tuple(manifest["control_joint_names"]) != CONTROL_ORDER):
        raise ValueError("Bundle is not the supported V5 single-frame SCUT35 interface")
    return contract, manifest, control_prior, metadata
