#!/usr/bin/env python3
import math
import numpy as np


# ============================================================
# Correct method used in this workspace
#
# 1. Do NOT use the incomplete MDH test model from
#    rmcs_core/src/controller/test/Kinematic_test.cpp
# 2. Use the actual URDF kinematic chain that MoveIt and link_6 use:
#    base_link -> joint_1 -> ... -> joint_6 -> link_6
# 3. Build T_06 as:
#       T = Π [ T(origin_xyz, origin_rpy) * Rot(axis, joint_angle) ]
# 4. Then construct the other 5 hexagon vertices in base_link.
#
# This script prints:
# - FK pose of the known top vertex
# - Standard poses: ideal hexagon plane, ideal orientation
# - Error poses: preserve the measured top-pose orientation error and
#   rotate the whole pose around base_link X to the other vertices
# ============================================================


# -----------------------------
# Input joint angles (radians)
# -----------------------------
joint_angles = [0.0, 0.465947, -0.094436, 0.0, -0.293182, 0.0]


# -----------------------------
# Hexagon side length (meters)
# -----------------------------
s = (113.0 * math.sqrt(3.0) / 2.0 + 75.0 / 2.0) / 1000.0


# ============================================================
# Actual URDF joint chain for link_6
# joint_i = (origin_xyz, origin_rpy, axis_xyz)
# ============================================================
urdf_joints = [
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, -0.042, 0.071), (1.5708, 0.0, -3.1416), (0.0, 0.0, -1.0)),
    ((0.0, 0.37, 0.007), (0.0, 0.0, 1.5708), (0.0, 0.0, -1.0)),
    ((0.03, 0.1419, 0.035), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.2591), (1.5708, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ((0.0, 0.119, 0.0), (1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
]


def rpy_to_rot(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def rot_to_rpy(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


def transform_from_origin(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_rot(*rpy)
    T[:3, 3] = np.array(xyz, dtype=float)
    return T


def axis_angle_to_rot(axis, theta):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(theta)
    s_ = math.sin(theta)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s_, x * z * C + y * s_],
            [y * x * C + z * s_, c + y * y * C, y * z * C - x * s_],
            [z * x * C - y * s_, z * y * C + x * s_, c + z * z * C],
        ],
        dtype=float,
    )


def fk_urdf(q):
    T = np.eye(4)
    for (origin_xyz, origin_rpy, axis), theta in zip(urdf_joints, q):
        T_origin = transform_from_origin(origin_xyz, origin_rpy)
        T_joint = np.eye(4)
        T_joint[:3, :3] = axis_angle_to_rot(axis, theta)
        T = T @ T_origin @ T_joint
    return T


def Rx(theta):
    c = math.cos(theta)
    s_ = math.sin(theta)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s_],
            [0.0, s_, c],
        ],
        dtype=float,
    )


def format_pose_target(name, pose):
    x, y, z, roll, pitch, yaw = pose
    print(f"// {name}")
    print("Action::PoseTarget{")
    print(f"          .x     = {x:.6f},")
    print(f"          .y     = {y:.6f},")
    print(f"          .z     = {z:.6f},")
    print(f"          .roll  = {roll:.6f},")
    print(f"          .pitch = {pitch:.6f},")
    print(f"          .yaw   = {yaw:.6f}}}")
    print()


def main():
    T_06 = fk_urdf(joint_angles)
    p_top = T_06[:3, 3]
    R_top = T_06[:3, :3]
    roll_top, pitch_top, yaw_top = rot_to_rpy(R_top)

    x0, y0, z0 = p_top
    Yc = y0
    Zc = z0 - s
    center = np.array([x0, Yc, Zc], dtype=float)

    ex_top = R_top[:, 0]
    ey_top = R_top[:, 1]
    ez_top = R_top[:, 2]

    # Error metrics against the ideal top pose:
    # ideal +z = -base_link.x, ideal +x = +base_link.z
    ideal_z = np.array([-1.0, 0.0, 0.0], dtype=float)
    z_angle_deg = math.degrees(
        math.acos(max(-1.0, min(1.0, float(np.dot(ez_top, ideal_z)))))
    )
    x_out_of_plane_deg = math.degrees(
        math.asin(max(-1.0, min(1.0, float(np.dot(ex_top, np.array([1.0, 0.0, 0.0]))))))
    )

    # Right = smaller y, left = larger y
    dy = math.sqrt(3.0) / 2.0 * s
    hex_vertices = [
        ("A_top", np.array([x0, Yc, Zc + s], dtype=float)),
        ("B_right_upper", np.array([x0, Yc - dy, Zc + s / 2.0], dtype=float)),
        ("C_right_lower", np.array([x0, Yc - dy, Zc - s / 2.0], dtype=float)),
        ("D_bottom", np.array([x0, Yc, Zc - s], dtype=float)),
        ("E_left_lower", np.array([x0, Yc + dy, Zc - s / 2.0], dtype=float)),
        ("F_left_upper", np.array([x0, Yc + dy, Zc + s / 2.0], dtype=float)),
    ]

    print(f"六边形边长 s = {s * 1000.0:.6f} mm")
    print()
    print("=== 已知顶点 A (上尖) 正运动学结果: URDF/link_6 ===")
    print(f"x = {x0:.12f} m")
    print(f"y = {y0:.12f} m")
    print(f"z = {z0:.12f} m")
    print(f"roll  = {roll_top:.12f} rad")
    print(f"pitch = {pitch_top:.12f} rad")
    print(f"yaw   = {yaw_top:.12f} rad")
    print()
    print("末端轴方向（base_link下）:")
    print(f"+X = {ex_top}")
    print(f"+Y = {ey_top}")
    print(f"+Z = {ez_top}")
    print()
    print("误差指标（相对理想上尖姿态）:")
    print(f"+Z 与 -base_link.X 的夹角 = {z_angle_deg:.6f} deg")
    print(f"+X 偏出 hex 平面量 = {x_out_of_plane_deg:.6f} deg")
    print()
    print("六边形中心:")
    print(f"center = ({center[0]:.12f}, {center[1]:.12f}, {center[2]:.12f})")
    print()

    # --------------------------------------------------------
    # Standard method
    # --------------------------------------------------------
    # Ideal plane: parallel to base_link.yz
    # Ideal orientation:
    #   +Z = -base_link.X
    #   +X = outward radial direction
    # so that -X always points to the center.
    standard_results = []
    z_axis_ideal = np.array([-1.0, 0.0, 0.0], dtype=float)

    for name, p in hex_vertices:
        radial = p - center
        radial = radial / np.linalg.norm(radial)
        x_axis = radial
        y_axis = np.cross(z_axis_ideal, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        R = np.column_stack([x_axis, y_axis, z_axis_ideal])
        roll, pitch, yaw = rot_to_rpy(R)
        standard_results.append((name, (p[0], p[1], p[2], roll, pitch, yaw)))

    # --------------------------------------------------------
    # Error-preserving method
    # --------------------------------------------------------
    # Preserve the measured top pose error and rotate the whole pose
    # around base_link.X to the other vertices.
    rel_top = p_top - center
    vertex_angles_deg = {
        "A_top": 0.0,
        "B_right_upper": 60.0,
        "C_right_lower": 120.0,
        "D_bottom": 180.0,
        "E_left_lower": -120.0,
        "F_left_upper": -60.0,
    }

    error_results = []
    for name, _ in hex_vertices:
        theta = math.radians(vertex_angles_deg[name])
        R_rot = Rx(theta)
        p = center + R_rot @ rel_top
        R = R_rot @ R_top
        roll, pitch, yaw = rot_to_rpy(R)
        error_results.append((name, (p[0], p[1], p[2], roll, pitch, yaw)))

    print("=== 标准版 xyzrpy ===")
    for name, pose in standard_results:
        print(name, pose)
    print()

    print("=== 误差版 xyzrpy ===")
    for name, pose in error_results:
        print(name, pose)
    print()

    print("=== C++ PoseTarget: 标准版 ===")
    for name, pose in standard_results:
        format_pose_target(name, pose)

    print("=== C++ PoseTarget: 误差版 ===")
    for name, pose in error_results:
        format_pose_target(name, pose)


if __name__ == "__main__":
    main()  