#!/usr/bin/env python3
"""Draw a schematic side view for the engineer chassis in SVG.

This is intentionally a communication sketch driven by the current leg inverse
kinematics, not a CAD reconstruction.

Topology assumption used here after the user's correction:
- rear wheel branches from the lower node
- front wheel branches from the upper node
- the middle omni wheel also branches from the upper node
- link1 and link2 share the lower pivot and keep a fixed 120 degree angle
- the omni branch is 130 mm long and keeps a fixed 70 degree angle to link3

Because the omni branch is lateral in top view, the side view draws it as a
projected branch below the upper node.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, cos, pi, sin
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


LINK1 = 240.0
LINK2 = 120.0
LINK3 = 160.0
OMNI_BRANCH = 130.0
OMNI_LINK3_ANGLE = 70.0 * pi / 180.0
FIXED_LINK1_LINK2_ANGLE = 2.0 * pi / 3.0
WHEEL_DIAMETER = 110.0
WHEEL_RADIUS = WHEEL_DIAMETER * 0.5

WORLD_MIN_X = -380.0
WORLD_MAX_X = 380.0
WORLD_MIN_Z = -280.0
WORLD_MAX_Z = 220.0

PANEL_WIDTH = 720
PANEL_HEIGHT = 460
PANEL_PADDING = 36
SHEET_MARGIN = 28

SHEET_WIDTH = PANEL_WIDTH * 2 + SHEET_MARGIN * 3
SHEET_HEIGHT = PANEL_HEIGHT * 2 + SHEET_MARGIN * 3 + 64


@dataclass(frozen=True)
class Point:
    x: float
    z: float


@dataclass(frozen=True)
class Pose:
    title: str
    theta_f: float
    theta_b: float
    subtitle: str
    show_step: bool = False


def leg_inverse_kinematic(
    f_x: float,
    b_x: float,
    is_front_ecd_obtuse: bool = False,
    is_back_ecd_obtuse: bool = False,
) -> tuple[float, float]:
    theta_b = asin(b_x / LINK1)
    if is_back_ecd_obtuse:
        theta_b = pi - theta_b

    x_link2 = LINK2 * sin(FIXED_LINK1_LINK2_ANGLE - theta_b)
    theta_f = asin((f_x - x_link2) / LINK3)
    if is_front_ecd_obtuse:
        theta_f = pi - theta_f

    return theta_f, theta_b


def infer_single_side(theta_f: float, theta_b: float) -> dict[str, Point]:
    lower_joint = Point(0.0, 0.0)
    upper_joint = Point(
        LINK2 * sin(FIXED_LINK1_LINK2_ANGLE - theta_b),
        -LINK2 * cos(FIXED_LINK1_LINK2_ANGLE - theta_b),
    )
    rear = Point(-LINK1 * sin(theta_b), -LINK1 * cos(theta_b))
    front = Point(
        upper_joint.x + LINK3 * sin(theta_f),
        upper_joint.z - LINK3 * cos(theta_f),
    )
    link3_angle = atan2(front.z - upper_joint.z, front.x - upper_joint.x)
    omni_angle = link3_angle - OMNI_LINK3_ANGLE
    omni = Point(
        upper_joint.x + OMNI_BRANCH * cos(omni_angle),
        upper_joint.z + OMNI_BRANCH * sin(omni_angle),
    )
    return {
        "lower_joint": lower_joint,
        "upper_joint": upper_joint,
        "rear": rear,
        "omni": omni,
        "front": front,
    }


def svg(tag: str, **attrs: object) -> ET.Element:
    return ET.Element(tag, {key.replace("_", "-"): str(value) for key, value in attrs.items()})


def add(parent: ET.Element, element: ET.Element) -> ET.Element:
    parent.append(element)
    return element


def fmt(value: float) -> str:
    return f"{value:.2f}"


def midpoint(a: Point, b: Point) -> Point:
    return Point((a.x + b.x) * 0.5, (a.z + b.z) * 0.5)


def panel_transform(panel_x: float, panel_y: float) -> tuple[float, float, float]:
    span_x = WORLD_MAX_X - WORLD_MIN_X
    span_z = WORLD_MAX_Z - WORLD_MIN_Z
    scale_x = (PANEL_WIDTH - PANEL_PADDING * 2) / span_x
    scale_z = (PANEL_HEIGHT - PANEL_PADDING * 2) / span_z
    scale = min(scale_x, scale_z)
    origin_x = panel_x + PANEL_PADDING
    origin_y = panel_y + PANEL_HEIGHT - PANEL_PADDING
    return origin_x, origin_y, scale


def to_svg(point: Point, panel_x: float, panel_y: float) -> tuple[float, float]:
    origin_x, origin_y, scale = panel_transform(panel_x, panel_y)
    x = origin_x + (point.x - WORLD_MIN_X) * scale
    y = origin_y - (point.z - WORLD_MIN_Z) * scale
    return x, y


def add_text(
    parent: ET.Element,
    x: float,
    y: float,
    text: str,
    *,
    font_size: int = 18,
    fill: str = "#13212f",
    anchor: str = "start",
    weight: str = "400",
) -> None:
    node = add(
        parent,
        svg(
            "text",
            x=fmt(x),
            y=fmt(y),
            fill=fill,
            font_size=font_size,
            font_family="DejaVu Sans Mono, monospace",
            text_anchor=anchor,
            font_weight=weight,
        ),
    )
    node.text = text


def add_line(
    parent: ET.Element,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    stroke: str = "#243848",
    width: float = 5.0,
    dash: str | None = None,
) -> None:
    attrs = {
        "x1": fmt(start[0]),
        "y1": fmt(start[1]),
        "x2": fmt(end[0]),
        "y2": fmt(end[1]),
        "stroke": stroke,
        "stroke_width": fmt(width),
        "stroke_linecap": "round",
    }
    if dash:
        attrs["stroke_dasharray"] = dash
    add(parent, svg("line", **attrs))


def add_circle(
    parent: ET.Element,
    center: tuple[float, float],
    radius: float,
    *,
    fill: str,
    stroke: str,
    width: float = 4.0,
) -> None:
    add(
        parent,
        svg(
            "circle",
            cx=fmt(center[0]),
            cy=fmt(center[1]),
            r=fmt(radius),
            fill=fill,
            stroke=stroke,
            stroke_width=fmt(width),
        ),
    )


def add_rect(
    parent: ET.Element,
    x: float,
    y: float,
    width: float,
    height: float,
    *,
    fill: str,
    stroke: str = "none",
    stroke_width: float = 0.0,
    radius: float = 0.0,
) -> None:
    add(
        parent,
        svg(
            "rect",
            x=fmt(x),
            y=fmt(y),
            width=fmt(width),
            height=fmt(height),
            rx=fmt(radius),
            ry=fmt(radius),
            fill=fill,
            stroke=stroke,
            stroke_width=fmt(stroke_width),
        ),
    )


def draw_ground(parent: ET.Element, panel_x: float, panel_y: float) -> None:
    start = to_svg(Point(WORLD_MIN_X + 8.0, -WHEEL_RADIUS * 2.0), panel_x, panel_y)
    end = to_svg(Point(WORLD_MAX_X - 8.0, -WHEEL_RADIUS * 2.0), panel_x, panel_y)
    add_line(parent, start, end, stroke="#8697a5", width=3.0)


def draw_step(parent: ET.Element, panel_x: float, panel_y: float) -> None:
    left_top = Point(175.0, -130.0)
    right_bottom = Point(350.0, -WHEEL_RADIUS * 2.0)
    top_left = to_svg(left_top, panel_x, panel_y)
    bottom_right = to_svg(right_bottom, panel_x, panel_y)
    add_rect(
        parent,
        top_left[0],
        top_left[1],
        bottom_right[0] - top_left[0],
        bottom_right[1] - top_left[1],
        fill="#d7d7db",
        stroke="#9c9ca3",
        stroke_width=2.0,
        radius=6.0,
    )
    add_text(parent, top_left[0] + 10.0, top_left[1] - 10.0, "stair", font_size=14, fill="#6c6c70")


def draw_dimension_label(
    parent: ET.Element,
    panel_x: float,
    panel_y: float,
    a: Point,
    b: Point,
    label: str,
    color: str,
) -> None:
    p = to_svg(midpoint(a, b), panel_x, panel_y)
    add_text(parent, p[0], p[1] - 12.0, label, font_size=14, fill=color, anchor="middle")


def draw_fixed_angle_note(
    parent: ET.Element,
    lower_joint_svg: tuple[float, float],
) -> None:
    note_anchor = (lower_joint_svg[0] + 20.0, lower_joint_svg[1] - 26.0)
    add_line(parent, lower_joint_svg, note_anchor, stroke="#8c2f39", width=2.5)
    add_text(
        parent,
        note_anchor[0] + 8.0,
        note_anchor[1] - 4.0,
        "fixed 120 deg",
        font_size=13,
        fill="#8c2f39",
        weight="700",
    )
    add_text(
        parent,
        note_anchor[0] + 8.0,
        note_anchor[1] + 14.0,
        "link1 + link2 rigid",
        font_size=12,
        fill="#8c2f39",
    )


def draw_upper_branch_note(
    parent: ET.Element,
    upper_joint_svg: tuple[float, float],
) -> None:
    note_anchor = (upper_joint_svg[0] + 16.0, upper_joint_svg[1] - 38.0)
    add_line(parent, upper_joint_svg, note_anchor, stroke="#8b5a2b", width=2.5)
    add_text(
        parent,
        note_anchor[0] + 8.0,
        note_anchor[1] - 4.0,
        "omni = 130",
        font_size=13,
        fill="#8b5a2b",
        weight="700",
    )
    add_text(
        parent,
        note_anchor[0] + 8.0,
        note_anchor[1] + 14.0,
        "angle to link3 = 70 deg",
        font_size=12,
        fill="#8b5a2b",
    )


def draw_pose(parent: ET.Element, pose: Pose, panel_x: float, panel_y: float) -> None:
    add_rect(
        parent,
        panel_x,
        panel_y,
        PANEL_WIDTH,
        PANEL_HEIGHT,
        fill="#f7f8fa",
        stroke="#d7dde3",
        stroke_width=2.0,
        radius=18.0,
    )

    add_text(parent, panel_x + 24.0, panel_y + 34.0, pose.title, font_size=26, weight="700")
    add_text(parent, panel_x + 24.0, panel_y + 60.0, pose.subtitle, font_size=14, fill="#526272")

    if pose.show_step:
        draw_step(parent, panel_x, panel_y)
    draw_ground(parent, panel_x, panel_y)

    pts = infer_single_side(pose.theta_f, pose.theta_b)
    lower_joint = pts["lower_joint"]
    upper_joint = pts["upper_joint"]
    rear = pts["rear"]
    omni = pts["omni"]
    front = pts["front"]

    rear_svg = to_svg(rear, panel_x, panel_y)
    lower_joint_svg = to_svg(lower_joint, panel_x, panel_y)
    upper_joint_svg = to_svg(upper_joint, panel_x, panel_y)
    omni_svg = to_svg(omni, panel_x, panel_y)
    front_svg = to_svg(front, panel_x, panel_y)

    body_bottom = to_svg(Point(-36.0, 110.0), panel_x, panel_y)
    body_top = to_svg(Point(155.0, 165.0), panel_x, panel_y)
    add_rect(
        parent,
        body_bottom[0],
        body_top[1],
        body_top[0] - body_bottom[0],
        body_bottom[1] - body_top[1],
        fill="#bfd5ea",
        stroke="#4e7192",
        stroke_width=3.0,
        radius=12.0,
    )
    add_text(
        parent,
        (body_bottom[0] + body_top[0]) * 0.5,
        body_top[1] + 30.0,
        "chassis / arm base",
        font_size=15,
        fill="#264863",
        anchor="middle",
        weight="700",
    )
    add_line(
        parent,
        upper_joint_svg,
        ((body_bottom[0] + body_top[0]) * 0.5, body_bottom[1]),
        stroke="#4e7192",
        width=5.0,
    )

    add_line(parent, lower_joint_svg, rear_svg, stroke="#0d5a8a", width=8.0)
    add_line(parent, lower_joint_svg, upper_joint_svg, stroke="#d97706", width=8.0)
    add_line(parent, upper_joint_svg, front_svg, stroke="#0d5a8a", width=8.0)
    add_line(parent, upper_joint_svg, omni_svg, stroke="#8b5a2b", width=6.0, dash="10 8")
    draw_fixed_angle_note(parent, lower_joint_svg)
    draw_upper_branch_note(parent, upper_joint_svg)

    joint_radius = 8.0
    add_circle(parent, lower_joint_svg, joint_radius, fill="#ffffff", stroke="#243848", width=3.0)
    add_circle(parent, upper_joint_svg, joint_radius, fill="#ffffff", stroke="#243848", width=3.0)

    wheel_scale = panel_transform(panel_x, panel_y)[2]
    wheel_r_px = WHEEL_RADIUS * wheel_scale
    for name, center, fill, stroke in [
        ("rear wheel", rear_svg, "#20262d", "#495764"),
        ("middle omni (projected)", omni_svg, "#6b3b13", "#b86c28"),
        ("front wheel", front_svg, "#20262d", "#495764"),
    ]:
        add_circle(parent, center, wheel_r_px, fill=fill, stroke=stroke, width=5.0)
        add_circle(parent, center, wheel_r_px * 0.36, fill="#dfe5ea", stroke="#778491", width=3.0)
        add_text(parent, center[0], center[1] + wheel_r_px + 24.0, name, font_size=13, fill="#485865", anchor="middle")

    draw_dimension_label(parent, panel_x, panel_y, lower_joint, rear, "link1 = 240", "#0d5a8a")
    draw_dimension_label(
        parent, panel_x, panel_y, lower_joint, upper_joint, "link2 = 120", "#d97706"
    )
    draw_dimension_label(parent, panel_x, panel_y, upper_joint, front, "link3 = 160", "#0d5a8a")
    draw_dimension_label(
        parent, panel_x, panel_y, upper_joint, omni, "pivot to omni = 130", "#8b5a2b"
    )

    angle_text = f"theta_f={pose.theta_f * 180.0 / pi:5.1f} deg, theta_b={pose.theta_b * 180.0 / pi:5.1f} deg"
    add_text(parent, panel_x + 24.0, panel_y + PANEL_HEIGHT - 18.0, angle_text, font_size=14, fill="#526272")


def iter_panels(poses: Iterable[Pose]) -> Iterable[tuple[Pose, float, float]]:
    positions = [
        (SHEET_MARGIN, SHEET_MARGIN + 64.0),
        (SHEET_MARGIN * 2 + PANEL_WIDTH, SHEET_MARGIN + 64.0),
        (SHEET_MARGIN, SHEET_MARGIN * 2 + PANEL_HEIGHT + 64.0),
        (SHEET_MARGIN * 2 + PANEL_WIDTH, SHEET_MARGIN * 2 + PANEL_HEIGHT + 64.0),
    ]
    for pose, (panel_x, panel_y) in zip(poses, positions):
        yield pose, panel_x, panel_y


def build_svg() -> ET.Element:
    root = svg(
        "svg",
        xmlns="http://www.w3.org/2000/svg",
        width=SHEET_WIDTH,
        height=SHEET_HEIGHT,
        viewBox=f"0 0 {SHEET_WIDTH} {SHEET_HEIGHT}",
    )

    add_rect(root, 0, 0, SHEET_WIDTH, SHEET_HEIGHT, fill="#ffffff")
    add_text(root, SHEET_MARGIN, 44.0, "Engineer Chassis Side View Sketch", font_size=32, weight="700")
    add_text(
        root,
        SHEET_MARGIN,
        70.0,
        "Schematic side view. Lower pivot is shared by link1 and link2, with a fixed 120 deg angle.",
        font_size=15,
        fill="#526272",
    )

    four_theta_f, four_theta_b = leg_inverse_kinematic(242.0, 458.0 - 242.0)
    six_theta_f, six_theta_b = leg_inverse_kinematic(250.0, 221.0)

    poses = [
        Pose("Four Wheel", four_theta_f, four_theta_b, "corner wheels preferred, middle omni output disabled"),
        Pose("Six Wheel", six_theta_f, six_theta_b, "all three wheel stations shown on one side"),
        Pose("Up Stairs: Initial", 1.600270, 1.550000, "pre-load pose before climbing", True),
        Pose("Up Stairs: Lift", 1.002222, 1.453787, "front section lifted toward the stair", True),
    ]

    for pose, panel_x, panel_y in iter_panels(poses):
        draw_pose(root, pose, panel_x, panel_y)

    add_text(
        root,
        SHEET_MARGIN,
        SHEET_HEIGHT - 16.0,
        "The middle omni wheel is drawn as a side-view projection from the upper node branch.",
        font_size=14,
        fill="#6b7784",
    )
    return root


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent
    output_path = repo_root / "ENGINEER_CHASSIS_SIDE_VIEW.svg"

    root = build_svg()
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()
