#!/usr/bin/env python3
"""Passive MuJoCo viewer mirroring the in-process simulation bridge.

The rmcs_core::hardware::WheelLegSimulation component steps MuJoCo headless.
This script loads the same MJCF in a separate process, subscribes to the raw
state it publishes ([qpos, qvel]) and feeds it into a passive MuJoCo viewer, so
you can watch the *controlled* simulation.

Run it through .script/launch-mujoco-viewer (which wires up rclpy + the MuJoCo
virtualenv), not directly. Close the window to exit.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import signal
import sys
import threading

import mujoco
import mujoco.viewer
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

DEFAULT_TOPIC = "/wheelleg/simulation/state"
DEFAULT_MODEL = os.path.join(
    os.environ.get("RMCS_PATH", "/workspaces/RMCS"),
    "rmcs_ws/src/rmcs_core/models/wheelleg/wheelleg.xml",
)


class StateBridge(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("wheelleg_mujoco_viewer")
        self.latest: list[float] | None = None
        self.create_subscription(Float64MultiArray, topic, self._on_state, 1)
        self.get_logger().info(f"mirroring '{topic}' into the MuJoCo viewer")

    def _on_state(self, message: Float64MultiArray) -> None:
        self.latest = message.data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", nargs="?", default=DEFAULT_MODEL, help="path to the MJCF")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="state topic to mirror")
    parser.add_argument("--rate", type=float, default=60.0, help="viewer refresh rate [Hz]")
    parser.add_argument(
        "--track-body",
        default="",
        help="body the camera follows (empty = static free camera)",
    )
    parser.add_argument("--distance", type=float, default=2.5, help="camera distance [m]")
    parser.add_argument("--azimuth", type=float, default=90.0, help="camera azimuth [deg]")
    parser.add_argument("--elevation", type=float, default=-20.0, help="camera elevation [deg]")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    model_path = pathlib.Path(args.model)
    if not model_path.is_file():
        print(f"ERROR: model not found: {model_path}", file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # Closing the window is the normal exit; also react to Ctrl-C / kill.
    stop = threading.Event()

    def request_stop(signum, _frame) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rclpy.init()
    bridge = StateBridge(args.topic)

    def spin() -> None:
        try:
            rclpy.spin(bridge)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    threading.Thread(target=spin, daemon=True).start()

    mismatched = False
    bridge.get_logger().info(
        f"[viewer args] topic={args.topic} track_body={args.track_body!r} "
        f"distance={args.distance} azimuth={args.azimuth} elevation={args.elevation}")
    viewer = mujoco.viewer.launch_passive(model, data)

    # Lock the camera onto a body so it follows the (moving) vehicle at a fixed
    # distance. Distance/azimuth/elevation stay mouse-adjustable; only the
    # tracking target is re-asserted each frame.
    track_body_id = (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, args.track_body)
        if args.track_body
        else -1
    )
    if track_body_id >= 0:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = track_body_id
        viewer.cam.distance = args.distance
        viewer.cam.azimuth = args.azimuth
        viewer.cam.elevation = args.elevation
        bridge.get_logger().info(
            f"camera TRACKING body '{args.track_body}' (follow)")
    else:
        # Explicitly pin a static free camera (viewer_follow=false / no track
        # body): do not re-assert it every frame so it stays put.
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.fixedcamid = -1
        viewer.cam.trackbodyid = -1
        bridge.get_logger().info(
            f"camera FREE (static); track-body was {args.track_body!r}")

    period = 1.0 / args.rate if args.rate > 0.0 else 0.0
    try:
        while viewer.is_running() and not stop.is_set():
            frame = bridge.latest
            if frame is not None:
                expected = model.nq + model.nv
                if len(frame) == expected:
                    data.qpos[:] = frame[: model.nq]
                    data.qvel[:] = frame[model.nq :]
                    mujoco.mj_forward(model, data)
                elif not mismatched:
                    bridge.get_logger().warn(
                        f"state size {len(frame)} != nq+nv {expected}; ignoring"
                    )
                    mismatched = True

            if track_body_id >= 0:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = track_body_id

            viewer.sync()
            if period:
                stop.wait(period)
    finally:
        # GL/GLFW teardown can block indefinitely under some drivers, so skip
        # interpreter/context cleanup and terminate the process directly.
        os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
