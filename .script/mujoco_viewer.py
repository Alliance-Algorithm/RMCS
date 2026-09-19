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
    viewer = mujoco.viewer.launch_passive(model, data)
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

            viewer.sync()
            if period:
                stop.wait(period)
    finally:
        # GL/GLFW teardown can block indefinitely under some drivers, so skip
        # interpreter/context cleanup and terminate the process directly.
        os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
