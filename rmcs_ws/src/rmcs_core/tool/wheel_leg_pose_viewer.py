#!/usr/bin/env python3
"""Display real wheel-leg telemetry in MuJoCo without sending control commands."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import socket
import subprocess
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from wheel_leg_mujoco_pose import BODY_FROM_RL, PoseModel
from wheel_leg_telemetry import FoxgloveReader, LatestSnapshot, Snapshot


class LocalReader(threading.Thread):
    def __init__(self, latest, replay=None):
        super().__init__(daemon=True)
        self.latest, self.replay = latest, replay
        self.stopped = threading.Event()

    def run(self):
        try:
            if self.replay:
                with open(self.replay) as source:
                    previous = None
                    for line in source:
                        value = json.loads(line)
                        stamp = value.get("record_time_s", value["stamp_ns"] * 1e-9)
                        if previous is not None and self.stopped.wait(max(0., stamp - previous)):
                            return
                        if self.stopped.is_set():
                            return
                        self.latest.put(Snapshot.from_dict(value), source=value.get("source", "unknown"))
                        previous = stamp
                self.latest.set_status("replay complete; showing final recorded pose")
                return
            start = time.monotonic()
            while not self.stopped.is_set():
                t = time.monotonic() - start
                r, p, y = .2 * np.sin(.7 * t), .12 * np.sin(.5 * t), .25 * np.sin(.2 * t)
                rd, pd, yd = .14 * np.cos(.7 * t), .06 * np.cos(.5 * t), .05 * np.cos(.2 * t)
                body = Rotation.from_euler("xyz", [r, p, y])
                gyro = np.array([rd - yd * np.sin(p), pd * np.cos(r) + yd * np.sin(r) * np.cos(p),
                                 -pd * np.sin(r) + yd * np.cos(r) * np.cos(p)])
                angle = .2 * np.sin(t)
                positions = (.42 + angle, -.13742282595254576 + angle,
                             -.42 - angle, .13741557625658019 - angle, t, -t)
                self.latest.put(Snapshot(time.time_ns(), positions, tuple(body.as_quat()[[3, 0, 1, 2]]),
                                         tuple(gyro), tuple((body * BODY_FROM_RL).inv().apply([0., 0., -1.])),
                                         tuple(BODY_FROM_RL.inv().apply(gyro))))
                self.stopped.wait(.02)
        except Exception as error:
            self.latest.set_status(f"source failed: {error}")

    def close(self):
        self.stopped.set()
        self.join(timeout=3)


@contextmanager
def ssh_tunnel(host, ssh_port, remote_port, local_port):
    if host.startswith("-"):
        raise ValueError("invalid SSH destination")
    if not local_port:
        with socket.socket() as reserved:
            reserved.bind(("127.0.0.1", 0))
            local_port = reserved.getsockname()[1]
    command = ["ssh", "-N", "-T", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
               "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=2",
               "-p", str(ssh_port), "-L", f"127.0.0.1:{local_port}:127.0.0.1:{remote_port}", host]
    process = subprocess.Popen(command)
    try:
        deadline = time.monotonic() + 10.
        while True:
            if process.poll() is not None:
                raise RuntimeError("SSH tunnel failed; check the SSH message and authenticate with ssh first")
            try:
                with socket.create_connection(("127.0.0.1", local_port), timeout=.1):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise RuntimeError("SSH forwarding did not become ready")
                time.sleep(.1)
        yield f"ws://127.0.0.1:{local_port}"
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def save_image(pose, destination, text):
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
    pose.model.vis.global_.offwidth = 1280
    pose.model.vis.global_.offheight = 900
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0., 0., pose.root_height]
    camera.distance, camera.azimuth, camera.elevation = 1.65, 135., -20.
    with mujoco.Renderer(pose.model, height=900, width=1280) as renderer:
        renderer.update_scene(pose.data, camera=camera)
        pose.draw_axes(renderer.scene)
        result = Image.fromarray(renderer.render().copy())
    drawing = ImageDraw.Draw(result)
    drawing.rectangle((0, 0, 1280, 180), fill=(24, 28, 33))
    font = ImageFont.truetype("DejaVuSans.ttf", 16)
    drawing.multiline_text((15, 10), text, fill="white", spacing=3, font=font)
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    result.save(destination)


def run(args, url=None):
    import mujoco
    from contextlib import nullcontext
    pose = PoseModel(args.bundle, args.root_height)
    latest = LatestSnapshot(args.record, source="demo" if args.demo else "unknown" if args.replay else "foxglove")
    reader = FoxgloveReader(url, latest) if url else LocalReader(latest, args.replay)
    viewer_context = nullcontext(None)
    if not args.headless:
        import mujoco.viewer
        viewer_context = mujoco.viewer.launch_passive(pose.model, pose.data,
                                                       show_left_ui=False, show_right_ui=False)
    reader.start()
    started, last_stamp, accepted, rejected = time.monotonic(), None, 0, 0
    error, last_text, max_gap = "", "waiting", 0.
    try:
        with viewer_context as viewer:
            if viewer:
                viewer.cam.lookat[:] = [0., 0., args.root_height]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.65, 135., -20.
                viewer.opt.geomgroup[3] = 0
            while (viewer is None or viewer.is_running()) and (
                    args.duration is None or time.monotonic() - started < args.duration):
                snapshot, age, status = latest.get()
                if snapshot is not None and snapshot.stamp_ns != last_stamp:
                    last_stamp = snapshot.stamp_ns
                    try:
                        pose.set_snapshot(snapshot)
                        accepted += 1
                        max_gap = max(max_gap, pose.metrics["loop_gap_m"])
                        error = ""
                    except ValueError as failure:
                        rejected += 1
                        error = str(failure)
                last_text = pose.text(status, age, error, args.demo)
                if args.replay:
                    label = "REPLAY: SYNTHETIC DEMO" if latest.source == "demo" else f"REPLAY: {latest.source}"
                    last_text = last_text.replace("LIVE TELEMETRY", label)
                if viewer:
                    viewer.set_texts((None, None, last_text, ""))
                    with viewer.lock():
                        viewer.user_scn.ngeom = 0
                        pose.draw_axes(viewer.user_scn)
                    viewer.sync()
                time.sleep(1. / args.fps)
    finally:
        reader.close()
        latest.close()
    report = {"source": "demo" if args.demo else "replay" if args.replay else "foxglove",
              "samples_received": latest.count, "poses_displayed": accepted, "poses_rejected": rejected,
              "max_loop_gap_m": max_gap, "last_metrics": pose.metrics, "status": latest.get()[2]}
    if args.report:
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
    if args.screenshot and accepted:
        save_image(pose, args.screenshot, last_text)
    print(json.dumps(report, indent=2))
    return 0 if accepted and rejected == 0 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True, help="V5 bundle containing robot.xml and manifest.json")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--url", help="Foxglove bridge URL, e.g. ws://127.0.0.1:8765")
    source.add_argument("--ssh", help="SSH user@host; forward remote localhost Foxglove to a local loopback port")
    source.add_argument("--replay", type=Path, help="Replay a JSONL recording")
    source.add_argument("--demo", action="store_true", help="Explicitly labeled synthetic motion for setup testing")
    parser.add_argument("--ssh-port", type=int, default=22)
    parser.add_argument("--remote-port", type=int, default=8765)
    parser.add_argument("--local-port", type=int, default=0)
    parser.add_argument("--root-height", type=float, default=.65, help="Display height only; no world-position estimate")
    parser.add_argument("--fps", type=float, default=30.)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--record", type=Path, help="Write new JSONL file; existing files are never overwritten")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--screenshot", type=Path)
    args = parser.parse_args()
    if not 0. < args.fps <= 200. or not np.isfinite(args.root_height):
        parser.error("fps must be in (0,200] and root height must be finite")
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0.):
        parser.error("duration must be finite and positive")
    if args.headless and args.duration is None:
        parser.error("headless mode requires --duration")
    if args.ssh:
        with ssh_tunnel(args.ssh, args.ssh_port, args.remote_port, args.local_port) as url:
            return run(args, url)
    return run(args, args.url)


if __name__ == "__main__":
    raise SystemExit(main())
