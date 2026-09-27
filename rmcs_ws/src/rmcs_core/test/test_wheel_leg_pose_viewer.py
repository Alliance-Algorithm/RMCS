"""Offline transport, coordinate and closed-chain tests; never opens a robot connection."""
import asyncio
import json
import os
from pathlib import Path
import struct
import sys
import unittest

import numpy as np
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation
import websockets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tool"))
from wheel_leg_telemetry import FoxgloveReader, LatestSnapshot, MOTOR_NAMES, Snapshot, SnapshotAssembler, TOPICS
from wheel_leg_mujoco_pose import BODY_FROM_RL, PoseModel

TYPES = get_typestore(Stores.ROS2_JAZZY)


def reference_snapshot(stamp=1_000_000_123):
    body = Rotation.from_euler("xyz", [25., -20., 70.], degrees=True)
    return Snapshot(stamp, (-1.6, -2.93, 1.6, 2.93, .2, -.3),
                    tuple(body.as_quat()[[3, 0, 1, 2]]), (.7, -1.2, 2.3),
                    tuple((body * BODY_FROM_RL).inv().apply([0., 0., -1.])), (-1.2, -.7, 2.3))


def messages(snapshot):
    types = TYPES.types
    stamp = types["builtin_interfaces/msg/Time"](snapshot.stamp_ns // 1_000_000_000,
                                                snapshot.stamp_ns % 1_000_000_000)
    header = types["std_msgs/msg/Header"](stamp, "rl_base")
    body_header = types["std_msgs/msg/Header"](stamp, "chassis_body")
    vec = types["geometry_msgs/msg/Vector3"]
    q = snapshot.quaternion_wxyz
    # Deliberately reverse JointState ordering to detect index-based consumers.
    joints = types["sensor_msgs/msg/JointState"](header, list(MOTOR_NAMES[::-1]),
              np.array(snapshot.positions[::-1]), np.zeros(6), np.zeros(6))
    imu = types["sensor_msgs/msg/Imu"](body_header, types["geometry_msgs/msg/Quaternion"](q[1], q[2], q[3], q[0]),
              np.zeros(9), vec(*snapshot.gyro_body), np.zeros(9), vec(0., 0., 0.), np.zeros(9))
    stamped = types["geometry_msgs/msg/Vector3Stamped"]
    return dict(zip(TOPICS, (joints, imu, stamped(header, vec(*snapshot.gravity_rl)),
                            stamped(header, vec(*snapshot.gyro_rl)))))


class AssemblyTest(unittest.TestCase):
    def test_exact_sample_alignment_and_joint_name_mapping(self):
        assembler = SnapshotAssembler()
        first, second = messages(reference_snapshot()), messages(reference_snapshot(2_000_000_000))
        topics = list(TOPICS)
        self.assertIsNone(assembler.accept(topics[0], first[topics[0]]))
        for topic in topics[1:]:
            self.assertIsNone(assembler.accept(topic, second[topic]))
        complete = assembler.accept(topics[0], second[topics[0]])
        self.assertEqual(complete.positions, reference_snapshot().positions)
        self.assertEqual(complete.stamp_ns, 2_000_000_000)
        # Old or out-of-order messages cannot rewind the visualization.
        for topic in topics:
            self.assertIsNone(assembler.accept(topic, first[topic]))

    def test_invalid_orientation_is_not_replaced_by_identity(self):
        values = reference_snapshot().__dict__ | {"quaternion_wxyz": (0., 0., 0., 0.)}
        with self.assertRaises(ValueError):
            Snapshot.from_dict(values)

    def test_reject_wrong_imu_coordinate_frame(self):
        assembler = SnapshotAssembler()
        msg = messages(reference_snapshot())["/wheel_leg/telemetry/imu_body"]
        msg.header.frame_id = "rl_base"
        with self.assertRaises(ValueError):
            assembler.accept("/wheel_leg/telemetry/imu_body", msg)


class FoxgloveTest(unittest.IsolatedAsyncioTestCase):
    async def test_real_cdr_packets_and_subscribe_only_client(self):
        received_operations = []
        expected = reference_snapshot()
        packets = messages(expected)

        async def serve(socket, _path=None):
            await socket.send(json.dumps({"op": "serverInfo", "name": "offline test", "capabilities": []}))
            await socket.send(json.dumps({"op": "advertise", "channels": [
                {"id": i, "topic": topic, "encoding": "cdr", "schemaName": TOPICS[topic][1], "schema": ""}
                for i, topic in enumerate(TOPICS, 1)]}))
            request = json.loads(await socket.recv())
            received_operations.append(request["op"])
            self.assertEqual(request["op"], "subscribe")
            self.assertEqual(len(request["subscriptions"]), 4)
            for sub in request["subscriptions"][::-1]:
                topic = list(TOPICS)[sub["channelId"] - 1]
                payload = TYPES.serialize_cdr(packets[topic], TOPICS[topic][1])
                await socket.send(struct.pack("<BIQ", 1, sub["id"], expected.stamp_ns) + bytes(payload))
            try:
                while True:
                    received_operations.append(json.loads(await socket.recv())["op"])
            except websockets.ConnectionClosed:
                pass

        latest = LatestSnapshot()
        async with websockets.serve(serve, "127.0.0.1", 0, subprotocols=["foxglove.websocket.v1"]) as server:
            reader = FoxgloveReader(f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}", latest)
            reader.start()
            try:
                for _ in range(100):
                    if latest.get()[0] is not None:
                        break
                    await asyncio.sleep(.02)
                actual = latest.get()[0]
                self.assertIsNotNone(actual, latest.get()[2])
                self.assertEqual(actual.positions, expected.positions)
                self.assertEqual(actual.quaternion_wxyz, expected.quaternion_wxyz)
            finally:
                await asyncio.to_thread(reader.close)
        self.assertEqual(received_operations, ["subscribe"])


@unittest.skipUnless(os.environ.get("WHEEL_LEG_MODEL_BUNDLE"), "set WHEEL_LEG_MODEL_BUNDLE for the V5 geometry test")
class GeometryTest(unittest.TestCase):
    def test_real_logged_phases_and_frame_conversion(self):
        model = PoseModel(os.environ["WHEEL_LEG_MODEL_BUNDLE"])
        captured = np.radians([-99.421532, 179.194987, 89.957478, 168.761840, 0., 0.])
        for position in (reference_snapshot().positions, captured,
                         captured + [2*np.pi, -2*np.pi, -2*np.pi, 2*np.pi, 0., 0.]):
            snapshot = Snapshot.from_dict(reference_snapshot().__dict__ | {"positions": tuple(position)})
            metric = model.set_snapshot(snapshot)
            self.assertLess(metric["loop_gap_m"], 2e-6)
            self.assertLess(metric["motor_phase_error_rad"], 1e-12)
            self.assertLess(metric["gravity_component_error"], 1e-12)
            self.assertLess(metric["gyro_error_rad_s"], 1e-12)
            self.assertEqual(model.data.time, 0.)
        # A flipped gravity convention remains visible as a mismatch.
        bad = Snapshot.from_dict(snapshot.__dict__ | {"gravity_rl": tuple(-np.array(snapshot.gravity_rl))})
        self.assertAlmostEqual(model.set_snapshot(bad)["gravity_error_deg"], 180., places=5)


if __name__ == "__main__":
    unittest.main()
