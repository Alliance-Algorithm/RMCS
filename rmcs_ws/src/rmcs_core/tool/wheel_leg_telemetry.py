"""Read-only Foxglove subscriber and exact-timestamp wheel-leg snapshots."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import struct
import threading
import time

MOTOR_NAMES = ("left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint",
               "left_wheel", "right_wheel")
TOPICS = {
    "/wheel_leg/telemetry/joint_states": ("joints", "sensor_msgs/msg/JointState", "rl_base"),
    "/wheel_leg/telemetry/imu_body": ("imu", "sensor_msgs/msg/Imu", "chassis_body"),
    "/wheel_leg/telemetry/rl_projected_gravity": ("gravity", "geometry_msgs/msg/Vector3Stamped", "rl_base"),
    "/wheel_leg/telemetry/rl_angular_velocity": ("gyro", "geometry_msgs/msg/Vector3Stamped", "rl_base"),
}
SUBPROTOCOLS = ("foxglove.sdk.v1", "foxglove.websocket.v1")


@dataclass(frozen=True)
class Snapshot:
    stamp_ns: int
    positions: tuple[float, ...]
    quaternion_wxyz: tuple[float, ...]
    gyro_body: tuple[float, ...]
    gravity_rl: tuple[float, ...]
    gyro_rl: tuple[float, ...]
    velocities: tuple[float, ...] = ()

    def validate(self):
        for name, count in (("positions", 6), ("quaternion_wxyz", 4), ("gyro_body", 3),
                            ("gravity_rl", 3), ("gyro_rl", 3)):
            values = getattr(self, name)
            if len(values) != count or not all(math.isfinite(v) for v in values):
                raise ValueError(f"invalid {name}")
        norm = sum(x * x for x in self.quaternion_wxyz)
        if not .99 < norm < 1.01:
            raise ValueError(f"IMU quaternion norm squared {norm:.5g} is not near one")
        if not .95 < sum(x * x for x in self.gravity_rl) < 1.05:
            raise ValueError("RL gravity is not a unit vector")
        if self.stamp_ns <= 0:
            raise ValueError("zero/negative sample timestamp")
        return self

    @classmethod
    def from_dict(cls, value):
        fields = {k: value[k] for k in cls.__dataclass_fields__ if k in value}
        return cls(**fields).validate()


def vector(value):
    return (float(value.x), float(value.y), float(value.z))


class SnapshotAssembler:
    """Never combine IMU and joints from different executor samples."""

    def __init__(self):
        self.pending = {}
        self.last_stamp = 0

    def accept(self, topic, message):
        field, _, expected_frame = TOPICS[topic]
        if message.header.frame_id != expected_frame:
            raise ValueError(f"{topic}: expected frame {expected_frame}, got {message.header.frame_id}")
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp <= self.last_stamp:
            return None
        bucket = self.pending.setdefault(stamp, {})
        bucket[field] = message
        while len(self.pending) > 8:
            del self.pending[min(self.pending)]
        if len(bucket) != 4:
            return None
        joints, imu = bucket["joints"], bucket["imu"]
        if len(set(joints.name)) != len(joints.name) or len(joints.position) != len(joints.name):
            raise ValueError("duplicate joint names or incomplete joint positions")
        index = [list(joints.name).index(name) for name in MOTOR_NAMES]
        velocity = tuple(float(joints.velocity[i]) for i in index) if len(joints.velocity) == len(joints.name) else ()
        q = imu.orientation
        result = Snapshot(stamp, tuple(float(joints.position[i]) for i in index),
                          (float(q.w), float(q.x), float(q.y), float(q.z)),
                          vector(imu.angular_velocity), vector(bucket["gravity"].vector),
                          vector(bucket["gyro"].vector), velocity).validate()
        self.last_stamp = stamp
        self.pending = {key: val for key, val in self.pending.items() if key > stamp}
        return result


class LatestSnapshot:
    def __init__(self, record=None, source="foxglove"):
        self.lock = threading.Lock()
        self.snapshot = None
        self.received = 0.0
        self.status = "waiting for telemetry"
        self.started = time.monotonic()
        self.record = open(record, "x") if record else None
        self.source = source
        self.count = 0

    def put(self, snapshot, source=None):
        snapshot.validate()
        with self.lock:
            self.snapshot, self.received = snapshot, time.monotonic()
            self.status = "receiving"
            if source is not None:
                self.source = source
            self.count += 1
            if self.record:
                self.record.write(json.dumps(asdict(snapshot) | {
                    "record_time_s": self.received - self.started, "source": self.source}) + "\n")
                self.record.flush()

    def set_status(self, status):
        with self.lock:
            self.status = status

    def get(self):
        with self.lock:
            return self.snapshot, time.monotonic() - self.received, self.status

    def close(self):
        if self.record:
            self.record.close()


class FoxgloveReader(threading.Thread):
    """Only emits subscribe operations; has no publish/service path."""

    def __init__(self, url, latest):
        super().__init__(daemon=True)
        self.url, self.latest = url, latest
        self.stopped = threading.Event()

    def run(self):
        import asyncio
        from rosbags.typesys import Stores, get_typestore
        self.typestore = get_typestore(Stores.ROS2_JAZZY)
        asyncio.run(self.receive())

    async def receive(self):
        import asyncio
        import websockets
        while not self.stopped.is_set():
            try:
                async with websockets.connect(self.url, subprotocols=list(SUBPROTOCOLS),
                                              open_timeout=3, close_timeout=1, max_size=4 * 1024 * 1024,
                                              max_queue=16) as socket:
                    if socket.subprotocol not in SUBPROTOCOLS:
                        raise ValueError("server did not negotiate a supported Foxglove subprotocol")
                    assembler, subscriptions = SnapshotAssembler(), {}
                    self.latest.set_status("connected; waiting for 4 telemetry topics")
                    while not self.stopped.is_set():
                        try:
                            packet = await asyncio.wait_for(socket.recv(), timeout=.5)
                        except asyncio.TimeoutError:
                            continue
                        if isinstance(packet, str):
                            event = json.loads(packet)
                            if event.get("op") == "advertise":
                                requests = []
                                for channel in event["channels"]:
                                    topic = channel["topic"]
                                    if topic not in TOPICS:
                                        continue
                                    if channel["encoding"] != "cdr" or channel["schemaName"] != TOPICS[topic][1]:
                                        raise ValueError(f"unsupported encoding/schema for {topic}")
                                    ident = channel["id"]
                                    if ident not in subscriptions:
                                        subscriptions[ident] = topic
                                        requests.append({"id": ident, "channelId": ident})
                                if requests:
                                    await socket.send(json.dumps({"op": "subscribe", "subscriptions": requests}))
                                missing = set(TOPICS) - set(subscriptions.values())
                                if missing:
                                    self.latest.set_status("missing topics: " + ", ".join(sorted(missing)))
                            elif event.get("op") == "unadvertise":
                                for ident in event["channelIds"]:
                                    subscriptions.pop(ident, None)
                                assembler = SnapshotAssembler()
                                self.latest.set_status("telemetry publisher disappeared")
                            elif event.get("op") == "status" and event.get("level", 0) >= 2:
                                self.latest.set_status(event.get("message", "Foxglove error"))
                            continue
                        if len(packet) < 13 or packet[0] != 1:
                            continue
                        ident, _ = struct.unpack_from("<IQ", packet, 1)
                        topic = subscriptions.get(ident)
                        if topic is None:
                            continue
                        message = self.typestore.deserialize_cdr(packet[13:], TOPICS[topic][1])
                        try:
                            snapshot = assembler.accept(topic, message)
                            if snapshot:
                                self.latest.put(snapshot)
                        except ValueError as error:
                            self.latest.set_status(f"rejected sample: {error}")
            except Exception as error:
                self.latest.set_status(f"disconnected: {error}")
                for _ in range(10):
                    if self.stopped.is_set():
                        break
                    await asyncio.sleep(.1)

    def close(self):
        self.stopped.set()
        self.join(timeout=5)
