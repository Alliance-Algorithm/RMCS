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
    "/wheel_leg/telemetry/joint_states": ("joints", "sensor_msgs/msg/JointState", "chassis_body"),
    "/wheel_leg/telemetry/imu_body": ("imu", "sensor_msgs/msg/Imu", "chassis_body"),
    "/wheel_leg/telemetry/rl_projected_gravity": ("gravity", "geometry_msgs/msg/Vector3Stamped", "chassis_body"),
    "/wheel_leg/telemetry/rl_angular_velocity": ("gyro", "geometry_msgs/msg/Vector3Stamped", "chassis_body"),
    "/wheel_leg/telemetry/diagnostics": ("diagnostics", "diagnostic_msgs/msg/DiagnosticArray", "chassis_body"),
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
    rmcs_knee: tuple[float, ...] = ()
    rmcs_length: tuple[float, ...] = ()
    rmcs_model_positions: tuple[float, ...] = ()
    model_offsets: tuple[float, ...] = ()
    feedback_fresh: bool = True
    state_valid: bool = True
    reset_count: int = 0
    calibration_sha256: str = ""
    calibration_model_spec_sha256: str = ""
    sensor_age_s: tuple[float, ...] = ()
    diagnostic_status: str = ""

    @property
    def positions_by_name(self):
        return dict(zip(MOTOR_NAMES, self.positions))

    @property
    def velocities_by_name(self):
        return dict(zip(MOTOR_NAMES, self.velocities))

    @property
    def model_positions_by_name(self):
        return dict(zip(MOTOR_NAMES, self.rmcs_model_positions))

    @property
    def offsets_by_name(self):
        return dict(zip(MOTOR_NAMES[:4], self.model_offsets))

    def validate(self):
        for name, count in (("positions", 6), ("quaternion_wxyz", 4), ("gyro_body", 3),
                            ("gravity_rl", 3), ("gyro_rl", 3)):
            values = getattr(self, name)
            if len(values) != count or not all(math.isfinite(v) for v in values):
                raise ValueError(f"invalid {name}")
        norm = sum(x * x for x in self.quaternion_wxyz)
        if self.state_valid and not .99 < norm < 1.01:
            raise ValueError(f"IMU quaternion norm squared {norm:.5g} is not near one")
        if self.state_valid and not .95 < sum(x * x for x in self.gravity_rl) < 1.05:
            raise ValueError("RL gravity is not a unit vector")
        if self.stamp_ns <= 0:
            raise ValueError("zero/negative sample timestamp")
        for name, count in (("velocities", 6), ("rmcs_knee", 2), ("rmcs_length", 2),
                            ("rmcs_model_positions", 6), ("model_offsets", 4), ("sensor_age_s", 7)):
            values = getattr(self, name)
            if values and (len(values) != count or not all(math.isfinite(v) for v in values)):
                raise ValueError(f"invalid {name}")
        if type(self.feedback_fresh) is not bool or type(self.state_valid) is not bool:
            raise ValueError("invalid validity flags")
        if type(self.reset_count) is not int or self.reset_count < 0:
            raise ValueError("invalid reset count")
        return self

    @classmethod
    def from_dict(cls, value):
        fields = {k: value[k] for k in cls.__dataclass_fields__ if k in value}
        for name in ("positions", "quaternion_wxyz", "gyro_body", "gravity_rl", "gyro_rl",
                     "velocities", "rmcs_knee", "rmcs_length", "rmcs_model_positions",
                     "model_offsets", "sensor_age_s"):
            if name in fields:
                fields[name] = tuple(fields[name])
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
        while len(self.pending) > 32:
            del self.pending[min(self.pending)]
        if len(bucket) != len(TOPICS):
            return None
        joints, imu = bucket["joints"], bucket["imu"]
        if len(set(joints.name)) != len(joints.name) or len(joints.position) != len(joints.name):
            raise ValueError("duplicate joint names or incomplete joint positions")
        index = [list(joints.name).index(name) for name in MOTOR_NAMES]
        if len(joints.velocity) != len(joints.name):
            raise ValueError("incomplete joint velocities")
        velocity = tuple(float(joints.velocity[i]) for i in index)
        matches = [s for s in bucket["diagnostics"].status if s.name == "wheel_leg/telemetry"]
        if len(matches) != 1:
            raise ValueError("missing/duplicate telemetry diagnostics")
        entries = matches[0].values
        values = {v.key: v.value for v in entries}
        if len(values) != len(entries):
            raise ValueError("duplicate telemetry diagnostic keys")
        if values.get("schema") != "wheel_leg_telemetry_v1" or values.get("coordinate") != "rmcs_hardware_output_shaft_v1":
            raise ValueError("unsupported telemetry coordinate/schema")
        def flag(key):
            if values.get(key) not in ("true", "false"):
                raise ValueError(f"missing/invalid {key}")
            return values[key] == "true"
        def pair(key):
            return tuple(float(values[side + "/" + key]) for side in ("left", "right"))
        q = imu.orientation
        result = Snapshot(stamp, tuple(float(joints.position[i]) for i in index),
                          (float(q.w), float(q.x), float(q.y), float(q.z)),
                          vector(imu.angular_velocity), vector(bucket["gravity"].vector),
                          vector(bucket["gyro"].vector), velocity,
                          rmcs_knee=pair("passive_knee_rad"), rmcs_length=pair("leg_length_m"),
                          rmcs_model_positions=tuple(float(values["model_position_rad/" + n]) for n in MOTOR_NAMES),
                          model_offsets=tuple(float(values["model_offset_rad/" + n]) for n in MOTOR_NAMES[:4]),
                          feedback_fresh=flag("feedback_fresh"), state_valid=flag("state_valid"),
                          reset_count=int(values["reset_count"]),
                          calibration_sha256=values["calibration_sha256"],
                          calibration_model_spec_sha256=values["calibration_model_spec_sha256"],
                          sensor_age_s=tuple(float(values["motor_age_s/" + n]) for n in MOTOR_NAMES)
                              + (float(values["imu_age_s"]),),
                          diagnostic_status=matches[0].message).validate()
        self.last_stamp = stamp
        self.pending = {key: val for key, val in self.pending.items() if key > stamp}
        return result


class LatestSnapshot:
    def __init__(self, record=None, source="foxglove", metadata=None):
        self.lock = threading.Lock()
        self.snapshot = None
        self.received = 0.0
        self.status = "waiting for telemetry"
        self.started = time.monotonic()
        self.record = open(record, "x") if record else None
        self.source = source
        self.count = 0
        if self.record:
            self.record.write(json.dumps({"kind": "metadata", "schema": "wheel_leg_mirror_record_v1",
                                          "metadata": metadata or {}}, allow_nan=False) + "\n")
            self.record.flush()

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
                    "record_time_s": self.received - self.started, "source": self.source}, allow_nan=False) + "\n")
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
        try:
            import asyncio
            from rosbags.typesys import Stores, get_typestore
            self.typestore = get_typestore(Stores.ROS2_JAZZY)
            asyncio.run(self.receive())
        except Exception as error:
            self.latest.set_status(f"receiver failed: {error}")

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
                    self.latest.set_status("connected; waiting for 5 telemetry topics")
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
                                    if ident not in subscriptions or subscriptions[ident] != topic:
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
                        try:
                            message = self.typestore.deserialize_cdr(packet[13:], TOPICS[topic][1])
                            snapshot = assembler.accept(topic, message)
                            if snapshot:
                                self.latest.put(snapshot)
                        except (ValueError, KeyError, IndexError, TypeError) as error:
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


def read_record(path, expected_metadata=None):
    """Stream recorded snapshots; verify identity before yielding any state."""
    with open(path) as stream:
        first = next(stream, None)
        if first is None:
            raise ValueError("empty wheel-leg mirror recording")
        header = json.loads(first)
        if header.get("kind") != "metadata" or header.get("schema") != "wheel_leg_mirror_record_v1":
            raise ValueError("not a wheel-leg mirror recording")
        for key, value in (expected_metadata or {}).items():
            if header.get("metadata", {}).get(key) != value:
                raise ValueError(f"recording {key} differs from the selected model/mapping")
        previous = -1.
        for line in stream:
            row = json.loads(line)
            elapsed = row["record_time_s"]
            if not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0 or elapsed < previous:
                raise ValueError("recording time must be finite, nonnegative and nondecreasing")
            previous = elapsed
            yield elapsed, Snapshot.from_dict(row)


class ReplayReader(threading.Thread):
    def __init__(self, path, latest, speed=1., loop=False, metadata=None):
        super().__init__(daemon=True)
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("replay speed must be positive")
        self.path, self.latest, self.speed, self.loop = path, latest, speed, loop
        self.metadata = metadata
        self.stopped = threading.Event()

    def run(self):
        try:
            while not self.stopped.is_set():
                started, count = time.monotonic(), 0
                for elapsed, snapshot in read_record(self.path, self.metadata):
                    if self.stopped.wait(max(0., started + elapsed / self.speed - time.monotonic())):
                        return
                    self.latest.put(snapshot, source="replay")
                    count += 1
                if not count:
                    raise ValueError("recording contains no snapshots")
                if not self.loop:
                    self.latest.set_status("replay complete")
                    return
        except Exception as error:
            self.latest.set_status(f"replay failed: {error}")

    def close(self):
        self.stopped.set()
        self.join(timeout=5)
