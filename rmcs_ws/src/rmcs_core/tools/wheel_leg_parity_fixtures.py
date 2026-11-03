"""Shared deterministic fixtures for the current deployment/training parity check."""
import hashlib
import json
import math
import numpy as np
import torch

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def require(condition,message):
    if not condition:
        raise ValueError(message)

def sensor_records(training, observer):
    records = []
    bank = json.loads((training / "model/wheel_leg_v2.0.2/training/pose_bank.json").read_text())
    geometry = observer.coordinate
    nominal32 = geometry.geometry.nominal
    nominal = nominal32.numpy().astype(np.float64)
    q = np.r_[nominal, 0., 0.]
    tick = 0

    def emit(label, *, dq=None, gyro=(0., 0., 0.), gravity=(0., 0., -1.), command=(0., 0., 0.),
             height=.305, enabled=True, reset=False, duplicate=False):
        nonlocal tick
        if not duplicate:
            tick += 1
        records.append(dict(label=label, tick=tick, reset=reset, enabled=enabled, q=q.copy(),
            dq=np.zeros(6) if dq is None else np.asarray(dq), gyro=np.asarray(gyro), gravity=np.asarray(gravity),
            command=np.asarray(command), height=height, applied=.2*np.sin(np.arange(6)+tick*.03)))

    def repeat(label, count, **kwargs):
        for _ in range(count):
            emit(label, **kwargs)

    def coord(legs, gravity=(0., 0., -1.)):
        position = torch.tensor(np.r_[legs, 0., 0.], dtype=torch.float32)[None]
        return geometry(position, torch.tensor([gravity], dtype=torch.float32))[0][0].numpy()

    for offset in (0., 1e8, -1e8):
        q = np.r_[nominal, offset, -offset]
        emit("origin", reset=True)
        repeat("capture", 24)
        for _ in range(80):
            q[4:] += [.03, -.03]
            emit("drift", dq=[0., 0., 0., 0., 3., -3.])
        repeat("hold", 80)
        emit("duplicate", duplicate=True)
        base_coordinate = coord(nominal)
        origin = np.array([offset+2.4, -offset-2.4])
        for name in ("stand_low", "stand_high", "stand"):
            row = bank["poses"][bank["diagnostic_height_starts"]["mapping"][name]]["full_joint_positions"]
            end = np.array([row[k] for k in ("L_joint1", "LL_joint1", "R_joint1", "RR_joint1")])
            start = q[:4].copy()
            for i in range(1, 101):
                previous = q.copy()
                q[:4] = start + (end-start)*i/100
                q[4:] = origin + (base_coordinate-coord(q[:4]))/np.array([.06, -.06])
                emit("height_geometry", dq=(q-previous)/.01, height=.23 if name == "stand_low" else .42)
            repeat("height_hold", 40)
        pitch = 0.
        for endpoint in (-.2, .2, 0.):
            start_pitch = pitch
            for i in range(1, 41):
                previous, previous_pitch = q.copy(), pitch
                pitch = start_pitch+(endpoint-start_pitch)*i/40
                gravity = (math.sin(pitch), 0., -math.cos(pitch))
                q[4:] = origin+(base_coordinate-coord(q[:4], gravity))/np.array([.06, -.06])
                emit("pitch_geometry", dq=(q-previous)/.01, gyro=(0., (pitch-previous_pitch)/.01, 0.),
                     gravity=gravity)
        repeat("invalid_tilt", 20, gravity=(math.sin(.3), 0., -math.cos(.3)))
        repeat("restored_tilt", 30)
        emit("inverted", gravity=(0., 0., 1.))
        repeat("recapture", 25)
        for direction in (1, -1):
            for _ in range(60):
                q[4:] += np.array([.5, -.5])*direction
                emit("saturation_and_return", dq=np.r_[np.zeros(4), np.array([50., -50.])*direction])
            repeat("settled", 100)
        q[4] += math.tau/15.62
        repeat("jump_latch", 30)
        emit("end_request", command=(.001, 0., 0.))
        repeat("recapture", 25)
        q[5] = float("nan")
        emit("nonfinite_wheel")
        q[5] = origin[1]
        repeat("nonfinite_latch", 25)
        emit("disable", enabled=False)
        repeat("recapture", 25)
        for command in ((2., 0., 0.), (-2., 0., 0.), (0., .1, 0.), (0., 0., 100*math.tau/60),
                        (0., 0., -100*math.tau/60), (.2, 0., .6)):
            repeat("motion", 3, command=command)
            repeat("recapture", 25)
        emit("invalid_gyro", gyro=(float("nan"), 0., 0.))
        repeat("restored_gyro", 5)
        emit("disable", enabled=False)
        repeat("fast_wheel_capture_denied", 25, dq=(0., 0., 0., 0., 2.5, -2.5))
        repeat("fast_leg_capture_denied", 25, dq=(3., 0., 0., 0., 0., 0.))
        repeat("capture", 25)
    # Physical endpoint / bounded sensor projection and a whole-turn leg chart.
    for side in (0, 1):
        knots = getattr(geometry.geometry, f"side{side}_delta_rad")
        for delta in (float(knots[0]), float(knots[-1]), float(knots[0])-.0005, float(knots[-1])+.0005,
                      float(knots[-1])+.002, float(knots[-1])+.02):
            q = np.r_[nominal, 0., 0.]
            q[side*2+1] = q[side*2]+delta
            emit("endpoint", reset=True)
            repeat("endpoint", 25)
    q = np.r_[nominal+math.tau, 0., 0.]
    emit("whole_turn_legs", reset=True)
    repeat("whole_turn_legs", 25)
    return records, nominal32


def python_frames(records, observer, nominal):
    results = []
    for row in records:
        if row["reset"]:
            observer.reset(torch.tensor([0]))
        frame = torch.zeros(1, 35)
        frame[0, :3] = torch.tensor(row["command"])
        frame[0, 3] = torch.tensor(row["height"], dtype=torch.float32)*5.
        frame[0, 4:7] = torch.tensor(row["gyro"], dtype=torch.float32)*.5
        frame[0, 7:10] = torch.tensor(row["gravity"], dtype=torch.float32)
        delta = torch.tensor(row["q"][:4], dtype=torch.float32)-nominal
        frame[0, 10:14] = torch.atan2(delta.sin(), delta.cos())
        frame[0, 16:22] = torch.tensor(row["dq"], dtype=torch.float32)*.1
        frame[0, 22:28] = torch.tensor(row["applied"], dtype=torch.float32)
        frame[0, 28] = 1.
        enabled = torch.tensor([row["enabled"]])
        frame = frame.clamp(-100., 100.)
        frame[:, 14:16] = observer.local_wheel_angles(frame,
            torch.tensor(row["q"][4:], dtype=torch.float64)[None], enabled, row["tick"])
        frame = observer.update(frame, nominal, enabled, row["tick"])
        results.append(frame[0].numpy() if torch.isfinite(frame).all() else None)
    return results
