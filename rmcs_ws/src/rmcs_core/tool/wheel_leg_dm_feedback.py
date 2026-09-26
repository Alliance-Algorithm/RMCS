"""Binary CAN loopback to the real RMCS DM driver and controller component."""
from collections import deque
from pathlib import Path
import struct
import subprocess

import numpy as np


class DmFeedbackBridge:
    RESPONSE = struct.Struct("<4d4d32s8s4B256s")

    def __init__(self, count, offsets, container, log_path, direction_comparison=False):
        self.count = count
        self.offsets = np.asarray(offsets, dtype=np.float64)
        self.status = np.zeros((count, 4), dtype=np.uint8)
        self.wrap_signed = (np.arange(count) % 2).astype(bool)
        self.feedback_delay = np.zeros((count, 4), dtype=int)
        # Half of the cases also test unequal sensor and velocity-loop latency.
        self.stressed = ((np.arange(count) // 2) % 2).astype(bool)
        self.feedback_delay[self.stressed] = [1, 3, 2, 4]
        self.command_delay = np.zeros_like(self.feedback_delay)
        self.command_delay[self.stressed] = [2, 5, 3, 6]
        self.independent_short_arc = np.zeros(count, dtype=bool)
        if direction_comparison:
            self.independent_short_arc[1::2] = True
            self.wrap_signed[:] = False
            self.stressed[:] = False
            self.feedback_delay[:] = 0
            self.command_delay[:] = 0
        self.feedback_history = deque(maxlen=5)
        self.command_history = deque(maxlen=7)
        self.ids = np.array([1, 1, 2, 2], dtype=np.uint8)
        self.tick = 0
        self.system_counts = {"clear": 0, "enable": 0, "disable": 0}
        self.first_fault = None
        self.encoder_samples = []
        self.command_samples = []
        self.max_phase_error = 0.0
        self.startup_nonzero_frames = 0
        self.disable_commands_while_requested = 0
        self.wrap_crossings = np.zeros((count, 4), dtype=np.int64)
        self.previous_raw = None
        self.log = Path(log_path).open("wb")
        command = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /workspaces/RMCS/rmcs_ws/install/setup.bash && "
            "export RCUTILS_LOGGING_USE_STDOUT=0 && "
            "exec /workspaces/RMCS/rmcs_ws/build/rmcs_core/wheel_leg_dm_sim_bridge "
            "--ros-args --params-file /workspaces/RMCS/rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml "
            "--log-level warn"
        )
        self.process = subprocess.Popen(
            ["docker", "exec", "-i", container, "bash", "-lc", command],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
        )
        self.process.stdin.write(struct.pack("<I4d", count, *self.offsets) + self.independent_short_arc.astype(np.uint8).tobytes())
        self.process.stdin.flush()
        if self._read(4) != struct.pack("<I", 0x444D5635):
            raise RuntimeError("DM feedback bridge initialization failed")

    def _read(self, count):
        chunks = []
        while count:
            chunk = self.process.stdout.read(count)
            if not chunk:
                raise RuntimeError("DM feedback bridge ended early; inspect its log")
            chunks.append(chunk)
            count -= len(chunk)
        return b"".join(chunks)

    @staticmethod
    def _unsigned(value, maximum, bits):
        return np.floor((np.clip(value, -maximum, maximum) + maximum)
                        / (2. * maximum) * ((1 << bits) - 1)).astype(np.uint16)

    def step(self, time, positions, velocities, torques, targets, requested, reset):
        # Encoder zero is fixed by the real calibration pose. Quantize its
        # single-turn reading before the 16-bit CAN POS field (P_MAX=12.5).
        raw = np.remainder(self.offsets - positions, 2. * np.pi)
        raw = np.remainder(np.rint(raw / (2. * np.pi) * 16384.) / 16384. * (2. * np.pi), 2. * np.pi)
        raw[self.wrap_signed] = (raw[self.wrap_signed] + np.pi) % (2. * np.pi) - np.pi
        if self.previous_raw is not None:
            self.wrap_crossings += np.abs(raw - self.previous_raw) > np.pi
        self.previous_raw = raw.copy()
        packet = np.zeros((self.count, 4, 8), dtype=np.uint8)
        pos = self._unsigned(raw, 12.5, 16)
        vel = self._unsigned(-velocities, 45., 12)
        torque = self._unsigned(-torques, 54., 12)
        packet[:, :, 0] = self.status * 16 + self.ids
        packet[:, :, 1] = pos >> 8
        packet[:, :, 2] = pos & 255
        packet[:, :, 3] = vel >> 4
        packet[:, :, 4] = ((vel & 15) << 4) | (torque >> 8)
        packet[:, :, 5] = torque & 255
        packet[:, :, 6:] = [40, 45]
        self.feedback_history.append((packet.copy(), positions.copy()))
        delayed = packet.copy()
        delayed_positions = positions.copy()
        for env in range(self.count):
            for joint in range(4):
                index = min(self.feedback_delay[env, joint], len(self.feedback_history) - 1)
                delayed[env, joint] = self.feedback_history[-1 - index][0][env, joint]
                delayed_positions[env, joint] = self.feedback_history[-1 - index][1][env, joint]
        request = bytearray(struct.pack("<Id", 1, time))
        for env in range(self.count):
            request.extend(delayed[env].tobytes())
            request.extend(struct.pack("<4dQB", *targets[env], reset, int(requested)))
        self.process.stdin.write(request)
        self.process.stdin.flush()
        response = self._read(self.RESPONSE.size * self.count)
        decoded = np.zeros_like(positions)
        decoded_velocity = np.zeros_like(velocities)
        wire_commands = np.zeros_like(positions)
        active = np.zeros(self.count, dtype=bool)
        healthy = np.zeros(self.count, dtype=bool)
        for env in range(self.count):
            values = self.RESPONSE.unpack_from(response, self.RESPONSE.size * env)
            decoded[env] = values[:4]
            decoded_velocity[env] = values[4:8]
            frames, system = values[8:10]
            healthy[env], active[env] = values[10:12]
            reason = values[14].split(b"\0", 1)[0].decode()
            for joint in range(4):
                frame = frames[joint * 8:(joint + 1) * 8]
                assert frame[4:] == b"\0" * 4, "invalid VEL reserved bytes"
                wire_commands[env, joint] = struct.unpack_from("<f", frame)[0]
            if not active[env] and np.any(wire_commands[env] != 0.):
                self.startup_nonzero_frames += 1
            if system[:7] == b"\xff" * 7:
                if system[7] == 0xFC:
                    self.status[env] = 1
                    self.system_counts["enable"] += 1
                elif system[7] == 0xFD:
                    self.disable_commands_while_requested += int(requested)
                    self.status[env] = 0
                    self.system_counts["disable"] += 1
                elif system[7] == 0xFB:
                    self.status[env] = 0
                    self.system_counts["clear"] += 1
                else:
                    raise RuntimeError("unexpected motor system command")
            if requested and not healthy[env] and reason and self.first_fault is None:
                self.first_fault = {"time_s": time, "environment": env, "reason": reason,
                                    "raw_feedback_hex": [v.tobytes().hex() for v in delayed[env]],
                                    "decoded_angles": decoded[env].tolist()}
        phase_error = (decoded - delayed_positions + np.pi) % (2. * np.pi) - np.pi
        self.max_phase_error = max(self.max_phase_error, float(np.abs(phase_error).max()))
        if self.tick == 0 or (len(self.encoder_samples) < 5 and np.any(np.abs(raw) < .001)):
            env = 0
            self.encoder_samples.append({"time_s": time, "urdf_angles": positions[env].tolist(),
                "raw_motor_angles": raw[env].tolist(), "can_feedback_hex": [v.tobytes().hex() for v in packet[env]],
                "rmcs_decoded_angles": decoded[env].tolist()})
        if np.any(np.abs(wire_commands) > .05) and len(self.command_samples) < 2:
            self.command_samples.append({"time_s": time, "wire_motor_velocity_rad_s": wire_commands[0].tolist(),
                                         "urdf_velocity_rad_s": (-wire_commands[0]).tolist()})
        # VEL wire values are motor-axis speeds; all four axes are reversed.
        commands = -wire_commands
        self.command_history.append(commands.copy())
        applied = commands.copy()
        for env in range(self.count):
            for joint in range(4):
                index = min(self.command_delay[env, joint], len(self.command_history) - 1)
                applied[env, joint] = self.command_history[-1 - index][env, joint]
        applied[self.status[:, 0] == 0] = 0.
        self.tick += 1
        return decoded, decoded_velocity, commands, applied, active, healthy

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write(struct.pack("<I", 0))
            self.process.stdin.flush()
            self.process.stdin.close()
            try:
                code = self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5)
                raise RuntimeError("DM feedback bridge failed to exit")
            if code != 0:
                raise RuntimeError(f"DM feedback bridge failed with code {code}")
        self.log.close()
