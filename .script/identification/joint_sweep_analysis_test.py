#!/usr/bin/env python3
"""Independent signal/model checks for joint sweep analysis; no hardware access."""

import csv
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.signal import lfilter

from analyze_joint_sweep import S, analyze, fit_delay, fit_local, read_csv, tune_pd, validate_rollouts


class JointSweepAnalysisTest(unittest.TestCase):
    def test_inactive_csv_cannot_produce_identification_or_gains(self):
        data = {S + "state": np.full(1000, 6), S + "reference_active": np.zeros(1000),
                S + "control_steady_ns": np.arange(1000, dtype=np.int64) * 1_000_000,
                S + "stop_reason": np.full(1000, 2)}
        report = analyze(data, [], fit=True)
        self.assertFalse(report["accepted"])
        self.assertFalse(report["real2sim_ready"])
        self.assertEqual(report["local_models"], [])

    def test_delay_uses_command_to_torque_and_holds_out_complete_segments(self):
        dt = .001
        t = np.arange(0, 8, dt)
        u = np.sin(2 * np.pi * .7 * t) + .5 * np.sin(2 * np.pi * 5.3 * t)
        delayed = np.r_[np.full(12, u[0]), u[:-12]]
        alpha = np.exp(-dt / .005)
        y = .9 * lfilter([1 - alpha], [1, -alpha], delayed, zi=[alpha * delayed[0]])[0] + .07
        train = t < 4
        report = fit_delay(t, u, t, y, train, ~train, [np.flatnonzero(train), np.flatnonzero(~train)])
        self.assertTrue(report["accepted"])
        self.assertEqual(report["delay_ms"], 12)
        self.assertEqual(report["time_constant_ms"], 5)
        self.assertAlmostEqual(report["gain"], .9, places=2)
        self.assertLess(report["holdout"]["nrmse"], .02)

    def test_coupled_parameters_recover_independent_noncollinear_motion(self):
        t = np.arange(0, 10, .002)
        q, v, acc = [], [], []
        for frequencies, phase in (((.37, 1.7, 3.3), .2), ((.63, 2.1, 4.9), 1.4)):
            w = 2 * np.pi * np.asarray(frequencies)
            angle = t[:, None] * w + phase
            q.append(np.sum(.012 * np.sin(angle), axis=1))
            v.append(np.sum(.012 * w * np.cos(angle), axis=1))
            acc.append(np.sum(-.012 * w * w * np.sin(angle), axis=1))
        q, v, acc = np.asarray(q).T, np.asarray(v).T, np.asarray(acc).T
        m = np.array([[.07, .012], [.012, .05]])
        b = np.array([[.18, .03], [.03, .12]])
        k = np.array([[2, -.3], [-.3, 1]])
        tau = acc @ m.T + v @ b.T + np.tanh(30 * v) * [.09, .08] + q @ k.T
        model = fit_local(q, v, acc, tau, t >= 7)
        self.assertTrue(model["accepted"])
        np.testing.assert_allclose(model["M"], m, atol=1e-5)
        np.testing.assert_allclose(model["B"], b, atol=1e-5)
        rollout = validate_rollouts(model, t, q, v, tau, t >= 7, np.where(t < 7, 0, 1))
        self.assertTrue(rollout["accepted"])
        delay = [{"gain": 1., "bias_nm": 0., "delay_ms": 4, "time_constant_ms": 2}] * 2
        tuned = tune_pd(model, delay, np.array([20., 20.]))
        self.assertTrue(tuned["available"])
        self.assertFalse(tuned["auto_applied"])

    def test_collinear_pair_motion_cannot_identify_two_axis_damping(self):
        t = np.arange(2000) * .001
        q = np.column_stack((np.sin(t), np.sin(t)))
        v = np.column_stack((np.cos(t), np.cos(t)))
        result = fit_local(q, v, -q, q, t > 1)
        self.assertFalse(result["accepted"])

    def test_reader_preserves_nanoseconds_above_float_integer_precision(self):
        timestamp = 18_014_398_509_481_985
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "recording.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["index", S + "control_steady_ns", S + "state"])
                writer.writerow([0, timestamp, 6])
                writer.writerow([1, timestamp + 1_000_001, 6])
                writer.writerow([2])
            data, incomplete = read_csv(path)
        self.assertEqual(int(data[S + "control_steady_ns"][0]), timestamp)
        self.assertEqual(int(np.diff(data[S + "control_steady_ns"])[0]), 1_000_001)
        self.assertEqual(len(incomplete), 1)

    def test_complete_csv_path_fits_local_models_and_separates_high_speed_delay(self):
        t = np.arange(0, 3, .001)
        n = len(t)
        pieces = []
        m = np.array([[.07, .012], [.012, .05]])
        b = np.array([[.18, .03], [.03, .12]])
        for center in (50, 75, 100):
            for validation in (0, 1):
                q, velocity, acceleration = [], [], []
                for frequencies, phase in (((.37, 1.7, 3.3), .2 + validation), ((.63, 2.1, 4.9), 1.4 + validation)):
                    w = 2 * np.pi * np.asarray(frequencies)
                    phase = t[:, None] * w + phase
                    q.append(np.sum(.012 * np.sin(phase), axis=1))
                    velocity.append(np.sum(.012 * w * np.cos(phase), axis=1))
                    acceleration.append(np.sum(-.012 * w * w * np.sin(phase), axis=1))
                q, velocity, acceleration = np.asarray(q).T, np.asarray(velocity).T, np.asarray(acceleration).T
                tau = acceleration @ m.T + velocity @ b.T + np.tanh(30 * velocity) * [.09, .08] + q
                pieces.append((center, validation, 6, q, velocity, tau))
        for validation in (0, 1):
            q = np.column_stack((np.pi * t, np.pi * t))
            velocity = np.full_like(q, np.pi)
            tau = np.column_stack((np.sin(t * 3) + 1, np.sin(t * 7) + 1))
            pieces.append((60 + 30 * validation, validation, 4, q, velocity, tau))
        count = n * len(pieces)
        stamp = 18_014_398_509_481_985 + np.arange(count, dtype=np.int64) * 1_000_000
        data = {S + "control_steady_ns": stamp, S + "state": np.full(count, 3),
                S + "reference_active": np.ones(count), S + "stop_reason": np.zeros(count),
                S + "side": np.zeros(count), S + "run_id": np.ones(count),
                S + "stage": np.repeat(np.arange(len(pieces)), n), S + "excitation_revision": np.full(count, 3),
                S + "stage_elapsed_s": np.tile(t, len(pieces)), S + "stage_duration_s": np.full(count, 3),
                S + "validation": np.repeat([p[1] for p in pieces], n),
                S + "waveform": np.repeat([p[2] for p in pieces], n),
                S + "left/beta_estimate": np.deg2rad(np.repeat([p[0] for p in pieces], n))}
        for axis, joint in enumerate(("hip", "knee")):
            sweep, motor = S + f"left_{joint}_joint/", f"/wheel_leg/left_{joint}_joint/"
            data[sweep + "model_angle"] = np.concatenate([p[3][:, axis] for p in pieces])
            data[sweep + "model_velocity"] = np.concatenate([p[4][:, axis] for p in pieces])
            data[motor + "velocity"] = -data[sweep + "model_velocity"]
            data[motor + "torque"] = -np.concatenate([p[5][:, axis] for p in pieces])
            data[motor + "tau_frame_api"] = data[motor + "torque"].copy()
            data[motor + "feedback_sequence"] = np.arange(1, count + 1)
            data[motor + "feedback_steady_ns"] = stamp.copy()
            data[motor + "tx_queued_steady_ns"] = stamp.copy()
            data[motor + "tx_kind"] = np.ones(count)
            data[sweep + "tracking_error"] = np.zeros(count)
            data[sweep + "saturated"] = np.zeros(count)
            data[sweep + "probe_torque_model"] = .2 * np.sin(np.arange(count) * .017)
            data[S + f"host/{joint}_torque_limit_nm"] = np.full(count, 20)
        report = analyze(data, [], fit=True)
        self.assertTrue(report["accepted"], report)
        self.assertFalse(report["real2sim_ready"])
        self.assertEqual(len(report["delay"]), 2)
        self.assertEqual(len(report["high_speed_delay"]), 2)
        for model in report["local_models"]:
            np.testing.assert_allclose(model["M"], m, atol=1e-3)
        data[S + "excitation_revision"].fill(4)
        report_v4 = analyze(data, [], fit=True)
        self.assertTrue(report_v4["accepted"], report_v4)
        data[S + "excitation_revision"].fill(5)
        self.assertFalse(analyze(data, [], fit=True)["accepted"])
        data[S + "excitation_revision"].fill(4)
        for joint in ("hip", "knee"):
            data[f"/wheel_leg/left_{joint}_joint/tx_kind"].fill(0)
        report = analyze(data, [], fit=True)
        self.assertFalse(report["accepted"])
        self.assertTrue(all(not row["accepted"] for row in report["delay"]))
        self.assertTrue(all("pd_search" not in model for model in report["local_models"]))


if __name__ == "__main__":
    unittest.main()
