#!/usr/bin/env python3
"""Check that velocity quantization is not reported as physical breakaway."""

import unittest

import numpy as np
import pandas as pd

from analyze_horizontal_knee_sweep import S, breakaway


class BreakawayTest(unittest.TestCase):
    def trace(self, velocity):
        time = pd.Series(np.arange(len(velocity)) * .001)
        motor = "/wheel_leg/left_knee_joint/"
        sweep = S + "left_knee_joint/"
        data = pd.DataFrame({motor + "feedback_sequence": np.arange(len(time)),
                             motor + "torque": np.full(len(time), -17.0),
                             sweep + "model_velocity": velocity,
                             sweep + "model_angle": np.cumsum(velocity) * .001,
                             S + "stage": np.ones(len(time))})
        return breakaway(data, time, pd.Series(np.full(len(time), 70.0)), -1)

    def test_quantized_stationary_feedback_is_not_breakaway(self):
        velocity = np.resize([-.010989, .010989, -.032967, .032967], 1000)
        self.assertEqual(self.trace(velocity), [])

    def test_rest_then_sustained_measured_displacement_is_a_load_candidate(self):
        events = self.trace(np.r_[np.full(500, .010989), np.full(500, .2)])
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["load_plus_friction_model_nm"], 17)
        self.assertNotIn("static_friction_nm", events[0])


if __name__ == "__main__":
    unittest.main()
