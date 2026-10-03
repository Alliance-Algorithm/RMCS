"""V5 motor-output coordinates and passive gas-spring forces, independent of Isaac."""
from __future__ import annotations

import torch

from wheeled_tasks.v40.core import motor_torque_limit


class V5Control:
    ACTIVE = ("L_joint1", "LL_joint1", "L_joint3", "R_joint1", "RR_joint1", "R_joint3")
    LEGS = (0, 1, 3, 4)
    WHEELS = (2, 5)

    def __init__(self, manifest, spec, fit, prior, settings, device):
        if tuple(manifest["control_joint_names"]) != self.ACTIVE:
            raise ValueError("V5 requires the two root-driven leg axes per side, not virtual knee actuation")
        self.nominal = torch.tensor([manifest["nominal_joint_pos"][n] for n in self.ACTIVE], device=device)
        self.spring_names = manifest["spring_joint_names"]
        self.s0 = torch.tensor([spec["spring_binding"][n]["compression_at_q_zero_m"] for n in self.spring_names], device=device)
        self.stroke = torch.tensor([spec["spring_binding"][n]["stroke_m"] for n in self.spring_names], device=device)
        self.coefficients = fit["monomial_coefficients_n"]
        by_name = {j["name"]: j for j in spec["joints"]}
        self.knee_bounds = torch.tensor([[float(by_name[n]["limit"][k]) for k in ("lower", "upper")]
                                        for n in ("L_joint2", "R_jonit2")], device=device)
        self.wheel_prior = prior["actuators"]["wheel"]
        self.settings = settings
        wheel_clip = settings.get("wheel_action_clip", settings["action_clip"])
        self.action_bounds = torch.tensor([settings["action_clip"], settings["action_clip"], wheel_clip,
                                          settings["action_clip"], settings["action_clip"], wheel_clip], device=device)

    def decode(self, actions, q):
        clipped = actions.clamp(-self.action_bounds, self.action_bounds)
        desired = self.nominal[list(self.LEGS)] + clipped[:, list(self.LEGS)] * self.settings["leg_position_scale"]
        delta = desired - q[:, list(self.LEGS)]
        legs = q[:, list(self.LEGS)] + torch.atan2(delta.sin(), delta.cos())
        wheels = clipped[:, list(self.WHEELS)] * self.settings["wheel_velocity_scale"]
        return legs, wheels, clipped

    def motor_efforts(self, q, dq, legs, wheels):
        # ACTIVE is two contiguous (hip, knee, wheel) triples. Views avoid
        # gathering/scattering joint subsets on every 1 kHz control tick.
        requested = torch.empty_like(q)
        pairs = requested.unflatten(-1, (2, 3))
        pairs[:, :, :2] = (self.settings["leg_kp"] * (legs.unflatten(-1, (2, 2))
                                                  - q.unflatten(-1, (2, 3))[:, :, :2])
                          - self.settings["leg_kd"] * dq.unflatten(-1, (2, 3))[:, :, :2])
        tau = torch.empty_like(q)
        tau.unflatten(-1, (2, 3))[:, :, :2] = pairs[:, :, :2].clamp(-40., 40.)
        wheel_speed = dq[:, 2::3]
        wheel = self.wheel_prior["kd"] * (wheels - wheel_speed)
        bound = motor_torque_limit(wheel_speed, self.wheel_prior)
        tau[:, 2::3] = torch.maximum(torch.minimum(wheel, bound), -bound)
        requested[:, 2::3] = wheel
        self.requested_motor_effort = requested
        self.current_motor_bounds = torch.full_like(q, 40.)
        self.current_motor_bounds[:, 2::3] = bound
        # These three outputs must remain independent and fresh: recovery and
        # transport mutate the diagnostics, and the caller scales torque in place.
        return tau

    def spring_state(self, position, velocity):
        return self.s0 - position, -velocity

    def spring_efforts(self, position):
        compression = self.s0 - position
        # Same explicit research extrapolation as the bounded MuJoCo test.
        # Force is held at the curve endpoint during numerical stop penetration;
        # mechanical limits and episode termination handle actual overtravel.
        u = (compression / self.stroke).clamp(0., 1.)
        a, b, c, d = self.coefficients
        return a + u * (b + u * (c + u * d))

    def working_margin_risk(self, knee_positions, spring_positions, margin_rad):
        compression = self.s0 - spring_positions
        spring_risk = ((compression / self.stroke - .9) / .1).clamp_min(0)
        lo, hi = self.knee_bounds[:, 0], self.knee_bounds[:, 1]
        joint_risk = torch.maximum((lo + margin_rad - knee_positions) / margin_rad,
                                  (knee_positions - hi + margin_rad) / margin_rad).clamp_min(0)
        return torch.maximum(spring_risk, joint_risk)

    def proprioception(self, omega, gravity, commands, q, dq, previous_actions):
        delta = q[:, list(self.LEGS)] - self.nominal[list(self.LEGS)]
        delta = torch.atan2(delta.sin(), delta.cos())
        return torch.cat((omega * .5, gravity, commands * commands.new_tensor([1., 1., 5.]),
                          delta, dq * .1, previous_actions), -1).clamp(-100., 100.)
