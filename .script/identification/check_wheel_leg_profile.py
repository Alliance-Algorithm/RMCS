#!/usr/bin/env python3
"""Reject incomplete wheel-leg bench profiles before launching the RMCS executor.

Passing this static check does not certify a motor, linkage, fixture or sensor.
The controller also checks each measured feedback and planned trajectory at runtime.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml


ARRAY_SIZES = {
    **{key: 4 for key in (
        "model_sign", "model_offset", "root_min", "root_max", "max_speed",
        "max_acceleration", "braking_acceleration", "max_torque",
        "kp_position", "kp_velocity", "ki_velocity", "integral_limit", "gravity_feedforward_model",
        "max_position_error", "max_velocity_error", "max_measured_acceleration",
    )},
    "spring_delta_min": 2,
    "spring_delta_max": 2,
}
TRAJECTORY_ARRAY_SIZES = {key: 4 for key in (
    "inertia_bound", "load_torque_bound", "viscous_bound",
)}
SCALARS = (
    "joint_margin", "spring_margin", "beta_margin_degrees", "hold_test_duration_s",
    "ready_timeout_s", "feedback_timeout_s",
    "max_tick_gap_s", "other_side_speed_limit",
)
TRAJECTORY_SCALARS = (
    "move_s", "hold_s", "excite_s", "max_hip_excitation", "max_beta_excitation_degrees",
)
PROBE_SCALARS = (
    "probe_common_amplitude", "probe_knee_amplitude", "probe_sine_common_amplitude",
    "probe_sine_knee_amplitude", "probe_move_s", "probe_hold_s", "probe_excite_s",
    "probe_validation_s", "probe_gravity_hold_s", "probe_step_amplitude", "probe_step_rise_s",
    "probe_triangle_amplitude", "probe_triangle_frequency_hz", "probe_triangle_s",
    "probe_min_interior_delta",
)
SWEEP_SCALARS = (
    "sweep_common_amplitude", "sweep_relative_amplitude", "sweep_common_step",
    "sweep_relative_step", "sweep_step_rise_s", "sweep_move_s", "sweep_quarter_turn_s",
    "sweep_dwell_s", "sweep_chirp_s", "sweep_validation_s", "sweep_ramp_s",
    "sweep_relative_margin",
)
MULTIBAND_SCALARS = (
    "multiband_shape_offset", "multiband_eighth_turn_s", "multiband_dwell_s",
    "multiband_ramp_s", "multiband_common_step", "multiband_relative_step",
    "multiband_step_rise_s", "multiband_validation_s",
)
MULTIBAND_ARRAYS = {
    "multiband_common_hz": 4, "multiband_relative_hz": 4,
    "multiband_common_amplitude": 3, "multiband_relative_amplitude": 3,
    "multiband_band_s": 3,
}
LISTS = (
    "left_beta_degrees", "left_delta_rad", "right_beta_degrees", "right_delta_rad",
)
TRAJECTORY_LISTS = (
    "pose_hip", "pose_beta_degrees", "frequencies_hz",
    "hip_amplitudes", "beta_amplitudes_degrees",
)
COMPONENTS = (
    "rmcs_core::hardware::WheelLeg -> wheel_leg",
    "rmcs_core::controller::identification::WheelLegPairIdentificationController -> wheel_leg_pair_identification_controller",
    "rmcs_core::controller::identification::WheelLegIdentificationRecorder -> wheel_leg_identification_recorder",
)
PASSIVE_COMPONENTS = (
    "rmcs_core::hardware::WheelLeg -> wheel_leg",
    "rmcs_core::controller::identification::WheelLegPassiveRecorderGate -> wheel_leg_passive_recorder_gate",
    "rmcs_core::controller::identification::WheelLegIdentificationRecorder -> wheel_leg_identification_recorder",
)
WHEEL_COMPONENTS = (
    "rmcs_core::hardware::WheelLeg -> wheel_leg",
    "rmcs_core::controller::identification::WheelLegWheelIdentificationController -> wheel_leg_wheel_identification_controller",
    "rmcs_core::controller::identification::WheelLegIdentificationRecorder -> wheel_leg_identification_recorder",
)


def check(profile: dict) -> list[str]:
    errors: list[str] = []
    graph = profile.get("rmcs_executor", {}).get("ros__parameters", {})
    components = graph.get("components", [])
    passive = PASSIVE_COMPONENTS[1] in components
    wheel = WHEEL_COMPONENTS[1] in components
    if graph.get("update_rate") != 1000.0:
        errors.append("executor update_rate must be 1000 Hz")
    for component in (PASSIVE_COMPONENTS if passive else WHEEL_COMPONENTS if wheel else COMPONENTS):
        if component not in components:
            errors.append(f"missing graph component: {component}")
    if any("RlController ->" in str(component) or "WheelLegReferenceController ->" in str(component)
           for component in components):
        errors.append("identification graph cannot share the six torque outputs")
    if passive and any("WheelLegPairIdentificationController ->" in str(component)
                        for component in components):
        errors.append("passive recorder cannot include an active torque controller")
    if wheel and any("WheelLegPairIdentificationController ->" in str(component)
                     for component in components):
        errors.append("wheel and paired-leg controllers cannot share torque outputs")

    hardware = profile.get("wheel_leg", {}).get("ros__parameters", {})
    for key, expected in (("joint_control_mode", "torque"),
                          ("require_enable_request", True), ("allow_set_zero", False)):
        if hardware.get(key) != expected:
            errors.append(f"wheel_leg.{key} must be {expected!r}")

    controller = profile.get("wheel_leg_pair_identification_controller", {}).get("ros__parameters", {})
    recorder = profile.get("wheel_leg_identification_recorder", {}).get("ros__parameters", {})
    if wheel:
        if recorder.get("experiment_kind") != "wheel" or recorder.get("record_on_double_middle") is not True:
            errors.append("wheel identification requires wheel recorder gated by both MIDDLE")
        if recorder.get("encoder_zero_inner_knee_deg") != 105.0:
            errors.append("wheel profile must preserve the real DM zero provenance")
        params = profile.get("wheel_leg_wheel_identification_controller", {}).get("ros__parameters", {})
        finite = lambda x: isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
        for key, size, upper in (("wheel_speeds", 5, 30.), ("wheel_low_speeds", 3, 1.)):
            values = params.get(key)
            if (not isinstance(values, list) or len(values) != size or not all(map(finite, values))
                    or not all(0 < v <= upper for v in values)
                    or not all(a < b for a, b in zip(values, values[1:]))):
                errors.append(f"{key}: {size} ascending output-shaft speeds <= {upper} required")
        required = ("wheel_ramp_s", "wheel_hold_s", "wheel_coast_s", "wheel_step_hold_s",
                    "wheel_chirp_s", "wheel_validation_s", "wheel_velocity_kp", "wheel_velocity_ki",
                    "wheel_velocity_kd", "wheel_feedforward", "wheel_torque_cap", "wheel_current_limit_a",
                    "wheel_reduction_ratio", "wheel_feedback_speed_cap", "arm_dwell_s",
                    "feedback_timeout_s", "max_tick_gap_s", "control_frequency_hz", "reference_frequency_hz")
        for key in required:
            if not finite(params.get(key)):
                errors.append(f"{key}: finite wheel experiment value required")
        for key, expected in (("trajectory_revision", "wheel_multiband_v2"),
                              ("control_frequency_hz", 1000.), ("reference_frequency_hz", 50.),
                              ("wheel_reduction_ratio", 15.8), ("wheel_velocity_ki", 0.),
                              ("wheel_velocity_kd", 0.), ("wheel_feedforward", 0.)):
            if params.get(key) != expected:
                errors.append(f"wheel controller {key} must be {expected!r}")
        for key, expected in (("trajectory_revision", "wheel_multiband_v2"),
                              ("control_frequency_hz", 1000.), ("sample_frequency_hz", 1000.),
                              ("reference_frequency_hz", 50.), ("wheel_reduction_ratio", 15.8),
                              ("wheels_off_ground", True)):
            if recorder.get(key) != expected:
                errors.append(f"wheel recorder {key} must be {expected!r}")
        for key, minimum in (("wheel_ramp_s", .2), ("wheel_hold_s", 1.), ("wheel_coast_s", 2.),
                             ("wheel_step_hold_s", .5), ("wheel_chirp_s", 8.), ("wheel_validation_s", 8.)):
            value = params.get(key)
            if finite(value) and (value < minimum or abs(value*50 - round(value*50)) > 1e-8):
                errors.append(f"{key}: >= {minimum} s and aligned to 50 Hz required")
        repeats = params.get("wheel_step_repetitions")
        if type(repeats) is not int or not 2 <= repeats <= 4:
            errors.append("wheel_step_repetitions must be an integer in [2,4]")
        amps, cap = params.get("wheel_current_limit_a"), params.get("wheel_torque_cap")
        if finite(amps) and not 0 < amps <= 20:
            errors.append("wheel_current_limit_a must be in (0,20] for C620")
        if finite(amps) and finite(cap) and not math.isclose(cap, amps*15.8*.3*187/3591, abs_tol=1e-9, rel_tol=0):
            errors.append("wheel_torque_cap disagrees with current limit / installed 15.8 conversion")
        for key, lower, upper in (("wheel_velocity_kp", 0., math.inf),
                                  ("wheel_feedback_speed_cap", 30., math.inf),
                                  ("arm_dwell_s", .1-1e-10, math.inf),
                                  ("feedback_timeout_s", 0., .05), ("max_tick_gap_s", .001, .02)):
            value = params.get(key)
            if finite(value) and not lower < value <= upper:
                errors.append(f"{key}: wheel controller range is ({lower},{upper}]")
        for key in ("control_contract", "initial_pose_provenance", "feedback_filter", "feedback_frequency"):
            if not isinstance(recorder.get(key), str) or not recorder[key].strip():
                errors.append(f"wheel recorder needs {key}")
        return errors
    if passive:
        if recorder.get("side") not in ("left", "right"):
            errors.append("passive recorder requires a left/right recording side")
        if recorder.get("record_on_double_middle") is not True:
            errors.append("passive recording requires record_on_double_middle: true")
    elif controller.get("side") not in ("left", "right") or recorder.get("side") != controller.get("side"):
        errors.append("controller and recorder must select the same left/right side")
    stage = controller.get("experiment_stage", "hold")
    if stage not in ("hold", "probe", "trajectory"):
        errors.append("experiment_stage must be hold, probe or trajectory")
    pattern = controller.get("probe_pattern")
    sweep = stage == "probe" and pattern == "rotation_chirp"
    multiband = stage == "probe" and pattern == "multiband_chirp"
    delta_policy = controller.get("probe_measured_delta_policy", "stop")
    if delta_policy not in ("stop", "diagnostic") or (delta_policy == "diagnostic" and stage != "probe"):
        errors.append("probe_measured_delta_policy must be stop, or diagnostic in probe stage")
    if multiband and "probe_measured_delta_policy" not in controller:
        errors.append("multiband must explicitly declare probe_measured_delta_policy")
    bounded = stage == "probe" and pattern == "bounded"
    if stage == "probe" and not (sweep or bounded or multiband):
        errors.append("probe_pattern must explicitly select bounded, rotation_chirp or multiband_chirp")
    if recorder.get("encoder_zero_inner_knee_deg") != 105.0:
        errors.append("this hardware run records the confirmed 105-degree DM set-zero pose")
    if passive:
        return errors
    law = controller.get("control_law", "cascade_pi")
    pd = law == "rl_pd"
    if law not in ("cascade_pi", "rl_pd"):
        errors.append("control_law must be cascade_pi or rl_pd")
    frequency = 1000.0
    if any(params.get("control_frequency_hz") != frequency for params in (controller, recorder)):
        errors.append(f"paired controller and recorder control_frequency_hz must both be {frequency:g} Hz")
    if pd:
        if controller.get("reference_frequency_hz") not in (50.0, 200.0):
            errors.append("PD reference_frequency_hz must explicitly be 50 or 200 Hz")
        for key in ("ki_velocity", "integral_limit", "gravity_feedforward_model"):
            if controller.get(key) != [0.0] * 4:
                errors.append(f"RL PD {key} must be four zeros")
        if (controller.get("dm_motor_family") != "DM-J8009"
                or controller.get("dm_rated_torque_nm") != 20.0
                or controller.get("dm_peak_torque_nm") != 40.0):
            errors.append("RL PD requires explicit DM-J8009 manufacturer rated20/peak40 basis")
    array_sizes = dict(ARRAY_SIZES)
    if pd:
        del array_sizes["kp_position"], array_sizes["kp_velocity"]
        array_sizes.update(pd_kp=4, pd_kd=4)

    for key, count in {**array_sizes,
                       **(TRAJECTORY_ARRAY_SIZES if stage == "trajectory" else {})}.items():
        values = controller.get(key)
        if not isinstance(values, list) or len(values) != count:
            errors.append(f"{key}: {count} measured finite values required")
        elif not all(isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)
                     for x in values):
            errors.append(f"{key}: all values must be finite numbers")
    signs = controller.get("model_sign")
    if isinstance(signs, list) and len(signs) == 4 and any(x not in (-1, 1) for x in signs):
        errors.append("model_sign: each measured sign must be +1 or -1")
    caps = hardware.get("dm_control_torque_max")
    if not isinstance(caps, list) or len(caps) != 4 or not all(
            isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and value > 0 for value in caps):
        errors.append("wheel_leg.dm_control_torque_max needs four positive measured limits")
    elif pd and any(value > 40.0 for value in caps):
        errors.append("RL PD hardware torque cap exceeds DM-J8009 peak 40 Nm")
    elif not pd and stage == "hold" and any(value > 20.0 for value in caps):
        errors.append("HOLD hardware torque command caps must stay at or below 20 Nm")
    elif not pd and stage == "probe" and any(value > 20.0 for value in caps):
        errors.append("PROBE hardware torque command caps must stay at or below 20 Nm")
    requested = controller.get("max_torque")
    if isinstance(caps, list) and len(caps) == 4 and isinstance(requested, list) \
            and len(requested) == 4 and all(isinstance(value, (int, float)) for value in requested):
        # Hardware D order [LH,RH,LK,RK]; controller P4 [LH,LK,RH,RK].
        hardware_in_p4 = [caps[index] for index in (0, 2, 1, 3)]
        if any(desired > allowed for desired, allowed in zip(requested, hardware_in_p4)):
            errors.append("max_torque P4 must not exceed the corresponding hardware D torque cap")
    for key in (*(k for k in SCALARS if not (pd and multiband and k in ("hold_test_duration_s", "beta_margin_degrees"))), *(TRAJECTORY_SCALARS if stage == "trajectory" else ()),
                *(PROBE_SCALARS if bounded else ()), *(SWEEP_SCALARS if sweep else ()),
                *(MULTIBAND_SCALARS if multiband else ())):
        value = controller.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            errors.append(f"{key}: measured finite scalar required")
    if isinstance(controller.get("ready_timeout_s"), (int, float)) \
            and controller["ready_timeout_s"] <= 0.15:
        errors.append("ready_timeout_s must exceed the paired DM 100-tick enable sequence")
    if isinstance(controller.get("feedback_timeout_s"), (int, float)) \
            and controller["feedback_timeout_s"] > .05:
        errors.append("feedback_timeout_s cannot exceed the hardware 50 ms freshness gate")
    tick_budget = controller.get("max_tick_gap_s")
    if isinstance(tick_budget, (int, float)) and not .001 < tick_budget <= .02:
        errors.append("max_tick_gap_s must be in (0.001, 0.020] seconds")
    for key in (*(LISTS if stage != "probe" else ()),
                *(TRAJECTORY_LISTS if stage == "trajectory" else ()),
                *(("probe_frequencies_hz", "probe_validation_frequencies_hz") if bounded else ()),
                *(("sweep_common_hz", "sweep_relative_hz") if sweep else ())):
        values = controller.get(key)
        if not isinstance(values, list) or not values or not all(
                isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                for x in values):
            errors.append(f"{key}: nonempty measured finite list required")
    duration = controller.get("hold_test_duration_s")
    if isinstance(duration, (int, float)) and not 0.1 <= duration <= 5.0:
        errors.append("hold_test_duration_s must be in [0.1, 5.0] seconds")
    if sweep or multiband:
        if controller.get("phase_api_angle") is not True:
            errors.append("rotation/multiband chirp requires phase_api_angle: true for continuous turns")
        if recorder.get("support_fixture") != "rigid_fixed_frame" or recorder.get("wheels_off_ground") is not True:
            errors.append("rotation/multiband chirp requires fixed chassis with wheels off ground")
    if multiband:
        load_offset = controller.get("multiband_load_offset", 0.0)
        validation_scale = controller.get("multiband_validation_relative_scale", 1.0)
        valid_load = isinstance(load_offset, (int, float)) and not isinstance(load_offset, bool) \
            and math.isfinite(load_offset) and 0 <= load_offset <= .2
        valid_scale = isinstance(validation_scale, (int, float)) and not isinstance(validation_scale, bool) \
            and math.isfinite(validation_scale) and validation_scale > 0
        if not valid_load:
            errors.append("multiband_load_offset must be finite in [0, 0.2] rad")
        if not valid_scale:
            errors.append("multiband_validation_relative_scale must be positive and finite")
        loaded_revision = recorder.get("trajectory_revision") == "pair_multiband_v4_pd_loaded_1khz"
        if loaded_revision and (not pd or "multiband_load_offset" not in controller
                                or "multiband_validation_relative_scale" not in controller):
            errors.append("loaded PD revision requires explicit loading and validation scale")
        if (valid_load and load_offset > 0 or valid_scale and validation_scale != 1) and not loaded_revision:
            errors.append("explicit loading/validation scaling requires loaded PD trajectory_revision")
        valid = {}
        for key, size in MULTIBAND_ARRAYS.items():
            values = controller.get(key)
            valid[key] = isinstance(values, list) and len(values) == size and all(
                isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) and v > 0 for v in values)
            if not valid[key]:
                errors.append(f"{key}: {size} positive finite values required")
            elif key.endswith("_hz") and any(a >= b for a, b in zip(values, values[1:])):
                errors.append(f"{key}: frequency edges must be strictly ordered")
        for key in MULTIBAND_SCALARS:
            value = controller.get(key)
            if isinstance(value, (int, float)) and value <= 0:
                errors.append(f"{key} must be positive")
        ramp, validation = controller.get("multiband_ramp_s"), controller.get("multiband_validation_s")
        if isinstance(ramp, (int, float)) and isinstance(validation, (int, float)) \
                and valid["multiband_band_s"] and 2 * ramp >= min(validation, *controller["multiband_band_s"]):
            errors.append("multiband_ramp_s must leave a full-amplitude interval")
        # Same schedule as PairMultibandPlan: six four-block pose experiments,
        # two outward/return grids, two step sets, and two validation records.
        needed = (*MULTIBAND_SCALARS, "ready_timeout_s")
        if valid["multiband_band_s"] and all(isinstance(controller.get(k), (int, float))
                and math.isfinite(controller[k]) and controller[k] > 0 for k in needed):
            p = controller
            low, mid, high = p["multiband_band_s"]
            duration = (6 * (2 * low + mid + high) + 77 * p["multiband_dwell_s"]
                        + 34 * p["multiband_eighth_turn_s"] + 16 * p["multiband_step_rise_s"]
                        + 2 * p["multiband_validation_s"])
            if valid_load:
                duration += math.ceil(load_offset / .05) * (p["multiband_eighth_turn_s"] + p["multiband_dwell_s"])
            if p["ready_timeout_s"] < 1 + math.ceil((math.ceil(duration / .001) + 1) / 128) * .001:
                errors.append("ready_timeout_s cannot cover multiband preflight plus enable")
        revision = "pair_multiband_v4_pd_loaded_1khz" if loaded_revision else \
            "pair_multiband_v3_pd_1khz" if pd else "pair_multiband_v2_1khz"
        if recorder.get("trajectory_revision") != revision:
            errors.append(f"multiband trajectory_revision must match {revision}")
    if sweep:
        for key in SWEEP_SCALARS:
            value = controller.get(key)
            if isinstance(value, (int, float)) and value <= 0:
                errors.append(f"{key} must be positive")
        for key in ("sweep_common_hz", "sweep_relative_hz"):
            values = controller.get(key)
            if isinstance(values, list) and (len(values) != 2 or not all(
                    isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in values) or not 0 < values[0] < values[1]):
                errors.append(f"{key} requires two positive ordered frequencies")
        def positive(key):
            value = controller.get(key)
            return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0
        if all(positive(key) for key in SWEEP_SCALARS):
            if 2 * controller["sweep_ramp_s"] >= min(controller["sweep_chirp_s"], controller["sweep_validation_s"]):
                errors.append("sweep_ramp_s must leave a full-amplitude chirp interval")
            for axis in ("common", "relative"):
                if controller[f"sweep_{axis}_step"] > controller[f"sweep_{axis}_amplitude"]:
                    errors.append(f"sweep_{axis}_step exceeds its amplitude")
            low, high = controller.get("spring_delta_min"), controller.get("spring_delta_max")
            if isinstance(low, list) and isinstance(high, list) and len(low) == len(high) == 2 \
                    and all(isinstance(v, (int, float)) for v in (*low, *high)):
                room = 2 * controller["sweep_relative_amplitude"] + controller["sweep_relative_margin"]
                margin = controller.get("spring_margin", 0)
                if isinstance(margin, (int, float)) and any(b - a <= 2 * (room + margin) for a, b in zip(low, high)):
                    errors.append("sweep_relative_amplitude leaves no interior for two linkage shapes")
    if bounded:
        bounds = {"probe_common_amplitude": .05, "probe_knee_amplitude": .10,
                  "probe_sine_common_amplitude": .025, "probe_sine_knee_amplitude": .02,
                  "probe_move_s": 5.0, "probe_hold_s": 2.0, "probe_excite_s": 15.0,
                  "probe_validation_s": 15.0, "probe_gravity_hold_s": 3.0,
                  "probe_step_amplitude": .04, "probe_step_rise_s": 2.0,
                  "probe_triangle_amplitude": .03, "probe_triangle_frequency_hz": .5,
                  "probe_triangle_s": 20.0, "probe_min_interior_delta": .05}
        for key, upper in bounds.items():
            value = controller.get(key)
            if isinstance(value, (int, float)) and (value <= 0 or value > upper):
                errors.append(f"{key} must be positive and <= {upper}")
        if isinstance(controller.get("probe_step_rise_s"), (float, int)) \
                and controller["probe_step_rise_s"] < .5:
            errors.append("probe_step_rise_s requires at least 0.5 seconds")
        frequencies = controller.get("probe_frequencies_hz")
        if isinstance(frequencies, list) and (len(frequencies) != 2
                or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in frequencies)
                or not 0 < frequencies[0] < frequencies[1] <= 1.0):
            errors.append("probe_frequencies_hz must contain two ordered frequencies <= 1 Hz")
        validation = controller.get("probe_validation_frequencies_hz")
        if isinstance(validation, list) and (len(validation) != 2
                or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in validation)
                or not 0 < validation[0] < validation[1] <= 1.0):
            errors.append("probe_validation_frequencies_hz requires two ordered frequencies <= 1 Hz")
        progress = controller.get("probe_min_interior_delta")
        knee = controller.get("probe_knee_amplitude")
        if isinstance(progress, (float, int)) and isinstance(knee, (float, int)) \
                and not 0 < progress < knee:
            errors.append("probe_min_interior_delta must be less than knee amplitude")
    if stage == "probe" and isinstance(requested, list) and all(isinstance(x, (int, float)) for x in requested) \
            and any(x > (40.0 if pd else 20.0) for x in requested):
        errors.append("PROBE torque cap exceeds its explicit motor/profile limit")
    if pd:
        for key, strictly_positive in (("pd_kp", True), ("pd_kd", False)):
            values = controller.get(key, [])
            if isinstance(values, list) and any(
                    isinstance(x, (int, float)) and (x <= 0 if strictly_positive else x < 0)
                    for x in values):
                errors.append(f"{key}: invalid PD gain")
    for side in ("left", "right"):
        beta = controller.get(f"{side}_beta_degrees")
        delta = controller.get(f"{side}_delta_rad")
        if (isinstance(beta, list) and isinstance(delta, list) and len(beta) >= 3
                and len(beta) == len(delta) and all(isinstance(x, (float, int)) for x in (*beta, *delta))
                and all(math.isfinite(x) for x in (*beta, *delta))):
            if (min(beta) < 40.0 or max(beta) > 105.0
                    or any(a >= b for a, b in zip(beta, beta[1:]))
                    or not (all(a < b for a, b in zip(delta, delta[1:]))
                            or all(a > b for a, b in zip(delta, delta[1:])))):
                errors.append(f"{side} LUT must be monotone and within the real 40..105 degree range")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    args = parser.parse_args()
    contents = args.profile.read_text(encoding="utf-8")
    if any(isinstance(token, (yaml.tokens.AnchorToken, yaml.tokens.AliasToken))
           for token in yaml.scan(contents)):
        print("profile incomplete: ROS 2 parameter YAML does not support anchors or aliases")
        return 2
    profile = yaml.safe_load(contents)
    errors = check(profile if isinstance(profile, dict) else {})
    for error in errors:
        print(f"profile incomplete: {error}")
    if errors:
        return 2
    print("Static profile complete; hardware feedback, linkage and torque limits still require on-rig validation.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
