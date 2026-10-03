#!/usr/bin/env python3
"""Compile the unmodified handoff JSON into C++ test data (stdlib only).

The expected outputs come from the exported policy, not a reimplementation of
its network. Keep the source JSON alongside the packaged ONNX for provenance.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def values(items: list[float], size: int, as_float: bool = True) -> str:
    if len(items) != size or not all(math.isfinite(float(x)) for x in items):
        raise ValueError(f"Expected {size} finite values")
    result = []
    for value in items:
        literal = format(float(value), ".17g")
        if "." not in literal and "e" not in literal:
            literal += ".0"
        result.append(literal + ("f" if as_float else ""))
    return "{" + ", ".join(result) + "}"


def generate(source: Path, destination: Path) -> None:
    fixture = json.loads(source.read_text())
    if fixture["schema"] == "rmcs-v6-native-controller-fixture-v1":
        return generate_controller(fixture, destination)
    if (fixture["schema"] != "wheel-leg-policy-abi-vectors-v1"
            or fixture["case_count"] != 25 or len(fixture["cases"]) != 25
            or not fixture["outputs_before_clipping"]
            or fixture["atol"] != 1e-5 or fixture["rtol"] != 1e-5):
        raise ValueError("Unexpected V6 policy ABI fixture contract")
    names = [case["name"] for case in fixture["cases"]]
    if len(names) != len(set(names)):
        raise ValueError("Duplicate ABI case name")
    lines = [
        "// Generated from the authoritative deployment/abi_test_vectors.json.",
        "#pragma once", '#include "policy.hpp"', "#include <array>",
        "#include <string_view>", "namespace rmcs::rl::test {",
        "struct AbiCase { std::string_view name; PolicyObservation observation; PolicyAction raw_action; };",
        f'inline constexpr std::string_view kOnnxSha256 = "{fixture["onnx_sha256"]}";',
        "inline constexpr std::array<AbiCase, 25> kAbiCases{{",
    ]
    for case in fixture["cases"]:
        lines.append("    {" + json.dumps(case["name"]) + ", "
                     + values(case["obs"], 35) + ", "
                     + values(case["raw_actions"], 6) + "},")
    lines += ["}};", "} // namespace rmcs::rl::test", ""]
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines))


def generate_controller(fixture: dict, destination: Path) -> None:
    lines = [
        "// Generated from the native Torch controller reference, not C++ results.",
        "#pragma once", '#include "policy.hpp"', "#include <array>",
        "#include <cstddef>", "#include <string_view>", "namespace rmcs::rl::test {",
        "struct ControllerCase { std::string_view name; std::size_t advance_ms; bool policy_step;",
        "std::array<double,6> q_api, dq_api; std::array<double,4> orientation_imu_wxyz;",
        "std::array<double,3> gyro_imu, command; PolicyObservation observation;",
        "PolicyAction raw_action, clipped_action; std::array<double,6> targets, torque_model, torque_api; };",
        f"inline constexpr std::array<ControllerCase, {len(fixture['cases'])}> kControllerCases{{{{",
    ]
    for case in fixture["cases"]:
        fields = [json.dumps(case["name"]), str(case["advance_ms"]),
                  "true" if case["policy_step"] else "false"]
        for key, size, is_float in [
                ("q_api", 6, False), ("dq_api", 6, False), ("orientation_imu_wxyz", 4, False),
                ("gyro_imu", 3, False), ("command", 3, False), ("obs", 35, True),
                ("raw_actions", 6, True), ("clipped_actions", 6, True), ("targets", 6, False),
                ("torque_model", 6, False), ("torque_api", 6, False)]:
            fields.append(values(case[key], size, is_float))
        lines.append("    {" + ", ".join(fields) + "},")
    lines += ["}};", "} // namespace rmcs::rl::test", ""]
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    generate(args.source, args.destination)
