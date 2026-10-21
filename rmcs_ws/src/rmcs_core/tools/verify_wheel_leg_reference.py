#!/usr/bin/env python3
"""Verify the split wheel-leg deployment against a fetched RMCS recovery branch.

Build rmcs_rl first. This compares configuration/assets and replays the native
controller and actor without constructing hardware or issuing control commands.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import yaml

CORE = Path(__file__).resolve().parents[1]
RL = CORE.parent / 'rmcs_rl'
ROOT = CORE.parents[2]
CONFIG = 'rmcs_ws/src/rmcs_bringup/config/'
REFERENCE_PACKAGE = 'rmcs_ws/src/rmcs_rl/'


def run(command, **kwargs):
    return subprocess.check_output(command, **kwargs)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def compare(expected, actual):
    left, right = expected.decode().splitlines(), actual.decode().splitlines()
    require(len(left) == len(right), 'Replay row count differs')
    counts = {'recovery': 0, 'model': 0}
    maximum = dict.fromkeys(counts, 0.0)
    for row, (a, b) in enumerate(zip(left, right), 1):
        a, b = a.split(), b.split()
        require(len(a) == len(b) and a[0] == b[0], f'Replay row {row} differs')
        kind = a[0]
        counts[kind] += 1
        for column, (x, y) in enumerate(zip(a[1:], b[1:]), 1):
            x, y = float(x), float(y)
            require(math.isfinite(x) and math.isfinite(y), f'Nonfinite row {row}')
            difference = abs(x - y)
            maximum[kind] = max(maximum[kind], difference)
            tolerance = 0.0 if kind == 'recovery' and column <= 8 else 1e-6
            require(difference <= tolerance,
                    f'{kind} row {row}, column {column}: reference={x}, current={y}')
    require(counts == {'recovery': 6400, 'model': 256}, f'Incomplete replay: {counts}')
    for kind, count in counts.items():
        print(f'{kind}: {count} snapshots, max absolute difference {maximum[kind]:.9g}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', default='origin/dev/wheel_leg_rl')
    parser.add_argument('--build-dir', type=Path, default=CORE.parents[1] / 'build/rmcs_rl')
    parser.add_argument('--policy-receipt', type=Path,
                        help='Allow the authenticated flat 200 Hz actor/time/command migration')
    args = parser.parse_args()
    reference = run(['git', 'rev-parse', args.reference], cwd=ROOT, text=True).strip()
    print(f'Reference: {reference}')

    def source(path):
        return run(['git', 'show', f'{reference}:{path}'], cwd=ROOT)

    original = yaml.safe_load(source(CONFIG + 'wheel-leg-infantry-recovery.yaml'))
    current = yaml.safe_load((ROOT / CONFIG / 'wheel-leg-infantry-rl.yaml').read_text())
    expected = original['rl_controller']['ros__parameters']
    controller = current['wheel_leg_controller']['ros__parameters']
    bridge = current['rl_bridge']['ros__parameters']
    model = RL / bridge['rl_model_path']
    overrides = {}
    expected_commands = dict(original['wheel_leg_chassis_controller']['ros__parameters'])
    if args.policy_receipt:
        receipt = json.loads(args.policy_receipt.read_text())
        require(receipt['schema'] == 'rmcs-flat-policy-deployment-v1'
                and receipt['model_filename'] == model.name
                and receipt['model_sha256'] == hashlib.sha256(model.read_bytes()).hexdigest(),
                'Policy receipt does not identify the deployed actor')
        require(receipt['policy_hz'] == 200 and receipt['physics_hz'] == 1000
                and receipt['action_timeout_seconds'] == 0.015
                and receipt['vx_max_m_s'] == 3.0 and receipt['yaw_rate_max_rad_s'] == 4.0
                and receipt['heading_hold_yaw_rate_max_rad_s'] == 4.0,
                'Unsupported policy migration')
        overrides = dict(rl_model_path=bridge['rl_model_path'], policy_rate=receipt['policy_hz'],
                         inference_timeout=receipt['action_timeout_seconds'],
                         heading_hold_yaw_rate_max=receipt['heading_hold_yaw_rate_max_rad_s'],
                         yaw_rate_max=receipt['yaw_rate_max_rad_s'])
        expected_commands.update(yaw_rate_max=4.0, spin_yaw_rate=4.0)
        print('Actor/time/command migration verified against the policy receipt')
    require(current['wheel_leg'] == original['wheel_leg'], 'Hardware parameters differ')
    require(current['wheel_leg_chassis_controller']['ros__parameters'] == expected_commands,
            'Chassis command parameters differ')
    renamed = {'model_path': 'rl_model_path', 'rl_inference_frequency': 'policy_rate',
               'rl_history_length': 'history_length'}
    for name, value in expected.items():
        if name == 'spin_yaw_rate':
            continue
        key = renamed.get(name, name)
        value = overrides.get(key, value)
        actual = controller.get(key, bridge.get(key))
        if key in ('recovery_profile_path', 'height_lookup_path'):
            value = value.replace('models/wheel_leg/deployment/', 'config/wheel_leg/')
        require(actual == value, f'Parameter {name}: reference={value}, current={actual}')
    names = bridge['joint_names']
    require([bridge['default_joint_pos'][name] for name in names]
            == controller['nominal_model_pos'], 'Bridge/controller nominal poses differ')
    require(bridge['inference_timeout'] == bridge['max_action_age']
            == controller['inference_timeout'], 'Action age contracts differ')
    components = current['rmcs_executor']['ros__parameters']['components']
    require('rmcs_rl::RlBridge -> rl_bridge' in components, 'Missing generic bridge')
    require('rmcs_core::controller::chassis::WheelLegController -> wheel_leg_controller'
            in components, 'Missing chassis actuator controller')
    require(all('RlController' not in item and 'Launcher' not in item for item in components),
            'Legacy inference/controller component in the profile')
    for name in ('recovery_profile_path', 'height_lookup_path'):
        asset = CORE / controller[name]
        require(asset.read_bytes() == source(REFERENCE_PACKAGE + expected[name]),
                f'Recovery asset differs: {name}')
        print(f'Asset SHA256 {asset.name}: {hashlib.sha256(asset.read_bytes()).hexdigest()}')
    if not args.policy_receipt:
        require(model.read_bytes() == source(REFERENCE_PACKAGE + expected['model_path']),
                'Actor bytes differ')
    print(f'Actor SHA256: {hashlib.sha256(model.read_bytes()).hexdigest()}')
    for path in ('hardware/wheel-leg.cpp', 'hardware/device/dm_motor.hpp',
                 'hardware/device/dr16.hpp', 'broadcaster/value_broadcaster.cpp'):
        full = 'rmcs_ws/src/rmcs_core/src/' + path
        require((ROOT / full).read_bytes() == source(full), f'Hardware source differs: {path}')
    print('PASS: component ownership, hardware, calibration and recovery parameters')

    build = args.build_dir.resolve()
    require((build / 'librmcs_rl.so').exists(), 'Build rmcs_rl first')
    headers = list(build.glob('onnxruntime-*/include/onnxruntime_cxx_api.h'))
    require(len(headers) == 1, 'Cannot identify ONNX Runtime build')
    ort = headers[0].parents[1]
    parameters = dict(controller)
    parameters['default_joint_pos'] = controller['nominal_model_pos']
    with tempfile.TemporaryDirectory(prefix='rmcs-wheel-reference-') as directory:
        temporary = Path(directory)
        for path in run(['git', 'ls-tree', '-r', '--name-only', reference,
                         REFERENCE_PACKAGE + 'src'], cwd=ROOT, text=True).splitlines():
            (temporary / Path(path).name).write_bytes(source(path))
        settings = temporary / 'parameters.json'
        settings.write_text(json.dumps(parameters))
        compiler = shlex.split(os.environ.get('CXX', 'c++'))
        common = ['-std=c++23', '-O2', '-I' + str(ort / 'include'),
                  str(CORE / 'test/wheel_leg_reference_replay.cpp')]
        reference_binary, current_binary = temporary / 'reference', temporary / 'current'
        subprocess.check_call(compiler + common + [
            '-DRMCS_WHEEL_LEG_REFERENCE', '-I' + str(temporary),
            str(temporary / 'joint_reference_recovery_controller.cpp'),
            str(temporary / 'recovery_profile.cpp'), str(temporary / 'onnx_policy.cpp'),
            '-L' + str(ort / 'lib'), '-lonnxruntime', '-lcrypto',
            '-Wl,-rpath,' + str(ort / 'lib'), '-o', str(reference_binary)])
        deployed = CORE / 'src/controller/chassis/wheel_leg'
        subprocess.check_call(compiler + common + [
            '-I' + str(deployed), '-I' + str(RL / 'src'), '-I' + str(RL / 'include'),
            str(deployed / 'joint_reference_recovery_controller.cpp'),
            str(deployed / 'recovery_profile.cpp'), '-L' + str(build), '-lrmcs_rl',
            '-L' + str(ort / 'lib'), '-lonnxruntime', '-lcrypto',
            '-Wl,-rpath,' + str(build), '-o', str(current_binary)])
        inputs = [str(CORE / controller['recovery_profile_path']),
                  str(CORE / controller['height_lookup_path']), str(model), str(settings)]
        compare(run([str(reference_binary)] + inputs), run([str(current_binary)] + inputs))
    print('PASS: offline replay parity; no vehicle was constructed or driven')


if __name__ == '__main__':
    main()
