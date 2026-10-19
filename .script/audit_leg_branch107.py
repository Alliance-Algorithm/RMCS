#!/usr/bin/env python3
"""Exercise the production pair decoder and ORT metadata gate on CPU only."""
import argparse
from datetime import datetime, timezone
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(argv, **kwargs):
    return subprocess.run(argv, check=True, capture_output=True, **kwargs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--training-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    training = args.training_root.resolve()
    sys.path.insert(0, str(training / 'src'))
    import numpy as np
    import onnx
    from onnx import TensorProto, helper
    import torch
    from wheeled_tasks.chassis.fixed_flat_rehearsal import (
        install_fixed_flat_rehearsal, load_fixed_flat_rehearsal)
    from wheeled_tasks.chassis.flat_episode_runtime import build_flat_episode_runtime
    from wheeled_tasks.chassis.v5_control import V5Control

    torch.set_num_threads(1)
    src = ROOT / 'rmcs_ws/src/rmcs_rl/src'
    tests = ROOT / 'rmcs_ws/src/rmcs_rl/test'
    models = ROOT / 'rmcs_ws/src/rmcs_rl/models/wheel_leg/deployment'
    ort = ROOT / 'rmcs_ws/build/rmcs_rl/_deps/ort_binary-src'
    assert (ort / 'VERSION_NUMBER').read_text().strip() == '1.20.0'
    baseline = command(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).stdout.strip()
    assert command(['git', 'diff', '--', 'rmcs_ws/src/rmcs_bringup/config',
                    'rmcs_ws/src/rmcs_rl/models', 'rmcs_ws/src/rmcs_rl/src/action.cpp'],
                   cwd=ROOT, text=True).stdout == ''
    configuration = (src / 'configuration.cpp').read_text()
    assert ('validate_leg_target_recovery_profile(\n'
            '            leg_target_winding_, recovery_enabled_, native.model_spec_sha256);') in configuration

    def read(name):
        return json.loads((training / name).read_text())

    cfg = read('configs/v7_formal_20261012/flat_continuation_034.json')
    cfg.update(formal_dynamic_nonsteady_birth='v7-dynamic-source-birth-v1',
               formal_foundation_nominal_birth=False)
    directory = cfg['asset_directory']
    manifest, spec, fit = (read(directory + '/' + name) for name in
                           ('manifest.json', 'model_spec.json', 'fit_10mpa.json'))
    prior = read(cfg['control_math_source'])
    assert sha(training / directory / 'model_spec.json') == V5Control.BRANCH107_MODEL_SPEC_SHA256
    owner = build_flat_episode_runtime(cfg, manifest, spec, training, 4096, 'cpu')
    rehearsal = load_fixed_flat_rehearsal(
        cfg, training, 4096, ordinary_lane_ids=tuple(range(4096)))
    install_fixed_flat_rehearsal(rehearsal, owner.commands)
    nominal_mask = owner.commands.fixed_request_mask.clone()
    nominal_q = torch.tensor([manifest['nominal_joint_pos'][name]
                             for name in owner.tree_joint_names], dtype=torch.float64).expand(4096, -1)
    state = owner.prepare_reset(
        torch.arange(4096), torch.Generator().manual_seed(290929),
        origins=torch.zeros(4096, 3), reset_xy_world=torch.zeros(4096, 2),
        ground_z_world=torch.zeros(4096), nominal_q=nominal_q, nominal_mask=nominal_mask)
    indices = [owner.tree_joint_names.index(name) for name in V5Control.ACTIVE]
    q32 = state.joint_position[:, indices].float()
    settings = {**cfg['v5_control'], 'leg_winding_version': 'paired_branch107_v1'}
    native = V5Control(manifest, spec, fit, prior, settings, 'cpu')
    math64 = V5Control(manifest, spec, fit, prior, settings, 'cpu')
    bounds = torch.tensor(V5Control.BRANCH107_SIGNED_DELTA_BOUNDS_RAD, dtype=torch.float64)
    signs = torch.tensor([1., -1.], dtype=torch.float64)
    tau = 2 * torch.pi

    with tempfile.TemporaryDirectory(prefix='rmcs-branch107-cpu-') as scratch:
        scratch = Path(scratch)
        winding_binary, new_test, loader = scratch / 'winding', scratch / 'branch107', scratch / 'loader'
        flags = ['g++', '-std=c++23', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-I', str(src)]
        command(flags + [str(tests / 'leg_target_winding_test.cpp'), '-o', str(winding_binary)])
        old_selftest = command([str(winding_binary)], text=True).stdout.strip()
        command(flags + [str(tests / 'leg_target_branch107_test.cpp'), str(src / 'recovery_profile.cpp'),
                         '-lcrypto', '-o', str(new_test)])
        new_selftest = command([str(new_test), str(models)], text=True).stdout.strip()
        command(flags + ['-DRMCS_WINDING_METADATA', '-I', str(ort / 'include'),
                         str(tests / 'onnx_winding_test.cpp'), str(src / 'onnx_policy.cpp'),
                         '-L', str(ort / 'lib'), '-Wl,-rpath,' + str(ort / 'lib'),
                         '-lonnxruntime', '-o', str(loader)])
        nominal = [float(value) for value in command(
            [str(winding_binary), '--nominal'], text=True).stdout.split()]
        math64.nominal = torch.tensor(
            [nominal[0], nominal[1], 0., nominal[2], nominal[3], 0.], dtype=torch.float64)
        # A second production helper compiled from HEAD establishes bytewise
        # output preservation for both existing winding versions.
        old_src = scratch / 'old'
        old_src.mkdir()
        for name in ('policy.hpp', 'leg_target_winding.hpp', 'onnx_policy.cpp'):
            (old_src / name).write_bytes(command(
                ['git', 'show', baseline + ':rmcs_ws/src/rmcs_rl/src/' + name], cwd=ROOT).stdout)
        old_binary, old_loader = scratch / 'old_winding', scratch / 'old_loader'
        command(['g++', '-std=c++23', '-O2', '-I', str(old_src),
                 str(tests / 'leg_target_winding_test.cpp'), '-o', str(old_binary)])
        command(['g++', '-std=c++23', '-O2', '-DRMCS_WINDING_METADATA', '-I', str(old_src),
                 '-I', str(ort / 'include'), str(tests / 'onnx_winding_test.cpp'),
                 str(old_src / 'onnx_policy.cpp'), '-L', str(ort / 'lib'),
                 '-Wl,-rpath,' + str(ort / 'lib'), '-lonnxruntime', '-o', str(old_loader)])

        def cpp(mode, current, desired, executable=winding_binary):
            payload = torch.cat((current, desired), -1).double().numpy().astype(np.float64).tobytes()
            output = command([str(executable), '--binary', mode, 'v6'], input=payload).stdout
            return torch.from_numpy(np.frombuffer(output, dtype=np.float64).copy().reshape(-1, 4))

        cases = old_cases = 0
        max_math_error = max_native_error = max_native_delta_error = max_mean_error = 0.
        precision_groups = {}
        cuts = []
        corner_results = []

        def compare(action, current, current64=None, precision_group='ordinary'):
            nonlocal cases, old_cases, max_math_error, max_native_error
            nonlocal max_native_delta_error, max_mean_error
            current64 = current.double() if current64 is None else current64
            target64, _, clipped64 = math64.decode(action.double(), current64)
            target32, _, clipped32 = native.decode(action.float(), current.float())
            desired64 = math64.nominal[list(math64.LEGS)] + clipped64[:, math64.LEGS] * settings['leg_position_scale']
            actual = cpp('paired_branch107_v1', current64[:, math64.LEGS], desired64)
            error = float((actual - target64).abs().max())
            scale = max(1., float(actual.abs().max()))
            assert error <= 64 * np.finfo(np.float64).eps * scale
            delta = signs * (actual[:, 1::2] - actual[:, ::2])
            native_delta = signs * (target32[:, 1::2].double() - target32[:, ::2].double())
            assert bool(((delta >= bounds[:, 0]) & (delta <= bounds[:, 1])).all())
            assert bool(((native_delta >= bounds[:, 0]) & (native_delta <= bounds[:, 1])).all())
            desired_mean = desired64.reshape(-1, 2, 2).mean(-1)
            current_legs = current64[:, math64.LEGS].reshape(-1, 2, 2)
            feedback_mean = current_legs[..., 0] * .5 + current_legs[..., 1] * .5
            turn = torch.floor((feedback_mean - desired_mean) / tau + .5)
            desired32 = native.nominal[list(native.LEGS)] + clipped32[:, native.LEGS] * settings['leg_position_scale']
            native_mean = desired32.double().reshape(-1, 2, 2).mean(-1)
            native_feedback = current.float().double()[:, native.LEGS].reshape(-1, 2, 2)
            native_feedback_mean = native_feedback[..., 0] * .5 + native_feedback[..., 1] * .5
            native_turn = torch.floor((native_feedback_mean - native_mean) / tau + .5)
            turn_difference = turn - native_turn
            aligned_error = float((actual - target32.double()
                                  - turn_difference.repeat_interleave(2, -1) * tau).abs().max())
            group = precision_groups.setdefault(precision_group, {
                'cases': 0, 'common_turn_different_lanes': 0,
                'max_abs_target_error_rad': 0., 'max_target_error_after_common_turn_alignment_rad': 0.})
            group['cases'] += len(actual)
            group['common_turn_different_lanes'] += int((turn_difference != 0).any(-1).sum())
            group['max_abs_target_error_rad'] = max(group['max_abs_target_error_rad'],
                                                     float((actual - target32.double()).abs().max()))
            group['max_target_error_after_common_turn_alignment_rad'] = max(
                group['max_target_error_after_common_turn_alignment_rad'], aligned_error)
            mean_error = float((actual.reshape(-1, 2, 2).mean(-1) - desired_mean - turn * tau).abs().max())
            assert mean_error <= 8 * np.finfo(np.float64).eps * scale
            for mode in ('independent_nearest_v1', 'paired_common_v1'):
                now = cpp(mode, current64[:, math64.LEGS], desired64)
                prior_output = cpp(mode, current64[:, math64.LEGS], desired64, old_binary)
                assert now.numpy().tobytes() == prior_output.numpy().tobytes()
                old_cases += len(now)
            cases += len(actual)
            max_math_error = max(max_math_error, error)
            max_mean_error = max(max_mean_error, mean_error)
            max_native_error = max(max_native_error, float((actual - target32.double()).abs().max()))
            max_native_delta_error = max(max_native_delta_error, float((delta - native_delta).abs().max()))
            raw_delta = signs * (desired64[:, 1::2] - desired64[:, ::2])
            return actual, target32, turn, int(((raw_delta < bounds[:, 0]) | (raw_delta > bounds[:, 1])).any(-1).sum())

        shifts = [(-1000000, 1000000), (-100, 100), (-20, 20), (0, 0), (20, -20), (100, -100), (1000000, -1000000)]
        for corner in (None, *itertools.product((-3., 3.), repeat=4)):
            action = torch.zeros(4096, 6)
            if corner is not None:
                action[:, native.LEGS] = torch.tensor(corner)
            for left, right in shifts:
                current64 = q32.double()
                current64[:, list(native.LEGS)] += torch.tensor([left, left, right, right], dtype=torch.float64) * tau
                group = 'birth_turns_at_most_20' if max(abs(left), abs(right)) <= 20 else 'large_turn_stress'
                _, _, _, projected = compare(action, current64.float(), current64, group)
                corner_results.append({'leg_actions': [0.] * 4 if corner is None else list(corner),
                                       'common_turns': [left, right], 'projected_lanes': projected})
        for offset in (-torch.pi - 1e-6, -torch.pi, -torch.pi + 1e-6,
                       torch.pi - 1e-6, torch.pi, torch.pi + 1e-6):
            current64 = math64.nominal[None].clone()
            current64[:, math64.LEGS] += offset
            actual, target32, turn, _ = compare(
                torch.zeros(1, 6), current64.float(), current64, 'half_turn_cut')
            cuts.append({'offset_rad': offset, 'rmcs_double_turns': turn[0].tolist(),
                         'rmcs_double_targets_rad': actual[0].tolist(),
                         'train_native_float32_targets_rad': target32[0].tolist()})
        generator = torch.Generator().manual_seed(230539)
        random_action = torch.rand(1024, 6, generator=generator) * 12 - 6
        random_current = torch.rand(1024, 6, generator=generator) * 200 - 100
        compare(random_action, random_current, precision_group='random_control_coordinates')

        fixtures = [(None, None), ('independent_nearest_v1', None), ('paired_common_v1', None),
                    ('independent_nearest_v1', 'wrong'), ('paired_common_v1', 'wrong'),
                    ('paired_branch107_v1', None), ('paired_branch107_v1', ''),
                    ('paired_branch107_v1', 'wrong'),
                    ('paired_branch107_v1', V5Control.BRANCH107_MODEL_SPEC_SHA256), ('unknown', None)]
        metadata_results = []
        for index, (version, model_sha) in enumerate(fixtures):
            model = helper.make_model(helper.make_graph([
                helper.make_node('Constant', [], ['actions'], value=helper.make_tensor(
                    'zero', TensorProto.FLOAT, [1, 6], [0.] * 6))], 'branch107_metadata',
                [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [1, 35])],
                [helper.make_tensor_value_info('actions', TensorProto.FLOAT, [1, 6])]),
                opset_imports=[helper.make_opsetid('', 17)], ir_version=10)
            props = {} if version is None else {'leg_winding_version': version}
            if model_sha is not None:
                props['leg_target_model_spec_sha256'] = model_sha
            helper.set_model_props(model, props)
            path = scratch / ('metadata_' + str(index) + '.onnx')
            onnx.save(model, path)
            for configured in (None, 'independent_nearest_v1', 'paired_common_v1', 'paired_branch107_v1'):
                argv = [str(loader), str(path)] + ([] if configured is None else [configured])
                result = subprocess.run(argv, capture_output=True, text=True)
                effective_model = version or 'independent_nearest_v1'
                effective_configured = configured or 'independent_nearest_v1'
                expected = (effective_model == effective_configured
                            and (version in (None, 'independent_nearest_v1', 'paired_common_v1')
                                 or version == 'paired_branch107_v1' and model_sha == V5Control.BRANCH107_MODEL_SPEC_SHA256))
                assert (result.returncode == 0) == expected, (version, model_sha, configured, result.stderr)
                metadata_results.append({'model_version': version, 'model_spec_sha256': model_sha,
                                         'configured_version': configured, 'accepted': expected,
                                         'message': (result.stdout if expected else result.stderr).strip()})

        legacy_models = []
        for name in command(['git', 'ls-files', 'rmcs_ws/src/rmcs_rl/models/wheel_leg/*.onnx'],
                            cwd=ROOT, text=True).stdout.splitlines():
            old = command([str(old_loader), str(ROOT / name)], text=True).stdout
            new = command([str(loader), str(ROOT / name)], text=True).stdout
            assert old == new
            legacy_models.append({'path': name, 'sha256': sha(ROOT / name), 'output_unchanged': True})

    sources = ['rmcs_ws/src/rmcs_rl/src/' + name for name in
               ('leg_target_winding.hpp', 'action.cpp', 'configuration.cpp', 'onnx_policy.cpp',
                'recovery_profile.cpp', 'recovery_profile.hpp')]
    sources += ['rmcs_ws/src/rmcs_rl/test/leg_target_branch107_test.cpp', '.script/audit_leg_branch107.py']
    receipt = {
        'schema': 'rmcs-paired-branch107-cpu-audit-v1', 'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'rmcs_base_commit': baseline, 'source_sha256': {name: sha(ROOT / name) for name in sources},
        'training_decoder_sha256': sha(training / 'src/wheeled_tasks/chassis/v5_control.py'),
        'model_spec_sha256': V5Control.BRANCH107_MODEL_SPEC_SHA256,
        'canonical_delta_bounds_rad': V5Control.BRANCH107_SIGNED_DELTA_BOUNDS_RAD,
        'selftests': [old_selftest, new_selftest], 'cross_language_math_cases': cases,
        'old_version_bytewise_cases': old_cases, 'math_float64_max_target_error_rad': max_math_error,
        'common_mean_max_rounding_error_rad': max_mean_error,
        'native_float32_vs_rmcs_double_max_target_error_rad': max_native_error,
        'native_float32_vs_rmcs_double_max_delta_error_rad': max_native_delta_error,
        'native_precision_groups': precision_groups,
        'native_and_rmcs_canonical_domain_violations': 0,
        'birth_sampler': {'population': 4096, 'seed': 290929, 'nominal_lanes': int(nominal_mask.sum())},
        'corner_cases': corner_results, 'angular_cut_cases': cuts,
        'ort_version': '1.20.0', 'actual_ort_metadata_cases': metadata_results,
        'tracked_legacy_models': legacy_models,
        'recovery_scope': 'New version + enabled script recovery rejects authenticated old105 profile; explicit authenticated current107 pair passes the SHA guard. Disabled script recovery retains prior configuration defaults. Neither case grants hardware readiness.',
        'unchanged': ['action clipping', 'real control_q selection', 'wheel outputs', 'action history',
                      'PD/FSM/torque limits', 'default YAML', 'tracked models', 'old two decoder versions'],
        'scope': 'CPU production decoder, production frozen-profile loading and real ORT metadata checks. No ROS node, hardware, SSH, native simulator, GPU, PPO, online run or budget change.',
        'limitations': ['The relative target projection changes PD preload; it does not prove closed-chain physics is repaired.',
                        'Double RMCS and native float32 retain different precision; common whole turns can differ at a half-turn cut, and coarse float32 large-turn coordinates can collapse a pair. No native bit parity is claimed.',
                        'No policy behavior, nonsteady takeover performance or hardware safety qualification was measured.'],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: receipt[key] for key in ('cross_language_math_cases', 'old_version_bytewise_cases',
                     'math_float64_max_target_error_rad', 'native_and_rmcs_canonical_domain_violations')}, indent=2))


if __name__ == '__main__':
    main()
