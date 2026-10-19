#!/usr/bin/env python3
"""Compare actual C++ winding/ONNX loading with production V7 CPU decoders.

No ROS node, native simulator, checkpoint, GPU, or PPO optimizer is started.
The C++ test clients call the production header and production ONNX loader.
"""
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


def command(arguments, **kwargs):
    return subprocess.run(arguments, check=True, capture_output=True, **kwargs)


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
    from wheeled_tasks.chassis.fixed_flat_rehearsal import load_fixed_flat_rehearsal, install_fixed_flat_rehearsal
    from wheeled_tasks.chassis.flat_episode_runtime import build_flat_episode_runtime
    from wheeled_tasks.chassis.v5_control import V5Control

    torch.set_num_threads(1)
    src = ROOT / 'rmcs_ws/src/rmcs_rl/src'
    baseline_action = command(['git', 'show', '137d5ece:rmcs_ws/src/rmcs_rl/src/action.cpp'], cwd=ROOT, text=True).stdout
    actual_action = (src / 'action.cpp').read_text()
    tail = '    if (!v6 && recovery_started_ && recovery_observer_) {'
    assert baseline_action[baseline_action.index(tail):] == actual_action[actual_action.index(tail):]
    prefix = '                                : q_;\n'
    assert baseline_action.partition(prefix)[0] == actual_action.partition(prefix)[0]
    assert 'current[i] = control_q[i];' in actual_action
    assert 'wind_leg_targets(desired, current, leg_target_winding_)' in actual_action
    unchanged_profiles = command(['git', 'diff', '--name-only', '137d5ece', '--',
        'rmcs_ws/src/rmcs_bringup/config', 'rmcs_ws/src/rmcs_rl/models'], cwd=ROOT, text=True).stdout.splitlines()
    assert not unchanged_profiles
    ort = ROOT / 'rmcs_ws/build/rmcs_rl/_deps/ort_binary-src'
    assert (ort / 'VERSION_NUMBER').read_text().strip() == '1.20.0'
    read = lambda name: json.loads((training / name).read_text())
    cfg = read('configs/v7_formal_20261012/flat_continuation_034.json')
    cfg.update(formal_dynamic_nonsteady_birth='v7-dynamic-source-birth-v1', formal_foundation_nominal_birth=False)
    directory = cfg['asset_directory']
    manifest, spec, fit = (read(directory + '/' + name) for name in ('manifest.json', 'model_spec.json', 'fit_10mpa.json'))
    prior = read(cfg['control_math_source'])
    owner = build_flat_episode_runtime(cfg, manifest, spec, training, 4096, 'cpu')
    rehearsal = load_fixed_flat_rehearsal(cfg, training, 4096, ordinary_lane_ids=tuple(range(4096)))
    install_fixed_flat_rehearsal(rehearsal, owner.commands)
    nominal_mask = owner.commands.fixed_request_mask.clone()
    nominal_q = torch.tensor([manifest['nominal_joint_pos'][name] for name in owner.tree_joint_names], dtype=torch.float64).expand(4096, -1)
    state = owner.prepare_reset(torch.arange(4096), torch.Generator().manual_seed(290929),
        origins=torch.zeros(4096, 3), reset_xy_world=torch.zeros(4096, 2), ground_z_world=torch.zeros(4096),
        nominal_q=nominal_q, nominal_mask=nominal_mask)
    indices = [owner.tree_joint_names.index(name) for name in V5Control.ACTIVE]
    q32 = state.joint_position[:, indices].float()
    paired = V5Control(manifest, spec, fit, prior, {**cfg['v5_control'], 'leg_winding_version': 'paired_common_v1'}, 'cpu')

    with tempfile.TemporaryDirectory(prefix='v7-rmcs-winding-audit-') as scratch:
        scratch = Path(scratch)
        winding_binary, loader_binary = scratch / 'winding', scratch / 'loader'
        flags = ['g++', '-std=c++23', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-I', str(src)]
        command(flags + [str(ROOT / 'rmcs_ws/src/rmcs_rl/test/leg_target_winding_test.cpp'), '-o', str(winding_binary)])
        self_test = command([str(winding_binary)], text=True).stdout.strip()
        command(flags + ['-DRMCS_WINDING_METADATA', '-I', str(ort / 'include'),
            str(ROOT / 'rmcs_ws/src/rmcs_rl/test/onnx_winding_test.cpp'), str(src / 'onnx_policy.cpp'),
            '-L', str(ort / 'lib'), '-Wl,-rpath,' + str(ort / 'lib'), '-lonnxruntime', '-o', str(loader_binary)])
        nominal_cpp = [float(value) for value in command([str(winding_binary), '--nominal'], text=True).stdout.split()]
        paired64 = V5Control(manifest, spec, fit, prior, {**cfg['v5_control'], 'leg_winding_version': 'paired_common_v1'}, 'cpu')
        paired64.nominal = torch.tensor([nominal_cpp[0], nominal_cpp[1], 0., nominal_cpp[2], nominal_cpp[3], 0.], dtype=torch.float64)

        def cpp(mode, current, desired):
            data = torch.cat((current, desired), -1).double().numpy().astype(np.float64).tobytes()
            result = command([str(winding_binary), '--binary', mode, 'v6'], input=data)
            return torch.from_numpy(np.frombuffer(result.stdout, dtype=np.float64).copy().reshape(-1, 4))

        summaries, lane67, lane67_counterexample = [], None, None
        shifts = [(-20, 20), (-2, -1), (-1, 2), (0, 0), (1, -2), (2, 1), (20, -20)]
        max64_error = max_relative_error = max_native32_error = max_native32_relative_error = 0.
        split_cpp = common_cut_difference = total = 0

        def compare(action32, current32, current64=None):
            nonlocal max64_error, max_relative_error, max_native32_error, max_native32_relative_error
            nonlocal split_cpp, common_cut_difference, total
            current64 = current32.double() if current64 is None else current64
            native_target, _, native_clipped = paired.decode(action32, current32)
            reference_target, _, reference_clipped = paired64.decode(action32.double(), current64)
            desired64 = paired64.nominal[list(paired64.LEGS)] + reference_clipped[:, paired64.LEGS] * cfg['v5_control']['leg_position_scale']
            actual = cpp('paired_common_v1', current64[:, paired64.LEGS], desired64)
            mathematical_error = float((actual - reference_target).abs().max())
            relative_error = float(((actual[:, 1::2] - actual[:, ::2]) - (desired64[:, 1::2] - desired64[:, ::2])).abs().max())
            scale = 1 + float(actual.abs().max())
            assert mathematical_error <= 64 * np.finfo(np.float64).eps * scale
            assert relative_error <= 64 * np.finfo(np.float64).eps * scale
            desired32 = paired.nominal[list(paired.LEGS)] + native_clipped[:, paired.LEGS] * cfg['v5_control']['leg_position_scale']
            turns_cpp = ((actual - desired64) / (2 * torch.pi)).round().long().reshape(-1, 2, 2)
            turns32 = ((native_target - desired32) / (2 * torch.pi)).round().long().reshape(-1, 2, 2)
            split = turns_cpp[:, :, 0] != turns_cpp[:, :, 1]
            assert not bool(split.any())
            difference = turns_cpp - turns32
            assert torch.equal(difference[:, :, 0], difference[:, :, 1])
            split_cpp += int(split.any(-1).sum())
            common_cut_difference += int((difference != 0).any(-1).any(-1).sum())
            total += len(actual)
            max64_error = max(max64_error, mathematical_error)
            max_relative_error = max(max_relative_error, relative_error)
            max_native32_error = max(max_native32_error, float((actual - native_target.double()).abs().max()))
            max_native32_relative_error = max(max_native32_relative_error, float((
                (actual[:, 1::2] - actual[:, ::2])
                - (native_target[:, 1::2] - native_target[:, ::2]).double()).abs().max()))
            return actual, native_target, desired64, reference_target, turns_cpp, turns32

        for corner in (None, *itertools.product((-3., 3.), repeat=4)):
            action = torch.zeros(4096, 6)
            if corner is not None:
                action[:, paired.LEGS] = torch.tensor(corner)
            for left, right in shifts:
                current = q32.clone()
                current[:, list(paired.LEGS)] += torch.tensor([left, left, right, right]) * (2 * torch.pi)
                actual, native, desired, reference, turns_cpp, turns32 = compare(action, current)
                old = cpp('independent_nearest_v1', current[:, paired.LEGS].double(), desired)
                old_turns = ((old - desired) / (2 * torch.pi)).round().long().reshape(-1, 2, 2)
                split_old = (old_turns[:, :, 0] != old_turns[:, :, 1]).any(-1)
                summaries.append({'leg_action_control_order': [0.] * 4 if corner is None else list(corner),
                    'common_left_right_turns': [left, right], 'independent_split_lanes': int(split_old.sum()),
                    'paired_split_lanes': 0, 'f32_vs_double_common_branch_differences': int((turns_cpp != turns32).any(-1).any(-1).sum())})
                if corner is None and (left, right) == (0, 0):
                    lane67 = {'lane': 67, 'nominal_lane': bool(nominal_mask[67]),
                        'ground_request': bool(state.requests.ground_birth[67]),
                        'control_joint_names': list(V5Control.ACTIVE), 'control_q_native_f32_rad': current[67].tolist(),
                        'raw_action_control_order': action[67].tolist(), 'desired_rmcs_f64_rad': desired[67].tolist(),
                        'independent_rmcs_rad': old[67].tolist(), 'paired_rmcs_rad': actual[67].tolist(),
                        'paired_train_native_f32_rad': native[67].tolist(), 'paired_train_math_f64_rad': reference[67].tolist(),
                        'paired_rmcs_turns': turns_cpp[67].tolist(), 'paired_train_native_turns': turns32[67].tolist()}
                if lane67_counterexample is None and (left, right) == (0, 0) and bool(split_old[67]):
                    lane67_counterexample = {'lane': 67, 'raw_leg_action_control_order': list(corner),
                        'control_q_native_f32_rad': current[67].tolist(), 'desired_rmcs_f64_rad': desired[67].tolist(),
                        'independent_rmcs_rad': old[67].tolist(), 'paired_rmcs_rad': actual[67].tolist(),
                        'independent_turns': old_turns[67].tolist(), 'paired_turns': turns_cpp[67].tolist(),
                        'paired_train_native_f32_rad': native[67].tolist()}

        cuts = []
        for offset in (-torch.pi - 1e-6, -torch.pi, -torch.pi + 1e-6, torch.pi - 1e-6, torch.pi, torch.pi + 1e-6):
            action = torch.zeros(1, 6)
            current64 = paired64.nominal[None].clone()
            current64[:, paired64.LEGS] += offset
            actual, native, _, _, turns_cpp, turns32 = compare(action, current64.float(), current64)
            cuts.append({'offset_rad': offset, 'rmcs_f64_turns': turns_cpp[0].tolist(),
                'train_native_f32_turns': turns32[0].tolist(), 'rmcs_targets_rad': actual[0].tolist(),
                'train_targets_rad': native[0].tolist(), 'difference_is_common_per_side': True})

        assert lane67_counterexample is not None
        metadata_results = []
        for metadata in (None, 'independent_nearest_v1', 'paired_common_v1', 'unknown', ''):
            model = helper.make_model(helper.make_graph([
                helper.make_node('Constant', [], ['actions'], value=helper.make_tensor('zero', TensorProto.FLOAT, [1, 6], [0.] * 6))],
                'metadata_test', [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [1, 35])],
                [helper.make_tensor_value_info('actions', TensorProto.FLOAT, [1, 6])]),
                opset_imports=[helper.make_opsetid('', 17)], ir_version=10)
            if metadata is not None:
                helper.set_model_props(model, {'leg_winding_version': metadata})
            path = scratch / ('model_' + str(len(metadata_results)) + '.onnx')
            onnx.save(model, path)
            for configured in (None, 'independent_nearest_v1', 'paired_common_v1', 'unknown'):
                argv = [str(loader_binary), str(path)] + ([] if configured is None else [configured])
                result = subprocess.run(argv, capture_output=True, text=True)
                expected = metadata in (None, 'independent_nearest_v1') and configured in (None, 'independent_nearest_v1') or metadata == 'paired_common_v1' and configured == 'paired_common_v1'
                assert (result.returncode == 0) == expected, (metadata, configured, result.stderr)
                metadata_results.append({'model_metadata': metadata, 'configured_version': configured,
                    'accepted': result.returncode == 0, 'message': result.stdout.strip() if expected else result.stderr.strip()})

        baseline = scratch / 'baseline'
        baseline.mkdir()
        for name in ('policy.hpp', 'onnx_policy.cpp'):
            (baseline / name).write_bytes(command(['git', 'show', '137d5ece:rmcs_ws/src/rmcs_rl/src/' + name], cwd=ROOT).stdout)
        baseline_binary = scratch / 'baseline_loader'
        command(['g++', '-std=c++23', '-O2', '-I', str(baseline), '-I', str(ort / 'include'),
            str(ROOT / 'rmcs_ws/src/rmcs_rl/test/onnx_winding_test.cpp'), str(baseline / 'onnx_policy.cpp'),
            '-L', str(ort / 'lib'), '-Wl,-rpath,' + str(ort / 'lib'), '-lonnxruntime', '-o', str(baseline_binary)])
        legacy_results = []
        for name in command(['git', 'ls-files', 'rmcs_ws/src/rmcs_rl/models/wheel_leg/*.onnx'], cwd=ROOT, text=True).stdout.splitlines():
            old = command([str(baseline_binary), str(ROOT / name)], text=True)
            new = command([str(loader_binary), str(ROOT / name)], text=True)
            assert old.stdout == new.stdout
            legacy_results.append({'model': name, 'sha256': sha(ROOT / name),
                'baseline_and_new_zero_observation_output_identical': True, 'output': new.stdout.strip()})

    files = ['rmcs_ws/src/rmcs_rl/src/' + name for name in
        ('leg_target_winding.hpp', 'action.cpp', 'configuration.cpp', 'rl_controller.hpp', 'policy.hpp', 'onnx_policy.cpp')]
    files += ['rmcs_ws/src/rmcs_rl/test/leg_target_winding_test.cpp',
        'rmcs_ws/src/rmcs_rl/test/onnx_winding_test.cpp', '.script/audit_leg_target_winding.py']
    receipt = {'schema': 'v7-rmcs-leg-target-winding-audit-v1',
        'created_at_utc': datetime.now(timezone.utc).isoformat(), 'rmcs_base_commit': '137d5ece',
        'training_commit': command(['git', 'rev-parse', 'HEAD'], cwd=training, text=True).stdout.strip(),
        'source_sha256': {name: sha(ROOT / name) for name in files},
        'training_decoder_sha256': sha(training / 'src/wheeled_tasks/chassis/v5_control.py'),
        'asset_manifest_sha256': sha(training / directory / 'manifest.json'),
        'cpp_self_test': self_test, 'birth_population': {'accepted': 4096, 'seed': 290929,
            'nominal_lanes': int(nominal_mask.sum()), 'dynamic_lanes': int((~nominal_mask).sum())},
        'case_count_including_cuts': total, 'action_corners_plus_zero': 17, 'common_turn_pairs': shifts,
        'paired_different_axis_turn_case_count': split_cpp,
        'f32_vs_double_common_branch_case_count': common_cut_difference,
        'math_f64_target_max_abs_error_rad': max64_error, 'paired_relative_target_max_abs_error_rad': max_relative_error,
        'native_f32_vs_rmcs_double_target_max_abs_error_rad': max_native32_error,
        'native_f32_vs_rmcs_double_relative_target_max_abs_error_rad': max_native32_relative_error,
        'precision_scope': 'RMCS paired arithmetic stays double; real native training is float32. Common 2pi differences at the cut are not relative-axis splits; no all-domain bitwise parity is claimed.',
        'corner_results': summaries, 'lane67': lane67, 'lane67_counterexample': lane67_counterexample,
        'angular_cut_cases': cuts,
        'ort_version': '1.20.0', 'actual_ort_model_metadata_load_cases': metadata_results,
        'historical_tracked_models': legacy_results,
        'production_source_checks': {'clipping_and_control_q_selection_unchanged_from_base': True,
            'recovery_project_feedback_remains_the_current_coordinate': True,
            'pd_fsm_history_wheels_soft_limits_after_target_block_unchanged_from_base': True,
            'winding_helper_is_called_by_actual_action_cpp': True,
            'default_yaml_and_tracked_models_unchanged': True},
        'scope': 'CPU production function/ORT loader checks only; no ROS node, native simulator, GPU, PPO, live process, YAML, policy model, assets, reward or FSM changes',
        'qualification': 'not_native_or_hardware_qualified'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: receipt[key] for key in ('case_count_including_cuts', 'paired_different_axis_turn_case_count',
        'f32_vs_double_common_branch_case_count', 'math_f64_target_max_abs_error_rad', 'paired_relative_target_max_abs_error_rad')}, indent=2))


if __name__ == '__main__':
    main()
