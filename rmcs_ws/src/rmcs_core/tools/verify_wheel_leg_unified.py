#!/usr/bin/env python3
"""Check the unified115 deployment offline without creating vehicle components."""

import argparse
import hashlib
import json
from pathlib import Path
import shlex

import numpy as np
import onnxruntime as ort
import yaml


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify(root, package, config_path):
    contract_path = package / 'deployment_contract.json'
    contract = json.loads(contract_path.read_text())
    vectors_path = package / 'inference_vectors.json'
    vectors = json.loads(vectors_path.read_text())
    require(contract['schema'] == 'unified-getup-user-trial-v1'
            and contract['behavior_qualified'] is False
            and contract['hardware_tested'] is False
            and (contract['actor_dim'], contract['action_dim'], contract['history_length'],
                 contract['policy_hz'], contract['physical_pd_hz']) == (35, 6, 1, 200, 1000),
            'Unsupported export contract')
    observation_hash = hashlib.sha256(json.dumps(
        contract['standing_position_observation'], sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    require(observation_hash == contract['observation_contract_sha256']
            == 'db0fc6121c73121dcd39eef0524ecfeab4b3aa69aa7f5700d4205499af2402a9',
            'Standing observation contract differs')
    configuration = yaml.safe_load(config_path.read_text())
    bridge = configuration['rl_bridge']['ros__parameters']
    controller = configuration['wheel_leg_controller']['ros__parameters']
    hardware = configuration['wheel_leg']['ros__parameters']
    commands = configuration['wheel_leg_chassis_controller']['ros__parameters']
    require((bridge['policy_rate'], bridge['history_length'], bridge['rl_obs_size'],
             bridge['rl_feature_size'], bridge['rl_action_size']) == (200, 1, 35, 35, 6),
            'Policy clock, dimensions or history differ')
    require(bridge['expected_observation_contract_sha256'] == observation_hash
            and bridge['observation_frame_sequence_interface']
            == '/wheel_leg/rl/input/frame_sequence'
            and bridge['observation_terms'] == [
                'index=0 path=/wheel_leg/rl/input/frame take=array type=double_array dim=35 name=frame'],
            'Expected one coherent controller-owned frame')
    require(bridge['enable_default'] is False and controller['auto_enter_rl'] is False
            and bridge['reset_interface'] == '/chassis/reset_count'
            and bridge['enable_interface'] == '/wheel_leg/rl/input/enable'
            and controller['unified_full_rl'] is True
            and controller['heading_hold_enabled'] is False
            and controller['pd_frequency']
            == configuration['rmcs_executor']['ros__parameters']['update_rate'] == 1000
            and hardware['joint_control_mode'] == 'torque',
            'Enable, reset, preprocessing or host PD contract differs')
    require(not any(key.startswith(('recovery_', 'takeover_')) for key in controller),
            'Obsolete script parameters remain in the deployment')
    require(bridge['inference_timeout'] == bridge['max_action_age']
            == controller['inference_timeout'] == .06, 'Action expiry differs')
    require(bridge['leg_winding_version'] == controller['leg_winding_version']
            == contract['leg_winding_version'] == 'paired_branch107_v1'
            and bridge['expected_model_spec_sha256'] == contract['leg_target_model_spec_sha256']
            and controller['nominal_model_pos'] == contract['nominal_model_pos'],
            'Mechanical model, winding or nominal pose differs')
    require(hardware['dm_control_torque_max'] == [40.] * 4
            and hardware['joint_position_kp'] == contract['leg_kp'] == 160.
            and hardware['joint_position_kd'] == contract['leg_kd'] == 2.5
            and commands['default_command_height'] == contract['suggested_initial_height_m'],
            'PD, torque cap or initial height differs')
    names = ['left_hip_joint', 'left_knee_joint', 'right_hip_joint',
             'right_knee_joint', 'left_wheel', 'right_wheel']
    require(bridge['joint_names'] == names and len(bridge['action_terms']) == 6,
            'Policy joint order differs')
    for i, (name, text) in enumerate(zip(names, bridge['action_terms'])):
        term = dict(token.split('=', 1) for token in shlex.split(text))
        scale, bound = (.25, 4.) if i < 4 else (10., 90.)
        require(term['index'] == str(i) and term['output'] == '/wheel_leg/rl/target/' + name
                and float(term['scale']) == scale
                and [float(value) for value in term['clip'].split(':')] == [-bound, bound],
                f'Action channel {i} differs')
    model_path = Path(bridge['rl_model_path'])
    if not model_path.is_absolute():
        model_path = root / 'rmcs_ws/src/rmcs_rl' / model_path
    model_hash = digest(model_path)
    require(model_hash == contract['model_sha256'] == vectors['model_sha256'],
            'ONNX SHA256 differs')
    require(digest(package / 'checkpoint.pt') == contract['checkpoint_sha256'],
            'Source checkpoint SHA256 differs')
    core = root / 'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg'
    helpers = [core / name for name in ('standing_model_geometry.hpp',
               'standing_reference_v3.hpp', 'unified_observation.hpp')]
    for helper in helpers:
        require(digest(helper) == contract['runtime']['source_files'][
            str(helper.relative_to(root))], f'Frozen preprocessing helper differs: {helper.name}')
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model_path), sess_options=options,
                                  providers=['CPUExecutionProvider'])
    metadata = session.get_modelmeta().custom_metadata_map
    required = dict(policy_hz='200', observation_dim='35', action_dim='6',
                    rmcs_observation_contract_sha256=observation_hash,
                    leg_winding_version='paired_branch107_v1',
                    leg_target_model_spec_sha256=contract['leg_target_model_spec_sha256'],
                    source_checkpoint_sha256=contract['checkpoint_sha256'],
                    behavior_qualified='false')
    require(all(metadata.get(key) == value for key, value in required.items()),
            'ONNX metadata differs')
    inputs, outputs = session.get_inputs(), session.get_outputs()
    require(len(inputs) == len(outputs) == 1 and inputs[0].name == 'obs'
            and inputs[0].shape == [1, 35] and outputs[0].name == 'actions'
            and outputs[0].shape == [1, 6]
            and inputs[0].type == outputs[0].type == 'tensor(float)',
            'Expected float32 obs[1,35] -> actions[1,6]')
    require(bool(vectors['vectors']), 'Frozen inference vectors are missing')
    worst = 0.
    bounds = np.asarray([16.] * 4 + [9.] * 2)
    for row in vectors['vectors']:
        actual = session.run(None, {'obs': np.asarray(
            row['observation'], dtype=np.float32)[None]})[0]
        expected = np.asarray(row['expected_raw_action'], dtype=np.float32)[None]
        error = float(np.abs(actual - expected).max())
        require(np.isfinite(actual).all() and np.isfinite(error)
                and error <= vectors['max_abs_tolerance']
                and (np.abs(actual) <= bounds + 1e-5).all(), 'Frozen inference vector differs')
        worst = max(worst, error)
    sources = list(core.glob('*.hpp')) + list(core.glob('*.cpp'))
    child = root / 'rmcs_ws/src/rmcs_rl'
    sources += list((child / 'src').rglob('*.cpp')) + list((child / 'src').rglob('*.hpp'))
    sources += list((child / 'include').rglob('*.hpp'))
    return dict(schema='rmcs-unified115-deployment-v1', status='offline_contract_passed',
                model_filename=model_path.name, model_sha256=model_hash,
                observation_contract_sha256=observation_hash,
                source_contract_sha256=digest(contract_path), source_vectors_sha256=digest(vectors_path),
                config=str(config_path.relative_to(root)), config_sha256=digest(config_path),
                policy_hz=200, host_pd_hz=1000, inference_vectors=len(vectors['vectors']),
                maximum_raw_action_error=worst,
                runtime_binding='reviewed_port_with_frozen_helpers',
                source_sha256={str(path.relative_to(root)): digest(path) for path in sorted(sources)},
                hardware_tested=False, behavior_qualified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-package', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[4])
    parser.add_argument('--config', type=Path)
    parser.add_argument('--receipt', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    config = args.config or root / 'rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-unified-rl.yaml'
    result = verify(root, args.source_package.resolve(), config.resolve())
    text = json.dumps(result, indent=2) + '\n'
    if args.receipt:
        args.receipt.write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
