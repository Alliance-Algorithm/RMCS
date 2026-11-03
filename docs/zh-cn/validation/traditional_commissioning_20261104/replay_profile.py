"""Offline ABI equivalence only; no Isaac process or hardware is started."""
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from wheelleg_tuning.native import NativeController, abi_inputs

deploy = Path('/home/noir/Documents/workspace/.worktrees/rmcs-wheelleg-tradition-deploy')
tuning = Path('/home/noir/Documents/workspace/wheelleg_controller_tuning')
archive = tuning / 'docs/validation/self_righting_retest_20261103_9inrlmwb'
profile_path = deploy / 'rmcs_ws/src/rmcs_bringup/config/wheel-leg-traditional-v202.yaml'
profile = yaml.safe_load(profile_path.read_text())
config = yaml.safe_load((archive / 'traditional_config.yaml').read_text())
isaac = Path(config['paths']['isaac'])
table = json.loads((isaac / config['paths']['postures']).read_text())
curve = json.loads((isaac / config['paths']['bundle'] / 'gas_spring_model.json').read_text())
expected = abi_inputs(config, table, curve, config.get('candidate'))
params = profile['wheelleg_traditional']['ros__parameters']
actual = tuple(np.ascontiguousarray(params[key], dtype=np.float64).reshape(shape)
               for key, shape in [('parameters', (28,)), ('posture_rows', (-1, 27)),
                                  ('spring_force_rows', (-1, 2))])
assert all(np.array_equal(a, b) for a, b in zip(expected, actual))

receipt = json.loads((archive / 'standing_forward30/receipt.json').read_text())
sources = {**receipt['baseline_build']['source_sha256'], **receipt['source_sha256']}
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert all(digest(deploy / path) == value for path, value in sources.items())
old_library = Path(receipt['baseline_build']['library'])
assert digest(old_library) == receipt['baseline_build']['library_sha256']
new_library = deploy / 'rmcs_ws/build/rmcs_core/libwheelleg_traditional_native.so'

old_recovery = yaml.safe_load((archive / 'recovery_config.yaml').read_text())['wheelleg_self_righting']['ros__parameters']
new_recovery = profile['wheelleg_self_righting']['ros__parameters']
changes = {k: [old_recovery.get(k), new_recovery.get(k)]
           for k in old_recovery.keys() | new_recovery.keys()
           if old_recovery.get(k) != new_recovery.get(k)}
assert set(changes) == {'enabled', 'simulation_only', 'hardware_calibration_id',
                        'hardware_commissioning_enabled', 'cad_axis_sign',
                        'cad_axis_offset_rad', 'flip_program'}
assert old_recovery['flip_program'] == [] and 'flip_program' not in new_recovery

cases = []
for case in ('forward30', 'standing_forward30'):
    path = archive / case / 'receipt.npz'
    trace = np.load(path)['trace']
    samples = trace[trace[:, 1] == 100]
    maximum_error = 0.0
    with NativeController(old_library, *expected) as old, NativeController(new_library, *actual) as new:
        for index, row in enumerate(samples):
            packet = row[11:28]
            command = np.array([0., 0., .334, 150.])
            first = old.step(packet, command, (index + 1) * 1_000_000)
            second = new.step(packet, command, (index + 1) * 1_000_000)
            assert np.isfinite(first).all() and np.isfinite(second).all()
            assert first[15] == 0 and second[15] == 0
            maximum_error = max(maximum_error, float(np.abs(first - second).max()))
    assert maximum_error == 0.0
    cases.append(dict(case=case, samples=len(samples), trace_sha256=digest(path),
                      maximum_output_difference=maximum_error, faults=0))

result = dict(schema='traditional-commissioning-offline-parity/v1', date='2026-11-04',
              scope='Archived sensor-packet replay through two native ABIs; no new physics or hardware',
              profile=str(profile_path), profile_sha256=digest(profile_path),
              standing_parameters_exact=True, tested_sources_identical=True,
              tested_source_sha256=sources, recovery_profile_changes=changes,
              old_library=str(old_library), old_library_sha256=digest(old_library),
              new_library=str(new_library), new_library_sha256=digest(new_library), cases=cases,
              hardware_motion_performed=False, new_native_physics_run=False)
Path('/tmp/wheelleg-imu-20261104/offline_parity.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
