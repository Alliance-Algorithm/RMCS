#!/usr/bin/env python3
"""Export full-rate selected-pair telemetry for offline control analysis."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rmcs_msgs.msg import WheelLegIdentificationSample

p = argparse.ArgumentParser()
p.add_argument('run', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
definition = Path(get_package_share_directory('rmcs_msgs')) / 'msg/WheelLegIdentificationSample.msg'
assert definition.read_bytes() == (a.run / definition.name).read_bytes()
profile = yaml.safe_load((a.run / 'profile.yaml').read_text())
params = profile['wheel_leg_pair_identification_controller']['ros__parameters']
axes = [0, 1] if params['side'] == 'left' else [2, 3]
metadata = yaml.safe_load((a.run / 'bag/metadata.yaml').read_text())
n = metadata['rosbag2_bagfile_information']['message_count']
floats = ('q_api', 'dq_api', 'torque_fb_api', 'tau_frame_api', 'tau_cmd_api',
          'q_ref_model', 'dq_ref_model', 'velocity_target_model', 'position_error_model',
          'speed_error_model', 'tau_preclip_model', 'torque_integral_model')
arrays = {k: np.empty((n, 2)) for k in floats}
for k in ('feedback_steady_ns', 'feedback_sequence', 'tx_queued_steady_ns'):
    arrays[k] = np.empty((n, 2), dtype=np.uint64)
for k in ('control_steady_ns', 'tick'):
    arrays[k] = np.empty(n, dtype=np.uint64)
for k in ('phase', 'segment_id', 'segment_role', 'actuation_scope', 'failure_reason', 'repetition_id', 'dropped_samples'):
    arrays[k] = np.empty(n, dtype=np.int32)
arrays['torque_limited'] = np.empty((n, 2), dtype=bool)
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(a.run / 'bag'), storage_id='mcap'),
            rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
i = 0
while reader.has_next():
    topic, payload, _ = reader.read_next()
    if topic != '/wheel_leg/identification/sample':
        continue
    m = deserialize_message(payload, WheelLegIdentificationSample)
    for k, v in arrays.items():
        field = getattr(m, k)
        if v.ndim == 2:
            v[i] = field[axes[0]], field[axes[1]]
        else:
            v[i] = field
    i += 1
    if i % 100000 == 0:
        print('Extracted', i, '/', n, flush=True)
assert i == n
a.output.parent.mkdir(parents=True, exist_ok=True)
np.savez_compressed(a.output, **arrays)
a.output.with_suffix('.json').write_text(json.dumps({
    'source_run': str(a.run), 'samples': n, 'axes': axes, 'controller': params,
    'profile_sha256': hashlib.sha256((a.run / 'profile.yaml').read_bytes()).hexdigest(),
    'note': 'Full-rate telemetry; no resampling, filtering or force reinterpretation.'}, indent=2)+'\n')
print(a.output, flush=True)
