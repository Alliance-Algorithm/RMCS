#!/usr/bin/env python3
"""Check archived host-submitted MIT frames against their PC PD contract."""
import argparse
import json
import math
from pathlib import Path

import yaml


def decode_mit(data):
    if len(data) != 8:
        raise ValueError('MIT command must have eight bytes')
    data = [int(value) for value in data]
    return {
        'position_code': (data[0] << 8) | data[1],
        'velocity_code': (data[2] << 4) | (data[3] >> 4),
        'kp_code': ((data[3] & 15) << 8) | data[4],
        'kd_code': (data[5] << 4) | (data[6] >> 4),
        'torque_code': ((data[6] & 15) << 8) | data[7],
    }


def main():
    import rosbag2_py
    from ament_index_python.packages import get_package_share_directory
    from rclpy.serialization import deserialize_message
    from rmcs_msgs.msg import WheelLegIdentificationSample

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    installed = Path(get_package_share_directory('rmcs_msgs')) / 'msg/WheelLegIdentificationSample.msg'
    if installed.read_bytes() != (args.run / installed.name).read_bytes():
        raise ValueError('Archived and installed message layouts differ')
    profile = yaml.safe_load((args.run / 'profile.yaml').read_text())
    cfg = profile['wheel_leg_pair_identification_controller']['ros__parameters']
    if cfg['control_law'] != 'rl_pd':
        raise ValueError('This audit requires the archived pure PC PD control law')
    axes = (0, 1) if cfg['side'] == 'left' else (2, 3)
    rows = frames = nonzero_gains = non_torque_rows = nonfinite_rows = 0
    enable_heartbeat_rows = unexpected_rows = 0
    algebra_max = 0.0
    examples = []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.run / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic != '/wheel_leg/identification/sample':
            continue
        m = deserialize_message(payload, WheelLegIdentificationSample)
        if m.phase != 2:
            continue
        rows += 1
        non_torque = any(m.tx_kind[i] != 1 for i in axes)
        non_torque_rows += int(non_torque)
        if non_torque:
            heartbeat = (
                all(m.tx_kind[i] == 2 and m.dm_status[i] == 1
                    and list(m.tx_frame_bytes[8*i:8*i+8]) == [255]*7+[252]
                    for i in axes)
                and all(m.dm_status[i] == 0 for i in range(4) if i not in axes))
            enable_heartbeat_rows += int(heartbeat)
            unexpected_rows += int(not heartbeat)
        for i in axes:
            if m.tx_kind[i] != 1:
                continue
            raw = list(m.tx_frame_bytes[8*i:8*i+8])
            decoded = decode_mit(raw)
            frames += 1
            nonzero_gains += int(decoded['kp_code'] != 0 or decoded['kd_code'] != 0)
            sign = cfg['model_sign'][i]
            if not all(math.isfinite(value) for value in
                       (m.position_error_model[i],m.dq_api[i],m.tau_cmd_api[i])):
                nonfinite_rows += 1
                continue
            expected = cfg['pd_kp'][i]*m.position_error_model[i] - cfg['pd_kd'][i]*sign*m.dq_api[i]
            expected = sign*max(-cfg['max_torque'][i], min(cfg['max_torque'][i], expected))
            algebra_max = max(algebra_max, abs(m.tau_cmd_api[i]-expected))
            if len(examples) < 2:
                examples.append({'axis': i, 'can_bus': int(m.tx_can_bus[i]), 'can_id': int(m.tx_can_id[i]),
                                 'bytes_hex': ' '.join(f'{b:02x}' for b in raw), **decoded})
    result = {'run': str(args.run), 'phase2_rows': rows, 'submitted_mit_torque_frames': frames,
              'nonzero_motor_internal_pd_frames': nonzero_gains,
              'rows_with_non_torque_frame': non_torque_rows,
              'selected_pair_enable_heartbeat_rows': enable_heartbeat_rows,
              'unexpected_frame_rows': unexpected_rows,
              'nonfinite_axis_samples': nonfinite_rows,
              'pc_pd_identity_max_error_nm': algebra_max,
              'verified': (rows > 0 and frames == 2*(rows-enable_heartbeat_rows)
                           and unexpected_rows == 0 and nonzero_gains == 0
                           and nonfinite_rows == 0 and algebra_max < 1e-9),
              'examples': examples,
              'scope': 'Recorded host-submitted payloads; not an independent CAN reception acknowledgement.',
              'heartbeat_note': 'The archived scheduler periodically reasserts 0xFC enable. These system frames are counted separately from normal MIT torque frames.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
