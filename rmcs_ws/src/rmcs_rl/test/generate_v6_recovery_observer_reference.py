#!/usr/bin/env python3
"""Regenerate native Torch parity vectors from the frozen V6 source commit."""
import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path
import torch

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('training_repository', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('v6_recovery_observer_reference.json'))
    args = parser.parse_args()
    root = args.training_repository.resolve()
    commit = '2778206b5905c2760ccc381f2592b08e1cad611a'
    classes = {}
    for name in ['mechanism_observer', 'continuous_encoders', 'step_trigger', 'recovery_observer']:
        source = subprocess.check_output(['git', '-C', str(root), 'show', f'{commit}:src/wheeled_tasks/chassis/{name}.py']).decode()
        namespace = {'__name__': name}
        exec(compile(source, name + '.py', 'exec'), namespace)
        classes[name] = namespace
    lookup = json.loads(subprocess.check_output(['git', '-C', str(root), 'show', f'{commit}:contracts/v6_height_lookup_v1.json']))
    geometry = classes['mechanism_observer']['ClosedChainHeightModel'](lookup, 'cpu')
    limits = torch.stack([s['delta_rad'][[0, -1]] for s in geometry.sides])
    streams = []
    for stream in range(4):
        encoder = classes['continuous_encoders']['ContinuousEncoderAngles'](1, 'cpu', limits)
        probe = classes['recovery_observer']['WheelSupportProbe']('cpu', 1, 0.005)
        frames = []
        for tick in range(180):
            q = torch.tensor([[-0.345100204, 0.122575374, 0.0, 0.345100204, -0.122567735, 0.0]])
            dq = torch.zeros_like(q)
            gyro = torch.zeros(1, 3)
            g = torch.tensor([[0.0, 0.0, -1.0]])
            a = torch.tensor([[0.0, 0.0, 9.81]])
            phase = 7 if tick < 100 else 8 if tick < 140 else 9
            eligible = phase < 8
            if stream == 1:
                q[:, 0:2] += 2 * math.pi
                q[:, 3:5] -= 2 * math.pi
                if tick < 90:
                    q[:, 1] -= 2 * math.pi
                    q[:, 4] += 2 * math.pi
            if stream == 2:
                angle = 0.25 * math.sin(tick * 0.025)
                g = torch.tensor([[math.sin(angle), 0.0, -math.cos(angle)]])
                if 40 <= tick < 55:
                    a[:, 2] = 0.0
                if 65 <= tick < 75:
                    gyro[:, 1] = 10.0
                if 110 <= tick < 125:
                    dq[:, 2] = tick * 2.0
            if stream == 3:
                q[:, 1] = q[:, 0] + (1.5 if tick < 60 else 0.1)
                q[:, 4] = q[:, 3] - (1.5 if tick < 60 else 0.1)
            if tick == 0:
                encoder.reset([0], q)
            else:
                encoder.update(q)
            cq = encoder.project(q)
            points, valid = geometry.wheel_positions(cq)
            heights, hvalid = classes['step_trigger']['estimate_height_if_grounded'](points, g, geometry.wheel_radius_m, grounded=valid, sensor_valid=valid, wheel_axle_b=geometry.wheel_axles_b)
            h = heights.min(-1).values
            plausible = hvalid.all(-1) & (h > 0.12) & ((heights[:, 0] - heights[:, 1]).abs() < 0.035) & (a.norm(dim=-1) > 6) & (a.norm(dim=-1) < 16) & (gyro.norm(dim=-1) < 8)
            confirmed = probe.observe(dq[:, [2, 5]], gyro, plausible)
            clear = (confirmed | (phase in (8, 9))) & plausible & (h > 0.27)
            pulse = probe.command(tick, plausible & eligible, confirmed)
            p = [0, 1, 3, 4, 2, 5]
            frames.append(dict(tick=tick, q=q[0, p].tolist(), dq=dq[0, p].tolist(), continuous_q=cq[0, p].tolist(), gravity=g[0].tolist(), gyro=gyro[0].tolist(), acceleration=a[0].tolist(), phase=phase, eligible=eligible, height=h.item(), geometry_valid=valid.all().item(), plausible=plausible.item(), confirmed=confirmed.item(), body_clear=clear.item(), pulse=pulse[0].tolist()))
        streams.append(frames)
    out = args.output.resolve()
    out.write_text(json.dumps(dict(source_commit=commit, streams=streams), separators=(',', ':')) + '\n')
    print(out, hashlib.sha256(out.read_bytes()).hexdigest())
if __name__ == '__main__':
    main()
