"""Fixed-chassis, installed-leg MuJoCo replay of recorded 50 Hz references.

The normalized CAD-to-hardware mapping remains a conditional prior. Only the
selected leg is retained: a rigid support decouples the opposite leg dynamically.
No measured state is injected after each segment's initial condition. This is an
offline identification experiment, never a controller or training release.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

TRAIN = Path('/home/yukikaze/Documents/workspace/robot_rl/isaac_wheeled_rl_train')
sys.path.insert(0, str(TRAIN / 'scripts'))
from audit_v5_motor_coupling import MotorCouplingAudit

BUNDLE = TRAIN / 'model/纯底盘_v5_232mm/urdf'
ROOT = Path(__file__).resolve().parent


class Replay:
    def __init__(self, run, side='left', physics_dt=.0005):
        raw = dict(np.load(run / 'response_1khz.npz'))
        self.a = {k: v[raw['phase'] == 2] for k, v in raw.items()}
        self.meta = json.loads((run / 'response_1khz.json').read_text())
        self.cfg = self.meta['controller']
        assert self.cfg['side'] == side and self.cfg['control_law'] == 'rl_pd'
        assert self.cfg['control_frequency_hz'] == 1000
        assert self.cfg['reference_frequency_hz'] == 50
        self.real_q = self.a['q_ref_model'] - self.a['position_error_model']
        self.side = side
        self.letter = 'L' if side == 'left' else 'R'
        self.joints = [self.letter + '_joint1', self.letter * 2 + '_joint1']
        axes = [0, 1] if side == 'left' else [2, 3]
        self.kp = np.array(self.cfg['pd_kp'])[axes]
        self.kd = np.array(self.cfg['pd_kd'])[axes]
        self.cap = np.array(self.cfg['max_torque'])[axes]
        self.audit = MotorCouplingAudit()
        self.spec = self.audit.spec
        root = ET.parse(BUNDLE / 'robot.xml').getroot()
        world = root.find('worldbody')
        for geom in list(world.findall('geom')):
            world.remove(geom)
        base = world.find('body')
        base.remove(base.find('freejoint'))
        base.set('pos', '0 0 0')
        base.set('quat', '1 0 0 0')
        for body in list(base.findall('body')):
            if not body.get('name').startswith(self.letter):
                base.remove(body)
        # Visual/collision meshes are irrelevant to an airborne, fixed rig.
        # Original collision masks permit ground contact only, no self-contact.
        for body in root.iter('body'):
            for geom in list(body.findall('geom')):
                body.remove(geom)
        root.remove(root.find('asset'))
        root.remove(root.find('keyframe'))
        for tag in ('equality', 'actuator'):
            parent = root.find(tag)
            for element in list(parent):
                if not element.get('name').startswith(self.letter):
                    parent.remove(element)
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.model.opt.timestep = physics_dt
        self.substeps = round(.001 / physics_dt)
        assert abs(self.substeps * physics_dt - .001) < 1e-12
        self.data = mujoco.MjData(self.model)
        self.qi = np.array([self.model.joint(n).qposadr[0] for n in self.joints])
        self.vi = np.array([self.model.joint(n).dofadr[0] for n in self.joints])
        self.ui = np.array([self.model.actuator(n + '_equivalent_torque').id for n in self.joints])
        self.spring = self.letter + '_spring_slide'
        self.si = self.model.joint(self.spring).qposadr[0]
        self.su = self.model.actuator(self.spring + '_gas_force').id
        self.binding = self.spec['spring_binding'][self.spring]
        self.curve = json.loads((BUNDLE / 'fit_10mpa.json').read_text())
        self.site_pairs = np.array([[self.model.site(e.get('site1')).id,
                                     self.model.site(e.get('site2')).id]
                                    for e in root.find('equality')])
        self.initial = {}

    def initial_state(self, segment):
        if segment in self.initial:
            return self.initial[segment]
        ix = np.flatnonzero(self.a['segment_id'] == segment)
        target = self.real_q[ix[0]]
        nom = self.spec['nominal_joint_pos']
        active = {n: nom[n] for n in self.audit.manifest['control_joint_names']}
        q = dict(nom)
        # Continue the two input axes to preserve the CAD assembly branch.
        origin = np.array([nom[n] for n in self.joints])
        for r in np.linspace(0, 1, max(24, int(np.max(abs(target-origin))/.08)+1)):
            prescribed = {**active, **dict(zip(self.joints, origin+r*(target-origin)))}
            q, gap = self.audit.solve(prescribed, q)
        full = np.zeros(self.model.nq)
        for j in range(self.model.njnt):
            joint = self.model.joint(j)
            full[joint.qposadr[0]] = q[joint.name]
        # Initial velocity follows the local closure tangent; measurements are
        # used here only, not as a persistent constraint during integration.
        vel = np.zeros(self.model.nv)
        for j, name in enumerate(self.joints):
            prescribed = {**active, **dict(zip(self.joints, target))}
            prescribed[name] += 1e-5
            perturbed, _ = self.audit.solve(prescribed, q)
            for k in range(self.model.njnt):
                joint = self.model.joint(k)
                vel[joint.dofadr[0]] += ((perturbed[joint.name]-q[joint.name])/1e-5
                                        * self.a['dq_api'][ix[0], j])
        self.initial[segment] = (ix, full, vel, gap)
        return self.initial[segment]

    def replay(self, segment, params, max_samples=None):
        ix, q0, v0, gap0 = self.initial_state(segment)
        if max_samples:
            ix = ix[:max_samples]
        m, data = self.model, self.data
        # Passive engine damping is fixed except at the identified motor axes.
        m.dof_armature[self.vi] = params.get('armature', [.015795, .015795])
        m.dof_damping[self.vi] = params.get('damping', [0., 0.])
        m.dof_frictionloss[self.vi] = params.get('frictionloss', [0., 0.])
        mujoco.mj_resetData(m, data)
        data.qpos[:] = q0
        data.qvel[:] = v0
        mujoco.mj_forward(m, data)
        tau_s = np.broadcast_to(params.get('time_constant_s', 0.), (2,))
        alpha = np.exp(-m.opt.timestep / np.maximum(tau_s, 1e-12))
        gain = np.broadcast_to(params.get('gain', 1.), (2,))
        effort = self.a['tau_cmd_api'][ix[0]].copy()
        prediction = np.empty((len(ix), 2))
        velocity = np.empty_like(prediction)
        torque = np.empty_like(prediction)
        compression = np.empty(len(ix))
        max_gap = gap0
        for k, sample in enumerate(ix):
            prediction[k] = data.qpos[self.qi]
            velocity[k] = data.qvel[self.vi]
            request = np.clip(self.kp*(self.a['q_ref_model'][sample]-prediction[k])
                              - self.kd*velocity[k], -self.cap, self.cap)
            torque[k] = request
            for _ in range(self.substeps):
                effort = alpha*effort + (1-alpha)*gain*request
                data.ctrl[self.ui] = effort
                s = self.binding['compression_at_q_zero_m'] - data.qpos[self.si]
                u = np.clip(s / self.curve['stroke_m'], 0., 1.)
                data.ctrl[self.su] = np.polynomial.polynomial.polyval(
                    u, self.curve['monomial_coefficients_n'])
                mujoco.mj_step(m, data)
            compression[k] = s
            if k % 10 == 0:
                gaps = data.site_xpos[self.site_pairs[:, 0]] - data.site_xpos[self.site_pairs[:, 1]]
                max_gap = max(max_gap, float(np.max(np.linalg.norm(gaps, axis=1))))
                if not np.isfinite(data.qpos).all() or np.max(np.abs(data.qvel)) > 500:
                    raise RuntimeError(f'Divergent replay in segment {segment} at sample {k}')
        trim = slice(min(1000, len(ix)//4), None)
        err = prediction[trim] - self.real_q[ix][trim]
        score = {'segment': segment, 'samples': len(ix),
                 'axis_rmse_rad': np.sqrt(np.mean(err**2, axis=0)).tolist(),
                 'common_rmse_rad': float(np.sqrt(np.mean(err.mean(axis=1)**2))),
                 'difference_rmse_rad': float(np.sqrt(np.mean(np.diff(err, axis=1)**2))),
                 'max_closure_gap_m': max_gap,
                 'compression_range_m': [float(compression.min()), float(compression.max())],
                 'peak_request_nm': np.max(np.abs(torque), axis=0).tolist(),
                 'warnings': {str(i): int(w.number) for i,w in enumerate(data.warning) if w.number}}
        return score, {'indices': ix, 'q_sim': prediction, 'dq_sim': velocity, 'tau_request': torque}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--segments', type=int, nargs='+', default=[2, 4, 6, 8, 9, 11])
    p.add_argument('--params', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--physics-dt', type=float, default=.0005)
    args = p.parse_args()
    replay = Replay(args.run, physics_dt=args.physics_dt)
    params = json.loads(args.params.read_text()) if args.params else {}
    scores, traces = [], {}
    for segment in args.segments:
        start = time.monotonic()
        score, trace = replay.replay(segment, params)
        scores.append(score)
        traces.update({f'{segment}_{k}': v for k,v in trace.items()})
        print(json.dumps({**score, 'runtime_s': time.monotonic()-start}), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'model.xml').write_text(replay.xml)
    report = {'source_run': replay.meta, 'mujoco_version': mujoco.__version__,
              'physics_dt_s': replay.model.opt.timestep, 'parameters': params, 'scores': scores,
              'manifest_sha256': hashlib.sha256((BUNDLE/'manifest.json').read_bytes()).hexdigest(),
              'training_release': False,
              'fixed_priors': ['CAD mass/inertia/geometry', 'nominal gas curve',
                               'level rigid chassis, gravity 9.81', 'recorded PD and held targets'],
              'limitations': ['CAD-to-hardware zero/axis mapping is not independently verified.',
                              'No identified torque gain or physical output torque sensor.',
                              'Nominal 1 ms replay, no modeled encoder/velocity estimator.']}
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    np.savez_compressed(args.output/'traces.npz', **traces)


if __name__ == '__main__':
    main()
