#!/usr/bin/env python3
"""Compare the migrated C++ adapters with the original Unitree observation/action math.

This is offline validation only. Python is a test dependency, never a deployment
runtime dependency. Equations/order come from the original observation_manager.h,
manager_term_cfg.h, observations.h, joint_actions.h and State_Mimic.cpp.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
import onnxruntime as ort
import yaml

F = np.float32


def mul(a, b):
    w, x, y, z = np.asarray(a, dtype=F)
    W, X, Y, Z = np.asarray(b, dtype=F)
    return np.array([w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
                     w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W], dtype=F)


def conjugate(q):
    return np.asarray(q, dtype=F) * np.array([1, -1, -1, -1], dtype=F)


def rotation(q):
    w, x, y, z = np.asarray(q, dtype=F)
    # Eigen Quaternion::toRotationMatrix, with float32 operations.
    tx, ty, tz = F(2)*x, F(2)*y, F(2)*z
    return np.array([[1-ty*y-tz*z, tx*y-tz*w, tx*z+ty*w],
                     [tx*y+tz*w, 1-tx*x-tz*z, ty*z-tx*w],
                     [tx*z-ty*w, ty*z+tx*w, 1-tx*x-ty*y]], dtype=F)


def torso(q, joints):
    out = np.asarray(q, dtype=F)
    for joint, axis in [(12, 3), (13, 1), (14, 2)]:
        angle = F(joints[joint]) / F(2)
        part = np.zeros(4, dtype=F)
        part[0], part[axis] = np.cos(angle), np.sin(angle)
        out = mul(out, part)
    return out


def yaw(q):
    w, x, y, z = np.asarray(q, dtype=F)
    angle = np.arctan2(F(2)*(w*z+x*y), F(1)-F(2)*(y*y+z*z))
    out = np.array([np.cos(angle*F(.5)), 0, 0, np.sin(angle*F(.5))], dtype=F)
    return out / np.linalg.norm(out)


class OriginalMath:
    def __init__(self, root, spec):
        self.spec = spec
        bundle = root / spec['bundle']
        self.cfg = yaml.safe_load((bundle / 'params/deploy.yaml').read_text())
        self.dt = F(self.cfg['step_dt'])
        self.mapping = self.cfg['joint_ids_map']
        self.default = np.array(self.cfg['default_joint_pos'], dtype=F)
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED
        self.session = ort.InferenceSession(str(bundle / 'exported/policy.onnx'), options,
                                           providers=['CPUExecutionProvider'])
        self.motion = None
        if spec['adapter'] == 'mimic':
            self.motion = np.load(root / spec['motion_file'])
            self.start = F(spec.get('time_start', 0))
            self.end = F(spec.get('time_end', len(self.motion['joint_pos'])*F(.02)))
        obs = self.cfg['observations']
        first_term = next(v for k, v in obs.items() if k not in ('scale_first', 'use_gym_history'))
        self.groups = {'obs': obs} if 'params' in first_term else obs

    def motion_frame(self, step):
        t = F(F(step)*self.dt + self.start)
        return min(int(np.floor(F(t/F(.02)))), len(self.motion['joint_pos'])-1)

    def command(self, state):
        if self.spec['adapter'] == 'balance':
            return np.zeros(3, dtype=F)
        ranges = self.cfg['commands']['base_velocity']['ranges']
        return np.array([np.clip(F(state['command'][i]), *np.array(ranges[k], dtype=F))
                         for i, k in enumerate(['lin_vel_x', 'lin_vel_y', 'ang_vel_z'])], dtype=F)

    def observation(self, name, term, state, step):
        q = np.array(state['q'], dtype=F)
        if name == 'base_ang_vel':
            return np.array(state['gyro'], dtype=F)
        if name == 'projected_gravity':
            return rotation(state['quat']).T @ np.array([0, 0, -1], dtype=F)
        if name == 'velocity_commands':
            return self.command(state)
        if name == 'gait_phase':
            delta = F(self.dt*F(F(1)/F(term['params']['period'])))
            self.phase = F(np.fmod(F(self.phase+delta), F(1)))
            cmd = self.command(state)
            if np.sqrt(np.sum(cmd*cmd)) < F(.1):
                return np.zeros(2, dtype=F)
            return np.array([np.sin(float(self.phase)*2*np.pi), np.cos(float(self.phase)*2*np.pi)], dtype=F)
        if name == 'last_action':
            return self.last.copy()
        if name in ('joint_pos', 'joint_pos_rel', 'joint_vel_rel'):
            out = np.array(state['dq'] if name == 'joint_vel_rel' else state['q'], dtype=F)[self.mapping]
            if name == 'joint_pos_rel':
                out = out-self.default
            indices = term.get('params', {}).get('asset_cfg', {}).get('joint_ids', [])
            return out[indices] if indices else out
        if name in ('motion_command', 'motion_anchor_ori_b'):
            i = self.motion_frame(step)
            mq = self.motion['joint_pos'][i]
            if name == 'motion_command':
                return np.concatenate((mq, self.motion['joint_vel'][i]))
            real = torso(state['quat'], q)
            ref = torso(self.motion['body_quat_w'][i, 0], mq)
            rot = rotation(mul(conjugate(mul(self.init, ref)), real)).T
            return rot[:, :2].reshape(-1)
        raise ValueError(name)

    @staticmethod
    def processed(value, term, scale_first):
        value = np.array(value, dtype=F)
        scale = term.get('scale')
        clip = term.get('clip')
        if scale_first and scale is not None:
            value *= np.array(scale, dtype=F)
        if clip is not None:
            value = np.clip(value, *np.array(clip, dtype=F))
        if not scale_first and scale is not None:
            value *= np.array(scale, dtype=F)
        return value

    def reset(self, state):
        self.phase = F(0)
        self.last = np.zeros(29, dtype=F)
        self.history = {}
        if self.motion is not None:
            i = self.motion_frame(0)
            self.init = mul(yaw(torso(state['quat'], state['q'])),
                            conjugate(yaw(self.motion['body_quat_w'][i, 0])))
        for group_name, group in self.groups.items():
            scale_first = False
            for name, term in group.items():
                if name == 'scale_first':
                    scale_first = term
                elif name != 'use_gym_history':
                    value = self.processed(self.observation(name, term, state, 0), term, scale_first)
                    self.history[group_name, name] = [value.copy() for _ in range(term.get('history_length', 1))]

    def infer(self, state, step):
        observations = {}
        for group_name, group in self.groups.items():
            values = []
            scale_first = False
            for name, term in group.items():
                if name == 'scale_first':
                    scale_first = term
                elif name != 'use_gym_history':
                    value = self.processed(self.observation(name, term, state, step), term, scale_first)
                    history = self.history[group_name, name]
                    history.pop(0)
                    history.append(value)
                    values.append(history)
            if group.get('use_gym_history', False):
                observations[group_name] = np.concatenate([v[h] for h in range(len(values[0])) for v in values])
            else:
                observations[group_name] = np.concatenate([v for history in values for v in history])
        raw = self.session.run(None, {k: v[None, :] for k, v in observations.items()})[0][0]
        self.last = raw.copy()
        processed = []
        index = 0
        for term in self.cfg['actions'].values():
            count = 29 if term.get('joint_ids') is None else len(term['joint_ids'])
            action = raw[index:index+count].copy()
            if term.get('scale') is not None:
                action *= np.array(term['scale'], dtype=F)
            if term.get('offset') is not None:
                action += np.array(term['offset'], dtype=F)
            if term.get('clip') is not None:
                limits = np.array(term['clip'], dtype=F)
                action = np.clip(action, limits[:, 0], limits[:, 1])
            processed.extend(action)
            index += count
        target = np.zeros(29, dtype=F)
        action_dds = np.zeros(29, dtype=F)
        target[self.mapping] = processed
        action_dds[self.mapping] = raw
        obs_flat = np.concatenate([observations[i.name] for i in self.session.get_inputs()])
        return obs_flat, action_dds, target


def run_case(root, probe, directory, label, spec, steps=32):
    spec_path = directory / f'{label}.yaml'
    spec_path.write_text(yaml.safe_dump(spec, sort_keys=False))
    completed = subprocess.run([str(probe), str(root), str(spec_path), str(steps)],
                               check=True, text=True, capture_output=True)
    records = [json.loads(x) for x in completed.stdout.splitlines()]
    assert len(records) == 2*steps
    reference = OriginalMath(root, spec)
    maxima = {'observation': 0., 'action': 0., 'target': 0.}
    for episode in range(2):
        episode_records = records[episode*steps:(episode+1)*steps]
        reference.reset(episode_records[0])
        for i, record in enumerate(episode_records):
            expected = reference.infer(record, i)
            for name, value in zip(maxima, expected):
                actual = np.array(record[name])
                error = float(np.max(np.abs(actual-value)))
                maxima[name] = max(maxima[name], error)
                np.testing.assert_allclose(actual, value, atol=3e-5, rtol=2e-5,
                                           err_msg=f'{label} episode={episode} step={i} {name}')
            if reference.motion is not None:
                f = reference.motion_frame(i)
                goal = np.concatenate((reference.motion['joint_pos'][f], reference.motion['joint_vel'][f]))
                np.testing.assert_array_equal(record['goal'], goal)
                np.testing.assert_allclose(record['height'], reference.motion['body_pos_w'][f, 0, 2], atol=1e-7)
                gravity = rotation(reference.motion['body_quat_w'][f, 0]).T @ np.array([0, 0, -1], dtype=F)
                np.testing.assert_allclose(record['gravity'], gravity, atol=3e-7)
            if episode == 1:
                for name in ('observation', 'action', 'target', 'goal'):
                    np.testing.assert_array_equal(record[name], records[i][name], err_msg=f'{label} reset leaked {name}')
    return {'case': label, 'steps_per_episode': steps, 'episodes': 2, 'maximum_absolute_error': maxima}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    root, probe = args.root.resolve(), args.probe.resolve()
    velocity = dict(adapter='velocity', name='Velocity', bundle='deploy/robots/g1/config/policy/velocity/v0')
    mimic = dict(adapter='mimic', name='Mimic_Dance1_subject2', bundle='deploy/robots/g1/config/policy/mimic/dance1_subject2',
                 motion_file='deploy/robots/g1/config/policy/mimic/dance1_subject2/params/dance1_subject2.npz')
    results = []
    with tempfile.TemporaryDirectory(prefix='g1_legacy_parity_') as tmp:
        directory = Path(tmp)
        cases = [('velocity_original', velocity, 40),
                 ('balance_zero_command', dict(velocity, adapter='balance', name='balance'), 40),
                 ('mimic_original', mimic, 40),
                 ('mimic_time_window', dict(mimic, time_start=7.013, time_end=7.5), 24)]
        baseline = yaml.safe_load((root/velocity['bundle']/'params/deploy.yaml').read_text())
        for variant in ['mapping_clip', 'term_history', 'gym_history']:
            bundle = directory/variant
            (bundle/'params').mkdir(parents=True)
            (bundle/'exported').symlink_to(root/velocity['bundle']/'exported', target_is_directory=True)
            cfg = copy.deepcopy(baseline)
            if variant == 'mapping_clip':
                cfg['joint_ids_map'] = list(reversed(range(29)))
                cfg['observations'] = dict(scale_first=True, **cfg['observations'])
                cfg['observations']['base_ang_vel']['scale'] = [2.0, .5, -1.0]
                cfg['observations']['base_ang_vel']['clip'] = [-.15, .15]
                cfg['observations']['joint_pos_rel']['params'] = {'asset_cfg': {'joint_ids': list(reversed(range(29)))}}
                cfg['actions']['JointPositionAction']['clip'] = [[-.3, .4]]*29
            elif variant == 'term_history':
                cfg['observations']['base_ang_vel']['history_length'] = 2
                cfg['observations'].pop('velocity_commands')
                cfg['observations']['joint_vel_rel']['scale'] = [.5]*29
                cfg['observations']['joint_vel_rel']['clip'] = [-.1, .1]
            else:
                names = ['base_ang_vel', 'projected_gravity', 'gait_phase', 'joint_pos_rel', 'last_action']
                cfg['observations'] = {'use_gym_history': True, **{n: cfg['observations'][n] for n in names}}
                joint = cfg['observations']['joint_pos_rel']
                joint['params'] = {'asset_cfg': {'joint_ids': list(range(12))}}
                joint['scale'] = [1.0]*12
                for name in names:
                    cfg['observations'][name]['history_length'] = 2
            (bundle/'params/deploy.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
            cases.append((variant, dict(velocity, bundle=str(bundle)), 16 if variant == 'gym_history' else 24))
        for label, spec, steps in cases:
            result = run_case(root, probe, directory, label, spec, steps)
            results.append(result)
            print(json.dumps(result))
    report = {'status': 'passed', 'cases': results, 'tolerance': {'atol': 3e-5, 'rtol': 2e-5},
              'scope': 'Offline observations, ONNX actions, targets, reset, mapping, history, motion references. No robot actuation.'}
    if args.report:
        args.report.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
