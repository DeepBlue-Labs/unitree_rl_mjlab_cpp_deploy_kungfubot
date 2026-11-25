#!/usr/bin/env python3
"""Replay original Python sessions with real ONNX actors against native Session.

No mock actor, fake gate, robot connection, or policy-weight modification is used.
Measured-state fixtures follow reference poses; this verifies control semantics,
not closed-loop physical stability (the separate simulation tests cover that).
"""
import argparse
import copy
from contextlib import nullcontext
import gc
import json
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode = True
import tempfile
import numpy as np
import yaml
import onnxruntime as ort

# Keep the authoritative implementation unchanged while avoiding one thread pool
# per actor on a machine with many cores.
_ort_session = ort.InferenceSession

def single_thread_session(*args, **kwargs):
    if 'sess_options' not in kwargs:
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        kwargs['sess_options'] = options
    return _ort_session(*args, **kwargs)

ort.InferenceSession = single_thread_session


def plain(value):
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, (np.integer, np.floating)):
        return plain(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, (tuple, list)):
        return [plain(x) for x in value]
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--original', type=Path, default=Path('/home/pfy/unitree_rl_mjlab_supermimic'))
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--quick', action='store_true')
    parser.add_argument('--work-dir', type=Path, help='retain generated fixtures for investigation')
    args = parser.parse_args()
    sys.path.insert(0, str(args.original))
    from unified_deploy.registry import read_registry, resolve, load_policy
    from unified_deploy.sequence import InteractiveSequence
    from hybrid_supermimic.standing import VelocityStandingPolicy
    from hybrid_supermimic.runtime import RobotState
    from scipy.spatial.transform import Rotation
    original_registry = read_registry()
    entry = resolve('entry_star')
    manifest, summaries = [], []

    def state_for(sequence, bad=False):
        h = sequence.handoff
        p = h.policy
        phase = sequence.phase
        quat = np.array([1., 0., 0., 0.])
        q = sequence.standing.default_q.astype(float)
        height = .78
        if phase in ('HOLD', 'READY_BLEND') and hasattr(h, 'ready_q'):
            q = h.ready_q.astype(float)
        elif phase in ('PREPARE', 'WARMUP'):
            q = p.entry_q.astype(float)
        elif phase in ('POLICY', 'FIGHT_POLICY', 'SETTLE', 'HOLD'):
            f = 0 if phase == 'POLICY' and h.policy_ticks == 0 else p.frame
            if hasattr(p, 'bundle'):
                q = p.bundle.ref_joint_pos[f].astype(float)
                quat = p.bundle.ref_root_quat[f].copy()
            else:
                q = p.core.clip.joint_pos[f].astype(float)
                xyzw = Rotation.from_matrix(p.core.clip.policy_reference['root_R'][f]).as_quat()
                quat = xyzw[[3, 0, 1, 2]]
            height = p.reference_heights[f]
        return RobotState(np.array([0., 0., height]), quat, np.zeros(3),
                          np.array([.2 if bad else 0., 0., 0.]), q, np.zeros(29))

    def produce(work, name, spec, mode='one', stop_at=None, error_mode=None, automatic=False):
        standing = VelocityStandingPolicy()
        p = load_policy(spec)
        sequence = InteractiveSequence(p, standing, spec,
                                       gravity_model=args.original/'assets/supermimic_g1/scene_29dof.xml')
        fixture = work/(name+'.jsonl')
        item = dict(name=name, spec=spec, fixture=str(fixture), q_tolerance=5e-5, obs_tolerance=2e-4, automatic=automatic)
        if automatic:
            sequence.phase = 'STAND_PRE'
        manifest.append(item)
        h = sequence.handoff
        stage = 0
        phase_ticks = {}
        counts = {}
        prior_phase = None
        timeout_done = False
        hold_request_step = 0
        stop_sent = False
        action_index = 0
        fighting_actions = list(spec.get('ready_hold', {}).get('fighting', {}).get('actions', {}))
        visited = []
        with fixture.open('w') as out:
            for tick in range(30000):
                phase = sequence.phase
                phase_ticks[phase] = phase_ticks.get(phase, 0)+1 if phase == prior_phase else 1
                age = phase_ticks[phase]
                if phase != prior_phase:
                    visited.append(phase)
                command = ''
                bad = False
                if stop_at and phase == stop_at and not stop_sent:
                    command, stop_sent = 'STOP', True
                elif phase == 'WAIT_TAKEOVER':
                    command = 'TAKEOVER' if age == 2 else 'START'
                elif phase == 'STAND_PRE':
                    command = 'START' if age == 3 else ''
                elif mode == 'one' and phase == 'STAND_POST':
                    command = 'START' if age == 1 else ('STOP' if age == 3 else '')
                elif mode == 'ready':
                    if phase == 'HOLD':
                        if age == 1: command = 'START'
                        elif age == 2: command = 'NEXT entry_star'  # rejected in HOLD
                        elif age == 4:
                            command = 'NEXT home_star' if stage == 0 else 'NEXT exit_star'
                            stage += 1
                    elif phase == 'STAND_POST' and age == 2:
                        if stage == 2: command, stage = 'NEXT guard_stand', 3
                        elif stage == 3: command, stage = 'NEXT entry_star', 4
                        else: command = 'STOP'
                    if stage == 4 and phase == 'HOLD' and age == 3: command = 'STOP'
                elif mode == 'fight':
                    if phase == 'HOLD':
                        if error_mode == 'request_timeout' or (not timeout_done and not error_mode):
                            bad = True
                            hold_request_step += 1
                            if hold_request_step == 1: command = 'straight'
                            elif hold_request_step == 5: command = 'NEXT hook'  # replacement resets timeout
                            elif hold_request_step == h.request_timeout_ticks+6:
                                timeout_done = True
                                if error_mode == 'request_timeout': command = 'STOP'
                        elif age == 1: command = 'START'
                        elif not h.pending_name:
                            if action_index < len(fighting_actions):
                                command = fighting_actions[action_index]
                                action_index += 1
                            elif stage == 0:
                                command, stage = 'NEXT exit_star', 1
                            elif stage == 2:
                                command = 'STOP'
                    elif phase == 'STAND_POST' and age == 2:
                        command, stage = 'NEXT entry_star', 2
                    if error_mode == 'acquire_timeout' and phase == 'READY_BLEND': bad = True
                    if error_mode == 'settle_timeout' and phase == 'SETTLE': bad = True
                    if error_mode == 'settle_timeout' and phase == 'SETTLE':
                        # The settle body-angular threshold is .7, larger than ready's.
                        pass
                state = state_for(sequence, bad)
                if error_mode == 'acquire_timeout' and phase == 'READY_BLEND':
                    state = RobotState(state.root_pos_w, state.root_quat_w, state.root_lin_vel_b,
                                       np.array([1., 0., 0.]), state.joint_pos, state.joint_vel)
                if error_mode == 'settle_timeout' and phase == 'SETTLE':
                    state = RobotState(state.root_pos_w, state.root_quat_w, state.root_lin_vel_b,
                                       np.array([1., 0., 0.]), state.joint_pos, state.joint_vel)
                record = dict(command=command, q=state.joint_pos, dq=state.joint_vel,
                              quat=state.root_quat_w, gyro=state.root_ang_vel_b,
                              root_pos=state.root_pos_w, root_vel=state.root_lin_vel_b)
                prior_phase = phase
                try:
                    result = sequence.step(state, command or None)
                except Exception as error:
                    if not error_mode:
                        raise RuntimeError(f'{name} tick {tick} {phase}: {error}') from error
                    record.update(error=str(error), phase=sequence.phase)
                    out.write(json.dumps(plain(record))+'\n')
                    counts['ERROR'] = 1
                    break
                counts[result.phase] = counts.get(result.phase, 0)+1
                record.update(out_phase=result.phase, phase=sequence.phase, frame=result.frame,
                              active=getattr(h, 'active_name', spec['name']),
                              pending=getattr(h, 'pending_name', None) or '',
                              policy_ticks=h.policy_ticks, hold_ticks=getattr(h, 'hold_ticks', 0),
                              completed=getattr(h, 'completed_motions', []), finished=sequence.finished,
                              target=result.q, kp=result.kp, kd=result.kd)
                if tick % 10 == 0 or result.phase in ('WARMUP', 'READY_BLEND', 'RETURN', 'SETTLE'):
                    record['observation'] = result.observation
                gate = getattr(h, 'gate_result', None)
                if gate is not None:
                    record['gate'] = dict(ready=gate.ready, stable_ticks=gate.stable_ticks,
                                          metrics=gate.metrics)
                out.write(json.dumps(plain(record))+'\n')
                if sequence.finished:
                    break
            else:
                raise RuntimeError(f'{name}: did not finish ({sequence.phase}, stage={stage})')
        summary = dict(case=name, ticks=tick+1, phase_ticks=counts, visited=visited)
        summaries.append(summary)
        print('Generated', name, tick+1, 'ticks', flush=True)
        del sequence, h, p, standing
        gc.collect()

    if args.work_dir:
        args.work_dir.mkdir(parents=True, exist_ok=True)
    with (nullcontext(str(args.work_dir)) if args.work_dir else tempfile.TemporaryDirectory(prefix='g1_session_parity_')) as folder:
        work = Path(folder)
        names = [s['name'] for s in original_registry['policies'] if not s.get('ready_hold')]
        if args.quick:
            names = ['straight_21001', 'ufo_guard_stand_7501']
        for name in names:
            produce(work, 'oneshot_'+name, resolve(name))
        produce(work, 'automatic_stand_pre', resolve('straight_21001'), automatic=True)
        ready = copy.deepcopy(entry)
        ready['ready_hold'].pop('fighting')
        produce(work, 'ready_hold_all_routes', ready, 'ready')
        produce(work, 'fighting_all_actions_and_reentry', entry, 'fight')
        if not args.quick:
            for phase in ('WAIT_TAKEOVER', 'TAKEOVER', 'STAND_PRE', 'PREPARE', 'POLICY',
                          'READY_BLEND', 'HOLD', 'WARMUP', 'FIGHT_POLICY', 'SETTLE', 'RETURN', 'STAND_POST'):
                produce(work, 'stop_'+phase, entry, 'fight', stop_at=phase)
            for error in ('request_timeout', 'acquire_timeout', 'settle_timeout'):
                produce(work, error, entry, 'fight', error_mode=error)
        manifest_path = work/'manifest.yaml'
        manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
        result = subprocess.run([str(args.probe.resolve()), str(args.root.resolve()), str(manifest_path)],
                                text=True, capture_output=True, timeout=300)
        print(result.stdout, end='')
        if result.returncode:
            raise RuntimeError(result.stderr)
        numerical = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
        report = dict(status='passed', original=str(args.original), numerical=numerical,
                      coverage=summaries, total_ticks=sum(x['ticks'] for x in summaries),
                      scope='Real original Python actors and MuJoCo kinematics; deterministic measured-state replay. No robot commands.')
        if args.report:
            args.report.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
