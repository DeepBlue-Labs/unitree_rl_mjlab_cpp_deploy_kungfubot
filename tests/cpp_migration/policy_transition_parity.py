#!/usr/bin/env python3
"""Compare policy state across warmup, feedback, hold, seek and re-entry.

Original Python sources are test dependencies only. No simulator or DDS is used.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--original", type=Path, required=True)
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--probe", type=Path, required=True)
    ap.add_argument("--report", type=Path)
    args = ap.parse_args()
    sys.path[:0] = [str(args.original), str(args.original / "supermimic_reference/src")]
    from scipy.spatial.transform import Rotation
    import unified_deploy.adapters as adapters
    from unified_deploy.ufo_jointtrack import UfoJointTrackPolicy
    from hybrid_supermimic.runtime import RobotState
    from hybrid_supermimic.standing import VelocityStandingPolicy
    adapters.ROOT = args.root
    registry = json.loads((args.root / "policies/registry.json").read_text())
    report = []
    specs = registry["policies"] + [{"name": "PythonStanding", "adapter": "standing"}]
    with tempfile.TemporaryDirectory(prefix="g1-policy-transition-") as tmp:
        tmp = Path(tmp)
        for spec in specs:
            kind = spec["adapter"]
            standing = kind == "standing"
            spec["stage"] = "ground"
            if standing:
                actor = VelocityStandingPolicy(args.root / "policies/standing")
                count = 1
            else:
                actor = {"hybrid": adapters.HybridPolicy, "reference": adapters.ReferencePolicy,
                         "ufo_jointtrack": UfoJointTrackPolicy}[kind](spec)
                count = actor.frame_count
            steps = [0, 0, 0, 1, 5, 12, count-1, count-1, 0, 7, 7]
            steps = [min(f, count-1) for f in steps]
            expected = {k: [] for k in ("observation", "goal", "action", "target", "reference_joints")}
            states, commands, feedback = [], [], []
            previous = None
            for i, frame in enumerate(steps):
                reset = 1 if i in (0, 8, 9) else 2 if i == 10 else 0
                holdop = (1 if i in (2, 6) else 2 if i == 3 else 0) if kind in ("hybrid", "ufo_jointtrack") else 0
                holdframe = min(2, count-1) if i == 2 else count-1
                use_feedback = not reset and kind != "reference"
                if standing:
                    ref_r = np.eye(3)
                    ref_q = actor.default_q
                    height = .78
                elif kind == "reference":
                    ref_r = actor.core.clip.policy_reference["root_R"][frame]
                    ref_q = actor.core.clip.joint_pos[frame]
                    height = actor.reference_heights[frame]
                else:
                    ref = actor.bundle.reference_at(frame)
                    ref_r = Rotation.from_quat(ref["root_quat_w"][[1, 2, 3, 0]]).as_matrix()
                    ref_q = ref["joint_pos"]
                    height = actor.reference_heights[frame]
                rot = Rotation.from_euler("z", .27+i*.019).as_matrix() @ ref_r @ Rotation.from_euler("x", .023).as_matrix()
                quat = Rotation.from_matrix(rot).as_quat()[[3, 0, 1, 2]]
                q = np.asarray(ref_q, np.float64) + .014*np.sin(np.arange(29)*.41+i*.29)
                dq = .17*np.cos(np.arange(29)*.33+i*.11)
                gyro, pos, vel = np.array([.09, -.11, .13+i*.007]), np.array([.02, -.04, height+.006]), np.array([.13, .04, -.01])
                state = RobotState(pos, quat, vel, gyro, q, dq)
                if reset == 1:
                    actor.reset() if standing else actor.begin(state)
                elif reset == 2:
                    if standing:
                        actor.reset()
                    elif kind == "reference":
                        actor.core.last_action.fill(0)
                    else:
                        actor.bundle.last_action.fill(0)
                if not standing:
                    if kind == "reference":
                        actor.core.frame = frame
                    else:
                        actor.bundle.frame = frame
                        if holdop:
                            actor.bundle.hold_frame = holdframe if holdop == 1 else None
                q_feedback = q if previous is None else .37*previous.astype(np.float64)+.63*q
                if use_feedback:
                    if standing:
                        actor.last_action = ((q_feedback-actor.action_offset)/actor.action_scale).astype(np.float32)
                    else:
                        actor.bundle.last_action = ((q_feedback-actor.bundle.default_q)/actor.bundle.action_scale).astype(np.float32)
                result = actor.infer(state)
                previous = result.q_target
                obs = result.observation
                expected["observation"].append(obs[:990] if kind == "reference" else obs)
                expected["goal"].append(obs[990:] if kind == "reference" else np.empty(0, np.float32) if standing else obs[-640:])
                expected["action"].append(result.raw_action)
                expected["target"].append(result.q_target)
                expected["reference_joints"].append(ref_q)
                states.append(np.concatenate([q, dq, quat, gyro, pos, vel]))
                commands.append([frame, reset, holdop, holdframe, use_feedback])
                feedback.append(q_feedback)
            fixture, output = tmp / "input.npz", tmp / "output.npz"
            np.savez(fixture, states=np.asarray(states, np.float64), commands=np.asarray(commands, np.float64), feedback=np.asarray(feedback, np.float64))
            subprocess.run([str(args.probe), str(args.root), spec["name"], str(fixture), str(output)], check=True)
            errors = {}
            with np.load(output) as actual:
                for key, values in expected.items():
                    reference = np.asarray(values)
                    errors[key] = float(np.max(np.abs(actual[key]-reference))) if reference.size else 0.
                    np.testing.assert_allclose(actual[key], reference, atol=3e-6 if key in ("observation", "goal") else 2e-4,
                                               rtol=2e-5, err_msg=spec["name"]+":"+key)
                # Quaternion sign is irrelevant; compare the represented orientation.
                quat = actual["reference_quaternion"]
                actual_r = Rotation.from_quat(quat[:, [1, 2, 3, 0]]).as_matrix()
                refs = []
                for f in steps:
                    if standing:
                        refs.append(np.eye(3))
                    elif kind == "reference":
                        refs.append(actor.core.clip.policy_reference["root_R"][f])
                    else:
                        refs.append(Rotation.from_quat(actor.bundle.ref_root_quat[f, [1, 2, 3, 0]]).as_matrix())
                np.testing.assert_allclose(actual_r, refs, atol=2e-6, rtol=2e-5)
            report.append({"policy": spec["name"], "steps": len(steps), "feedback_and_reference_freeze": kind in ("hybrid", "ufo_jointtrack"), "max_abs_error": errors})
            print(json.dumps(report[-1]), flush=True)
    summary = {"policies_passed": len(report), "steps": sum(r["steps"] for r in report), "cases": ["repeated-frame warmup", "actual blended-target action feedback", "arbitrary reference-frame freeze with zero reference velocity", "clear reference freeze", "seek without reset", "begin after hold", "begin then seek", "last-action-only reset", "reference pose/quaternion access"], "results": report}
    if args.report:
        args.report.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "results"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
