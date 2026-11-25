#!/usr/bin/env python3
"""Compare C++ migration with the original Python adapters, entirely offline.

The original tree and its Python environment are required only for this test;
neither is a deployment runtime dependency.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--original", type=Path, required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--policy", action="append")
    args = parser.parse_args()
    sys.path.insert(0, str(args.original))
    sys.path.insert(0, str(args.original / "supermimic_reference/src"))
    from scipy.spatial.transform import Rotation
    import unified_deploy.adapters as adapters
    from unified_deploy.ufo_jointtrack import UfoJointTrackPolicy
    from hybrid_supermimic.runtime import RobotState
    adapters.ROOT = args.root
    registry = json.loads((args.root / "policies/registry.json").read_text())
    report = []
    guards_checked = False
    with tempfile.TemporaryDirectory(prefix="g1-motion-parity-") as directory:
        directory = Path(directory)
        for spec in registry["policies"]:
            if args.policy and spec["name"] not in args.policy:
                continue
            if spec["adapter"] not in ("hybrid", "reference", "ufo_jointtrack"):
                continue
            spec["stage"] = "ground"
            cls = {"hybrid": adapters.HybridPolicy, "reference": adapters.ReferencePolicy,
                   "ufo_jointtrack": UfoJointTrackPolicy}[spec["adapter"]]
            actor = cls(spec)
            n = actor.frame_count
            frames = [0, 1, 3, min(12, n-1), n-2, n-1, n-1, 0, 1]
            commands, states, expected = [], [], {k: [] for k in ("observation", "goal", "action", "target")}
            for i, frame in enumerate(frames):
                reset = i in (0, 7)
                hold = i == 6 and spec["adapter"] == "ufo_jointtrack"
                if spec["adapter"] == "reference":
                    ref_r = actor.core.clip.policy_reference["root_R"][frame]
                    ref_q = actor.core.clip.joint_pos[frame]
                else:
                    ref = actor.bundle.reference_at(frame)
                    ref_r = Rotation.from_quat(ref["root_quat_w"][[1,2,3,0]]).as_matrix()
                    ref_q = ref["joint_pos"]
                # Nonzero heading/roll, velocities and measured-pose perturbations
                # distinguish column-major mistakes, history layouts and reset seeds.
                rot = Rotation.from_euler("z", .37).as_matrix() @ ref_r @ Rotation.from_euler("x", .017).as_matrix()
                quat = Rotation.from_matrix(rot).as_quat()[[3,0,1,2]]
                q = np.asarray(ref_q, np.float64) + .013*np.sin(np.arange(29)*.47+i*.31)
                dq = .19*np.cos(np.arange(29)*.23+i*.17)
                gyro = np.array([.071+i*.003, -.13, .27])
                pos = np.array([.12, -.07, actor.reference_heights[frame]+.011])
                velocity = np.array([.21, -.14, .023])
                state = RobotState(pos, quat, velocity, gyro, q, dq)
                if reset:
                    actor.begin(state)
                while actor.frame < frame:
                    actor.advance()
                if hold:
                    actor.hold_final()
                step = actor.infer(state)
                assert actor.frame == frame
                obs = step.observation
                if spec["adapter"] == "reference":
                    observation, goal = obs[:990], obs[990:]
                else:
                    observation, goal = obs, obs[-640:]
                expected["observation"].append(observation)
                expected["goal"].append(goal)
                expected["action"].append(step.raw_action)
                expected["target"].append(step.q_target)
                states.append(np.concatenate([q, dq, quat, gyro, pos, velocity]))
                commands.append([frame, int(reset), int(hold)])
            fixture, output = directory / "input.npz", directory / "output.npz"
            np.savez(fixture, states=np.asarray(states, np.float64), commands=np.asarray(commands, np.float64))
            subprocess.run([str(args.probe), str(args.root), spec["name"], str(fixture), str(output)], check=True)
            with np.load(output) as actual:
                errors = {}
                for key, values in expected.items():
                    reference = np.asarray(values)
                    errors[key] = float(np.max(np.abs(actual[key] - reference)))
                    # Inference kernels can differ between ORT patch versions.
                    # Inputs must agree tightly; actor/target use 2e-4 abs tolerance.
                    tol = 2e-6 if key in ("observation", "goal") else 2e-4
                    np.testing.assert_allclose(actual[key], reference, atol=tol, rtol=2e-5, err_msg=spec["name"]+":"+key)
            report.append({"policy": spec["name"], "steps": len(frames), "max_abs_error": errors})
            print(json.dumps(report[-1]), flush=True)
            if spec["adapter"] == "reference" and not guards_checked:
                # Every rejected sample must fail before any target is returned.
                for fault in ("quaternion", "gyro", "position", "velocity", "tilt"):
                    bad = np.asarray(states[0], np.float64).copy()
                    if fault == "quaternion":
                        bad[58:62] = 0
                    elif fault == "gyro":
                        bad[62] = np.nan
                    elif fault == "position":
                        bad[0] = np.nan
                    elif fault == "velocity":
                        bad[29] = 100
                    elif fault == "tilt":
                        ref_r = actor.core.clip.policy_reference["root_R"][0]
                        bad[58:62] = Rotation.from_matrix(ref_r @ Rotation.from_euler("x", .6).as_matrix()).as_quat()[[3,0,1,2]]
                    np.savez(fixture, states=bad[None], commands=np.array([[0., 1., 0.]]))
                    result = subprocess.run([str(args.probe), str(args.root), spec["name"], str(fixture), str(output)], capture_output=True, text=True)
                    assert result.returncode != 0 and "reference guard:" in result.stderr, (fault, result)
                guards_checked = True
                print("Reference IMU, finite state, velocity and relative tilt guards passed", flush=True)
    print(json.dumps({"policies_passed": len(report), "steps": sum(r["steps"] for r in report), "results": report}, indent=2))


if __name__ == "__main__":
    main()
