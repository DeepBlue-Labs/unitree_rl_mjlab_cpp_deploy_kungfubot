"""Build the offline C++ probe and compare to the untrimmed Python modules.

Run this with the source project's existing Python environment. By default,
compile against that Python installation's MuJoCo headers/library to compare
the same physics engine version. --bundled additionally smoke-tests the bundled
library; its version can differ, so only estimator/gravity parity is asserted.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import tempfile

import mujoco
import numpy as np


def compile_probe(root, output, mujoco_root):
    libraries = list(mujoco_root.glob("libmujoco.so*")) or list((mujoco_root / "lib").glob("libmujoco.so*"))
    if not libraries:
        raise RuntimeError(f"MuJoCo shared library missing in {mujoco_root}")
    library = min(libraries, key=lambda item: len(item.name))
    subprocess.run([
        "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "deploy/robots/g1/include"), "-I", "/usr/include/eigen3",
        "-I", str(mujoco_root / "include"),
        str(root / "tests/cpp_migration/physics_probe.cpp"),
        str(root / "deploy/robots/g1/src/control/Physics.cpp"),
        str(library), "-lyaml-cpp", f"-Wl,-rpath,{library.parent}", "-o", str(output),
    ], check=True)


def state_dict(state):
    return dict(q=state.joint_pos, dq=state.joint_vel, quat=state.root_quat_w,
                gyro=state.root_ang_vel_b, root_pos=state.root_pos_w, root_vel_b=state.root_lin_vel_b)


def check(actual, expected, path="", atol=2e-10):
    if isinstance(expected, dict):
        return max(check(actual[key], value, f"{path}/{key}", atol) for key, value in expected.items())
    if isinstance(expected, list) and expected and isinstance(expected[0], dict):
        return max(check(a, e, f"{path}/{i}", atol) for i, (a, e) in enumerate(zip(actual, expected, strict=True)))
    np.testing.assert_allclose(actual, expected, rtol=2e-10, atol=atol, err_msg=path)
    return float(np.max(np.abs(np.asarray(actual, dtype=float)-np.asarray(expected, dtype=float))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original", type=pathlib.Path,
                        default=pathlib.Path("/home/pfy/unitree_rl_mjlab_supermimic"))
    parser.add_argument("--bundled", action="store_true")
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(args.original))
    from hybrid_supermimic.plant import MujocoPlant
    from hybrid_supermimic.estimator import G1FootKinematicEstimator
    from hybrid_supermimic.runtime import quat_to_matrix
    from unified_deploy.preparation import UpperBodyGravity
    from unified_deploy.interactive_sim import VirtualHoist

    config_path = root / "policies/supermimic_horse_stance_v2/deploy_config.json"
    config = json.loads(config_path.read_text())
    names = tuple(config["joint_names"])
    model_path = root / "assets/supermimic_g1/scene_29dof.xml"
    worst = 0.
    with tempfile.TemporaryDirectory(prefix="g1_physics_parity_") as work:
        work = pathlib.Path(work)
        executable = work / "physics_probe"
        compile_probe(root, executable, pathlib.Path(mujoco.__file__).parent)
        bundled = work / "physics_bundled"
        if args.bundled:
            compile_probe(root, bundled, root / "simulate/mujoco")
        for index in range(4):
            rng = np.random.default_rng(83+index)
            q = np.asarray(config["default_joint_pos"]) + rng.uniform(-.08, .08, 29)
            quat = np.array([1., .02, -.06, .10]) + rng.uniform(-.03, .03, 4)
            quat /= np.linalg.norm(quat)
            fixture = dict(q=q, dq=rng.uniform(-.2, .2, 29), quat=quat,
                           gyro=rng.uniform(-.1, .1, 3), root_pos=np.array([.2, -.3, .82]),
                           root_vel_b=rng.uniform(-.05, .05, 3), kp=np.asarray(config["kp"]),
                           kd=np.asarray(config["kd"]), effort=np.asarray(config["effort_limit"]),
                           target=q+rng.uniform(-.6, .6, 29))
            if index == 3:
                fixture["effort"] *= .02  # Verify clipping and saturation accounting.
            fixture_path = work / f"fixture_{index}.json"
            fixture_path.write_text(json.dumps({k: v.tolist() for k, v in fixture.items()}))
            plant = MujocoPlant(model_path, names, physics_dt=.002, control_dt=.02)
            rotation = quat_to_matrix(quat)
            plant.reset(root_pos_w=fixture["root_pos"], root_quat_w=quat,
                        root_lin_vel_w=rotation @ fixture["root_vel_b"], root_ang_vel_w=rotation @ fixture["gyro"],
                        joint_pos=q, joint_vel=fixture["dq"])
            initial = plant.state()
            estimator = G1FootKinematicEstimator(model_path, names)
            gravity = UpperBodyGravity(names, fixture["effort"], model_path)
            expected = dict(reset=state_dict(initial),
                            estimate=state_dict(estimator.estimate(quat, fixture["gyro"], q, fixture["dq"])),
                            gravity=gravity.offset(initial, fixture["kp"]))
            gravity.max_torque *= .01
            expected["gravity_limited"] = gravity.offset(initial, fixture["kp"])
            hoist = VirtualHoist(plant)
            expected["steps"] = []
            for tick in range(12):
                hoist.set_enabled(3 <= tick < 9)
                if tick == 8:
                    plant.substeps = 1
                state = plant.step(fixture["target"], fixture["kp"], fixture["kd"], fixture["effort"],
                                   before_substep=hoist.apply)
                expected["steps"].append(state_dict(state))
            expected.update(torque=plant.last_torque, saturation_count=plant.torque_saturation_count,
                            time=plant.data.time, validation=True)
            output = subprocess.check_output([str(executable), str(model_path), str(config_path), str(fixture_path)])
            actual = json.loads(output)
            worst = max(worst, check(actual, expected))
            if args.bundled:
                bundled_result = json.loads(subprocess.check_output([
                    str(bundled), str(model_path), str(config_path), str(fixture_path)]))
                check(bundled_result, {k: expected[k] for k in ("reset", "estimate", "gravity", "gravity_limited")})
                if not np.isfinite([v for step in bundled_result["steps"] for values in step.values() for v in values]).all():
                    raise AssertionError("bundled physics produced non-finite state")
        print(f"PASS: 4 fixtures, reset/estimator/gravity/12 plant steps/support toggle/PD saturation; max error {worst:.3g}")
        print(f"Exact parity MuJoCo {mujoco.__version__}" +
              (f"; bundled {bundled_result['version']} estimator/gravity parity and integration smoke passed" if args.bundled else ""))


if __name__ == "__main__":
    main()
