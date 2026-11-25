#!/usr/bin/env python3
"""Exercise the actual C++ CLI and MuJoCo plant; never connect to DDS.

Checks terminal FIFO/EOF, pause, bounded damping, all motion frames, physical
fighting handoffs, automatic screening, inspect and pre-DDS argument rejection.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time
import numpy as np


class Suite:
    def __init__(self, args):
        self.args = args
        self.root = args.root.resolve()
        self.binary = args.binary.resolve()
        self.work = args.output.resolve()
        self.work.mkdir(parents=True, exist_ok=False)
        self.reports = {}
        self.registry = json.loads((self.root / "policies/registry.json").read_text())

    def resolve(self, key):
        return next(s for s in self.registry["policies"] if key in [s["name"], *s.get("aliases", [])])

    def frames(self, key):
        spec = self.resolve(key)
        with np.load(self.root / spec.get("clip", spec["bundle"]+"/reference.npz")) as z:
            return len(z["joint_pos"])

    def command(self, name, *flags):
        return [str(self.binary), "--root", str(self.root), "--output", str(self.work/name), *map(str, flags)]

    def run(self, name, *flags, input_text=None):
        result = subprocess.run(self.command(name, *flags), input=input_text, text=True, capture_output=True, timeout=120)
        (self.work/(name+".log")).write_text(result.stdout+result.stderr)
        assert result.returncode == 0, (name, result.returncode, result.stdout[-5000:], result.stderr)
        return result

    def load(self, name):
        directory = self.work/name
        summary = json.loads((directory/"summary.json").read_text())
        with np.load(directory/"trace.npz") as z:
            data = {k: z[k] for k in z.files}
        assert not summary["error"], (name, summary)
        assert summary["robot_commands_published"] == 0, (name, "simulation opened robot control")
        assert data["phase"].dtype.kind == "U" and data["active_policy"].dtype.kind == "U", "NPZ must preserve original Unicode string contract"
        assert dict(Counter(data["phase"])) == summary["phase_ticks"], name
        assert np.isfinite(data["target"]).all() and np.isfinite(data["kp"]).all() and np.isfinite(data["kd"]).all()
        active = np.isin(data["phase"], ["STAND_PRE", "PREPARE", "POLICY", "RETURN", "STAND_POST", "HOLD", "READY_BLEND", "WARMUP", "SETTLE"])
        if active.any():
            assert not data["support"][active].any(), (name, "support applied during active control")
        self.reports[name] = {"phase_ticks": summary["phase_ticks"], "completed_motions": summary.get("completed_motions", []), "stop_reason": summary["stop_reason"], "samples": len(data["phase"])}
        return summary, data

    @staticmethod
    def policy_runs(data):
        runs = []
        current = None
        for phase, name, frame in zip(data["phase"], data["active_policy"], data["motion_frame"]):
            if phase != "POLICY":
                current = None
            else:
                if current is None or current["policy"] != name:
                    current = {"policy": str(name), "frames": []}
                    runs.append(current)
                current["frames"].append(int(frame))
        return runs

    def assert_runs(self, data, names):
        expected = [{"policy": self.resolve(name)["name"], "frames": list(range(self.frames(name)))} for name in names]
        assert self.policy_runs(data) == expected, (self.policy_runs(data), expected)

    def scripted(self, name, policy, commands, steps):
        script = self.work/(name+".yaml")
        # JSON is a YAML subset; no Python YAML package is required by this test.
        script.write_text(json.dumps([{"tick": tick, "command": command} for tick, command in commands]))
        trace = self.work/(name+".csv")
        result = self.run(name, "--mode", "sim2sim", "--policy", policy, "--headless", "--fast", "--steps", steps,
                          "--script", script, "--trace", trace)
        with trace.open() as stream:
            rows = {int(row["tick"]): row for row in csv.DictReader(stream)}
        return result, rows, *self.load(name)

    def one_shot(self):
        result, rows, summary, data = self.scripted("one_shot", "straight", [
            (0, "START"), (1, "TAKEOVER"), (2, "START"),
            (210, "PAUSE"), (211, "START"), (212, "NEXT hook"), (220, "RESUME"),
            (370, "START"), (550, "START"), (600, "PAUSE"),
            (605, "START"), (605, "STOP"), (605, "TAKEOVER")], 710)
        counts = summary["phase_ticks"]
        assert counts["TAKEOVER"] == 200 and counts["DAMP"] == 50, counts
        assert counts["PREPARE"] == 20 and counts["RETURN"] == 25 and counts["STAND_PRE"] >= 100
        assert "START ignored in WAIT_TAKEOVER" in result.stdout and "START ignored in TAKEOVER" in result.stdout
        assert "START ignored" in result.stdout and counts["POLICY"] == self.frames("straight")
        self.assert_runs(data, ["straight"])
        for begin, end in [(210, 219), (600, 604)]:
            for tick in range(begin, end+1):
                assert rows[tick]["paused"] == "1"
                assert rows[tick]["time"] == rows[begin]["time"]
                assert rows[tick]["policy_frame"] == rows[begin]["policy_frame"]
        damp = data["phase"] == "DAMP"
        assert np.all(data["kp"][damp] == 0) and np.all(data["kd"][damp] == 3)
        assert np.all(data["phase"][-50:] == "DAMP") and summary["stop_reason"] == "STOP requested"
        takeover = data["phase"] == "TAKEOVER"
        assert np.all(data["support"][takeover])
        np.testing.assert_allclose(np.diff(data["sim_time"][takeover]), .02, atol=1e-9)

    def terminal_fifo_eof(self):
        name = "terminal_fifo_eof"
        trace = self.work/(name+".csv")
        process = subprocess.Popen(self.command(name, "--mode", "sim2sim", "--policy", "straight", "--headless", "--fast", "--trace", trace),
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        lines, transcript = queue.Queue(), []
        def reader():
            for line in process.stdout:
                transcript.append(line)
                lines.put(line)
            lines.put(None)
        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
        def wait_for(text):
            deadline = time.monotonic()+45
            while time.monotonic() < deadline:
                line = lines.get(timeout=max(.01, deadline-time.monotonic()))
                assert line is not None, (text, "".join(transcript))
                if text in line:
                    return
            raise AssertionError((text, "".join(transcript)))
        try:
            wait_for("WAIT_TAKEOVER")
            # Both lines enter one OS read; neither may overwrite the other.
            process.stdin.write("TAKEOVER\nSTART\n")
            process.stdin.flush()
            wait_for("Stand/STAND_PRE/")
            process.stdin.write("PAUSE\n")
            process.stdin.flush()
            time.sleep(.12)
            process.stdin.close()  # EOF must unpause and complete all damping ticks.
            process.wait(timeout=15)
            thread.join(timeout=2)
            assert process.returncode == 0, "".join(transcript)
        finally:
            if process.poll() is None:
                process.kill(); process.wait()
            thread.join(timeout=2)
            process.stdout.close()
            (self.work/(name+".log")).write_text("".join(transcript))
        summary, data = self.load(name)
        assert summary["phase_ticks"]["TAKEOVER"] == 200
        assert summary["phase_ticks"]["DAMP"] == 50
        assert summary["phase_ticks"].get("POLICY", 0) == 0, "START queued during TAKEOVER was replayed later"
        with trace.open() as stream:
            paused = [row for row in csv.DictReader(stream) if row["paused"] == "1"]
        assert paused and len({r["time"] for r in paused}) == 1
        assert np.all(data["phase"][-50:] == "DAMP")
        # An emergency in the same input batch wins even without a final newline.
        self.run("stop_priority_eof", "--mode", "sim2sim", "--policy", "straight", "--headless", "--fast", input_text="TAKEOVER\n  sToP  ")
        summary, data = self.load("stop_priority_eof")
        assert summary["phase_ticks"].get("TAKEOVER", 0) == 0
        assert summary["phase_ticks"]["DAMP"] == 50 and np.all(data["kp"] == 0)

    def fighting(self):
        result, _, summary, data = self.scripted("fighting", "entry_star", [
            (0, "TAKEOVER"), (350, "START"), (600, "HELP"), (601, "STATUS"), (800, "straight"),
            (1400, "NEXT hook"), (2100, "NEXT exit_star"),
            (2500, "entry_star"), (3000, "STOP")], 3100)
        names = ["entry_star", "straight", "hook", "exit_star", "entry_star"]
        self.assert_runs(data, names)
        expected = [{"policy": self.resolve(name)["name"], "frames": self.frames(name)} for name in names]
        assert summary["completed_motions"] == expected
        assert summary["phase_ticks"]["TAKEOVER"] == 200 and summary["phase_ticks"]["DAMP"] == 50
        assert summary["phase_ticks"]["WARMUP"] == 3*26
        assert summary["phase_ticks"]["STAND_POST"] >= 100
        assert summary["active_policy"] == self.resolve("home_star")["name"]
        assert summary["last_motion"] == self.resolve("entry_star")["name"]
        assert summary["motion_complete"]
        identities = json.loads((self.work/"fighting/session.json").read_text())["artifact_sha256"]
        assert len(identities) == 10
        active = ~np.isin(data["phase"], ["WAIT_TAKEOVER", "TAKEOVER", "DAMP"])
        assert data["height"][active].min() > .6 and data["tilt"][active].max() < .45
        assert np.isfinite(data["raw_action"][data["phase"] == "POLICY"]).all()
        help_line = next(line for line in result.stdout.splitlines() if "In HOLD verified actions:" in line)
        shown = set(help_line.split("In HOLD verified actions:", 1)[1].split(";", 1)[0].split())
        expected_actions = set(self.resolve("entry_star")["ready_hold"]["fighting"]["actions"])
        assert len(expected_actions) == 7 and shown == expected_actions, help_line
        status_line = next(line for line in result.stdout.splitlines() if " | motion=" in line)
        assert " | gate=" in status_line and f"motion={self.resolve('entry_star')['name']} {self.frames('entry_star')}/{self.frames('entry_star')}" in status_line, status_line

    def ui_selection(self):
        _, _, summary, data = self.scripted("ui_selection", "straight", [
            (0, "TAKEOVER"), (350, "SELECT hook"), (351, "START"),
            (600, "passive"), (610, "TAKEOVER"),
            (730, "SELECT straight"), (731, "START"), (1000, "STOP")], 1100)
        # S/START must execute the displayed selection, including a selection
        # made while a different native session is waiting in STAND_PRE.
        self.assert_runs(data, ["hook", "straight"])
        assert summary["phase_ticks"]["TAKEOVER"] == 200
        assert summary["phase_ticks"]["DAMP"] == 50
        assert summary["phase_ticks"]["PREPARE"] == 40
        assert summary["phase_ticks"]["RETURN"] == 50

        _, _, summary, data = self.scripted("ui_selection_hold", "straight", [
            (0, "SELECT entry_star"), (1, "TAKEOVER"), (350, "START"),
            (800, "SELECT straight"), (801, "START"),
            (1150, "SELECT hook"), (1151, "NEXT hook"),
            (1500, "START"), (1800, "STOP")], 1900)
        # Selection before TAKEOVER chooses the complete entry-star session;
        # selection in HOLD follows the fighting NEXT handoff, while a later
        # unqualified START must not replay the action.
        self.assert_runs(data, ["entry_star", "straight", "hook"])
        assert summary["phase_ticks"]["TAKEOVER"] == 200
        assert summary["phase_ticks"]["DAMP"] == 50
        assert summary["phase_ticks"]["WARMUP"] == 2*26
        assert summary["active_policy"] == self.resolve("home_star")["name"]
        assert summary["last_motion"] == self.resolve("hook")["name"]

    def automatic(self):
        for name, policy in [("automatic_straight", "straight"), ("automatic_hold", "entry_star")]:
            self.run(name, "--mode", "sim", "--policy", policy, "--fast")
            summary, data = self.load(name)
            self.assert_runs(data, [policy])
            assert summary["complete"] and summary["screen_passed"]
            terminal = "HOLD" if policy == "entry_star" else "STAND_POST"
            assert summary["phase_ticks"][terminal] == 500
            assert not data["support"].any()
            assert summary["phase_ticks"]["STAND_PRE"] == 250
            required = {"phase", "active_policy", "motion_frame", "target", "measured", "height", "tilt", "pitch", "max_joint_speed", "observation", "raw_action", "kp", "kd", "sim_time"}
            assert required <= data.keys()
            if policy == "entry_star":
                assert summary["hold_stable_last_1s"]
                assert summary["hold_pose_rmse_last_1s"] < .15
                assert summary["hold_pitch_ptp_deg_last_1s"] < .2
                assert summary["hold_max_joint_speed_last_1s"] < .1
            else:
                assert summary["standing_recovered_last_1s"]

    def cli(self):
        result = subprocess.run([str(self.binary), "--root", str(self.root), "--policy", "entry_star", "--inspect", "--mode", "real", "--iface", "this_interface_does_not_exist", "--domain", "0"],
                                text=True, capture_output=True, timeout=45)
        assert result.returncode == 0, result.stderr
        inspect = json.loads(result.stdout)
        assert len(inspect["session_artifacts"]) == 10 and inspect["policy"]["prepare_s"] == .4
        bad = [(["--mode", "real", "--iface", "lo", "--domain", "0"], "wired domain 0"),
               (["--mode", "real", "--auto-start", "--domain", "0"], "interactive"),
               (["--mode", "real", "--steps", "10", "--domain", "0"], "interactive"),
               (["--mode", "real", "--watchdog-ms", "0"], "invalid setting"),
               (["--mode", "real", "--pre-s", ".401"], "ticks"),
               (["--mode", "real", "--render"], "require"),
               (["--mode", "readonly", "--domain", "-1"], "nonnegative")]
        for i, (flags, message) in enumerate(bad):
            name = "invalid_"+str(i)
            result = subprocess.run(self.command(name, "--policy", "straight", *flags), text=True, capture_output=True, timeout=15)
            assert result.returncode != 0 and message in result.stderr, (flags, result.stdout, result.stderr)
            assert not (self.work/name).exists(), "argument rejection occurred after run/transport startup"
        self.reports["cli"] = {"invalid_cases": len(bad), "inspect_json": True, "DDS_opened": False}

    def save(self):
        (self.work/"validation.json").write_text(json.dumps(self.reports, ensure_ascii=False, indent=2)+"\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", type=Path, required=True)
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--output", type=Path, help="new results directory; default a unique temporary run")
    ap.add_argument("--only", choices=["one_shot", "terminal_fifo_eof", "fighting", "ui_selection", "automatic", "cli"])
    args = ap.parse_args()
    if args.output is None:
        args.output = Path(tempfile.mkdtemp(prefix="g1-session-integration-"))/"run"
    suite = Suite(args)
    for name in ([args.only] if args.only else ["cli", "one_shot", "terminal_fifo_eof", "fighting", "ui_selection", "automatic"]):
        try:
            getattr(suite, name)()
            suite.save()
            print(name+": passed", flush=True)
        except Exception:
            suite.save()
            raise
    print("C++ session integration passed; results: "+str(suite.work/"validation.json"))


if __name__ == "__main__":
    main()
