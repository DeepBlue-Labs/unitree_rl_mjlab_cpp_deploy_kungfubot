#!/usr/bin/env python3
"""Validate restored registry metadata, list/inspect and identities offline."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--original", type=Path, required=True)
ap.add_argument("--root", type=Path, required=True)
ap.add_argument("--probe", type=Path, required=True)
args = ap.parse_args()
original = json.loads((args.original / "unified_deploy/policies.json").read_text())
restored = json.loads((args.root / "policies/registry.json").read_text())
assert restored["defaults"] == original["defaults"]
assert len(restored["policies"]) == len(original["policies"]) == 28
for old, new in zip(original["policies"], restored["policies"]):
    for key, value in old.items():
        if key not in ("bundle", "clip"):
            assert new[key] == value, (old["name"], key)
for key, old in original["training_sources"].items():
    new = restored["training_sources"][key]
    assert new["label"] == old["label"] and new["note"] == old["note"]
    for source, copied in zip(old.get("evidence", []), new.get("evidence", [])):
        assert (args.original/source).read_bytes() == (args.root/copied).read_bytes()
subprocess.run([str(args.probe), str(args.root), "--self-test"], check=True)
listed = subprocess.check_output([str(args.probe), str(args.root), "--list"], text=True)
for source in original["training_sources"].values():
    assert source["label"] in listed
for entry in original["policies"]:
    assert entry["motion_label"] in listed and entry["name"] in listed
manifest = yaml.safe_load(subprocess.check_output([str(args.probe), str(args.root), "entry_star"], text=True))
assert manifest["policy"]["ready_hold"] == original["policies"][19]["ready_hold"]
assert manifest["policy"]["prepare_s"] == .4
assert manifest["standing_source"]["waist_yaw_action_scale"] == .55
assert manifest["standing_source"]["original_cpp_velocity_waist_yaw_action_scale"] == .35
assert len(manifest["session_artifacts"]) == 10
for identity in [manifest["artifact_sha256"], *manifest["session_artifacts"].values()]:
    for path, digest in identity.items():
        assert hashlib.sha256((args.root/path).read_bytes()).hexdigest() == digest
for path, digest in manifest["standing_sha256"].items():
    assert hashlib.sha256((args.root/"policies/standing"/path).read_bytes()).hexdigest() == digest
with tempfile.TemporaryDirectory(prefix="g1-registry-invalid-") as tmp:
    file = Path(tmp)/"registry.json"
    bad = json.loads(json.dumps(restored))
    bad["policies"][1]["aliases"].append("official")
    file.write_text(json.dumps(bad))
    result = subprocess.run([str(args.probe), str(args.root), "--list", str(file)], capture_output=True, text=True)
    assert result.returncode != 0 and "duplicate" in result.stderr
    bad = json.loads(json.dumps(restored))
    bad["defaults"]["prepare_s"] = .401
    file.write_text(json.dumps(bad))
    result = subprocess.run([str(args.probe), str(args.root), "official", str(file)], capture_output=True, text=True)
    assert result.returncode != 0 and "control ticks" in result.stderr
print("Registry parity passed: 28 contracts, 6 Chinese provenance groups, defaults/ready_hold, aliases, detached resolves, 10 fighting identities, standing source split and malformed input rejection")
