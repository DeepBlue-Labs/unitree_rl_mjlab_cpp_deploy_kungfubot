#!/usr/bin/env python3
"""Generate real NumPy fixtures and exercise the bundled C++ NPZ reader."""
import argparse
from pathlib import Path
import subprocess
import tempfile
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--probe", required=True, type=Path)
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix="g1-npz-compat-") as directory:
    directory = Path(directory)
    matrix = np.arange(12, dtype=np.float64).reshape(3, 4) / 8
    arrays = dict(names=np.array(["alpha", "站立"], dtype="<U5"), scalar=np.array(3.25), matrix=matrix)
    np.savez(directory / "stored.npz", **arrays)
    np.savez_compressed(directory / "compressed.npz", **arrays)
    with (directory / "v2.npy").open("wb") as stream:
        np.lib.format.write_array(stream, matrix, version=(2, 0))
    subprocess.run([str(args.probe), str(directory)], check=True)
