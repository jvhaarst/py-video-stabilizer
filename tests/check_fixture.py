"""Assert stabilize.py still produces the committed conformance fixture.

Compared with a tolerance rather than byte-for-byte: the fixture holds
full-precision floats, and a different numpy or BLAS build can move the last
bit without anything being wrong.
"""
import json
import pathlib
import sys

import numpy as np

import make_fixture as F

TOL = 1e-9


def main():
    path = pathlib.Path(__file__).parent / "fixture.json"
    ref = json.loads(path.read_text())
    fresh = F.compute()

    worst = 0.0
    problems = []
    for key in ("steps", "rollDeg", "poses", "bbox"):
        a = np.asarray(fresh[key], dtype=float).ravel()
        b = np.asarray(ref[key], dtype=float).ravel()
        if a.shape != b.shape:
            problems.append(f"{key}: shape {a.shape} but fixture has {b.shape}")
            continue
        diff = float(np.max(np.abs(a - b))) if a.size else 0.0
        worst = max(worst, diff)
        print(f"{key:8s} max diff {diff:.3e}")
        if diff > TOL:
            problems.append(f"{key}: max diff {diff:.3e} exceeds {TOL:g}")

    if problems:
        print("\nFAILED — stabilize.py no longer matches tests/fixture.json:", file=sys.stderr)
        for p in problems:
            print("  " + p, file=sys.stderr)
        print("\nIf the change was intended, regenerate with "
              "`python tests/make_fixture.py` and commit the result.", file=sys.stderr)
        sys.exit(1)
    print(f"\nOK — stabilize.py reproduces the fixture (worst {worst:.3e})")


if __name__ == "__main__":
    main()
