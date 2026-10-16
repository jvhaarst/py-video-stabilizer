"""Generate the conformance fixture the browser port is checked against.

The fixture holds synthetic per-frame step matrices and the poses, matrices and
canvas bounds that stabilize.py derives from them. Synthetic input keeps the
test deterministic and free of any video file, so it runs anywhere.
"""
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import stabilize as S

W, H = 1920, 1080
N = 400
LOCK, SIGMA, OFFSET_DEG = 1.0, 20.0, -4.7


def synthetic_steps(n, seed=20261016):
    """A plausible flight: a drifting roll and a dive, plus per-frame shake."""
    rng = np.random.default_rng(seed)
    steps = [np.eye(3)]
    for i in range(1, n):
        t = i / n
        angle = np.radians(0.9 * np.sin(6.3 * t) + 0.25) + rng.normal(0, 0.004)
        scale = 1.0 + 0.0015 * np.sin(3.1 * t) + rng.normal(0, 0.0007)
        tx = 6.0 * np.cos(4.7 * t) + rng.normal(0, 1.6)
        ty = 4.0 * np.sin(2.9 * t) + rng.normal(0, 1.6)
        steps.append(S.similarity(angle, scale, tx, ty))
    return steps


def main():
    steps = synthetic_steps(N)
    cx, cy = W / 2, H / 2
    to_c, from_c = S.translation(-cx, -cy), S.translation(cx, cy)

    cum = [np.eye(3)]
    for st in steps[1:]:
        cum.append((to_c @ st @ from_c) @ cum[-1])
    roll = np.unwrap(np.array([S.decompose(m)[0] for m in cum]))

    lo, hi = S.segment_bounds([], len(steps))
    radius = int(3 * SIGMA)
    offset = np.radians(OFFSET_DEG)
    cap = 0.35 * min(W, H)

    poses = []
    for i in range(len(steps)):
        j0, j1 = max(lo[i], i - radius), min(hi[i], i + radius)
        a, sc, tx, ty = S.local_average_pose(cum, i, j0, j1, SIGMA)
        angle = (1 - LOCK) * a + LOCK * (offset - roll[i])
        shift = np.hypot(tx, ty)
        if shift > cap:
            tx, ty = tx * cap / shift, ty * cap / shift
        poses.append((angle, sc, tx, ty))

    mats = [from_c @ S.similarity(*p) @ to_c for p in poses]
    x0, x1, y0, y1 = S.warped_bounds(mats, W, H)

    out = {
        "width": W, "height": H, "lock": LOCK, "sigma": SIGMA, "offsetDeg": OFFSET_DEG,
        "steps": [[float(v) for v in m.flatten()] for m in steps],
        "rollDeg": [float(np.degrees(r)) for r in roll],
        "poses": [[float(v) for v in p] for p in poses],
        "bbox": [float(x0), float(x1), float(y0), float(y1)],
    }
    path = pathlib.Path(__file__).parent / "fixture.json"
    path.write_text(json.dumps(out))
    print(f"wrote {path} — {len(steps)} frames, canvas {x1-x0:.0f}x{y1-y0:.0f}")


if __name__ == "__main__":
    main()
