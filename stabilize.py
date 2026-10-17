#!/usr/bin/env python3
"""Stabilize shaky video by smoothing the camera path locally.

Each frame is placed at the weighted average of the poses its neighbours had,
measured relative to that frame itself. Because the average is taken relative
to the current frame, the correction stays bounded by local motion: fast but
deliberate movement (an FPV dive, a pan) survives, while jitter does not.

Rotation is handled separately and can be locked absolutely (--lock 1.0), so
the picture holds the orientation of the first frame for the whole clip
instead of rolling with the aircraft.

The frame is drawn zoomed *out* inside the canvas (--zoom, or --fit to compute
it automatically), so corrections are never paid for with a crop; uncovered
canvas is left black.
"""
import argparse
import subprocess
import sys

import cv2
import numpy as np


def similarity(angle, scale, tx, ty):
    """Build a 3x3 similarity matrix from its decomposed parameters."""
    c, s = scale * np.cos(angle), scale * np.sin(angle)
    return np.array([[c, -s, tx], [s, c, ty], [0.0, 0.0, 1.0]])


def decompose(m):
    """Split a 3x3 similarity matrix into (angle, scale, tx, ty)."""
    return (
        np.arctan2(m[1, 0], m[0, 0]),
        np.hypot(m[0, 0], m[1, 0]),
        m[0, 2],
        m[1, 2],
    )


def translation(tx, ty):
    return np.array([[1.0, 0.0, tx], [0.0, 1.0, ty], [0.0, 0.0, 1.0]])


def build_mask(h, w, top_frac, bottom_frac):
    """Mask out horizontal bands that hold burned-in HUD graphics."""
    mask = np.full((h, w), 255, np.uint8)
    mask[: int(h * top_frac), :] = 0
    mask[int(h * (1 - bottom_frac)) :, :] = 0
    return mask


def estimate_steps(path, track_width, mask_top, mask_bottom):
    """Measure the transform from each frame to the next.

    Returns the per-frame step matrices, the indices where the footage cuts to
    a different shot, the frame size and the frame rate.
    """
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"cannot open {path}")
    ok, prev = cap.read()
    if not ok:
        sys.exit("empty video")

    h, w = prev.shape[:2]
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    down = min(1.0, track_width / w)

    def small(frame):
        resized = cv2.resize(frame, None, fx=down, fy=down, interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    prev_gray = small(prev)
    mask = build_mask(*prev_gray.shape[:2], mask_top, mask_bottom)

    steps = [np.eye(3)]
    cuts = []
    lost = 0
    diag = {"inlier": [], "split": [], "trans": []}

    feature_args = dict(maxCorners=800, qualityLevel=0.01, minDistance=10, blockSize=7)
    lk_args = dict(
        winSize=(21, 21),
        maxLevel=4,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = small(frame)

        step, failed = np.eye(3), True
        p0 = cv2.goodFeaturesToTrack(prev_gray, mask=mask, **feature_args)
        if p0 is not None and len(p0) >= 10:
            p1, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk_args)
            # Backward check: a point must track back to where it came from.
            p0r, status_b, _ = cv2.calcOpticalFlowPyrLK(gray, prev_gray, p1, None, **lk_args)
            good = (
                status.ravel().astype(bool)
                & status_b.ravel().astype(bool)
                & (np.linalg.norm(p0 - p0r, axis=2).ravel() < 1.0)
            )
            if good.sum() >= 10:
                m, inl = cv2.estimateAffinePartial2D(
                    p0[good], p1[good], method=cv2.RANSAC, ransacReprojThreshold=3.0
                )
                if m is not None:
                    diag["inlier"].append(float(inl.ravel().mean()) if inl is not None else 0.0)
                    diag["trans"].append(float(np.hypot(m[0, 2], m[1, 2]) / down / w))
                    # Fit the halves of the frame separately. On a far, planar
                    # scene they agree; under parallax or an independently moving
                    # subject they do not, which is exactly when the rotation
                    # estimate stops being trustworthy. Sampled, since it costs
                    # two more fits.
                    if len(steps) % 3 == 0:
                        sp = p0[good].reshape(-1, 2)
                        dp = p1[good].reshape(-1, 2)
                        up = sp[:, 1] < prev_gray.shape[0] / 2
                        if up.sum() >= 12 and (~up).sum() >= 12:
                            mt, _ = cv2.estimateAffinePartial2D(
                                sp[up], dp[up], method=cv2.RANSAC, ransacReprojThreshold=3.0)
                            mb, _ = cv2.estimateAffinePartial2D(
                                sp[~up], dp[~up], method=cv2.RANSAC, ransacReprojThreshold=3.0)
                            if mt is not None and mb is not None:
                                diag["split"].append(abs(np.degrees(
                                    np.arctan2(mt[1, 0], mt[0, 0])
                                    - np.arctan2(mb[1, 0], mb[0, 0]))))
                    m[0, 2] /= down  # translation was measured downscaled
                    m[1, 2] /= down
                    step, failed = np.vstack([m, [0.0, 0.0, 1.0]]), False

        _, scale, tx, ty = decompose(step)
        # A jump this large is a shot change, not camera motion. Trying to
        # stabilize across it would fling the frame off the canvas.
        if failed or np.hypot(tx, ty) > 0.5 * w or not 0.5 < scale < 2.0:
            if failed:
                lost += 1
            else:
                cuts.append(len(steps))
            step = np.eye(3)

        steps.append(step)
        prev_gray = gray

    cap.release()
    if lost:
        print(f"  no usable motion on {lost} frame(s)", file=sys.stderr)
    if cuts:
        print(f"  {len(cuts)} shot change(s) at frame(s) {cuts[:8]}", file=sys.stderr)
    return steps, cuts, (w, h), fps, diag


def classify(diag):
    """Read the settings off the tracking diagnostics.

    Two measurements decide them. Whether the halves of the frame agree on
    rotation, together with how much of the scene the model explains, says
    whether rotation can be trusted at all. How far the picture travels per
    frame sets the smoothing window, through a continuous mapping rather than
    bands, because the two implementations measure travel slightly differently
    and a threshold would make a small difference flip the result.
    """
    inlier = float(np.median(diag["inlier"])) if diag["inlier"] else 0.0
    split = float(np.median(diag["split"])) if diag["split"] else 99.0
    travel = float(np.median(diag["trans"])) if diag["trans"] else 0.0

    trust_rotation = inlier >= 0.5 and split <= 0.5
    smooth = float(np.clip(round(0.6 / max(travel, 1e-4)), 5, 25))

    return {
        "smooth": smooth,
        "trust_rotation": trust_rotation,
        "evidence": (f"inlier ratio {inlier:.2f}, frame halves disagree on rotation by "
                     f"{split:.2f} deg, picture travels {travel * 100:.1f}% of its width "
                     f"per frame"),
    }


def segment_bounds(cuts, n):
    """For each frame, the first and last frame of the shot it belongs to."""
    edges = [0] + list(cuts) + [n]
    lo = np.empty(n, int)
    hi = np.empty(n, int)
    for a, b in zip(edges, edges[1:]):
        lo[a:b], hi[a:b] = a, b - 1
    return lo, hi


def local_average_pose(cumulative, i, j0, j1, sigma):
    """Weighted mean of the poses of frames j0..j1, measured relative to i."""
    inv_i = np.linalg.inv(cumulative[i])
    weights, sin_a, cos_a, log_s, txs, tys = [], [], [], [], [], []
    for j in range(j0, j1 + 1):
        angle, scale, tx, ty = decompose(cumulative[j] @ inv_i)
        wgt = np.exp(-0.5 * ((j - i) / sigma) ** 2)
        weights.append(wgt)
        sin_a.append(wgt * np.sin(angle))
        cos_a.append(wgt * np.cos(angle))
        log_s.append(wgt * np.log(max(scale, 1e-6)))
        txs.append(wgt * tx)
        tys.append(wgt * ty)
    total = sum(weights)
    return (
        np.arctan2(sum(sin_a), sum(cos_a)),
        np.exp(sum(log_s) / total),
        sum(txs) / total,
        sum(tys) / total,
    )


def horizon_angle(frame, sample_width=320):
    """Angle of the sky-ground boundary in this frame, or None if unclear.

    Sky is picked out by its blue-minus-red bias, which separates it cleanly
    from the yellow-green ground of aerial footage.
    """
    scale = sample_width / frame.shape[1]
    small = cv2.GaussianBlur(
        cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA), (5, 5), 0
    )
    score = small[..., 0].astype(np.int16) - small[..., 2].astype(np.int16)
    if score.max() - score.min() < 20:
        return None  # flat colour: no sky/ground split to find

    h, w = score.shape
    norm = cv2.normalize(score, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    thr, _ = cv2.threshold(norm, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = norm >= thr
    if not 0.03 < mask.mean() < 0.5:
        return None

    grey = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    if grey[mask].mean() - grey[~mask].mean() < 20:
        return None  # sky is brighter than ground; a dark region is not sky
    if np.nonzero(mask)[0].mean() > 0.45 * score.shape[0]:
        return None  # and it sits in the upper part of the frame

    # Walk down each column to the end of the unbroken run of sky.
    boundary = np.argmin(mask, axis=0).astype(float)
    boundary[mask.all(axis=0)] = h
    valid = (boundary > 1) & (boundary < h - 1)
    if valid.sum() < 0.4 * w:
        return None

    x = np.nonzero(valid)[0].astype(float)
    y = boundary[valid]
    for _ in range(3):  # refit without the outliers each time
        slope, intercept = np.polyfit(x, y, 1)
        residual = y - (slope * x + intercept)
        spread = np.median(np.abs(residual - np.median(residual))) + 1e-6
        keep = np.abs(residual - np.median(residual)) < 3 * spread
        if keep.sum() < 0.3 * len(x):
            break
        x, y = x[keep], y[keep]

    slope, _ = np.polyfit(x, y, 1)
    if abs(slope) > 1.0:
        return None  # steeper than 45 degrees is not a horizon
    return np.arctan(slope)


def estimate_level(path, roll, fps=30.0):
    """Find the true horizon by pooling every frame that shows sky.

    Each frame's measured horizon is rotated back into the orientation of
    frame 0 by subtracting that frame's roll, so measurements taken seconds
    apart, at completely different camera attitudes, can be averaged together.
    Line angles wrap at 180 degrees, so the average is taken on doubled angles.
    """
    cap = cv2.VideoCapture(path)
    measurements, seen = [], []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i < len(roll):
            angle = horizon_angle(frame)
            if angle is not None:
                measurements.append(angle - roll[i])
                seen.append(i)
        i += 1
    cap.release()

    if len(measurements) < 10:
        return None

    # Consecutive frames of one view are one observation, not many. A real
    # horizon has to be seen repeatedly, at camera attitudes far enough apart
    # that a stray road or river cannot masquerade as one.
    bursts = []
    for i in seen:
        if bursts and i - bursts[-1][-1] <= fps:
            bursts[-1].append(i)
        else:
            bursts.append([i])
    span = (seen[-1] - seen[0]) / max(len(roll), 1)
    if len(bursts) < 3 or span < 0.2:
        print(f"  horizon seen in only {len(bursts)} burst(s) spanning {span * 100:.0f}% "
              f"of the clip: not enough to trust", file=sys.stderr)
        return None

    doubled = np.exp(2j * np.array(measurements))
    world = np.angle(doubled.mean()) / 2.0
    agreement = np.abs(doubled.mean())  # 1.0 = every frame agrees
    residual = np.angle(np.exp(2j * (np.array(measurements) - world))) / 2.0
    return -world, len(measurements), np.degrees(np.std(residual)), agreement


def static_tail(steps, fps, min_seconds=0.4):
    """Index where the trailing run of motionless frames begins, or None.

    An end card is static, so stabilizing it only tilts it.
    """
    start = len(steps)
    for i in range(len(steps) - 1, 0, -1):
        angle, scale, tx, ty = decompose(steps[i])
        if np.hypot(tx, ty) > 1.5 or abs(angle) > np.radians(0.3) or abs(scale - 1) > 0.005:
            break
        start = i
    if len(steps) - start < min_seconds * fps:
        return None
    return start


def minimum_jerk(u):
    """Quintic ease with zero velocity and zero acceleration at both ends.

    Blending with this keeps the incoming motion's velocity at the start, so
    the picture drifts to rest instead of stopping dead (Flash & Hogan, 1985).
    """
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def warped_bounds(transforms, w, h):
    """Bounding box of the frame corners across every warp, in canvas pixels."""
    corners = np.array([[0.0, 0.0, 1.0], [w, 0.0, 1.0], [w, h, 1.0], [0.0, h, 1.0]]).T
    pts = np.concatenate([(t @ corners)[:2].T for t in transforms])
    return pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--zoom", type=float, default=0.6,
                    help="frame size inside the canvas; <1 zooms out and leaves black corners")
    ap.add_argument("--fit", action="store_true",
                    help="shrink the frame automatically so no frame is ever clipped")
    ap.add_argument("--expand", action="store_true",
                    help="grow the canvas to the bounding box instead, keeping native scale")
    ap.add_argument("--max-canvas", type=int, default=3840,
                    help="longest side an --expand canvas may reach before it scales down")
    ap.add_argument("--lock", type=float, default=None,
                    help="rotation lock: 0 only smooths roll, 1 holds frame-0 orientation. "
                         "1 needs a horizon reference and a scene the tracker can measure "
                         "rotation in reliably; see --roll-offset")
    ap.add_argument("--roll-offset", type=float, default=0.0,
                    help="degrees of fixed roll to add, to level a tilted first frame")
    ap.add_argument("--level-average", action="store_true",
                    help="level the clip's median attitude instead (no horizon needed)")
    ap.add_argument("--auto-level", action="store_true",
                    help="find the real horizon by pooling the sky-ground boundary over all frames")
    ap.add_argument("--straighten-end", action="store_true",
                    help="leave a static end card unrotated and centred")
    ap.add_argument("--end-ramp", type=float, default=0.6,
                    help="seconds over which to ease into a straightened end card")
    ap.add_argument("--smooth", type=float, default=None,
                    help="Gaussian sigma in frames for the local path average. Larger means "
                         "steadier but a bigger canvas and larger corrections; 20-30 suits "
                         "aerial footage, 5-10 suits handheld. Measured from the "
                         "footage when not given")
    ap.add_argument("--max-shift", type=float, default=0.35,
                    help="cap on translation correction, as a fraction of frame size")
    ap.add_argument("--track-width", type=int, default=960,
                    help="downscale width used for motion estimation")
    ap.add_argument("--mask-top", type=float, default=0.14,
                    help="fraction of the top of frame ignored when picking features")
    ap.add_argument("--mask-bottom", type=float, default=0.12,
                    help="fraction of the bottom of frame ignored when picking features")
    ap.add_argument("--crf", type=int, default=18)
    args = ap.parse_args()

    print("pass 1: measuring camera path", file=sys.stderr)
    steps, cuts, (w, h), fps, diag = estimate_steps(
        args.input, args.track_width, args.mask_top, args.mask_bottom
    )
    profile = classify(diag)
    smooth = args.smooth if args.smooth is not None else profile["smooth"]
    lock = args.lock if args.lock is not None else 0.0
    n = len(steps)
    print(f"  {n} frames, {w}x{h} @ {fps:.3f} fps", file=sys.stderr)
    print(f"  measured: {profile['evidence']}", file=sys.stderr)
    chose = [] if args.smooth is not None else ["--smooth %g" % smooth]
    print(f"  using --smooth {smooth:g} --lock {lock:g}"
          + (f"  ({', '.join(chose)} measured from the footage)" if chose else ""),
          file=sys.stderr)
    if lock > 0 and not profile["trust_rotation"]:
        print("  note: the halves of the frame disagree about rotation, so rotation cannot be\n"
              "  measured reliably here and the figure --lock holds will be mostly drift.\n"
              "  Parallax and large moving subjects do this; --lock 0 is immune to it.",
              file=sys.stderr)

    # Work around the frame centre, not the origin: under large rolls the
    # origin's path says nothing useful about where the picture actually went.
    cx, cy = w / 2.0, h / 2.0
    to_centre, from_centre = translation(-cx, -cy), translation(cx, cy)

    cumulative = [np.eye(3)]
    for step in steps[1:]:
        cumulative.append((to_centre @ step @ from_centre) @ cumulative[-1])

    roll = np.unwrap(np.array([decompose(m)[0] for m in cumulative]))
    print(f"  measured roll: {np.degrees(roll).min():.1f}..{np.degrees(roll).max():.1f} deg "
          f"(peak-to-peak {np.degrees(roll.max() - roll.min()):.1f})", file=sys.stderr)

    offset_deg = args.roll_offset
    if args.level_average:
        # No gravity reference exists, but a pilot holds the aircraft roughly
        # level most of the time, so the median attitude is a fair stand-in.
        median_roll = np.degrees(np.median(roll))
        offset_deg -= median_roll
        print(f"  levelling median attitude: roll offset {-median_roll:+.1f} deg",
              file=sys.stderr)
    if args.auto_level:
        print("  pooling sky-ground boundary across frames", file=sys.stderr)
        found = estimate_level(args.input, roll, fps)
        if found is None:
            print("  no usable horizon found; keeping --roll-offset", file=sys.stderr)
        else:
            level, used, spread, agreement = found
            offset_deg = np.degrees(level) + args.roll_offset
            print(f"  horizon from {used} frame(s): roll offset {np.degrees(level):+.1f} deg "
                  f"(scatter {spread:.1f} deg, agreement {agreement:.2f})", file=sys.stderr)

    # Whether a large net roll is real or tracker drift cannot be decided from the
    # numbers alone, so state it and let the operator judge against the footage.
    net = abs(np.degrees(roll[-1] - roll[0]))
    if lock > 0 and net > 25:
        print(f"  note: roll ends {net:.0f} deg from where it started, and --lock "
              f"{lock:g} will hold the frame at that first orientation throughout.\n"
              f"  That is right only if the camera really turned that far. Parallax and large\n"
              f"  moving subjects bias the tracker the same way; --lock 0 is immune to it.",
              file=sys.stderr)

    lo, hi = segment_bounds(cuts, n)
    radius = max(1, int(3 * smooth))
    offset = np.radians(offset_deg)
    cap_px = args.max_shift * min(w, h)

    poses = []
    clamped = 0
    for i in range(n):
        j0, j1 = max(lo[i], i - radius), min(hi[i], i + radius)
        mean_angle, mean_scale, tx, ty = local_average_pose(cumulative, i, j0, j1, smooth)

        # Rotation is the one component worth correcting absolutely: holding
        # frame-0's orientation costs only black corners, not framing.
        angle = (1.0 - lock) * mean_angle + lock * (offset - roll[i])

        shift = np.hypot(tx, ty)
        if shift > cap_px:
            tx, ty = tx * cap_px / shift, ty * cap_px / shift
            clamped += 1

        poses.append((angle, mean_scale, tx, ty))

    if clamped:
        print(f"  translation clamped on {clamped} frame(s)", file=sys.stderr)

    def build(params):
        return [from_centre @ similarity(*p) @ to_centre for p in params]

    stabilized = build(poses)

    # The end card is a static graphic: stabilizing it only tilts it. Ease the
    # correction away so the card lands square and centred, carrying the
    # incoming motion into the settle rather than cutting it dead.
    tail = static_tail(steps, fps) if args.straighten_end else None
    bounds = warped_bounds(stabilized[:tail] if tail else stabilized, w, h)
    if tail:
        x0, x1, y0, y1 = bounds
        target = (0.0, 1.0, (x0 + x1) / 2.0 - cx, (y0 + y1) / 2.0 - cy)
        ramp = max(1, int(args.end_ramp * fps))
        start = max(0, tail - ramp)
        for i in range(start, n):
            ease = minimum_jerk((i - start) / ramp)
            angle, scale, tx, ty = poses[i]
            poses[i] = (
                (1 - ease) * angle + ease * target[0],
                np.exp((1 - ease) * np.log(max(scale, 1e-6))),
                (1 - ease) * tx + ease * target[2],
                (1 - ease) * ty + ease * target[3],
            )
        stabilized = build(poses)
        print(f"  end card straightened from frame {tail}, "
              f"{ramp}-frame minimum-jerk settle", file=sys.stderr)

    out_w, out_h = w, h
    if args.expand:
        # Pass 1 already told us every corner every frame reaches, so size the
        # canvas to that box and leave the pixels at their original scale.
        x0, x1, y0, y1 = bounds
        box_w, box_h = x1 - x0, y1 - y0
        z = min(1.0, args.max_canvas / max(box_w, box_h))
        out_w = int(np.ceil(box_w * z)) // 2 * 2
        out_h = int(np.ceil(box_h * z)) // 2 * 2
        place = np.array([[z, 0.0, -z * x0], [0.0, z, -z * y0], [0.0, 0.0, 1.0]])
        print(f"  --expand canvas {out_w}x{out_h} "
              f"(bounding box {box_w:.0f}x{box_h:.0f}, scale {z:.3f})", file=sys.stderr)
    elif args.fit:
        x0, x1, y0, y1 = bounds
        z = min(w / (x1 - x0), h / (y1 - y0))
        place = np.array([[z, 0.0, cx - z * (x0 + x1) / 2.0],
                          [0.0, z, cy - z * (y0 + y1) / 2.0],
                          [0.0, 0.0, 1.0]])
        print(f"  --fit chose zoom {z:.3f} (frame drawn at {z * 100:.0f}% size)", file=sys.stderr)
    else:
        z = args.zoom
        place = np.array([[z, 0.0, (1 - z) * cx], [0.0, z, (1 - z) * cy], [0.0, 0.0, 1.0]])

    ffmpeg = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{out_w}x{out_h}", "-r", f"{fps}", "-i", "-",
        "-i", args.input,
        "-map", "0:v", "-map", "1:a?",
        "-c:v", "libx264", "-preset", "medium", "-crf", str(args.crf),
        "-pix_fmt", "yuv420p", "-c:a", "copy", "-shortest", args.output,
    ]
    print("pass 2: warping and encoding", file=sys.stderr)
    proc = subprocess.Popen(ffmpeg, stdin=subprocess.PIPE)

    cap = cv2.VideoCapture(args.input)
    for i in range(n):
        ok, frame = cap.read()
        if not ok:
            break
        warp = place @ stabilized[i]
        out = cv2.warpAffine(frame, warp[:2], (out_w, out_h),
                             flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        proc.stdin.write(out.tobytes())
        if i % 200 == 0:
            print(f"  frame {i}/{n}", file=sys.stderr)
    cap.release()
    proc.stdin.close()
    if proc.wait() != 0:
        sys.exit("ffmpeg failed")
    print(f"wrote {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
