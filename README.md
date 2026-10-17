# py-video-stabilizer

Stabilizes shaky video by smoothing the camera path locally, locking the horizon,
and expanding the output canvas so corrections are never paid for with a crop.

Built for FPV drone footage, where the camera rolls hard and dives fast. Most
stabilizers smooth the *absolute* camera path and then crop to hide the
correction; on footage like this the absolute path scales 17x during a dive, so
that approach collapses. This one averages each frame's pose over its immediate
neighbours *relative to that frame*, which keeps corrections bounded under fast
but deliberate motion, and accepts black wedges instead of throwing away pixels.

Two ways to run it, one algorithm.

## In the browser

**<https://jvhaarst.github.io/py-video-stabilizer/>**

Drop in a video, mark the horizon on a few frames, get a stabilized file back.
Nothing is uploaded: decoding, tracking and encoding all happen in your browser,
and the video never leaves your machine. No build step, no CDN and no network at
runtime — `index.html` plus the vendored muxer in `vendor/`, which also works
offline from `file://`.

Tracking runs through OpenCV compiled to WebAssembly, 3.34 MB, about 0.85 MB
over the wire once compressed. It is the slow part: measuring a 204-frame clip
takes around 145 seconds in the browser against a few seconds on the command
line, because direct frame alignment runs per frame.

Decoding uses the `<video>` element and a canvas, which every browser has. For
MP4 with audio and correct timing it also needs WebCodecs (`VideoEncoder` and
`AudioEncoder`): Chrome and Edge 94+, Firefox 130+ on desktop, Safari 16.4+.
Without WebCodecs the page falls back to `MediaRecorder` and says so.

## On the command line

```bash
uvx --from git+https://github.com/jvhaarst/py-video-stabilizer stabilize \
  input.mp4 output.mp4 --expand
```

That is the right starting point for ordinary footage. Add the horizon lock only
when you have measured an offset and the camera really does roll:

```bash
... --expand --lock 1.0 --straighten-end --roll-offset -5.1
```

Faster than the browser, and the output is whatever your ffmpeg build encodes.
`stabilize --help` lists every option.

## What it does

- **Measures** frame-to-frame motion with Shi-Tomasi corners, pyramidal
  Lucas-Kanade with a backward-tracking consistency check, and a RANSAC
  similarity fit. Burned-in HUD graphics are masked out of feature selection.
- **Smooths locally.** Each frame is placed at the Gaussian-weighted average of
  its neighbours' poses, measured relative to itself. Poses are decomposed
  around the frame centre, not the origin — under large rolls the origin's path
  says nothing useful about where the picture went.
- **Locks rotation absolutely** (`--lock 1.0`), so the picture holds one
  orientation for the whole clip instead of rolling with the aircraft.
- **Expands the canvas** to the bounding box of every warped frame corner, so no
  frame is ever clipped.
- **Lands the end card square.** A static end card is detected and the
  correction eases off onto it with a quintic minimum-jerk blend, which keeps
  the incoming motion's velocity and arrives at rest.

## Settings chosen from the footage

Both implementations measure the clip during the tracking pass and set the
smoothing window from what they find. Anything you pass explicitly wins; the
automatic value only fills in what you left out, and the chosen figure is
printed so nothing is decided silently.

Two measurements do the work:

- **Can rotation be trusted?** The model is fitted separately to the top and
  bottom halves of the frame. On a far, planar scene the halves agree. Under
  parallax, or with a subject moving independently, they do not, and that is
  exactly when the rotation estimate stops meaning anything. Combined with the
  share of points the fit explains, this decides whether the rotation lock is
  usable at all.
- **How far does the picture travel per frame?** This sets the smoothing
  window, as `0.6 / travel` clamped to 5–25. A continuous mapping rather than
  bands, because the two implementations measure travel slightly differently
  and a threshold would let a small difference flip the result.

Measured on three clips:

| clip | inliers | halves disagree | travel/frame | smoothing |
| --- | --- | --- | --- | --- |
| FPV drone | 0.91 | 0.00° | 0.4% | 25 |
| handheld, walking | 0.23 | 1.06° | 6.0% | 10 |
| timelapse | 0.73 | 0.36° | 9.8% | 6 |

**Automatic selection never switches the rotation lock on.** It can only report
that rotation is untrustworthy, and warn if you asked for a lock anyway. Turning
the lock on needs a horizon offset, which is a judgement about the footage, not
something to guess.

## Where it does badly

The motion model is a similarity transform: translation, rotation and uniform
scale, four degrees of freedom. That fits aerial footage well, because the scene
is far away, effectively planar, and nothing in it moves on its own. Two common
situations break it.

**Parallax.** Walking with a camera makes near things move much faster than far
things. No single similarity transform fits both, so the least-squares fit
absorbs the mismatch as rotation and scale. The error is small per frame and
always in the same direction, so it accumulates.

**Large moving subjects.** RANSAC keeps the biggest consistent set of points. If
a subject fills enough of the frame and is well textured, that set can be the
subject rather than the scene.

Measured on a 6.8-second handheld clip of a dog walking ahead of the camera: the
tracker reported 112 degrees of accumulated roll where the footage barely
rotates at all. Taking features only from the distant upper part of the frame
roughly halved it, to 66 degrees, which is what identifies parallax and the
subject as the cause:

| features taken from | accumulated roll |
| --- | --- |
| the whole frame | 112° |
| the top 40% only | 66° |
| the bottom 40% only, holding the subject and the nearest ground | 131° |

Richer motion models do not help: on identical correspondences, affine reached
127 degrees and a homography 135, against 110 for the similarity transform.
Neither does spacing the features out, which made it worse. What helps is not
relying on the features at all, which is what the refinement pass does.

The command-line version can work around it. When the inlier ratio shows the
tracker is not fitting the scene, each estimate is polished by aligning the two
frames directly, which does not share the feature tracker's bias. On the
handheld clip that cut accumulated roll from 116 degrees to 37.

It is applied only when the measurements say it will help. Direct alignment is
not better in general — where the tracker fits well it is markedly worse, by
-11 degrees against 68 on the timelapse and 19 against 57 on the aerial clip —
so `--refine auto`, the default, enables it only where the tracker is
struggling. `--refine on` and `--refine off` force the choice. The browser
version does not have this yet.

What to do about it:

- **Leave `--lock` at 0**, which is the default. Local smoothing only ever
  compares a frame with its neighbours, so slow drift cancels instead of
  accumulating. `--lock 1.0` applies the accumulated figure as a correction, so
  on drifting footage it rotates the picture right over.
- **Keep `--smooth` small for handheld**, 5 to 10. Each step up widens the
  window, which enlarges the corrections, the canvas and the amount of clamping:
  on that clip `--smooth 40` clamped 129 of 204 frames.
- **Exclude a region** with `--mask-top` and `--mask-bottom` when a subject or
  very near ground dominates one part of the frame.

Fixing the drift itself rather than working around it needs a motion model that
can represent a 3D scene. That is open work.

## Finding the horizon

Footage without gyro or accelerometer data carries no gravity reference, so
"level" has to come from somewhere else.

A colour-based sky detector was built and **rejected**: blue-minus-red cannot
tell sky from water, and on the test clip it confidently locked onto a river.
Colour alone is not enough.

What works is marking the horizon by hand on a few frames. Each mark is rotated
into world coordinates by subtracting that frame's measured roll, and the marks
are pooled as doubled angles. The browser page picks which frames to show by
farthest-point sampling on **roll**, and sends you to the frame whose attitude
differs most from what you have marked.

Roll spread matters for *falsification*, not for precision. Extra marks do not
reduce the variance of the mean, which is sigma squared over n regardless. But
marks on a fixed ground feature such as a road still agree with each other at
similar attitudes — only widely separated rolls expose the error. So mark frames
whose roll differs widely, and judge precision by the standard error rather than
the raw scatter.

Snapping a line to the nearest strong edge is offered as a button and never
applied automatically: measured against a known-good offset it moved lines
between -7 and +5 degrees, because field boundaries are sharper than a hazy
horizon. That is the size of the angle being measured.

## Known differences between the two implementations

The camera-path maths is identical. `tests/conformance.mjs` asserts the browser
port reproduces `stabilize.py` within 1e-9 on angles and scales, 1e-7 on
translations and 1e-6 on canvas pixels; it currently matches to 1e-12 or better
on all of them.

The **trackers** now agree too, because both call OpenCV. Measured frame by
frame on identical frames:

| | hand-written JS | OpenCV.js |
| --- | --- | --- |
| rotation correlation, handheld | 0.206 | **0.988** |
| translation correlation, handheld | 0.28 / 0.13 | **0.989 / 0.999** |
| rotation correlation, aerial | 0.993 | **1.000** |

End to end on the handheld clip the page reports 36.9 degrees of accumulated
roll against the command line's 36.7.

Residual differences come from RANSAC's randomness, not from different
algorithms.

## Browser output

The page writes MP4: video encoded with WebCodecs `VideoEncoder` (VP9, falling
back to H.264) and audio re-encoded as AAC, muxed by
[mp4-muxer](https://github.com/Vanilagy/mp4-muxer), vendored into `vendor/`.

Timing is the reason for WebCodecs rather than `MediaRecorder`. `MediaRecorder`
timestamps frames as they arrive from the canvas, which is wall-clock time, so a
render slower than real time produces a file whose duration is wrong. The W3C
issue tracker has this as intended behaviour and no flag changes it. Stamping
each frame at `frameIndex * 1e6 / fps` microseconds instead makes the output's
timing the source's timing. A verification run bears this out: 90 frames
rendered at 14 fps wall clock came back as `avg_frame_rate=30/1`, video duration
exactly 3.000000s.

VP9 is preferred over H.264 because the expanded canvas routinely exceeds the
macroblock limits that H.264 levels advertise.

Two things to know about the audio. It is **re-encoded, not copied**: the
`<video>` element hands this page pixels and never the original encoded audio
packets, so passthrough is not reachable from this design. And the audio track
runs slightly longer than the video track, because AAC is framed in blocks of
1024 samples and the encoder adds its own priming: 59ms on the full test clip
and 72ms on a 3-second one. Players follow the container duration, so this is
cosmetic.

Browsers without `VideoEncoder` fall back to `MediaRecorder`, which produces
WebM with no audio and wall-clock timing. The page says so and prints the
`ffmpeg` command that fixes the timing.

Verified at full size: 804 frames at 2606x2210 rendered in 140s (5.7 fps) came
back as `avg_frame_rate=30/1`, 26.800s of VP9 plus an AAC stereo track, 36 MB.
At 5.7 fps against a 30 fps source, `MediaRecorder` would have produced a file
roughly six times too long.

One difference from the command-line version: the browser counts frames as
`container duration x frame rate`, and a container's duration is the longest of
its tracks. On the test clip the audio runs to 26.800s while the video track
ends at 26.733s, so the page renders 804 frames where `stabilize.py` renders
802, repeating the last image for the extra two. Reading the video track's own
duration would need a demuxer, which this page deliberately does not have.

### Vendored dependencies

`vendor/` holds `mp4-muxer`, a byte-identical copy of the build that
`package.json` pins, and `opencv.js`, which is **built from source** rather than
taken from a registry: `estimateAffinePartial2D` is missing from the upstream
JavaScript whitelist (opencv/opencv#20538, open since 2021) and the stock build
is 10.9 MB against 3.34 MB for one holding only the eleven functions this page
calls. `tools/build-opencv-js.sh` and `tools/opencv_js.config.py` reproduce it,
`vendor/opencv.js.provenance` records the commit, toolchain and hash, and the
vendor check verifies that hash. Neither needs a bundler, a CDN or a network. `tools/sync-vendor.mjs`
refreshes it and `--check` fails the build when the two drift, which is what
makes a Renovate version bump actually reach the page instead of sitting unused
in `node_modules`.

## Development

```bash
uv venv && uv pip install -e .
npm ci

.venv/bin/python tests/check_fixture.py      # stabilize.py still matches the fixture
node tests/conformance.mjs                   # the browser port still matches stabilize.py
node tools/sync-vendor.mjs --check           # vendor/ still matches package.json
```

These are the three checks CI runs. The fixture is synthetic and deterministic,
so none of them needs a video file. To change the algorithm deliberately,
regenerate the fixture with `.venv/bin/python tests/make_fixture.py` and commit
the result.

Serving the page over HTTP is not required — opening `index.html` directly
works — but `python3 -m http.server` is handy for testing.

## License

MIT — see [LICENSE](LICENSE).

`vendor/mp4-muxer.js` is mp4-muxer by Vanilagy, also MIT; its licence travels
with it in `vendor/mp4-muxer.LICENSE`.
