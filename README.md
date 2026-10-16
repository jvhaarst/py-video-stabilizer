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
and the video never leaves your machine. No dependencies, no build step — the
page is one HTML file that also works offline from `file://`.

Needs a current Chrome, Edge or Firefox for `canvas.captureStream` and
`MediaRecorder`.

## On the command line

```bash
uvx --from git+https://github.com/jvhaarst/py-video-stabilizer stabilize \
  input.mp4 output.mp4 --expand --lock 1.0 --straighten-end --roll-offset -4.7
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

The camera-path maths is identical — `tests/conformance.mjs` asserts the browser
port reproduces `stabilize.py` to within 1e-9, and it currently matches to 1e-12.

The **trackers** do not match exactly. On the test clip:

| | `stabilize.py` | `index.html` |
| --- | --- | --- |
| roll peak-to-peak | 81.6° | 85.2° |
| roll min / max | -23.1° / 58.6° | -24.2° / 61.0° |

About 4% apart, from different Lucas-Kanade window sizes and iteration counts.
Each implementation is self-consistent, so a roll offset measured in the browser
applies to the browser; carrying an offset between the two leaves a degree or
two on the table. Matching the trackers exactly is open work.

## Browser output

Encoded with WebCodecs `VideoEncoder` (VP9, falling back to VP8) and muxed into
WebM here, rather than through `MediaRecorder`. That matters for timing:
`MediaRecorder` timestamps frames as they arrive from the canvas, which is
wall-clock time, so a render slower than real time produces a file whose
duration is wrong. The W3C issue tracker has this as intended behaviour, and no
flag changes it.

Setting each frame's timestamp to `frameIndex * 1e6 / fps` microseconds instead
makes the output's timing the source's timing, however slowly it renders. A
verification run bears this out: 90 frames rendered at 13 fps wall clock came
back as 90 frames, `avg_frame_rate=30/1`, duration exactly 3.000000s.

VP9 is preferred over H.264 because the expanded canvas routinely exceeds the
macroblock limits that H.264 levels advertise.

Browsers without `VideoEncoder` fall back to `MediaRecorder`. Every frame is
still present and in order, but the duration follows render speed, and the page
says so and prints the `ffmpeg` command that fixes it.

## Development

```bash
uv venv && uv pip install -e .
python tests/make_fixture.py     # regenerate the conformance fixture
node tests/conformance.mjs       # assert the browser port still matches
```

The fixture is synthetic and deterministic, so the test needs no video file.

## License

MIT
