# Obstacle Detection + Curb Following v1 — Implementation Plan

## Overview

RC car on Raspberry Pi 3B/3B+ with USB webcam. Classical CV only (no ML,
no accelerator). Detect road vs non-road via adaptive color segmentation,
find curb line via Canny+Hough, figure out drivable corridor, spot
obstacles in it, turn that into a speed tier + steer value, ship it to an
Arduino-class MCU over serial. Includes a one-time camera calibration
step (homography) so distances are real-world cm, not pixels.

This repo does NOT contain the Arduino firmware — only the Pi-side
serial client and the wire protocol spec it talks.

Repo is currently empty and not a git repo. Plan starts by initializing
git and working in a dedicated worktree.

## Scope

### In scope (v1)
- USB webcam capture via OpenCV/V4L2, 320x240, live feed AND recorded
  video file through the same code path.
- Adaptive road-color segmentation from a trapezoid ROI, rebuilt every
  frame or every few frames.
- Curb-line detection via Canny + Hough, auto-detects which side
  (left/right) has the curb.
- Obstacle detection = non-road blobs inside the drivable corridor.
- Decision-level temporal smoothing (majority vote over last 2-3
  frames). NOT object tracking.
- One-time manual camera calibration -> homography matrix, stored in a
  config file, loaded at runtime.
- 3-tier speed control (full/slow/stop) off two tunable distance
  thresholds (defaults 30cm stop, 100cm slow) in config.
- Steering offset to hold target lateral distance from curb.
- Curb-confidence fallback: no curb seen in last N frames (N
  configurable) -> drive straight/lane-center, keep obstacle avoidance
  running independently. Never stop just because curb is gone.
- Versioned newline-delimited ASCII serial protocol, e.g.
  `S,<speed>,<steer>\n` with header/version.
- Pi-side watchdog: stale frame (>500ms), stalled processing, or failed
  serial write -> force a stop command.
- Debug/viz tool: overlay mask + curb line + decision onto frames, save
  to video file or show live window.
- Curated small set of test video clips (varied light/curb
  visibility/obstacles) as regression fixtures.
- Early, explicit on-hardware FPS validation task (see Architecture
  summary — this is deliberately NOT the last task).

### Out of scope (v1)
- Dusk/night/low-light. Daytime only.
- ML/neural nets, hardware accelerators (Coral/NPU/GPU), object
  classification, frame-to-frame object tracking.
- Indoor floors, curbs on both sides, curb-less roads.
- Arduino firmware implementation (only Pi-side client + protocol spec;
  Arduino-side failsafe is flagged as an open coordination item, not
  built here).
- Full serial-robustness hardening (partial reads/garbled bytes) beyond
  basic defensive parsing — flagged as a risk, not solved fully in v1.

## Architecture Summary

Modules (all under `src/rccar/`):

- `capture/` — source-agnostic frame iterator. One interface, two
  backends: `cv2.VideoCapture(device_index)` (live) and
  `cv2.VideoCapture(filepath)` (recorded file). All downstream code
  consumes only the iterator, never touches VideoCapture directly.
- `segmentation/` — adaptive road-color model (ROI sampling -> HSV
  histogram -> classify frame).
- `curb/` — Canny + Hough curb-line detection, auto side-detection,
  confidence tracking over last N frames.
- `obstacles/` — non-road blob detection inside drivable corridor.
- `calibration/` — homography calc from manual measurements, YAML/JSON
  config load/save, pixel->ground-plane conversion.
- `decision/` — distance thresholds -> speed tier, curb offset ->
  steer value, temporal smoothing (majority vote), curb-fallback logic.
- `serial_client/` — protocol encode/decode, serial port wrapper,
  write-with-timeout, failure detection.
- `watchdog/` — frame staleness timer, processing stall timer, forces
  stop command through serial_client.
- `viz/` — debug overlay renderer, video/window output.
- `tests/fixtures/` — curated test clips + calibration reference data.

**Ordering choice (risk mitigation):** the 5-10 FPS end-to-end target on
real Pi 3B hardware is genuinely uncertain (weak quad-core ARM, no
accelerator, Hough transform is not cheap). Rather than build the whole
pipeline and find out at the end it's too slow, task T10 (right after
scaffolding + capture abstraction + a bare-bones segmentation/Hough
stub) runs an early on-hardware perf smoke test. If it fails, we know
before curb detection, decision logic, serial protocol, watchdog, and
viz tool are all built on top of a bad assumption.

## Task List

### T0 — repo init + worktree
Set up git and an isolated worktree for this feature.
- `git init` in `/Users/mdoraiswamy/work/rccar`
- initial empty commit if needed so worktree add has something to
  branch from
- `git worktree add ../worktree-obstacle-detection-v1 -b obstacle-detection/v1`
- all subsequent work happens inside
  `/Users/mdoraiswamy/work/rccar/../worktree-obstacle-detection-v1`
  (i.e. `rccar/../worktree-obstacle-detection-v1`)
Files: none (repo-level)
Depends on: —
Acceptance: `git status` clean, worktree dir exists, branch
`obstacle-detection/v1` checked out there.

### T1 — python project scaffold: venv + requirements.txt
Make a venv, pin deps.
Files: `requirements.txt` (opencv-python, numpy, pyserial, pyyaml,
pytest — pinned versions), `.gitignore` (venv/, __pycache__, *.pyc,
*.mp4 fixtures if large — note fixtures dir exception later)
Depends on: T0
Acceptance: `pip install -r requirements.txt` succeeds in a fresh venv
on dev machine.

### T2 — package layout skeleton
Make the module dirs from Architecture summary, each with `__init__.py`
and a one-line docstring, no logic yet.
Files: `src/rccar/__init__.py`, `src/rccar/capture/__init__.py`,
`src/rccar/segmentation/__init__.py`, `src/rccar/curb/__init__.py`,
`src/rccar/obstacles/__init__.py`, `src/rccar/calibration/__init__.py`,
`src/rccar/decision/__init__.py`, `src/rccar/serial_client/__init__.py`,
`src/rccar/watchdog/__init__.py`, `src/rccar/viz/__init__.py`
Depends on: T1
Acceptance: `python -c "import rccar"` works with `src/` on PYTHONPATH
(add `setup.cfg`/`pyproject.toml` minimal or `src` layout note).

### T3 — pytest scaffold
Placeholder test runner config + one dummy test so CI/local pytest run
proves green from day 1.
Files: `pyproject.toml` (pytest config, testpaths=tests), `tests/__init__.py`,
`tests/test_smoke.py` (asserts True)
Depends on: T2
Acceptance: `pytest` runs, 1 passed.

### T4 — frame source interface (capture abstraction)
Define `FrameSource` protocol/ABC with `read() -> frame|None` and
`is_live() -> bool`. Two impls: `LiveCameraSource(device_index)`,
`VideoFileSource(filepath)`. Both wrap `cv2.VideoCapture` under the hood,
same call surface.
Files: `src/rccar/capture/source.py`, `src/rccar/capture/live.py`,
`src/rccar/capture/file.py`
Depends on: T2
Test cases:
- happy: `VideoFileSource` on a short test clip yields N frames then
  None at EOF
- edge: `LiveCameraSource` with a bad device index raises/returns None
  cleanly, doesn't hang
- error: file source pointed at nonexistent path raises clear error at
  construction, not deep in a loop
Acceptance: unit test constructs both source types against a fixture
file and a mocked/no-op device, confirms same interface shape.

### T5 — code-path parity test (file vs live)
Prove the exact same processing function runs unchanged against both
source types. Use a fake/mock live source (e.g. wraps the same test
video file but through the "live" code path with device index faked
via monkeypatch) vs the real file source, run one shared
`process_frame()` stub over both, assert identical output sequence.
Files: `tests/capture/test_parity.py`
Depends on: T4
Test cases:
- happy: identical frame count and identical stub-processing output
  for both source types over the same underlying video
- edge: source exhausted mid-stream (early EOF) handled the same way
  by both
Acceptance: test passes, documents parity guarantee — this is the
"first-class architectural requirement" proof point.

### T6 — test fixture clips (curated set)
Record/gather 4-6 short clips (10-20s each): well-lit clean road w/
clear curb, curb on left, curb on right, obstacle present in corridor,
low-contrast pavement, no-curb-visible segment. Store under
`tests/fixtures/videos/`.
Files: `tests/fixtures/videos/*.mp4` (or .avi), 
`tests/fixtures/videos/MANIFEST.md` (one line per clip: what it's for)
Depends on: T0
Test cases: n/a (data task)
Acceptance: files present, each <10s-20s, MANIFEST describes what
scenario each covers and which later tasks consume it.

### T7 — ROI trapezoid + road-color sampler (segmentation core)
Define the fixed trapezoid ROI geometry (near-field, ahead-of-car) as
config-driven pixel coords for 320x240. Sample pixels inside it, build
an HSV histogram model.
Files: `src/rccar/segmentation/roi.py`, `src/rccar/segmentation/model.py`,
`config/roi.yaml` (trapezoid corner coords)
Depends on: T2
Test cases:
- happy: known synthetic frame (solid color road) produces a tight
  histogram peak
- edge: ROI partly out of frame bounds (e.g. car nose visible) clipped
  safely, no crash
Acceptance: unit test on a synthetic frame confirms histogram model
built and non-empty.

### T8 — adaptive road/non-road classifier
Use the histogram model from T7 to classify every pixel (or block) of
a full frame as road/non-road. Rebuild model every K frames (config
value).
Files: `src/rccar/segmentation/classify.py`
Depends on: T7
Test cases:
- happy: synthetic 2-tone frame (road-color rectangle + obstacle-color
  rectangle) segments correctly
- edge: frame that's 100% "road color" (no obstacle) still classifies
  without div-by-zero/empty-mask errors
Acceptance: unit test asserts correct mask on synthetic 2-tone frame,
IoU > 0.9 vs ground truth mask.

### T9 — bare-bones Hough curb stub (perf-test fodder only)
Minimal Canny+Hough call on the ROI, no side-detection logic yet — just
enough real CV workload to make the perf test in T10 representative.
Files: `src/rccar/curb/hough_stub.py`
Depends on: T4, T8
Acceptance: runs on a sample frame without crashing, returns 0+ line
segments.

### T10 — ON-HARDWARE FPS VALIDATION (early, risk-mitigation gate)
Run capture(320x240) + T8 segmentation + T9 Hough stub + a no-op
decision step, in a loop, ON ACTUAL PI 3B/3B+ HARDWARE (not dev
laptop, not simulated). Measure sustained FPS over >=100 frames from a
fixture video (T6) and, if a webcam is available on the test rig, from
live camera too.
Files: `scripts/perf_smoke_test.py`, `docs/perf_results_pi3b.md`
(record raw numbers + Pi model/OS/Python version used)
Depends on: T6, T9
Test cases:
- happy: sustained >=5 FPS over a 100-frame run on Pi hardware
- edge: cold-start first-frame latency (histogram rebuild) measured
  separately from steady-state, doesn't blow the budget
- failure: if <5 FPS, this task's acceptance FAILS and must be flagged
  back — do not proceed to build curb-side-detection/decision/serial
  layers on an unvalidated perf assumption until addressed (e.g. via
  frame skip, lower-res ROI-only Hough, block-based segmentation)
Acceptance: measured FPS >=5 (target 5-10) logged in
`docs/perf_results_pi3b.md` with hardware specs. This is a HARD gate —
note in the doc if it fails and what mitigation was applied before
continuing.

### T11 — curb-line side auto-detection
On top of T9's raw Hough lines, pick the best curb-candidate line and
decide left vs right automatically (e.g. based on which side of frame
center it falls, line angle/slope filtering to reject noise).
Files: `src/rccar/curb/detect.py`
Depends on: T10
Test cases:
- happy: fixture clip with curb-on-left correctly reports "left"
- happy: fixture clip with curb-on-right correctly reports "right"
- edge: fixture clip with no curb visible returns "none"/low confidence,
  not a wrong guess
Acceptance: unit test over T6 fixtures gets side right on the
clear-curb clips.

### T12 — curb confidence tracker + fallback state
Track curb-found/not-found over last N frames (N from config). Expose
"curb_available: bool" + which side, or "fallback: lane-center" state.
Files: `src/rccar/curb/confidence.py`, `config/curb.yaml` (N, min
confidence threshold)
Depends on: T11
Test cases:
- happy: curb visible every frame -> curb_available stays True
- edge: curb disappears for N+1 frames -> flips to fallback mode
- edge: curb reappears -> flips back within N frames
Acceptance: unit test drives a fake sequence of found/not-found flags
through the tracker, asserts correct state transitions at the N
boundary.

### T13 — calibration script (camera height/tilt -> homography)
Interactive/manual CLI script: operator inputs camera height, tilt
angle (or points at 4 known ground-plane reference points in a test
frame), script computes homography matrix via `cv2.getPerspectiveTransform`
or `findHomography`, writes to config file.
Files: `scripts/calibrate_camera.py`,
`config/homography.yaml.example`
Depends on: T2
Test cases:
- happy: given 4 known image points + 4 known real-world (cm) points,
  script computes homography matching expected transform on a 5th
  held-out point within tolerance
- edge: degenerate/collinear input points -> script errors clearly
  instead of producing garbage homography
Acceptance: unit test feeds synthetic point correspondences (known
ground-truth H), confirms recovered H reprojects held-out point within
2cm at 100cm range.

### T14 — homography sanity-check tool
Given the stored homography + a test frame with a known physical
measurement (e.g. a tape-measure mark at a known distance in frame),
compute pixel->cm and report error vs the known measurement. This is
the "calibration accuracy" acceptance task called out separately from
T13's math-only unit test — it's the real-world check.
Files: `scripts/check_calibration.py`, `tests/fixtures/calibration_ref.yaml`
(recorded known measurements from a physical setup)
Depends on: T13
Test cases:
- happy: known 50cm mark measured via homography reports within +/-5cm
- error: homography file missing/malformed -> clear error, not a
  silent wrong number
Acceptance: script run against `calibration_ref.yaml` reports measured
vs actual distance and pass/fail against a tolerance (e.g. +/-10%).

### T15 — pixel-to-ground-plane distance API
Wrap the homography into a clean function: image point (or obstacle
blob centroid) -> real-world (x, y) cm on the ground plane, plus
nearest-obstacle-distance helper.
Files: `src/rccar/calibration/homography.py`
Depends on: T13
Test cases:
- happy: point at image center maps to expected forward distance
  (matches T14 reference within tolerance)
- edge: point above horizon line (not on ground plane, e.g. sky) ->
  returns None/invalid instead of nonsense negative distance
Acceptance: unit test round-trips known correspondences from T13/T14
fixtures.

### T16 — obstacle blob detection in drivable corridor
Combine T8 road/non-road mask + T11 curb side to define "drivable
corridor" (between curb line and opposite frame edge, or lane-center
band in fallback mode). Find non-road blobs inside corridor, filter by
min blob size (noise rejection).
Files: `src/rccar/obstacles/detect.py`
Depends on: T8, T12, T15
Test cases:
- happy: synthetic frame with obstacle blob in corridor detected with
  correct centroid
- edge: non-road blob OUTSIDE corridor (off to the side, off-road) is
  ignored, not flagged as obstacle
- edge: tiny noise speckle below min-size threshold ignored
Acceptance: unit test on synthetic frames confirms correct
in-corridor-only detection.

### T17 — nearest-obstacle real-world distance
Feed T16 blob centroids through T15's homography API, get nearest
obstacle distance in cm.
Files: `src/rccar/obstacles/distance.py`
Depends on: T16
Test cases:
- happy: multiple obstacles, correctly picks nearest by real-world
  distance (not pixel distance, which can be misleading due to
  perspective)
- edge: zero obstacles -> returns None/inf sentinel cleanly
Acceptance: unit test with multiple synthetic blobs at known mapped
distances picks the right minimum.

### T18 — speed tier decision logic
Two tunable thresholds (stop<30cm, slow<100cm defaults) from config,
map nearest-obstacle distance -> speed tier enum (STOP/SLOW/FULL).
Thresholds must be config values, not magic numbers in code.
Files: `src/rccar/decision/speed.py`, `config/thresholds.yaml`
Depends on: T17
Test cases:
- happy: distance=200cm -> FULL; distance=50cm -> SLOW; distance=10cm
  -> STOP
- edge: distance exactly at threshold boundary (30cm, 100cm) — defined
  behavior (document which side wins)
- edge: no obstacle (None distance) -> FULL
Acceptance: unit test table-driven over threshold boundaries.

### T19 — steering offset from curb distance
Given curb line position + target lateral offset (config value, cm),
compute steer value to hold that distance. Uses fallback (T12) to go
straight when no curb.
Files: `src/rccar/decision/steer.py`, `config/steer.yaml` (target
offset cm, steer value range)
Depends on: T12, T15
Test cases:
- happy: car too close to curb -> steer away; too far -> steer toward
- edge: fallback/no-curb mode -> steer value = 0 (straight), doesn't
  chase stale curb data
Acceptance: unit test on synthetic curb-line positions confirms steer
sign/magnitude direction.

### T20 — temporal smoothing (decision-level majority vote)
Smooth speed tier (and optionally steer) over last 2-3 frames
(config-controlled window) via majority vote, to kill single-frame
jitter. Explicitly not object tracking — just a small ring buffer over
decision outputs.
Files: `src/rccar/decision/smoothing.py`
Depends on: T18, T19
Test cases:
- happy: single-frame flicker (STOP,FULL,FULL) smooths to FULL by
  majority
- edge: window not yet full at startup (first 1-2 frames) — defined
  behavior, doesn't crash
Acceptance: unit test drives a sequence through the smoother, checks
output against expected majority-vote result at each step.

### T21 — serial wire protocol spec
Write the protocol doc precisely: message format `S,<speed>,<steer>\n`,
version header line sent on connect, field ranges/encoding (speed
tier as int 0/1/2, steer as signed int range), framing/delimiter
rules, what a malformed/partial message looks like.
Files: `docs/serial_protocol.md`
Depends on: T2
Acceptance: doc reviewed for completeness against the "deliverable"
checklist in requirements (format, ranges, versioning, framing,
encoding all present).

### T22 — serial protocol encode/decode
Implement encode (decision -> wire bytes) per T21 spec, plus a decoder
for tests/debugging.
Files: `src/rccar/serial_client/protocol.py`
Depends on: T21
Test cases:
- happy: encode(FULL, steer=5) round-trips through decode to same
  values
- edge: steer value at min/max of allowed range encodes without
  overflow/truncation
- error: decode() on garbled/partial bytes raises a specific
  `ProtocolError`, doesn't silently return wrong values (flagged per
  risk: full robustness not required in v1, but must fail loudly not
  silently)
Acceptance: unit test round-trips several values incl. boundary cases,
confirms garbled input raises cleanly.

### T23 — serial client (port wrapper + write-with-timeout)
Wrap `pyserial` with open/write/close, write() has a timeout, reports
failure (raises/returns False) instead of hanging on write.
Files: `src/rccar/serial_client/client.py`, `config/serial.yaml` (port,
baud, timeout)
Depends on: T22
Test cases:
- happy: write succeeds against a mocked serial port
- edge: port busy/unavailable at open -> clear error, not a hang
- error: write times out (mocked slow port) -> raises/reports failure
  within timeout window, doesn't block forever
Acceptance: unit test mocks pyserial `Serial` object, confirms timeout
behavior and failure signaling.

### T24 — watchdog: stale frame / stall / serial failure -> forced stop
Wire up: frame timeout (>500ms since last frame), processing loop
stall detection, serial write failure -> all route to sending (or
ensuring) a STOP command via T23's client.
Files: `src/rccar/watchdog/watchdog.py`, `config/watchdog.yaml`
(frame_timeout_ms=500)
Depends on: T23, T20
Test cases:
- happy: normal frame cadence -> no watchdog trip, no forced stop
- edge: frame gap of 600ms (mocked clock) -> watchdog fires STOP
- error: serial write raises inside normal loop -> watchdog catches,
  still attempts STOP send (or logs last-known-good failure if that
  also fails), doesn't crash the whole process silently
Acceptance: unit test with a mocked clock/frame source and mocked
serial client confirms STOP fired on each of the 3 trigger conditions.

### T25 — main pipeline wiring (capture -> decision -> serial)
Glue T4 capture, T8/T11/T12 segmentation+curb, T16/T17 obstacles,
T18/T19/T20 decision, T22/T23 serial, T24 watchdog into one runnable
loop. CLI flags to select live vs file source.
Files: `src/rccar/main.py`
Depends on: T20, T24, T15
Test cases:
- happy: run end-to-end against a T6 fixture file, confirms it
  produces a sequence of valid protocol messages (no crash, no None
  where a value's expected)
- edge: run against fixture with no-curb segment, confirms fallback
  steering (0) kicks in without stopping the car
Acceptance: integration test runs `main.py`-equivalent function over a
fixture clip end-to-end, asserts well-formed output messages for every
frame.

### T26 — debug/viz overlay tool
Render segmentation mask (semi-transparent), curb line, and decision
text (speed tier + steer) onto each frame. Support: save to output
video file, or live `cv2.imshow` window.
Files: `src/rccar/viz/overlay.py`, `scripts/run_with_viz.py`
Depends on: T25
Test cases:
- happy: run against a T6 fixture, output video file is created,
  non-zero size, same frame count as input
- edge: run with `--no-display` in a headless/CI env doesn't try to
  open a GUI window and crash
Acceptance: manual visual check of output video on one fixture (curb
line + mask + decision text visibly overlaid and roughly correct) +
automated check that output file exists with expected frame count.

### T27 — end-to-end FPS re-check with FULL pipeline on Pi hardware
Now that everything's wired (not just the T10 stub), re-measure
sustained FPS with the real full pipeline (T25) on Pi 3B/3B+ hardware,
320x240, over a fixture clip.
Files: `docs/perf_results_pi3b_full_pipeline.md`
Depends on: T25
Test cases:
- happy: >=5 FPS sustained over >=100 frames with full pipeline
- failure: if below target, document what was cut/optimized (e.g.
  segmentation rebuild interval increased, Hough restricted tighter to
  ROI) and re-measure
Acceptance: measured FPS logged, >=5 FPS, ideally 5-10 FPS as scoped.

### T28 — code-path parity re-check on full pipeline
Re-run T5-style parity check but against the FULL wired pipeline
(T25), not just the capture stub — confirms live-vs-file parity holds
all the way through decision output.
Files: `tests/integration/test_full_pipeline_parity.py`
Depends on: T25
Test cases:
- happy: same fixture run through file source vs mocked-live source
  produces identical decision/output sequence
Acceptance: test passes, byte-identical (or tolerance-equal for any
float rounding) output sequences from both source types.

### T29 — commit all work in worktree
Stage and commit everything built T1-T28 with a clear commit message
on branch `obstacle-detection/v1`.
Files: n/a (git operation)
Depends on: T6, T10, T14, T27, T28 (i.e. all substantive tasks done)
Acceptance: `git log` shows commit(s), `git status` clean in worktree.

### T30 — cleanup: remove worktree
Switch back to main worktree dir, remove the feature worktree.
- `cd /Users/mdoraiswamy/work/rccar`
- `git worktree remove ../worktree-obstacle-detection-v1`
Files: none
Depends on: T29
Acceptance: `git worktree list` no longer shows the feature worktree;
branch `obstacle-detection/v1` still exists with the commits (worktree
removal doesn't delete the branch).

## Test Strategy

- Unit tests (pytest) for every module with pure-function-shaped logic:
  segmentation classify, curb side-detect, homography math, distance
  calc, speed/steer decision, smoothing, protocol encode/decode,
  watchdog trigger conditions. Synthetic frames/data preferred for
  determinism; T6 fixture clips used where real imagery matters (curb
  detection accuracy, calibration).
- Two dedicated parity tests (T5 early on capture stub, T28 late on
  full pipeline) proving live-camera code path and video-file code
  path produce identical results through the same functions.
- Two dedicated on-hardware performance tasks (T10 early/gating, T27
  late/full-pipeline confirm) — both MUST be run on real Pi 3B/3B+, not
  simulated on a dev laptop, since ARM perf characteristics differ.
- One dedicated calibration-accuracy task (T14) checking homography
  output against a physically-measured reference distance, separate
  from the math-only unit test in T13.
- Visual/manual check for the viz tool (T26) since overlay correctness
  isn't easily numerically assertable — combined with an automated
  file-exists/frame-count check.
- Integration test (T25's test) running the whole pipeline over a
  fixture clip end to end.

## Risks

- **Pi 3B compute headroom is genuinely uncertain.** Adaptive
  segmentation + Canny + Hough at 5-10fps on a weak quad-core ARM with
  no accelerator might just not fit. Mitigated by running T10 (perf
  gate) early, before curb/decision/serial/watchdog/viz are built on
  top of an unvalidated assumption, and T27 as a full-pipeline
  re-check. If it fails, fallback mitigations (frame skip, smaller
  ROI-only Hough, less frequent histogram rebuild) need to be applied
  and re-measured — this may ripple into task estimates for T11+.
- **Homography calibration accuracy is error-prone.** It depends on
  careful manual measurement of camera height/tilt or accurate
  physical reference points; a sloppy calibration silently produces
  wrong distance estimates that cascade into wrong stop/slow
  decisions. T14's physical sanity-check task exists specifically to
  catch this, but it's a one-time manual step that could be redone
  wrong on a different physical car mount.
- **USB webcam V4L2 quirks vary by camera model.** Resolution/FPS
  negotiation, auto-exposure/auto-white-balance behavior, and
  `cv2.VideoCapture` backend selection can behave differently across
  webcam models — code that works on the dev webcam may misbehave on
  the actual deployed one. Not fully solvable without the real
  hardware in hand; flagged as a known integration risk, not solved by
  a specific task.
- **Serial protocol robustness is intentionally limited in v1.** Full
  handling of partial reads / garbled bytes / desync recovery is out
  of scope for v1, but the protocol WILL see noisy real-world serial
  conditions eventually. T22 requires decode() to fail loudly
  (raise) rather than silently return wrong values, as a minimum bar,
  but real resync-on-garbage logic is not built here — flagged
  explicitly as a gap to revisit post-v1.
- **Arduino-side failsafe is out of scope and unbuilt.** If the Pi
  process dies outright (not just stalls), it cannot send a stop
  command at all. The Pi-side watchdog (T24) only helps for stalls/
  timeouts the Pi process is alive to detect. See Open Questions.

## Open Questions

- Who owns/builds the Arduino firmware, and when? It MUST implement
  its own independent timeout-based failsafe (auto-stop if no serial
  command received within some window, e.g. 1s) since the Pi process
  itself could crash and never send anything. This is a hard
  coordination requirement, not optional — flagging here since it's
  outside this repo's scope but safety-critical.
- Exact camera mount position/angle on the physical car chassis isn't
  specified — calibration (T13/T14) assumes a fixed, known mount;
  if the mount is adjustable/removable, calibration needs to be redone
  each time and that process isn't automated.
- Serial baud rate / physical port device path (e.g. `/dev/ttyUSB0` vs
  `/dev/ttyACM0`) not yet pinned down — depends on final Arduino-class
  MCU choice, left as a `config/serial.yaml` value to fill in at
  integration time.
- What happens on repeated/persistent low FPS after T27's mitigations
  (e.g. genuinely can't hit 5fps even after optimization) — is 3-4fps
  an acceptable fallback for v1, or is that a hard blocker requiring
  hardware change? Not resolved in this plan; assumed acceptable to
  revisit after T27's real numbers are in.

## Post-change Doc-Update Note

This repo currently has no README or AGENTS.md-equivalent (it's
greenfield). Once T29 lands actual code, add a `README.md` (setup,
running live vs file mode, calibration workflow) and an
`AGENTS.md`-equivalent (module map, conventions, how to run
tests/perf-checks) as a follow-up task — not included as a numbered
task in this plan since no such file exists yet to update, but flagged
here so it isn't forgotten once the code exists.
