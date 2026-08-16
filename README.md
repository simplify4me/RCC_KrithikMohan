# rccar — RC-Car Obstacle-Detection & Curb-Following Pipeline

`rccar` is a Python library + CLI that turns a single forward-facing camera feed
into real-time drive commands for a small RC-car chassis:

```
capture -> segmentation (road mask) -> curb detection -> obstacle detection
        -> speed/steer decision -> smoothing -> serial link -> MCU -> motors
```

It's designed to run on a Raspberry Pi (or any machine with a camera and a
serial link to a motor-controller MCU), but every stage is a plain,
dependency-injectable Python object, so you can also embed just the pieces
you need inside a larger app.

This README explains how to install it, calibrate a camera, run it as a
standalone app, and — the main use case — **how to integrate it into your
own application**.

---

## 1. Requirements

- Python >= 3.9
- A camera (USB webcam, Pi Camera via `cv2.VideoCapture`, or a video file for
  testing)
- (Optional, for driving real hardware) a serial-connected MCU speaking the
  wire protocol in [`docs/serial_protocol.md`](docs/serial_protocol.md)

Dependencies (see `requirements.txt`):

```
opencv-python==4.9.0.80
numpy==1.26.4
pyserial==3.5
pyyaml==6.0.1
pytest==8.1.1        # dev/test only
```

## 2. Installation

From the repo root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e .        # installs the `rccar` package (src/ layout) in editable mode
```

`pyproject.toml` declares the package as `rccar`, sourced from `src/`, so
after `pip install -e .` you can `import rccar` from anywhere in your app,
not just from the repo root.

Run the test suite to confirm the install:

```bash
pytest
```

## 3. One-time camera calibration

The pipeline needs a **homography** matrix mapping camera pixels to
ground-plane centimeters (used to turn obstacle/curb pixel positions into
real-world distances). This is a one-time step per physical camera
mount/angle.

1. Point the camera at the ground and measure at least 4 known points in
   front of it (e.g. tape-measure marks, or corners of a rectangle taped to
   the floor).
2. Record each point's pixel location (open a saved frame in an image
   viewer) and its real-world ground-plane coordinates in cm, in a YAML file:

   ```yaml
   image_points:
     - [45, 210]
     - [275, 210]
     - [200, 120]
     - [120, 120]
   world_points:
     - [-30, 40]
     - [30, 40]
     - [15, 100]
     - [-15, 100]
   ```

3. Compute and save the homography:

   ```bash
   python3 scripts/calibrate_camera.py --points-file points.yaml \
       --output config/homography.yaml
   ```

4. (Recommended) sanity-check the result against independently-measured
   reference points:

   ```bash
   python3 scripts/check_calibration.py \
       --homography config/homography.yaml \
       --ref-file tests/fixtures/calibration_ref.yaml \
       --tolerance-pct 10.0
   ```

`config/homography.yaml.example` shows the expected file shape if you want
to hand-edit or generate it programmatically instead.

## 4. Configuration

All tunable parameters live in `config/*.yaml` and are loaded with sane
fallback defaults if a file is missing, so the library works out of the box
and can be tuned without touching code:

| File | Controls |
|---|---|
| `config/homography.yaml` | Camera pixel -> ground-plane cm mapping (from step 3) |
| `config/roi.yaml` | Fixed near-field trapezoid ROI for road segmentation |
| `config/curb.yaml` | Curb tracker's confidence threshold / fallback window |
| `config/thresholds.yaml` | `stop_distance_cm` / `slow_distance_cm` for speed tiers |
| `config/steer.yaml` | Target curb offset, proportional gain, steer clamp range |
| `config/smoothing.yaml` | Majority-vote smoothing window size |
| `config/watchdog.yaml` | Stale-frame timeout before forced STOP |
| `config/serial.yaml` | Serial port / baud / write timeout for the MCU link |

Copy `config/homography.yaml.example` → `config/homography.yaml` (after
running your own calibration) — it's the only file that ships without a
default in place, since it's inherently camera-specific.

## 5. Running it as a standalone app

The package ships a ready-made pipeline runner, useful as-is or as a
reference for your own integration.

**Live camera, driving a real MCU over serial:**

```bash
python3 -m rccar.main --device-index 0 \
    --serial-port /dev/ttyUSB0 --baud 9600 \
    --homography config/homography.yaml
```

**Recorded video file (no hardware needed) — good for development:**

```bash
python3 -m rccar.main --source-file path/to/video.mp4 \
    --serial-port /dev/ttyUSB0 --max-frames 500
```

**With a live debug overlay** (segmentation mask, curb line, decision text)
rendered on each frame, optionally written to a video file:

```bash
python3 scripts/run_with_viz.py --device-index 0 \
    --output debug_run.mp4 --no-display
```

Run `python3 -m rccar.main --help` / `python3 scripts/run_with_viz.py --help`
for the full flag list.

## 6. Integrating into your own app

There are three levels of integration, from "just reuse the whole loop" to
"cherry-pick one function." Pick the one that matches your app.

### 6.1 Reuse the whole pipeline (recommended default)

If your app just needs "give me a frame source and a serial client, drive
the car," call `run_pipeline` directly — it's exactly what `rccar.main`
does, minus CLI parsing:

```python
from rccar.calibration.homography_api import load_homography
from rccar.capture.live import LiveCameraSource
from rccar.serial_client.client import SerialClient
from rccar.watchdog.watchdog import Watchdog
from rccar.main import run_pipeline

homography = load_homography("config/homography.yaml")
source = LiveCameraSource(device_index=0)
serial_client = SerialClient(port="/dev/ttyUSB0", baud=9600, timeout=1.0)
watchdog = Watchdog(serial_client)

try:
    results = run_pipeline(source, serial_client, homography, watchdog)
finally:
    source.release()
```

- Swap `LiveCameraSource` for `rccar.capture.file.VideoFileSource(path)` to
  run over a recorded video instead — both implement the same
  `FrameSource` interface (`read() -> Optional[np.ndarray]`,
  `is_live()`, `release()`), so your app can target the interface and
  accept either.
- Pass `max_frames=N` to `run_pipeline` to bound a run (useful for tests, or
  an app that processes a fixed clip rather than running forever).
- `run_pipeline` returns a list of per-frame result dicts (see 6.2) — use
  that for logging, a UI, or post-run analysis.

### 6.2 Drive your own loop, one frame at a time

If your app already owns its own capture loop (e.g. frames arrive from
somewhere other than `cv2.VideoCapture` — a ROS topic, a socket, a
different camera SDK) call `process_frame` per frame instead of
`run_pipeline`:

```python
import numpy as np
from rccar.calibration.homography_api import load_homography
from rccar.curb.confidence import CurbConfidenceTracker
from rccar.decision.smoothing import MajorityVoteSmoother
from rccar.decision.speed import load_thresholds
from rccar.segmentation.classify import AdaptiveClassifier
from rccar.main import PipelineState, process_frame

stop_cm, slow_cm = load_thresholds()
state = PipelineState(
    classifier=AdaptiveClassifier(),
    curb_tracker=CurbConfidenceTracker(),
    homography=load_homography("config/homography.yaml"),
    speed_smoother=MajorityVoteSmoother(),
    steer_smoother=MajorityVoteSmoother(),
    stop_distance_cm=stop_cm,
    slow_distance_cm=slow_cm,
)

def on_new_frame(frame: np.ndarray):
    result = process_frame(frame, state)
    # result = {
    #     "speed": SpeedTier.STOP | SLOW | FULL,
    #     "steer": int,               # -100..100, negative=left
    #     "curb_state": "tracking" | "fallback",
    #     "obstacle_distance_cm": float | None,
    #     "curb_side": "left" | "right" | None,
    #     "current_offset_cm": float | None,
    # }
    send_to_your_motor_controller(result["speed"], result["steer"])
```

Key properties that make this safe to embed:

- **`PipelineState` is the only mutable state.** `process_frame` reads/writes
  nothing else (no globals), so you can run multiple independent pipelines
  (e.g. two cameras) side by side, each with its own `PipelineState`.
- **Deterministic given `(frame, state)`** — this is what makes the live vs.
  recorded-video code paths behave identically in tests, and it means you
  can unit-test your integration by feeding canned frames.
- You are responsible for actually sending `result["speed"]`/`result["steer"]`
  somewhere — `process_frame` does not touch serial/watchdog at all. Wire it
  into your own actuator/transport as shown above, or reuse
  `rccar.watchdog.watchdog.Watchdog.write_command(speed, steer)` if you also
  want the staleness/fail-safe behavior (see 6.4).

### 6.3 Cherry-pick individual stages

Every stage is independently usable if you only need part of the pipeline
(e.g. just obstacle distance for a collision-avoidance feature bolted onto
an unrelated app):

| Stage | Entry point |
|---|---|
| Road segmentation | `rccar.segmentation.classify.AdaptiveClassifier().process_frame(frame) -> mask` |
| Curb side/confidence | `rccar.curb.detect.detect_curb_side(frame) -> (side, confidence)` |
| Curb temporal tracking | `rccar.curb.confidence.CurbConfidenceTracker` |
| Obstacle corridor + detection | `rccar.obstacles.detect.define_corridor` / `detect_obstacles` |
| Pixel -> real-world distance | `rccar.obstacles.distance.nearest_obstacle_real_distance(obstacles, homography)` |
| Speed-tier decision | `rccar.decision.speed.decide_speed_tier(distance_cm, stop_distance_cm, slow_distance_cm)` |
| Steering decision | `rccar.decision.steer.compute_steer(curb_side, current_offset_cm)` |
| Temporal smoothing | `rccar.decision.smoothing.MajorityVoteSmoother` |
| Pixel <-> ground-plane math | `rccar.calibration.homography_api.image_point_to_ground(homography, (px, py))` |

Each module's docstrings/tests under `tests/` double as usage examples for
that stage in isolation.

### 6.4 Talking to the MCU (serial protocol)

If you're driving real hardware, either:

- send `result["speed"]`/`result["steer"]` yourself using
  `rccar.serial_client.protocol.encode_command(speed, steer) -> str` and
  `rccar.serial_client.client.SerialClient.write(line)`, or
- wrap the serial client in `rccar.watchdog.watchdog.Watchdog`, which adds:
  - **stale-frame protection** — forces `STOP` if `on_frame_received()`
    hasn't been called within `frame_timeout_ms` (`config/watchdog.yaml`)
  - **write-failure fail-safe** — if a serial write raises, the watchdog
    forces a `STOP` write instead of propagating a silent drive-command loss

The full wire format (message framing, valid ranges, malformed-input
handling) is specified in [`docs/serial_protocol.md`](docs/serial_protocol.md)
— read this before implementing MCU-side firmware or any alternative
transport (e.g. swapping serial for a socket), since the encode/decode
contract must match exactly:

```
V,<version>\n              sent once on connect
S,<speed>,<steer>\n        drive command, sent every decision tick

speed ::= 0 (STOP) | 1 (SLOW) | 2 (FULL)
steer ::= integer in [-100, 100], negative = left, positive = right
```

`decode_command`/`encode_command` (`rccar.serial_client.protocol`) raise
`ProtocolError` on any malformed input rather than silently clamping or
defaulting — mirror that behavior if you write your own MCU-side decoder.

### 6.5 Non-Python / non-Pi integration

If your "app" is on a different host or in a different language (e.g. a
mobile app, a web dashboard, or an MCU-side integration), you don't consume
this Python package directly — instead:

- Point your integration at the **serial protocol**
  ([`docs/serial_protocol.md`](docs/serial_protocol.md)), which is the
  actual boundary between this pipeline and any drive-motor consumer.
- Run this package as the Pi-side process (`python3 -m rccar.main ...`,
  section 5) and have your app read/observe state over whatever channel you
  add (e.g. tap `run_pipeline`'s returned result list, or extend
  `process_frame`'s call site to also publish `result` to MQTT/a socket/a
  log your app tails).

## 7. Testing your integration

Run the existing suite as a smoke test before wiring in your app:

```bash
pytest                      # full suite
pytest tests/integration     # end-to-end pipeline tests
pytest tests/test_main.py    # main() / run_pipeline() CLI-level tests
```

`tests/fixtures/` contains sample frames/videos and a calibration reference
file usable as canned input while developing your integration without a
live camera.

## 8. Repository layout reference

```
src/rccar/
  capture/        FrameSource interface + live camera / video file backends
  segmentation/    Road-mask classifier + ROI handling
  curb/            Curb-line detection (Hough) + temporal confidence tracking
  obstacles/       Corridor definition, obstacle detection, pixel->cm distance
  decision/        Speed-tier / steering decision + majority-vote smoothing
  calibration/     Homography computation + pixel<->ground-plane mapping
  serial_client/   Wire protocol encode/decode + pyserial wrapper
  watchdog/        Stale-frame / write-failure fail-safe wrapper
  viz/             Debug overlay rendering
  main.py          CLI entry point + run_pipeline()/process_frame()
config/            YAML configuration for every tunable stage
scripts/           calibrate_camera.py, check_calibration.py, run_with_viz.py
docs/              serial_protocol.md, performance results
tests/             Unit + integration tests, mirroring src/rccar/ layout
```
