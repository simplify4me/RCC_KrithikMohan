# T10 — On-Hardware FPS Validation Results (Pi 3B/3B+)

Early risk-mitigation gate: capture + T8 segmentation + T9 Hough curb stub +
no-op decision, run on real Pi hardware over >=100 frames. Must hit >=5 FPS
(target 5-10 FPS) before curb-side-detection, decision logic, serial
protocol, watchdog, and viz tool are built on top of this assumption.

Run with:

```
python3 scripts/perf_smoke_test.py --source-file tests/fixtures/videos/<clip>.mp4 --device-index 0
```

(Omit `--device-index` if no webcam is attached to the test rig; the file-source
run alone still satisfies the gate.)

## Status: NOT YET RUN

This file is a template. Paste the actual script output below once run on
real Pi hardware -- do not fill in placeholder numbers from a dev laptop.

## Hardware / software specs

- Pi model:
- OS / kernel:
- Python version:
- OpenCV version:

## Results

### File-source run

- Fixture clip used:
- Frames processed:
- Cold-start (1st frame) latency:
- Steady-state sustained FPS:
- Verdict (PASS if >=5 FPS):

### Live-camera run (if applicable)

- Camera model:
- Frames processed:
- Cold-start (1st frame) latency:
- Steady-state sustained FPS:
- Verdict (PASS if >=5 FPS):

## Overall verdict

PASS / FAIL:

## If FAIL: mitigation applied

(e.g. frame skip, smaller ROI-only Hough, less frequent histogram rebuild)
Describe what was changed and the re-measured FPS after the change.
