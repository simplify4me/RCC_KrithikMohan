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

=== T10 on-hardware FPS validation ===
platform: Linux-6.18.39+rpt-rpi-v7-armv7l-with-glibc2.41
machine:  armv7l
python:   3.13.5
pi model: Raspberry Pi 3 Model B Plus Rev 1.3

--- file: /home/pi/recordings/outing_2026-08-11_112525.mp4 ---
frames processed:        100
cold-start (1st frame):  181.5 ms
steady-state duration:   14.33 s
sustained FPS:           6.91
verdict:                 PASS (gate: >= 5.0 FPS)

--- live: device 0 ---
frames processed:        100
cold-start (1st frame):  514.7 ms
steady-state duration:   14.82 s
sustained FPS:           6.68
verdict:                 PASS (gate: >= 5.0 FPS)

=== Summary ===
file: /home/pi/recordings/outing_2026-08-11_112525.mp4: 6.91 FPS (PASS)
live: device 0: 6.68 FPS (PASS)

Overall: PASS (target range 5.0-10.0 FPS, hard floor 5.0 FPS)

=== T10 on-hardware FPS validation ===
platform: Linux-6.18.39+rpt-rpi-v7-armv7l-with-glibc2.41
machine:  armv7l
python:   3.13.5
pi model: Raspberry Pi 3 Model B Plus Rev 1.3

--- live: device 0 ---
frames processed:        100
cold-start (1st frame):  528.8 ms
steady-state duration:   14.65 s
sustained FPS:           6.76
verdict:                 PASS (gate: >= 5.0 FPS)

=== Summary ===
live: device 0: 6.76 FPS (PASS)

Overall: PASS (target range 5.0-10.0 FPS, hard floor 5.0 FPS)

