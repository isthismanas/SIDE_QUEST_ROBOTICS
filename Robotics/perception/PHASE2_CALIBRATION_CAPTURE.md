# Phase 2 Calibration Capture

This is the perception-only data collection step for camera-to-robot
calibration.

## Purpose

Use the confirmed top-down camera from Phase 1 and collect paired samples:

- camera-frame pose of `marker 0`
- matching robot-frame coordinates measured manually at the same physical point

These paired samples are the input for solving the camera-to-robot transform in
the next step.

## File

- `Robotics/perception/phase2_calibration_capture.py`

## What it does

- attaches to a single OAK-D device
- tracks one marker continuously
- keeps a rolling buffer of recent detections
- computes median and standard deviation over that buffer
- lets you save a stable sample together with manually entered robot
  coordinates
- writes one JSON record per captured calibration point

## Recommended device for current lab state

Phase 1 identified the active top-down marker-view camera as:

- device id: `169.254.1.223`
- label: `OAK_2`
- marker: `0`

## Run

From the repo root on the Pi:

```bash
source ~/regolith-robotics-env/bin/activate
python3 Robotics/perception/phase2_calibration_capture.py --device-id 169.254.1.223 --marker-id 0
```

## Commands

At the prompt:

- `status`
  Prints frame counts, detection ratio, buffered sample count, and the current
  median/std pose summary.

- `capture`
  Saves one calibration point. The tool will ask for:
  - sample label
  - robot pose in `x y z` or `x y z rx ry rz`
  - optional notes

- `quit`
  Stops the tool cleanly.

Pressing Enter on an empty prompt is treated like `capture`.

## Output

By default, records are written to:

```text
Robotics/perception/calibration_data/phase2_capture_<timestamp>.jsonl
```

Each line contains:

- sample index and label
- UTC timestamp
- device id and marker id
- camera pose summary from the rolling window
  - median pose
  - mean pose
  - std pose
  - sample count
  - capture time span
- manually entered robot pose
- optional notes

## Lab procedure

1. Move the robot TCP to a known calibration point.
2. Place `marker 0` at the corresponding physical reference point.
3. Wait until `status` shows a healthy buffer and stable standard deviation.
4. Run `capture`.
5. Enter the robot coordinates for that point.
6. Repeat across at least 6 well-spread points in the working area.

## Notes

- This tool does not command the robot.
- It does not talk to Unity.
- It is safe to use as a perception-only calibration logger while the live
  controller is not running.
