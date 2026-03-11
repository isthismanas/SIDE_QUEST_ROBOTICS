# Phase 1 Camera Probe

This probe is isolated from the live controller. Use it to confirm which OAK-D
device is the top-down perception camera before doing any coordinate
calibration.

## What it does

- Enumerates available OAK-D devices by `MxId`
- Connects to each device independently
- Reads the `CAM_B` mono feed used by the current perception stack
- Runs the existing `ArucoTracker` on that feed
- Prints marker coordinates per device and a final summary

## Why this is Phase 1

Before solving any camera-to-robot transform, you need to know that the marker
detections are coming from the correct physical camera.

The current controller assumes perception attaches to the device labeled
`INSPECTOR`. This probe lets you verify that assumption outside the robot motion
runtime.

## Run

From the repo root on the Pi:

```bash
source ~/regolith-robotics-env/bin/activate
python3 Robotics/perception/phase1_camera_probe.py --duration-s 20
```

If you want to watch a single marker id:

```bash
python3 Robotics/perception/phase1_camera_probe.py --marker-id 0 --duration-s 20
```

## How to use it in the lab

1. Place the ArUco marker where only the intended top-down camera should see it clearly.
2. Run the probe.
3. Watch which device reports repeated detections with a stable detection ratio.
4. Confirm that device's `MxId` matches the role you expect for the top-down stream.

## Interpreting the output

- `marker=<id>` lines mean that device is currently seeing the marker.
- `heartbeat` lines show the number of processed frames and the detection ratio.
- The final summary ranks devices by detection ratio and prints the most likely
  active marker-view camera.

This is a perception-only diagnostic. It does not move the robot, talk to Unity,
or modify task-controller state.
