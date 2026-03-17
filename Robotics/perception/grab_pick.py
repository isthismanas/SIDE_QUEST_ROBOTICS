#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from uuid import uuid4

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MOTION_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "motion"))

if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)
if MOTION_DIR not in sys.path:
    sys.path.append(MOTION_DIR)

import actions
import robot_config as cfg
import vision_bridge
from dh_gripper import DHGripperPGE
from dobot_driver import DobotDriver
from logger import set_jsonl_context, write_jsonl_event
from phase2_calibration_capture import (
    DEFAULT_TOPDOWN_DEVICE_ID,
    DEFAULT_TOPDOWN_LABEL,
    CaptureSession,
    _format_pose_block,
    _resolve_device,
    _start_capture_thread,
)


def _timestamp_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Guarded single-block vision pick helper. Detect one marker, plan one pick, optionally execute only the pick.",
    )
    parser.add_argument("--marker-id", type=int, required=True, help="Marker id on the source block to pick.")
    parser.add_argument(
        "--device-id",
        default=DEFAULT_TOPDOWN_DEVICE_ID,
        help="DepthAI device id. Defaults to the validated top-down inspector camera.",
    )
    parser.add_argument(
        "--device-label",
        default=DEFAULT_TOPDOWN_LABEL,
        help="Free-form device label for logging.",
    )
    parser.add_argument("--fps", type=float, default=20.0, help="Perception capture FPS.")
    parser.add_argument("--buffer-size", type=int, default=60, help="Rolling detection buffer size.")
    parser.add_argument("--min-samples", type=int, default=20, help="Minimum buffered detections before planning.")
    parser.add_argument("--timeout-s", type=float, default=20.0, help="Maximum wait time for a stable detection window.")
    parser.add_argument(
        "--max-xy-std-mm",
        type=float,
        default=5.0,
        help="Maximum allowed median-window std on camera-frame X and Y before a pick is considered stable.",
    )
    parser.add_argument(
        "--workspace-x-min-mm",
        type=float,
        default=None,
        help="Optional override for safe pickup workspace minimum X in robot mm.",
    )
    parser.add_argument(
        "--workspace-x-max-mm",
        type=float,
        default=None,
        help="Optional override for safe pickup workspace maximum X in robot mm.",
    )
    parser.add_argument(
        "--workspace-y-min-mm",
        type=float,
        default=None,
        help="Optional override for safe pickup workspace minimum Y in robot mm.",
    )
    parser.add_argument(
        "--workspace-y-max-mm",
        type=float,
        default=None,
        help="Optional override for safe pickup workspace maximum Y in robot mm.",
    )
    parser.add_argument(
        "--stack-level",
        type=int,
        default=None,
        help="Optional explicit stack level for neutral routing. Defaults to the marker's position in PICK_SEQUENCE.",
    )
    parser.add_argument(
        "--participant-name",
        default="grab_pick",
        help="Optional participant/label written into JSONL logs.",
    )
    parser.add_argument(
        "--home-after",
        action="store_true",
        help="Return to safe home after the pick completes.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually arm the robot, connect the gripper, and execute the pick. Without this flag the tool only plans and logs.",
    )
    return parser.parse_args()


def _reverse_pick_marker_map() -> dict[int, str]:
    reverse: dict[int, str] = {}
    for target_id, marker_id in dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {})).items():
        reverse[int(marker_id)] = str(target_id)
    return reverse


def _infer_stack_level(target_id: str) -> int:
    sequence = [str(value).strip().upper() for value in getattr(cfg, "PICK_SEQUENCE", [])]
    target_key = str(target_id).strip().upper()
    if target_key in sequence:
        return int(sequence.index(target_key))
    return 0


def _workspace_bounds(args: argparse.Namespace) -> tuple[tuple[float, float], tuple[float, float]]:
    cfg_x = tuple(float(v) for v in getattr(cfg, "VISION_PICK_WORKSPACE_X_MM", (210.0, 430.0)))
    cfg_y = tuple(float(v) for v in getattr(cfg, "VISION_PICK_WORKSPACE_Y_MM", (-80.0, 60.0)))
    x_bounds = (
        float(args.workspace_x_min_mm) if args.workspace_x_min_mm is not None else cfg_x[0],
        float(args.workspace_x_max_mm) if args.workspace_x_max_mm is not None else cfg_x[1],
    )
    y_bounds = (
        float(args.workspace_y_min_mm) if args.workspace_y_min_mm is not None else cfg_y[0],
        float(args.workspace_y_max_mm) if args.workspace_y_max_mm is not None else cfg_y[1],
    )
    return x_bounds, y_bounds


def _write_grab_pick_event(event_name: str, **fields) -> None:
    payload = {
        "event": event_name,
        "module": "PERCEPTION",
        "tool": "grab_pick",
    }
    payload.update(fields)
    write_jsonl_event("grab_pick", payload)


def _wait_for_stable_summary(session: CaptureSession, marker_id: int, min_samples: int, max_xy_std_mm: float, timeout_s: float):
    deadline = time.time() + float(timeout_s)
    last_status_print = 0.0

    while time.time() < deadline:
        summary = session.capture_summary(marker_id, min_samples=min_samples)
        if summary is not None:
            std_pose = summary["std_pose"]
            std_x_mm = float(std_pose["x_m"]) * 1000.0
            std_y_mm = float(std_pose["y_m"]) * 1000.0
            if std_x_mm <= max_xy_std_mm and std_y_mm <= max_xy_std_mm:
                return summary

        now = time.time()
        if (now - last_status_print) >= 1.0:
            status = session.status()
            marker_status = status["markers"].get(marker_id, {})
            print(
                "[GRAB_PICK] "
                f"waiting marker={marker_id} frames={status['frames']} "
                f"buffer={marker_status.get('buffer_count')} "
                f"ratio={float(marker_status.get('detection_ratio', 0.0)):.3f} "
                f"last_detection_age_s={marker_status.get('last_detection_age_s')}"
            )
            last_status_print = now

        time.sleep(0.1)

    raise RuntimeError(
        f"Timed out waiting for stable marker {marker_id} detection window "
        f"(min_samples={min_samples}, max_xy_std_mm={max_xy_std_mm})."
    )


def _plan_pick_from_marker(
    marker_id: int,
    summary: dict[str, object],
    stack_level_override: int | None,
    workspace_x_mm: tuple[float, float],
    workspace_y_mm: tuple[float, float],
):
    reverse_map = _reverse_pick_marker_map()
    target_id = reverse_map.get(marker_id, f"marker_{marker_id}")
    template_target_id = reverse_map.get(
        marker_id,
        str(getattr(cfg, "VISION_PICK_TEMPLATE_TARGET", "P3")).strip().upper(),
    )
    template_pose = cfg.pick_target_pose(template_target_id)
    if stack_level_override is None:
        stack_level = _infer_stack_level(target_id) if marker_id in reverse_map else 0
    else:
        stack_level = int(stack_level_override)

    median_pose = summary["median_pose"]
    camera_x_m = float(median_pose["x_m"])
    camera_y_m = float(median_pose["y_m"])
    robot_xy, reason = vision_bridge.camera_xy_to_robot_xy_mm(camera_x_m, camera_y_m)
    if robot_xy is None:
        raise RuntimeError(f"camera_xy_to_robot_xy_mm failed: {reason}")

    robot_x = float(robot_xy[0])
    robot_y = float(robot_xy[1])
    if not (workspace_x_mm[0] <= robot_x <= workspace_x_mm[1]):
        raise RuntimeError(
            f"Computed pick X={robot_x:.3f}mm is outside workspace bounds "
            f"[{workspace_x_mm[0]:.3f}, {workspace_x_mm[1]:.3f}]"
        )
    if not (workspace_y_mm[0] <= robot_y <= workspace_y_mm[1]):
        raise RuntimeError(
            f"Computed pick Y={robot_y:.3f}mm is outside workspace bounds "
            f"[{workspace_y_mm[0]:.3f}, {workspace_y_mm[1]:.3f}]"
        )

    pick_pose = (
        robot_x,
        robot_y,
        float(template_pose[2]),
        float(template_pose[3]),
        float(template_pose[4]),
        float(template_pose[5]),
    )
    hover_pose = cfg.pick_target_hover_pose(template_target_id)
    planned_hover_pose = (
        robot_x,
        robot_y,
        float(hover_pose[2]),
        float(hover_pose[3]),
        float(hover_pose[4]),
        float(hover_pose[5]),
    )
    return {
        "marker_id": int(marker_id),
        "target_id": target_id,
        "template_target_id": template_target_id,
        "stack_level": int(stack_level),
        "template_pose": template_pose,
        "planned_pick_pose": pick_pose,
        "planned_hover_pose": planned_hover_pose,
        "camera_pose_summary": summary,
        "workspace_x_mm": workspace_x_mm,
        "workspace_y_mm": workspace_y_mm,
    }


def _execute_pick(plan: dict[str, object], home_after: bool) -> None:
    robot = DobotDriver(robot_ip=cfg.ROBOT_IP, dashboard_port=cfg.DASHBOARD_PORT, timeout_s=cfg.SOCKET_TIMEOUT_S)
    gripper = DHGripperPGE(
        port=cfg.GRIPPER_PORT,
        baudrate=cfg.GRIPPER_BAUDRATE,
        device_id=cfg.GRIPPER_SLAVE_ID,
    )
    handles = actions.SystemHandles(robot=robot, gripper=gripper)
    handles.combo_active = False

    try:
        actions.arm_robot_once(handles)
        actions.connect_gripper_once(handles)
        actions.initialize_stack_session(handles)
        actions.execute_pick_pose(
            handles,
            pick_pose=plan["planned_pick_pose"],
            stack_level=int(plan["stack_level"]),
            target_label=f"marker_{plan['marker_id']}->{plan['target_id']}",
        )
        if home_after:
            actions.do_home(handles)
    finally:
        try:
            gripper.disconnect()
        except Exception:
            pass
        try:
            robot.close()
        except Exception:
            pass


def main() -> int:
    args = parse_args()
    participant_name = str(args.participant_name).strip() or "grab_pick"
    session_id = f"grab-{int(time.time())}-{uuid4().hex[:8]}"
    run_id = uuid4().hex
    set_jsonl_context(
        participant_name=participant_name,
        session_id=session_id,
        run_id=run_id,
        leaderboard_mode="LAB",
    )

    marker_id = int(args.marker_id)
    device_info, resolved_device_id = _resolve_device(args.device_id)
    session = CaptureSession(buffer_size=int(args.buffer_size), marker_ids=[marker_id])
    stop_event = threading.Event()
    capture_thread = _start_capture_thread(
        device_info=device_info,
        fps=float(args.fps),
        session=session,
        stop_event=stop_event,
    )

    def _request_stop(_signum=None, _frame=None) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        time.sleep(1.0)
        if not capture_thread.is_alive():
            raise RuntimeError("Perception capture thread failed to start.")

        print(
            f"[GRAB_PICK] marker_id={marker_id} device_id={resolved_device_id} "
            f"device_label={args.device_label} execute={bool(args.execute)}"
        )
        _write_grab_pick_event(
            "grab_pick_start",
            marker_id=marker_id,
            device_id=resolved_device_id,
            device_label=args.device_label,
            execute=bool(args.execute),
            started_at_utc=_timestamp_utc(),
        )

        summary = _wait_for_stable_summary(
            session=session,
            marker_id=marker_id,
            min_samples=int(args.min_samples),
            max_xy_std_mm=float(args.max_xy_std_mm),
            timeout_s=float(args.timeout_s),
        )
        print("[GRAB_PICK] stable summary " + _format_pose_block(summary))

        workspace_x_mm, workspace_y_mm = _workspace_bounds(args)
        plan = _plan_pick_from_marker(
            marker_id=marker_id,
            summary=summary,
            stack_level_override=args.stack_level,
            workspace_x_mm=workspace_x_mm,
            workspace_y_mm=workspace_y_mm,
        )
        print(
            "[GRAB_PICK] plan "
            f"target={plan['target_id']} stack_level={plan['stack_level']} "
            f"pick_xy=({plan['planned_pick_pose'][0]:.3f},{plan['planned_pick_pose'][1]:.3f}) "
            f"workspace_x={workspace_x_mm} workspace_y={workspace_y_mm}"
        )
        _write_grab_pick_event(
            "grab_pick_plan",
            marker_id=marker_id,
            target_id=plan["target_id"],
            template_target_id=plan["template_target_id"],
            stack_level=int(plan["stack_level"]),
            planned_pick_pose=plan["planned_pick_pose"],
            planned_hover_pose=plan["planned_hover_pose"],
            template_pose=plan["template_pose"],
            workspace_x_mm=list(workspace_x_mm),
            workspace_y_mm=list(workspace_y_mm),
            camera_pose_summary=summary,
        )

        if not args.execute:
            print("[GRAB_PICK] dry-run only. Re-run with --execute to move the robot.")
            _write_grab_pick_event("grab_pick_dry_run", marker_id=marker_id, target_id=plan["target_id"])
            return 0

        _write_grab_pick_event(
            "grab_pick_execute_start",
            marker_id=marker_id,
            target_id=plan["target_id"],
            stack_level=int(plan["stack_level"]),
        )
        _execute_pick(plan, home_after=bool(args.home_after))
        _write_grab_pick_event(
            "grab_pick_execute_complete",
            marker_id=marker_id,
            target_id=plan["target_id"],
            stack_level=int(plan["stack_level"]),
            completed_at_utc=_timestamp_utc(),
        )
        print("[GRAB_PICK] pick complete.")
        return 0
    finally:
        stop_event.set()
        capture_thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
