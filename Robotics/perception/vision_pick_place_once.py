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
        description=(
            "Standalone guarded vision pick-and-place cycle. "
            "Select a live mapped pickup marker, pick that block, and place it "
            "onto an explicit tower level while preserving deterministic corridor logic."
        )
    )
    parser.add_argument(
        "--device-id",
        default=DEFAULT_TOPDOWN_DEVICE_ID,
        help="DepthAI device id for the marker-view camera.",
    )
    parser.add_argument(
        "--device-label",
        default=DEFAULT_TOPDOWN_LABEL,
        help="Free-form camera label for logging.",
    )
    parser.add_argument("--fps", type=float, default=20.0, help="Perception capture FPS.")
    parser.add_argument(
        "--use-depth-module",
        action="store_true",
        help="Enable the optional perception-side depth module for this workflow.",
    )
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
        "--remaining-targets",
        nargs="+",
        default=None,
        help="Optional subset of pickup targets to consider, e.g. P3 P4 P7. Defaults to cfg.PICK_SEQUENCE.",
    )
    parser.add_argument(
        "--place-level",
        type=int,
        default=0,
        help="0-indexed tower level to place onto. Use 0 for T1, 1 for T2, etc.",
    )
    parser.add_argument(
        "--max-count",
        type=int,
        default=None,
        help=(
            "Optional safety cap on how many detected blocks to stack. "
            "If omitted, the workflow keeps stacking detected mapped targets until none remain "
            "or tower capacity is reached."
        ),
    )
    parser.add_argument(
        "--max-detection-age-s",
        type=float,
        default=1.0,
        help="Maximum allowed age for the last marker detection before a candidate is considered stale.",
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
        "--participant-name",
        default="vision_pick_place_once",
        help="Optional participant/label written into JSONL logs.",
    )
    parser.add_argument(
        "--enable-pick-ml",
        action="store_true",
        help="Enable the optional non-depth pickup residual model for this guarded run only.",
    )
    parser.add_argument(
        "--pick-ml-model-json",
        default=None,
        help="Optional override path for the pickup residual model JSON used with --enable-pick-ml.",
    )
    parser.add_argument(
        "--home-after",
        action="store_true",
        help="Return to safe home after the cycle completes.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually execute the pick-and-place cycle. Without this flag the tool only plans.",
    )
    return parser.parse_args()


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


def _candidate_targets(args: argparse.Namespace) -> list[str]:
    if args.remaining_targets:
        return [str(target).strip().upper() for target in args.remaining_targets if str(target).strip()]
    return [str(target).strip().upper() for target in getattr(cfg, "PICK_SEQUENCE", [])]


def _candidate_markers(target_ids: list[str]) -> list[int]:
    marker_map = dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {}))
    marker_ids: list[int] = []
    for target_id in target_ids:
        marker_id = marker_map.get(target_id)
        if marker_id is not None:
            marker_ids.append(int(marker_id))
    return list(dict.fromkeys(marker_ids))


def _write_cycle_event(event_name: str, **fields) -> None:
    payload = {
        "event": event_name,
        "module": "PERCEPTION",
        "tool": "vision_pick_place_once",
    }
    payload.update(fields)
    write_jsonl_event("grab_pick", payload)


def _configure_pick_ml(args: argparse.Namespace) -> str:
    cfg.VISION_PICK_ML_ENABLED = bool(args.enable_pick_ml)
    if args.pick_ml_model_json:
        cfg.VISION_PICK_ML_MODEL_JSON = str(args.pick_ml_model_json)
    return str(getattr(cfg, "VISION_PICK_ML_MODEL_JSON", "")).strip()


def _projection_details_to_log(details: dict[str, object]) -> dict[str, object]:
    base_xy = details["base_xy_mm"]
    ml_xy = details["ml_corrected_xy_mm"]
    final_xy = details["final_xy_mm"]
    offsets = details["offset_xy_mm"]
    ml_delta = dict(details["ml_delta_xy_mm"])
    return {
        "base_xy_mm": {"x": round(float(base_xy[0]), 3), "y": round(float(base_xy[1]), 3)},
        "ml_corrected_xy_mm": {"x": round(float(ml_xy[0]), 3), "y": round(float(ml_xy[1]), 3)},
        "final_xy_mm": {"x": round(float(final_xy[0]), 3), "y": round(float(final_xy[1]), 3)},
        "offset_xy_mm": {"x": round(float(offsets[0]), 3), "y": round(float(offsets[1]), 3)},
        "ml_delta_xy_mm": {
            "x": round(float(ml_delta["x"]), 3),
            "y": round(float(ml_delta["y"]), 3),
            "norm": round(float(ml_delta["norm"]), 3),
        },
        "base_reason": details["base_reason"],
        "ml_reason": details["ml_reason"],
    }


def _pose_dict(pose: tuple[float, float, float, float, float, float]) -> dict[str, float]:
    return {
        "x": round(float(pose[0]), 3),
        "y": round(float(pose[1]), 3),
        "z": round(float(pose[2]), 3),
        "rx": round(float(pose[3]), 3),
        "ry": round(float(pose[4]), 3),
        "rz": round(float(pose[5]), 3),
    }


def _pickup_grid_diagnostics(robot_x: float, robot_y: float, target_id: str) -> dict[str, object]:
    source_pose = cfg.pick_target_pose(target_id)
    dx = float(robot_x - source_pose[0])
    dy = float(robot_y - source_pose[1])
    nearest_target_id = None
    nearest_pose = None
    nearest_norm = None
    for candidate_id, candidate_pose in getattr(cfg, "PICKUP_POINTS", {}).items():
        cdx = float(robot_x - candidate_pose[0])
        cdy = float(robot_y - candidate_pose[1])
        cnorm = (cdx * cdx + cdy * cdy) ** 0.5
        if nearest_norm is None or cnorm < nearest_norm:
            nearest_target_id = str(candidate_id)
            nearest_pose = candidate_pose
            nearest_norm = cnorm
    return {
        "source_target_id": str(target_id),
        "source_template_pose_mm_deg": _pose_dict(source_pose),
        "source_delta_mm": {
            "dx": round(dx, 3),
            "dy": round(dy, 3),
            "norm": round((dx * dx + dy * dy) ** 0.5, 3),
        },
        "nearest_pick_target_id": nearest_target_id,
        "nearest_pick_pose_mm_deg": _pose_dict(nearest_pose) if nearest_pose is not None else None,
        "nearest_pick_delta_mm": {
            "dx": round(float(robot_x - nearest_pose[0]), 3) if nearest_pose is not None else None,
            "dy": round(float(robot_y - nearest_pose[1]), 3) if nearest_pose is not None else None,
            "norm": round(float(nearest_norm), 3) if nearest_norm is not None else None,
        },
    }


def _select_stable_target(
    session: CaptureSession,
    target_ids: list[str],
    min_samples: int,
    max_xy_std_mm: float,
    timeout_s: float,
    max_detection_age_s: float,
) -> tuple[str, int, dict[str, object]]:
    marker_map = dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {}))
    deadline = time.time() + float(timeout_s)
    last_status_print = 0.0

    while time.time() < deadline:
        candidates: list[tuple[float, int, str, dict[str, object]]] = []
        status = session.status()
        for target_id in target_ids:
            marker_id = marker_map.get(target_id)
            if marker_id is None:
                continue
            summary = session.capture_summary(int(marker_id), min_samples=min_samples)
            if summary is None:
                continue
            std_pose = summary["std_pose"]
            std_x_mm = float(std_pose["x_m"]) * 1000.0
            std_y_mm = float(std_pose["y_m"]) * 1000.0
            if std_x_mm > max_xy_std_mm or std_y_mm > max_xy_std_mm:
                continue
            marker_status = status["markers"].get(int(marker_id), {})
            age_s = marker_status.get("last_detection_age_s")
            if age_s is None:
                continue
            if float(age_s) > float(max_detection_age_s):
                continue
            candidates.append((float(age_s), int(marker_id), str(target_id), summary))

        if candidates:
            candidates.sort(key=lambda item: (item[0], item[1], item[2]))
            age_s, marker_id, target_id, summary = candidates[0]
            print(
                f"[VISION_PICK_PLACE] selected target={target_id} marker={marker_id} "
                f"age_s={age_s:.3f} summary={_format_pose_block(summary)}"
            )
            return target_id, marker_id, summary

        now = time.time()
        if (now - last_status_print) >= 1.0:
            print(
                f"[VISION_PICK_PLACE] waiting for stable candidate among targets={target_ids} "
                f"frames={status['frames']}"
            )
            last_status_print = now
        time.sleep(0.1)

    raise RuntimeError(
        f"Timed out waiting for a stable pickup marker among targets={target_ids} "
        f"(min_samples={min_samples}, max_xy_std_mm={max_xy_std_mm})."
    )


def _plan_cycle(
    target_id: str,
    marker_id: int,
    summary: dict[str, object],
    place_level: int,
    workspace_x_mm: tuple[float, float],
    workspace_y_mm: tuple[float, float],
) -> dict[str, object]:
    template_pose = cfg.pick_target_pose(target_id)
    median_pose = summary["median_pose"]
    projection_details, reason = vision_bridge.camera_pose_to_pick_robot_xy_mm_details(
        float(median_pose["x_m"]),
        float(median_pose["y_m"]),
        camera_pose=median_pose,
    )
    if projection_details is None:
        raise RuntimeError(f"camera_xy_to_robot_xy_mm failed: {reason}")
    robot_xy = projection_details["final_xy_mm"]

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
    pick_hover_pose = (
        robot_x,
        robot_y,
        float(template_pose[2] + cfg.PICK_CLEARANCE_MM),
        float(template_pose[3]),
        float(template_pose[4]),
        float(template_pose[5]),
    )
    pickup_grid = _pickup_grid_diagnostics(robot_x, robot_y, target_id)
    tower_target_id = cfg.build_target_id_for_level(place_level)
    tower_place_pose = cfg.tower_place_pose(place_level)
    tower_hover_pose = cfg.tower_hover_pose(place_level)
    return {
        "target_id": target_id,
        "marker_id": int(marker_id),
        "stack_level": int(place_level),
        "tower_target_id": tower_target_id,
        "planned_pick_pose": pick_pose,
        "planned_pick_hover_pose": pick_hover_pose,
        "planned_place_pose": tower_place_pose,
        "planned_tower_hover_pose": tower_hover_pose,
        "camera_pose_summary": summary,
        "computed_robot_xy_mm": (robot_x, robot_y),
        "pickup_grid_diagnostics": pickup_grid,
        "pick_projection": _projection_details_to_log(projection_details),
        "workspace_x_mm": workspace_x_mm,
        "workspace_y_mm": workspace_y_mm,
    }


def _create_system_handles() -> actions.SystemHandles:
    robot = DobotDriver(robot_ip=cfg.ROBOT_IP, dashboard_port=cfg.DASHBOARD_PORT, timeout_s=cfg.SOCKET_TIMEOUT_S)
    gripper = DHGripperPGE(
        port=cfg.GRIPPER_PORT,
        baudrate=cfg.GRIPPER_BAUDRATE,
        device_id=cfg.GRIPPER_SLAVE_ID,
    )
    handles = actions.SystemHandles(robot=robot, gripper=gripper)
    handles.combo_active = False
    return handles


def _close_system_handles(handles: actions.SystemHandles) -> None:
    try:
        handles.gripper.disconnect()
    except Exception:
        pass
    try:
        handles.robot.close()
    except Exception:
        pass


def _initialize_cycle_session(handles: actions.SystemHandles) -> None:
    actions.arm_robot_once(handles)
    actions.connect_gripper_once(handles)
    handles.gripper.ensure_initialized()
    actions.initialize_stack_session(handles)


def _execute_cycle_with_handles(handles: actions.SystemHandles, plan: dict[str, object]) -> None:
    actions.execute_pick_pose(
        handles,
        pick_pose=plan["planned_pick_pose"],
        stack_level=int(plan["stack_level"]),
        target_label=f"marker_{plan['marker_id']}->{plan['target_id']}",
    )
    actions.move_to_tower_hover(
        handles,
        int(plan["stack_level"]),
        target_id=str(plan["target_id"]),
    )
    actions.complete_place_sequence(
        handles,
        int(plan["stack_level"]),
        place_pose=plan["planned_place_pose"],
        perform_neutral_exit=True,
        target_id=str(plan["target_id"]),
    )


def main() -> int:
    args = parse_args()
    participant_name = str(args.participant_name).strip() or "vision_pick_place_once"
    pick_ml_model_json = _configure_pick_ml(args)
    session_id = f"visionpickplace-{int(time.time())}-{uuid4().hex[:8]}"
    run_id = uuid4().hex
    set_jsonl_context(
        participant_name=participant_name,
        session_id=session_id,
        run_id=run_id,
        leaderboard_mode="LAB",
    )

    target_ids = _candidate_targets(args)
    requested_max_count = None if args.max_count is None else max(1, int(args.max_count))
    available_levels = max(0, int(cfg.stack_target_count()) - int(args.place_level))
    if requested_max_count is not None and requested_max_count > len(target_ids):
        raise RuntimeError(
            f"Requested max_count={requested_max_count} exceeds available target set size={len(target_ids)} "
            f"for targets={target_ids}"
        )
    if requested_max_count is not None and requested_max_count > available_levels:
        raise RuntimeError(
            f"Requested max_count={requested_max_count} exceeds remaining tower capacity={available_levels} "
            f"from place_level={int(args.place_level)}"
        )
    marker_ids = _candidate_markers(target_ids)
    if not marker_ids:
        raise RuntimeError(f"No pickup markers mapped for remaining targets={target_ids}")

    device_info, resolved_device_id = _resolve_device(args.device_id)
    session = CaptureSession(
        buffer_size=int(args.buffer_size),
        marker_ids=marker_ids,
        use_depth_module=bool(args.use_depth_module),
    )
    stop_event = threading.Event()
    capture_thread = _start_capture_thread(
        device_info=device_info,
        fps=float(args.fps),
        session=session,
        stop_event=stop_event,
        use_depth_module=bool(args.use_depth_module),
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
            f"[VISION_PICK_PLACE] device_id={resolved_device_id} device_label={args.device_label} "
            f"targets={target_ids} place_level={int(args.place_level)} max_count={requested_max_count} execute={bool(args.execute)} "
            f"use_depth_module={bool(args.use_depth_module)} pick_ml={bool(args.enable_pick_ml)}"
        )
        _write_cycle_event(
            "vision_pick_place_start",
            device_id=resolved_device_id,
            device_label=args.device_label,
            use_depth_module=bool(args.use_depth_module),
            pick_ml_enabled=bool(args.enable_pick_ml),
            pick_ml_model_json=pick_ml_model_json,
            candidate_targets=target_ids,
            place_level=int(args.place_level),
            max_count=requested_max_count,
            execute=bool(args.execute),
            started_at_utc=_timestamp_utc(),
        )
        workspace_x_mm, workspace_y_mm = _workspace_bounds(args)

        remaining_targets = list(target_ids)
        plans: list[dict[str, object]] = []
        max_cycles = min(
            len(remaining_targets),
            available_levels,
            requested_max_count if requested_max_count is not None else min(len(remaining_targets), available_levels),
        )
        for cycle_index in range(max_cycles):
            try:
                target_id, marker_id, summary = _select_stable_target(
                    session=session,
                    target_ids=remaining_targets,
                    min_samples=int(args.min_samples),
                    max_xy_std_mm=float(args.max_xy_std_mm),
                    timeout_s=float(args.timeout_s),
                    max_detection_age_s=float(args.max_detection_age_s),
                )
            except RuntimeError:
                if cycle_index == 0:
                    raise
                print(
                    f"[VISION_PICK_PLACE] no additional stable targets found after stacking {len(plans)} block(s); stopping."
                )
                break
            plan = _plan_cycle(
                target_id=target_id,
                marker_id=marker_id,
                summary=summary,
                place_level=int(args.place_level) + cycle_index,
                workspace_x_mm=workspace_x_mm,
                workspace_y_mm=workspace_y_mm,
            )
            plans.append(plan)
            remaining_targets = [candidate for candidate in remaining_targets if candidate != target_id]

            print(
                "[VISION_PICK_PLACE] plan "
                f"cycle={cycle_index + 1}/{max_cycles} "
                f"source={plan['target_id']} marker={plan['marker_id']} "
                f"pick_xy=({plan['planned_pick_pose'][0]:.3f},{plan['planned_pick_pose'][1]:.3f}) "
                f"tower_target={plan['tower_target_id']} place_xy=({plan['planned_place_pose'][0]:.3f},{plan['planned_place_pose'][1]:.3f}) "
                f"ml_reason={plan['pick_projection']['ml_reason']}"
            )
            if "depth_summary" in summary:
                print(f"[VISION_PICK_PLACE] depth summary={summary['depth_summary']}")
            grid = plan["pickup_grid_diagnostics"]
            print(
                "[VISION_PICK_PLACE] pickup_grid "
                f"source_template={grid['source_target_id']} "
                f"delta_mm=({grid['source_delta_mm']['dx']:.3f},{grid['source_delta_mm']['dy']:.3f}) "
                f"norm={grid['source_delta_mm']['norm']:.3f} "
                f"nearest={grid['nearest_pick_target_id']} "
                f"nearest_norm={grid['nearest_pick_delta_mm']['norm']:.3f}"
            )
            _write_cycle_event(
                "vision_pick_place_plan",
                cycle_index=int(cycle_index),
                cycle_count=max_cycles,
                target_id=plan["target_id"],
                marker_id=plan["marker_id"],
                stack_level=int(plan["stack_level"]),
                tower_target_id=plan["tower_target_id"],
                planned_pick_pose=plan["planned_pick_pose"],
                planned_pick_hover_pose=plan["planned_pick_hover_pose"],
                planned_place_pose=plan["planned_place_pose"],
                planned_tower_hover_pose=plan["planned_tower_hover_pose"],
                workspace_x_mm=list(workspace_x_mm),
                workspace_y_mm=list(workspace_y_mm),
                camera_pose_summary=summary,
                pickup_grid_diagnostics=plan["pickup_grid_diagnostics"],
                pick_projection=plan["pick_projection"],
                pick_ml_enabled=bool(args.enable_pick_ml),
                pick_ml_model_json=pick_ml_model_json,
            )
            if "depth_summary" in summary:
                _write_cycle_event(
                    "vision_pick_place_depth_summary",
                    cycle_index=int(cycle_index),
                    target_id=plan["target_id"],
                    marker_id=plan["marker_id"],
                    depth_summary=summary["depth_summary"],
                )
                write_jsonl_event(
                    "depth_dataset",
                    {
                        "event": "depth_pick_place_summary",
                        "module": "PERCEPTION",
                        "tool": "vision_pick_place_once",
                        "cycle_index": int(cycle_index),
                        "cycle_count": max_cycles,
                        "target_id": plan["target_id"],
                        "marker_id": int(plan["marker_id"]),
                        "stack_level": int(plan["stack_level"]),
                        "tower_target_id": plan["tower_target_id"],
                        "camera_pose_summary": summary,
                        "depth_summary": summary["depth_summary"],
                        "computed_robot_xy_mm": {
                            "x": round(float(plan["computed_robot_xy_mm"][0]), 3),
                            "y": round(float(plan["computed_robot_xy_mm"][1]), 3),
                        },
                        "planned_pick_pose": plan["planned_pick_pose"],
                        "planned_place_pose": plan["planned_place_pose"],
                        "workspace_x_mm": list(workspace_x_mm),
                        "workspace_y_mm": list(workspace_y_mm),
                        "use_depth_module": bool(args.use_depth_module),
                    },
                )

        if not args.execute:
            print("[VISION_PICK_PLACE] dry-run only. Re-run with --execute to move the robot.")
            _write_cycle_event(
                "vision_pick_place_dry_run",
                planned_cycle_count=len(plans),
                planned_targets=[plan["target_id"] for plan in plans],
                planned_tower_targets=[plan["tower_target_id"] for plan in plans],
            )
            return 0

        if not plans:
            raise RuntimeError("No stable pickup targets were available to plan.")

        handles = _create_system_handles()
        try:
            _initialize_cycle_session(handles)
            for cycle_index, plan in enumerate(plans):
                _write_cycle_event(
                    "vision_pick_place_execute_start",
                    cycle_index=int(cycle_index),
                    cycle_count=len(plans),
                    target_id=plan["target_id"],
                    marker_id=plan["marker_id"],
                    stack_level=int(plan["stack_level"]),
                    tower_target_id=plan["tower_target_id"],
                )
                _execute_cycle_with_handles(handles, plan)
                _write_cycle_event(
                    "vision_pick_place_execute_complete",
                    cycle_index=int(cycle_index),
                    cycle_count=len(plans),
                    target_id=plan["target_id"],
                    marker_id=plan["marker_id"],
                    stack_level=int(plan["stack_level"]),
                    tower_target_id=plan["tower_target_id"],
                    completed_at_utc=_timestamp_utc(),
                )
            if bool(args.home_after):
                actions.do_home(handles)
        finally:
            _close_system_handles(handles)
        print(f"[VISION_PICK_PLACE] cycle complete. stacked={len(plans)}")
        return 0
    finally:
        stop_event.set()
        capture_thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
