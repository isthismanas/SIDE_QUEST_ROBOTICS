import time, socket, struct, os, threading, json, sys, subprocess, signal
import warnings
import secrets
from datetime import timedelta
import hashlib
from uuid import uuid4
from state_machine import State, Event, step, parse_event
import leaderboard as lb
from leaderboard import normalize_leaderboard_mode

import actions
from actions import SystemHandles
import block_tracker
import drift_engine
import tolerance_engine
import vision_controller
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE  # NEW: RS485 Modbus gripper driver
import robot_config as cfg
from logger import info, warn, error, set_jsonl_context, write_jsonl_event

# --- Add perception module ---
VISION_MODE_ENABLED = vision_controller.VISION_MODE_ENABLED
CAMERA_STREAM_ENABLED = bool(getattr(cfg, "CAMERA_STREAM_ENABLED", True))

dai = None
if VISION_MODE_ENABLED or CAMERA_STREAM_ENABLED:
    try:
        import depthai as dai
    except Exception:
        dai = None

warnings.filterwarnings("ignore", category=DeprecationWarning)

if bool(getattr(cfg, "DEBUG_ENABLED", False)):
    info("CONTROL", f"USING ACTIONS FROM: {actions.__file__}")


# --- 0. STABILITY & PORT CONFIG ---
os.environ["DEPTHAI_WATCHDOG_TIMEOUT"] = "5000"

UNITY_PORT_INSPECTOR = 8085
UNITY_PORT_MANAGER   = 8086
UNITY_PORT_COMMANDS  = 8088

MXID_INSPECTOR = tuple(getattr(cfg, "INSPECTOR_CAMERA_IDS", ("19443010B14C872F00",)))
MXID_MANAGER   = tuple(getattr(cfg, "MANAGER_CAMERA_IDS", ("194430108183F12E00",)))

# --- Gripper ---
LAST_GRIP_TS = 0.0
GRIP_DEBOUNCE_S = 0.40
_last_nudge_t = 0.0

# Deterministic gripper positions (calibrated by you)
GRIP_OPEN_POS  = 900
GRIP_CLOSE_POS = 50

# --- Robot driver instance ---
robot = DobotDriver()

# --- Gripper driver instance (RS485 Modbus) ---
gripper = DHGripperPGE(
    port=cfg.GRIPPER_PORT,
    baudrate=cfg.GRIPPER_BAUDRATE,
    device_id=cfg.GRIPPER_SLAVE_ID,
)

# --- System handles (dependency injection) ---
handles = SystemHandles(robot=robot, gripper=gripper)
handles.combo_active = False

gripper_connected = False


def _initialize_gripper_on_startup() -> None:
    global gripper_connected

    if not gripper.connect():
        raise RuntimeError("Failed to connect to DH gripper")

    print("[GRIPPER] Ensuring initialization...")
    gripper.ensure_initialized()
    print("[GRIPPER] Gripper ready")
    gripper_connected = True

# --- Simple control mode/state (via State Machine) ---
STATE = State.IDLE

# --- NEW: Arm state tied to VR connection lifecycle ---
robot_armed = False
vr_connected = False

# --- Graceful shutdown ---
STOP_EVENT = threading.Event()
_SOCKETS_LOCK = threading.Lock()
_SERVER_SOCKETS: set[socket.socket] = set()
_CLIENT_SOCKETS: set[socket.socket] = set()
_WORKER_THREADS: list[threading.Thread] = []

# --- Session-level participant + telemetry state ---
participant_name = None
session_id = None
tower_attempt_start_ts = None
run_start_time = None
block_attempt_start_ts = None
drop_committed_this_window = False
decision_seq = 0
holding_block = False
current_session_token = uuid4()
proposed_place_pose = None
proposed_place_stack_level = None
current_zone = "GREEN"
current_zone_stack_level = None
quality_score = 0
green_count = 0
yellow_count = 0
red_count = 0
green_place_streak = 0
combo_active = False
_unity_command_conn = None
_last_sent_zone = None
run_id = None
run_finalized = False
_logged_raw_getpose_probe = False
current_run_seed = None
active_pick_target_id = None
active_pick_marker_target_id = None
active_pick_claim_target_id = None
placed_pick_target_ids: list[str] = []
picked_marker_target_ids: list[str] = []
expected_workbench_brick_count = None
committed_stack_level = 0
pending_commit_level = None
pending_commit_deadline = None
completion_finalize_pending = False
completion_end_mono = None
last_finalized_run_id = None
last_finalized_mode = None
last_finalized_session_id = None
last_finalized_participant_name = None
_score_state_lock = threading.Lock()
DEBUG_ENABLED = bool(getattr(cfg, "DEBUG_ENABLED", False))
CONSOLE_QUIET = not DEBUG_ENABLED
_DEFAULT_LOG_MODULE_LEVELS = dict(getattr(cfg, "LOG_MODULES", {}))
_last_ready_level_printed = None
QUIET_ALLOWLIST = {"PROMPT", "SUMMARY", "FAULT", "ERROR", "FATAL"}
LEADERBOARD_MODE = "DEV"
OFFICIAL_EVENT_ID = "ARC2026"

LEADERBOARD_PORT = 8090
VISION_ASSIST_STATE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "perception",
    "logs",
    "vision_pick_assist_state.json",
)
REPO_ROOT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
TRAIN_PICK_ML_SCRIPT_PATH = os.path.join(REPO_ROOT_DIR, "Robotics", "perception", "train_pick_ml_residual.py")
_TERMINAL_INPUT_LOCK = threading.Lock()

# Shared mutable context read by the leaderboard HTTP handler at request time.
# Kept in sync with LEADERBOARD_MODE / OFFICIAL_EVENT_ID whenever they change.
lb_ctx = lb.LeaderboardContext(mode=LEADERBOARD_MODE, official_event_id=OFFICIAL_EVENT_ID)

# Hard timeout: 
HARD_TIMEOUT_S = 300.0 # 300s for competition


def _track_server_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _SERVER_SOCKETS.add(sock)


def _untrack_server_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _SERVER_SOCKETS.discard(sock)


def _track_client_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _CLIENT_SOCKETS.add(sock)


def _untrack_client_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _CLIENT_SOCKETS.discard(sock)


def _close_socket_quietly(sock: socket.socket) -> None:
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass


def _close_all_tracked_sockets() -> list[Exception]:
    errors: list[Exception] = []
    with _SOCKETS_LOCK:
        client_sockets = list(_CLIENT_SOCKETS)
        server_sockets = list(_SERVER_SOCKETS)

    for sock in client_sockets:
        try:
            _close_socket_quietly(sock)
        except Exception as e:
            errors.append(e)
        finally:
            _untrack_client_socket(sock)

    for sock in server_sockets:
        try:
            _close_socket_quietly(sock)
        except Exception as e:
            errors.append(e)
        finally:
            _untrack_server_socket(sock)

    return errors


def _join_worker_threads(timeout_s: float = 1.0) -> list[str]:
    alive: list[str] = []
    for thread in _WORKER_THREADS:
        if not thread.daemon:
            thread.join(timeout=timeout_s)
            if thread.is_alive():
                alive.append(thread.name)
    return alive


def _is_official_mode() -> bool:
    return normalize_leaderboard_mode(LEADERBOARD_MODE) == "OFFICIAL"


# ---------------------------------------------------------------------------
# Thin wrappers — delegate to leaderboard.py, injecting the current globals.
# Call sites in handle_command are unchanged.
# ---------------------------------------------------------------------------

def log_event(event: str, **fields) -> None:
    payload = {
        "leaderboard_mode": LEADERBOARD_MODE,
        "session_id": session_id,
        "participant_name": participant_name,
        "state": STATE,
        "current_stack_level": globals().get("current_stack_level", 0),
        "current_pick_index": globals().get("current_pick_index", 0),
        "event_type": event,
    }
    payload.update(fields)
    return lb.log_event(**payload)


def _sync_json_log_context() -> None:
    participant = participant_name.strip() if isinstance(participant_name, str) and participant_name.strip() else None
    set_jsonl_context(
        participant_name=participant,
        session_id=session_id,
        run_id=run_id,
        leaderboard_mode=LEADERBOARD_MODE,
    )


def _current_pick_target_id() -> str | None:
    if isinstance(active_pick_target_id, str) and active_pick_target_id.strip():
        return str(active_pick_target_id)
    try:
        if 0 <= int(current_pick_index) < len(cfg.PICK_SEQUENCE):
            return str(cfg.PICK_SEQUENCE[int(current_pick_index)])
    except Exception:
        return None
    return None


def _reset_pick_runtime_cache() -> None:
    global active_pick_target_id, active_pick_marker_target_id, active_pick_claim_target_id
    global placed_pick_target_ids, picked_marker_target_ids, expected_workbench_brick_count
    active_pick_target_id = None
    active_pick_marker_target_id = None
    active_pick_claim_target_id = None
    placed_pick_target_ids = []
    picked_marker_target_ids = []


def _write_vision_assist_state() -> None:
    payload = {
        "participant_name": participant_name,
        "session_id": session_id,
        "run_id": run_id,
        "pick_mode": str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower(),
        "expected_workbench_brick_count": expected_workbench_brick_count,
        "claimed_pick_slot_ids": list(placed_pick_target_ids),
        "picked_marker_target_ids": list(picked_marker_target_ids),
        "placed_pick_target_ids": list(placed_pick_target_ids),
        "active_pick_target_id": active_pick_target_id,
        "active_pick_marker_target_id": active_pick_marker_target_id,
        "active_pick_claim_target_id": active_pick_claim_target_id,
        "current_stack_level": int(current_stack_level) if isinstance(current_stack_level, int) else current_stack_level,
        "current_pick_index": int(current_pick_index) if isinstance(current_pick_index, int) else current_pick_index,
        "remaining_pick_slots": _remaining_pick_slots(),
        "remaining_marker_targets": _remaining_marker_targets(),
        "remaining_targets": _remaining_pick_slots(),
        "ts_unix": time.time(),
    }
    os.makedirs(os.path.dirname(VISION_ASSIST_STATE_PATH), exist_ok=True)
    with open(VISION_ASSIST_STATE_PATH, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


def _nearest_pick_slot_claim(target_id: str) -> str:
    tracking = block_tracker.track_pick_target(str(target_id))
    if tracking.get("available", False):
        robot_x, robot_y = tracking["robot_xy"]
        best_target_id = None
        best_norm = None
        for candidate_id, candidate_pose in getattr(cfg, "PICKUP_POINTS", {}).items():
            dx = float(robot_x) - float(candidate_pose[0])
            dy = float(robot_y) - float(candidate_pose[1])
            norm = (dx * dx + dy * dy) ** 0.5
            if best_norm is None or norm < best_norm:
                best_target_id = str(candidate_id)
                best_norm = norm
        claim_radius_mm = float(getattr(cfg, "VISION_PICK_SLOT_CLAIM_RADIUS_MM", 70.0))
        if best_target_id is not None and best_norm is not None and best_norm <= claim_radius_mm:
            return best_target_id
    return str(target_id)


def _prompt_workbench_brick_count() -> int | None:
    pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
    if pick_mode != "vision":
        return None

    target_height = int(getattr(cfg, "TOWER_LEVELS", 7))
    prompt = f"How many bricks are on the workbench? [1-{target_height}] "
    with _TERMINAL_INPUT_LOCK:
        while True:
            try:
                raw = input(prompt).strip()
            except EOFError:
                return None
            if not raw:
                continue
            try:
                value = int(raw)
            except ValueError:
                print(f"[VISION] Enter an integer between 1 and {target_height}.")
                continue
            if 1 <= value <= target_height:
                return value
            print(f"[VISION] Enter an integer between 1 and {target_height}.")


def _effective_target_stack_count() -> int:
    configured = int(cfg.stack_target_count())
    if str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower() == "vision":
        if expected_workbench_brick_count is not None:
            return max(0, min(configured, int(expected_workbench_brick_count)))
    return configured


def _retrain_pick_ml_before_run() -> bool:
    pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
    if pick_mode != "vision":
        return True
    if not bool(getattr(cfg, "VISION_PICK_ML_ENABLED", False)):
        console_info("VISION", "[VISION] Pickup ML disabled in config; skipping retraining.", essential=True)
        return True
    if not os.path.exists(TRAIN_PICK_ML_SCRIPT_PATH):
        warn("VISION", f"[VISION] Pickup ML trainer missing: {TRAIN_PICK_ML_SCRIPT_PATH}")
        return False

    output_json = str(getattr(cfg, "VISION_PICK_ML_MODEL_JSON", "")).strip()
    if not output_json:
        warn("VISION", "[VISION] Pickup ML model path is unset; cannot retrain.")
        return False
    if not os.path.isabs(output_json):
        output_json = os.path.abspath(os.path.join(REPO_ROOT_DIR, output_json))

    cmd = [
        sys.executable,
        TRAIN_PICK_ML_SCRIPT_PATH,
        "--output-json",
        output_json,
        "--use-log-confirmation-samples",
        "--use-runtime-residual-samples",
    ]
    console_info("VISION", "[VISION] Retraining pickup ML from previous logs before run start...", essential=_is_debug_enabled())
    started_at = time.monotonic()
    try:
        result = subprocess.run(
            cmd,
            cwd=REPO_ROOT_DIR,
            capture_output=True,
            text=True,
            timeout=120.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        warn("VISION", "[VISION] Pickup ML retraining timed out after 120s. Continuing with the existing model.")
        write_jsonl_event(
            "motion_trace",
            {
                "event": "pick_ml_retrain_timeout",
                "module": "VISION",
                "pick_mode": pick_mode,
                "output_json": output_json,
            },
        )
        return False
    except Exception as exc:
        warn("VISION", f"[VISION] Pickup ML retraining failed: {exc}. Continuing with the existing model.")
        write_jsonl_event(
            "motion_trace",
            {
                "event": "pick_ml_retrain_error",
                "module": "VISION",
                "pick_mode": pick_mode,
                "output_json": output_json,
                "error": str(exc),
            },
        )
        return False

    duration_s = time.monotonic() - started_at
    stdout_lines = [line.strip() for line in str(result.stdout or "").splitlines() if line.strip()]
    stderr_lines = [line.strip() for line in str(result.stderr or "").splitlines() if line.strip()]
    metrics: dict[str, str] = {}
    for line in stdout_lines:
        if line.startswith("[PICK_ML] ") and "=" in line:
            key, value = line[len("[PICK_ML] ") :].split("=", 1)
            metrics[key.strip()] = value.strip()

    if result.returncode != 0:
        warn(
            "VISION",
            f"[VISION] Pickup ML retraining exited with code {result.returncode}. Continuing with the existing model.",
        )
        for line in stdout_lines[-6:]:
            console_info("VISION", line, essential=True)
        for line in stderr_lines[-6:]:
            warn("VISION", line)
        write_jsonl_event(
            "motion_trace",
            {
                "event": "pick_ml_retrain_failed",
                "module": "VISION",
                "pick_mode": pick_mode,
                "output_json": output_json,
                "returncode": int(result.returncode),
                "duration_s": duration_s,
                "stdout_tail": stdout_lines[-6:],
                "stderr_tail": stderr_lines[-6:],
            },
        )
        return False

    summary = (
        "[VISION] Pickup ML retrain complete: "
        f"samples={metrics.get('sample_count', '?')} "
        f"runtime={metrics.get('runtime_residual_sample_count', '?')} "
        f"loo_rmse_total_mm={metrics.get('loo_rmse_total_mm', '?')} "
        f"duration_s={duration_s:.1f}"
    )
    console_info("VISION", summary, essential=_is_debug_enabled())
    write_jsonl_event(
        "motion_trace",
        {
            "event": "pick_ml_retrain_complete",
            "module": "VISION",
            "pick_mode": pick_mode,
            "output_json": output_json,
            "duration_s": duration_s,
            "returncode": int(result.returncode),
            "sample_count": metrics.get("sample_count"),
            "calibration_sample_count": metrics.get("calibration_sample_count"),
            "log_confirmation_sample_count": metrics.get("log_confirmation_sample_count"),
            "runtime_residual_sample_count": metrics.get("runtime_residual_sample_count"),
            "base_rmse_total_mm": metrics.get("base_rmse_total_mm"),
            "train_rmse_total_mm": metrics.get("train_rmse_total_mm"),
            "loo_rmse_total_mm": metrics.get("loo_rmse_total_mm"),
        },
    )
    return True


def _remaining_pick_slots() -> list[str]:
    completed = {str(target_id) for target_id in placed_pick_target_ids}
    if isinstance(active_pick_claim_target_id, str) and active_pick_claim_target_id.strip():
        completed.add(str(active_pick_claim_target_id))
    return [
        str(target_id)
        for target_id in getattr(cfg, "PICK_SEQUENCE", [])
        if str(target_id) not in completed
    ]


def _remaining_marker_targets() -> list[str]:
    completed = {str(target_id) for target_id in picked_marker_target_ids}
    if isinstance(active_pick_marker_target_id, str) and active_pick_marker_target_id.strip():
        completed.add(str(active_pick_marker_target_id))
    return [
        str(target_id)
        for target_id in getattr(cfg, "PICK_SEQUENCE", [])
        if str(target_id) not in completed
    ]


def _remaining_pick_targets() -> list[str]:
    pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
    if pick_mode == "vision":
        return _remaining_pick_slots()
    return _remaining_pick_slots()


def _pick_selection_reason(selected_target_id: str, reason: str) -> None:
    write_jsonl_event(
        "block_state",
        {
            "event": "pick_target_selected",
            "module": "CONTROL",
            "pick_mode": str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower(),
            "selected_target_id": str(selected_target_id),
            "selection_reason": str(reason),
            "remaining_pick_slots": _remaining_pick_slots(),
            "remaining_marker_targets": _remaining_marker_targets(),
            "remaining_targets": _remaining_pick_slots(),
            "placed_pick_target_ids": list(placed_pick_target_ids),
            "picked_marker_target_ids": list(picked_marker_target_ids),
        },
    )


def _clear_active_pick_targets() -> None:
    global active_pick_target_id, active_pick_marker_target_id, active_pick_claim_target_id
    active_pick_target_id = None
    active_pick_marker_target_id = None
    active_pick_claim_target_id = None


def _set_active_pick_targets(pick_target_id: str, pick_mode_override: str | None) -> None:
    global active_pick_target_id, active_pick_marker_target_id, active_pick_claim_target_id
    active_pick_target_id = str(pick_target_id)
    pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
    if pick_mode == "vision" and pick_mode_override is None:
        active_pick_marker_target_id = str(pick_target_id)
    else:
        active_pick_marker_target_id = None
    active_pick_claim_target_id = _nearest_pick_slot_claim(str(pick_target_id))


def _deterministic_remaining_pick_fallback_target(
    reason: str,
    remaining_slots: list[str] | None = None,
) -> tuple[str | None, str | None]:
    pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
    if pick_mode != "vision":
        return None, None

    ordered_slots = [str(target_id) for target_id in (remaining_slots or _remaining_pick_slots())]
    if not ordered_slots:
        return None, None

    fallback_target_id = str(ordered_slots[0])
    selection_reason = f"deterministic_remaining_slot:{reason}"
    console_info(
        "CONTROL",
        f"[VISION] deterministic remaining-slot fallback target={fallback_target_id} reason={reason}",
        essential=True,
    )
    _pick_selection_reason(fallback_target_id, selection_reason)
    return fallback_target_id, "deterministic"


def _resolve_pick_target_for_cycle() -> tuple[str, str | None]:
    pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()

    if pick_mode == "vision":
        remaining_marker_targets = _remaining_marker_targets()
        remaining_pick_slots = _remaining_pick_slots()
        if not remaining_marker_targets and not remaining_pick_slots:
            raise RuntimeError("no_remaining_pick_targets")

        selected_target_id = vision_controller.select_next_pick_target(
            remaining_marker_targets,
            debug_enabled=_is_debug_enabled(),
            allow_cached=True,
        )
        if selected_target_id is not None:
            _pick_selection_reason(selected_target_id, "vision_tracker")
            return str(selected_target_id), None

        fallback_target_id, pick_mode_override = _deterministic_remaining_pick_fallback_target(
            "vision_pick_target_unavailable",
            remaining_slots=remaining_pick_slots,
        )
        if fallback_target_id is not None:
            return fallback_target_id, pick_mode_override

        raise RuntimeError("vision_pick_target_unavailable")

    try:
        return str(cfg.PICK_SEQUENCE[int(current_pick_index)]), None
    except Exception as exc:
        raise RuntimeError(f"pick_sequence_unavailable:{exc}")


def _log_inferred_block_state(event_name: str, **fields) -> None:
    payload = {
        "event": event_name,
        "module": "CONTROL",
    }
    payload.update(fields)
    write_jsonl_event("block_state", payload)


def _quality_points_for_zone(zone: str) -> int:
    normalized = str(zone).strip().upper()
    if normalized == "GREEN":
        return 3
    if normalized == "YELLOW":
        return 2
    if normalized == "RED":
        return 1
    return 0


def _reset_quality_tracking() -> None:
    global quality_score, green_count, yellow_count, red_count
    with _score_state_lock:
        quality_score = 0
        green_count = 0
        yellow_count = 0
        red_count = 0


def _snapshot_quality_tracking() -> tuple[int, int, int, int]:
    with _score_state_lock:
        return int(quality_score), int(green_count), int(yellow_count), int(red_count)


def _record_quality_placement(zone: str) -> int:
    global quality_score, green_count, yellow_count, red_count
    normalized = str(zone).strip().upper()
    delta = _quality_points_for_zone(normalized)
    with _score_state_lock:
        if normalized == "GREEN":
            green_count += 1
        elif normalized == "YELLOW":
            yellow_count += 1
        elif normalized == "RED":
            red_count += 1
        quality_score += delta
    return delta


def _clear_score_commit_state(*, reset_committed: bool = True) -> None:
    global committed_stack_level, pending_commit_level, pending_commit_deadline, completion_finalize_pending, completion_end_mono
    with _score_state_lock:
        if reset_committed:
            committed_stack_level = 0
        pending_commit_level = None
        pending_commit_deadline = None
        completion_finalize_pending = False
        completion_end_mono = None


def _set_pending_commit(level: int, deadline_mono: float) -> None:
    global pending_commit_level, pending_commit_deadline
    with _score_state_lock:
        pending_commit_level = int(level)
        pending_commit_deadline = float(deadline_mono)


def _snapshot_score_state() -> tuple[int, object, object, bool]:
    with _score_state_lock:
        return committed_stack_level, pending_commit_level, pending_commit_deadline, completion_finalize_pending


def _resolve_official_score(as_of_mono: float | None = None) -> int:
    effective_now = time.monotonic() if as_of_mono is None else float(as_of_mono)
    with _score_state_lock:
        score = int(committed_stack_level or 0)
        if pending_commit_level is not None and pending_commit_deadline is not None and effective_now >= pending_commit_deadline:
            score = int(pending_commit_level)
        return score


def _promote_pending_commit_if_ready(now_mono: float | None = None) -> bool:
    global committed_stack_level, pending_commit_level, pending_commit_deadline, completion_finalize_pending, run_finalized
    global current_pick_index, current_stack_level, holding_block, proposed_place_pose, proposed_place_stack_level
    global block_attempt_start_ts, drop_committed_this_window, tower_attempt_start_ts, run_start_time, participant_name
    global run_id, session_id
    global current_run_seed, _last_ready_level_printed, last_finalized_run_id, last_finalized_mode
    global last_finalized_session_id, last_finalized_participant_name, completion_end_mono

    effective_now = time.monotonic() if now_mono is None else float(now_mono)
    finalize_complete = False
    latched_end_mono = None

    with _score_state_lock:
        if pending_commit_level is not None and pending_commit_deadline is not None and effective_now >= pending_commit_deadline:
            committed_stack_level = int(pending_commit_level)
            pending_commit_level = None
            pending_commit_deadline = None
            finalize_complete = completion_finalize_pending
            completion_finalize_pending = False
            latched_end_mono = completion_end_mono

    if not finalize_complete:
        return False

    run_quality_score, run_green_count, run_yellow_count, run_red_count = _snapshot_quality_tracking()

    run_time_s = lb.emit_run_summary(
        "COMPLETE",
        run_start_time=run_start_time,
        participant_name=participant_name,
        current_stack_level=_resolve_official_score(effective_now),
        quality_score=run_quality_score,
        end_time_mono=latched_end_mono,
    )
    if lb.finalize_run(
        "COMPLETE",
        ctx=lb_ctx,
        run_id=run_id,
        session_id=session_id,
        participant_name=participant_name,
        current_stack_level=_resolve_official_score(effective_now),
        run_start_time=run_start_time,
        quality_score=run_quality_score,
        green_count=run_green_count,
        yellow_count=run_yellow_count,
        red_count=run_red_count,
        already_finalized=run_finalized,
        end_time_mono=latched_end_mono,
    ):
        run_finalized = True
        last_finalized_run_id = run_id
        last_finalized_mode = normalize_leaderboard_mode(lb_ctx.mode)
        last_finalized_session_id = session_id
        last_finalized_participant_name = participant_name

    log_event(
        "EVENT_RUN_SUMMARY",
        participant=participant_name,
        blocks_placed=_resolve_official_score(effective_now),
        run_time_s=run_time_s,
        quality_score=run_quality_score,
        green_count=run_green_count,
        yellow_count=run_yellow_count,
        red_count=run_red_count,
        source="COMPLETE",
    )

    _send_line_to_unity(f"RUN_COMPLETE {current_stack_level}")
    time.sleep(5.0)
    console_emit("[STACK] COMPLETE summary emitted (post-wait)", tag="SUMMARY", level="INFO", module="CONTROL", allow_in_quiet=True)

    current_pick_index = 0
    current_stack_level = 0
    _reset_pick_runtime_cache()
    holding_block = False
    proposed_place_pose = None
    proposed_place_stack_level = None
    block_attempt_start_ts = None
    drop_committed_this_window = False
    tower_attempt_start_ts = None
    run_start_time = None
    participant_name = None
    run_id = None
    current_run_seed = None
    cfg.DRIFT_RUNTIME_RUN_SEED = None
    cfg.DRIFT_RUNTIME_PARTICIPANT = ""
    _last_ready_level_printed = None
    _clear_score_commit_state()
    _reset_quality_tracking()
    _sync_json_log_context()
    return True


def emit_run_summary(reason: str) -> float:
    run_quality_score, _, _, _ = _snapshot_quality_tracking()
    return lb.emit_run_summary(
        reason,
        run_start_time=run_start_time,
        participant_name=participant_name,
        current_stack_level=_resolve_official_score(),
        quality_score=run_quality_score,
    )


def finalize_run(end_state: str) -> None:
    global run_finalized, last_finalized_run_id, last_finalized_mode, last_finalized_session_id, last_finalized_participant_name
    run_quality_score, run_green_count, run_yellow_count, run_red_count = _snapshot_quality_tracking()
    if lb.finalize_run(
        end_state,
        ctx=lb_ctx,
        run_id=run_id,
        session_id=session_id,
        participant_name=participant_name,
        current_stack_level=_resolve_official_score(),
        run_start_time=run_start_time,
        quality_score=run_quality_score,
        green_count=run_green_count,
        yellow_count=run_yellow_count,
        red_count=run_red_count,
        already_finalized=run_finalized,
    ):
        run_finalized = True
        last_finalized_run_id = run_id
        last_finalized_mode = normalize_leaderboard_mode(lb_ctx.mode)
        last_finalized_session_id = session_id
        last_finalized_participant_name = participant_name


def _check_run_timeout() -> bool:
    """Check if run is active and has exceeded 5-minute hard limit."""
    if run_start_time is None or run_finalized:
        return False
    now = time.monotonic()
    elapsed = now - run_start_time
    return elapsed >= HARD_TIMEOUT_S


def _handle_run_timeout() -> None:
    """Handler for 5-minute hard timeout (end_state='TIMEOUT')."""
    global run_finalized, last_finalized_run_id, last_finalized_mode, last_finalized_session_id, last_finalized_participant_name
    global STATE, current_pick_index, current_stack_level, holding_block, participant_name, tower_attempt_start_ts
    global run_start_time, block_attempt_start_ts, drop_committed_this_window, proposed_place_pose, proposed_place_stack_level
    global current_zone, current_zone_stack_level, green_place_streak, combo_active, run_id, current_run_seed, _last_ready_level_printed
    global controller_busy, current_session_token
    
    module = "CONTROL"
    current_session_token = uuid4()
    timeout_mono = time.monotonic()
    official_score = _resolve_official_score(timeout_mono)
    blocks_placed = official_score
    run_quality_score, run_green_count, run_yellow_count, run_red_count = _snapshot_quality_tracking()
    
    run_time_s = lb.emit_run_summary(
        "TIMEOUT",
        run_start_time=run_start_time,
        participant_name=participant_name,
        current_stack_level=official_score,
        quality_score=run_quality_score,
        end_time_mono=timeout_mono,
    )
    
    if lb.finalize_run(
        "TIMEOUT",
        ctx=lb_ctx,
        run_id=run_id,
        session_id=session_id,
        participant_name=participant_name,
        current_stack_level=official_score,
        run_start_time=run_start_time,
        quality_score=run_quality_score,
        green_count=run_green_count,
        yellow_count=run_yellow_count,
        red_count=run_red_count,
        already_finalized=run_finalized,
        end_time_mono=timeout_mono,
    ):
        run_finalized = True
        last_finalized_run_id = run_id
        last_finalized_mode = normalize_leaderboard_mode(lb_ctx.mode)
        last_finalized_session_id = session_id
        last_finalized_participant_name = participant_name
    
    _send_line_to_unity("RUN_FAIL TIMEOUT")
    
    log_event(
        "EVENT_RUN_SUMMARY",
        participant=participant_name,
        blocks_placed=blocks_placed,
        run_time_s=run_time_s,
        quality_score=run_quality_score,
        green_count=run_green_count,
        yellow_count=run_yellow_count,
        red_count=run_red_count,
        source="TIMEOUT",
    )
    
    waited = False
    while controller_busy:
        if not waited:
            info(module, "[TIMEOUT] Controller busy; waiting to execute timeout recovery...")
            waited = True
        time.sleep(0.05)

    if waited:
        info(module, "[TIMEOUT] Controller free; executing timeout recovery.")

    controller_busy = True
    try:
        info(module, f"[TIMEOUT] enter state={STATE.name} holding_block_flag={holding_block}")
        detected_holding = actions.execute_tumble_sequence(handles, fallback_holding=holding_block)
        info(module, f"[TIMEOUT] completed detected_holding={detected_holding}")
        log_event(
            "EVENT_TIMEOUT_DUMP",
            source="TIMEOUT",
            state_before=STATE.name,
            holding_block_flag=holding_block,
            holding_detected=detected_holding,
        )
    except Exception as e:
        warn(module, f"[TIMEOUT] dump sequence failed: {e}")
        log_event("EVENT_TIMEOUT_DUMP_ERROR", source="TIMEOUT", error=str(e))
    finally:
        controller_busy = False
    
    STATE = State.IDLE
    current_pick_index = 0
    current_stack_level = 0
    _reset_pick_runtime_cache()
    holding_block = False
    participant_name = None
    tower_attempt_start_ts = None
    run_start_time = None
    block_attempt_start_ts = None
    drop_committed_this_window = False
    proposed_place_pose = None
    proposed_place_stack_level = None
    current_zone = "GREEN"
    current_zone_stack_level = None
    green_place_streak = 0
    if combo_active:
        send_boost_end()
    combo_active = False
    handles.combo_active = combo_active
    send_boost_state(0, False)
    run_id = None
    run_finalized = False
    current_run_seed = None
    cfg.DRIFT_RUNTIME_RUN_SEED = None
    cfg.DRIFT_RUNTIME_PARTICIPANT = ""
    _last_ready_level_printed = None
    _clear_score_commit_state()
    _reset_quality_tracking()
    _sync_json_log_context()
    if vr_connected:
        console_emit("Waiting for participant name...", tag="PROMPT", level="INFO", module="CONTROL", allow_in_quiet=True)


def _is_debug_enabled() -> bool:
    return bool(DEBUG_ENABLED)


def _is_console_quiet() -> bool:
    return bool(CONSOLE_QUIET)


def _apply_console_verbosity() -> None:
    if not isinstance(getattr(cfg, "LOG_MODULES", None), dict):
        return

    if _is_console_quiet():
        for key in list(cfg.LOG_MODULES.keys()):
            cfg.LOG_MODULES[key] = "WARN"
    else:
        for key, value in _DEFAULT_LOG_MODULE_LEVELS.items():
            cfg.LOG_MODULES[key] = value


def console_emit(
    message: str,
    tag: str,
    level: str,
    module: str = "CONTROL",
    allow_in_quiet: bool = False,
) -> None:
    tag_u = str(tag).strip().upper()
    level_u = str(level).strip().upper()

    if _is_console_quiet():
        allow = (level_u in {"WARN", "ERROR", "FATAL"}) or (tag_u in QUIET_ALLOWLIST) or allow_in_quiet
        if not allow:
            return
        print(f"[{module}] {message}")
        return

    if level_u in {"ERROR", "FATAL"}:
        error(module, message)
    elif level_u in {"WARN", "WARNING"}:
        warn(module, message)
    else:
        info(module, message)


def console_info(module: str, message: str, essential: bool = False) -> None:
    console_emit(
        message=message,
        tag="PROMPT" if essential else "INFO",
        level="INFO",
        module=module,
        allow_in_quiet=essential,
    )


def _emit_ready_prompt(level) -> None:
    global _last_ready_level_printed
    if level == _last_ready_level_printed:
        return
    _last_ready_level_printed = level
    console_info("CONTROL", f"READY: waiting for DROP/FIX (stack_level={level})", essential=_is_debug_enabled())


def _set_debug_enabled(enabled: bool) -> None:
    global DEBUG_ENABLED, CONSOLE_QUIET
    DEBUG_ENABLED = bool(enabled)
    CONSOLE_QUIET = not DEBUG_ENABLED
    cfg.DEBUG_ENABLED = DEBUG_ENABLED
    _apply_console_verbosity()


# (leaderboard path helpers, record helpers, _LeaderboardHandler, and
#  leaderboard_http_server have been moved to leaderboard.py)
lb.load_leaderboard_mode(lb_ctx)
LEADERBOARD_MODE = lb_ctx.mode
OFFICIAL_EVENT_ID = lb_ctx.official_event_id
_apply_console_verbosity()
_initialize_gripper_on_startup()


# --- SIGNIFIER: prove which file is running ---
def _startup_banner():
    try:
        path = os.path.abspath(__file__)
        with open(path, "rb") as f:
            h = hashlib.sha1(f.read()).hexdigest()[:10]
        info("CONTROL", "=" * 72)
        info("CONTROL", "SIDE QUEST TASK CONTROLLER — ARM-ON-CONNECT BUILD (NO PER-COMMAND ARM)")
        info("CONTROL", f"RUNNING FILE: {path}")
        info("CONTROL", f"FILE SHA1 (first10): {h}")
        info("CONTROL", "=" * 72)
    except Exception as e:
        warn("CONTROL", f"Could not print startup banner: {e}")

_startup_banner()


# --- 2. CAMERA PIPELINE ---
camera_streamer = None
CAMERA_STREAM_RUNTIME_ENABLED = bool(CAMERA_STREAM_ENABLED)
if CAMERA_STREAM_RUNTIME_ENABLED:
    try:
        import camera_streamer
    except ModuleNotFoundError:
        camera_streamer = None
        CAMERA_STREAM_RUNTIME_ENABLED = False
        warn("PERC", "camera_streamer missing; camera stream disabled")

def camera_server_wrapper(mxid, port, label):
    if (not CAMERA_STREAM_RUNTIME_ENABLED) or (dai is None) or (camera_streamer is None):
        return
    enable_rawL, perc_engine = vision_controller.camera_stream_perception_binding(label)
    
    camera_streamer.start_camera_server(
        mxid=mxid, 
        port=port, 
        label=label, 
        enable_rawL=enable_rawL, 
        stop_event=STOP_EVENT, 
        perc_engine=perc_engine if enable_rawL else None
    )

def ensure_ready(precision: bool = True):
    """
    IMPORTANT:
    - We DO NOT ClearError/EnableRobot/SpeedFactor per command anymore.
    - Arming happens ONCE on VR connect in command_server().
    """
    global robot_armed
    if not robot_armed:
        warn("CONTROL", "[SAFETY] Motion ignored: robot DISARMED (no active VR connection).")
        return False
    return True


def ensure_gripper_ready():
    if not gripper_connected:
        warn("CONTROL", "[SAFETY] Gripper ignored: gripper NOT connected (RS485).")
        return False
    return True


def _module_for_source(source: str) -> str:
    return "ADMIN" if source == "ADMIN" else "CONTROL"


def _send_line_to_unity(line: str) -> bool:
    if _unity_command_conn is None:
        return False
    try:
        _unity_command_conn.sendall(f"{line}\n".encode("utf-8"))
        return True
    except Exception:
        return False


def send_ack(cmd: str) -> bool:
    return _send_line_to_unity(f"ACK {cmd}")


def send_nack(cmd: str, reason: str) -> bool:
    return _send_line_to_unity(f"NACK {cmd} {reason}")


def send_boost_state(combo_count: int, boost_active: bool) -> bool:
    combo = max(0, min(3, int(combo_count)))
    boost = 1 if bool(boost_active) else 0
    return _send_line_to_unity(f"BOOST_STATE {combo} {boost}")


def send_boost_end() -> bool:
    return _send_line_to_unity("BOOST_END")


def _reset_drift_scale_for_run(boundary: str) -> None:
    default_scale = float(getattr(cfg, "DRIFT_SCALE_DEFAULT", getattr(cfg, "DRIFT_SCALE", 1.0)))
    cfg.DRIFT_SCALE = default_scale
    if _is_debug_enabled():
        info("DRIFT", f"[RUN] DRIFT_SCALE reset to {cfg.DRIFT_SCALE:.3f} boundary={boundary}")


def _generate_runtime_run_seed() -> int:
    forced_seed = getattr(cfg, "DRIFT_FORCE_RUN_SEED", None)
    if forced_seed is not None:
        return int(forced_seed)
    return int(secrets.randbits(64))


def _push_zone_if_needed(force: bool = False) -> None:
    global _last_sent_zone
    if _unity_command_conn is None:
        return
    if not force and _last_sent_zone == current_zone:
        return
    sent = _send_line_to_unity(f"ZONE {current_zone}")
    if sent:
        _last_sent_zone = current_zone


def _handle_post_tower_hover(module: str, my_token) -> bool:
    """Handle post-hover fault check and AT_TOWER_HOVER transition in one place."""
    global STATE, block_attempt_start_ts, drop_committed_this_window, decision_seq, proposed_place_pose, proposed_place_stack_level
    global current_zone, current_zone_stack_level

    m = handles.robot.robot_mode()
    if m in (9, 11):
        if my_token != current_session_token:
            return False
        console_emit(f"[FAULT] RobotMode={m} -> entering FAULT", tag="FAULT", level="WARN", module=module, allow_in_quiet=True)
        fault_result = step(STATE, Event.FAULT)
        if fault_result.allowed:
            STATE = fault_result.next_state
            info(module, f"[SM] -> {STATE.name} (FAULT)")
        return False

    if my_token != current_session_token:
        return False
    result = step(STATE, Event.AT_TOWER_HOVER)
    if result.allowed:
        STATE = result.next_state
        info(module, f"[SM] -> {STATE.name} (AT_TOWER_HOVER)")
        if STATE == State.WAITING_FOR_DECISION:
            drop_committed_this_window = False
            decision_seq += 1
            if current_stack_level is not None and current_stack_level >= 0:
                base_place = cfg.tower_place_pose(current_stack_level)
                if proposed_place_pose is None or proposed_place_stack_level != current_stack_level:
                    proposed_place_pose = drift_engine.inject_drift(
                        base_place,
                        current_stack_level,
                        run_seed=current_run_seed,
                        participant=participant_name,
                    )
                    proposed_place_stack_level = current_stack_level
                    log_event(
                        "EVENT_DRIFT_LEVEL",
                        source="INTERNAL",
                        stack_level=current_stack_level,
                        drift_dx_mm=round(proposed_place_pose[0] - base_place[0], 3),
                        drift_dy_mm=round(proposed_place_pose[1] - base_place[1], 3),
                        runtime_run_seed=current_run_seed,
                        drift_run_seed_baseline=int(getattr(cfg, "DRIFT_RUN_SEED", 0)),
                    )
                ex = proposed_place_pose[0] - base_place[0]
                ey = proposed_place_pose[1] - base_place[1]

                tcp_before = handles.robot.get_tcp_pose()
                tcp_after = handles.robot.get_tcp_pose()

                if tcp_after is not None:
                    measured_pose = (
                        tcp_after[0],
                        tcp_after[1],
                        proposed_place_pose[2],
                        proposed_place_pose[3],
                        proposed_place_pose[4],
                        proposed_place_pose[5],
                    )
                    current_zone = tolerance_engine.classify_pose(
                        measured_pose,
                        center_xy=(base_place[0], base_place[1]),
                    )
                else:
                    current_zone = tolerance_engine.classify_pose(
                        proposed_place_pose,
                        center_xy=(base_place[0], base_place[1]),
                    )
                vision_controller.log_shadow_pose(
                    context=f"ZONE_INIT lvl={current_stack_level}",
                    pose=measured_pose if tcp_after is not None else proposed_place_pose,
                    authoritative_zone=current_zone,
                    debug_enabled=_is_debug_enabled(),
                )
                vision_controller.log_drop_tracking(
                    stack_level=current_stack_level,
                    debug_enabled=_is_debug_enabled(),
                )

                tcp_before_x = tcp_before[0] if tcp_before is not None else float("nan")
                tcp_before_y = tcp_before[1] if tcp_before is not None else float("nan")
                tcp_after_x = tcp_after[0] if tcp_after is not None else float("nan")
                tcp_after_y = tcp_after[1] if tcp_after is not None else float("nan")
                if _is_debug_enabled():
                    info(
                        "CONTROL",
                        f"[ZONE_INIT] lvl={current_stack_level} nominal_xy=({base_place[0]:.2f},{base_place[1]:.2f}) "
                        f"drift_xy=({ex:.2f},{ey:.2f}) proposed_xy=({proposed_place_pose[0]:.2f},{proposed_place_pose[1]:.2f}) "
                        f"tcp_before=({tcp_before_x:.2f},{tcp_before_y:.2f}) tcp_after=({tcp_after_x:.2f},{tcp_after_y:.2f}) zone={current_zone}",
                    )
                    if tcp_after is not None:
                        ex_tcp = tcp_after[0] - base_place[0]
                        ey_tcp = tcp_after[1] - base_place[1]
                        info(
                            "CONTROL",
                            f"[POSE] lvl={current_stack_level} tcp=({tcp_after[0]:.2f},{tcp_after[1]:.2f},{tcp_after[2]:.2f}) "
                            f"err=({ex_tcp:.2f},{ey_tcp:.2f}) zone={current_zone}",
                        )
                    else:
                        info(
                            "CONTROL",
                            f"[POSE] lvl={current_stack_level} tcp=(nan,nan,nan) err=(nan,nan) zone={current_zone}",
                        )
                current_zone_stack_level = current_stack_level
                _push_zone_if_needed(force=True)
                _send_line_to_unity(f"DECISION_READY {decision_seq}")
            else:
                proposed_place_pose = None
                proposed_place_stack_level = None
                current_zone = "GREEN"
                current_zone_stack_level = None
                _push_zone_if_needed(force=True)
                _send_line_to_unity(f"DECISION_READY {decision_seq}")
            _emit_ready_prompt(current_stack_level)
            block_attempt_start_ts = time.time()

    return True


# --- Robot actions ---
# NOTE: Robot action functions (do_home, do_drop, do_nudge_xy, etc.) are now
# imported from actions.py module along with their SystemHandles dependency.


# --- 3.5. COMMAND HANDLER ---
def handle_command(cmd_str: str, source: str) -> None:
    """
    Process a single incoming command string.
    Uses globals: STATE, robot_armed, gripper_connected, controller_busy,
                  current_pick_index, current_stack_level, stacking_enabled, target_stack_count
    
    source: Log prefix, e.g., "CONTROL", "ADMIN"
    """
    global STATE, robot_armed, gripper_connected, controller_busy
    global current_pick_index, current_stack_level, stacking_enabled, target_stack_count
    global participant_name, session_id, tower_attempt_start_ts, run_start_time, block_attempt_start_ts, drop_committed_this_window, decision_seq, holding_block, current_session_token
    global proposed_place_pose, proposed_place_stack_level
    global current_zone, current_zone_stack_level, green_place_streak, combo_active
    global run_id, run_finalized, current_run_seed, DEBUG_ENABLED, _last_ready_level_printed
    global active_pick_target_id, active_pick_marker_target_id, active_pick_claim_target_id
    global placed_pick_target_ids, picked_marker_target_ids, expected_workbench_brick_count
    global LEADERBOARD_MODE, OFFICIAL_EVENT_ID
    global committed_stack_level, pending_commit_level, pending_commit_deadline, completion_finalize_pending, completion_end_mono
    global last_finalized_run_id, last_finalized_mode, last_finalized_session_id, last_finalized_participant_name
    global _last_nudge_t

    _promote_pending_commit_if_ready()

    # Normalize command
    if cmd_str == "COMMIT":
        cmd_str = "DROP"

    module = _module_for_source(source)
    if _is_debug_enabled():
        console_info(module, f"[{source}] Received: {cmd_str}   (STATE={STATE.name})  (ARMED={robot_armed})  (GRIPPER={gripper_connected})")

    # Raw session commands (before parse_event)
    upper_cmd = cmd_str.upper()
    if upper_cmd == "NAME" or upper_cmd.startswith("NAME "):
        # Accept: NAME <free text>
        name_value = cmd_str[4:].strip()
        if not name_value:
            warn(module, "[GATE] NAME command requires free text (e.g., NAME Alice).")
            return
        participant_name = name_value
        current_stack_level = 0
        current_pick_index = 0
        _reset_pick_runtime_cache()
        expected_workbench_brick_count = _prompt_workbench_brick_count()
        target_stack_count = _effective_target_stack_count()
        session_id = f"{int(time.time())}-{uuid4().hex[:8]}"
        tower_attempt_start_ts = None
        run_start_time = None
        block_attempt_start_ts = None
        drop_committed_this_window = False
        holding_block = False
        controller_busy = False
        current_session_token = uuid4()
        proposed_place_pose = None
        proposed_place_stack_level = None
        current_zone = "GREEN"
        current_zone_stack_level = None
        green_place_streak = 0
        if combo_active:
            send_boost_end()
        combo_active = False
        handles.combo_active = combo_active
        send_boost_state(0, False)
        run_id = None
        run_finalized = False
        current_run_seed = None
        cfg.DRIFT_RUNTIME_RUN_SEED = None
        cfg.DRIFT_RUNTIME_PARTICIPANT = participant_name
        _last_ready_level_printed = None
        _clear_score_commit_state()
        _reset_quality_tracking()
        _reset_drift_scale_for_run("NAME")
        vision_controller.reset_pick_tracking_memory()
        _write_vision_assist_state()
        _sync_json_log_context()
        _send_line_to_unity("NAME_SET")
        log_event("EVENT_NAME_SET", participant=participant_name, source=source)
        if expected_workbench_brick_count is not None:
            console_info(
                "CONTROL",
                f"Workbench brick count set to {expected_workbench_brick_count}. Ready to START.",
                essential=True,
            )
            return
        console_info("CONTROL", f"Participant set: {participant_name}. Ready to START.", essential=True)
        return

    if upper_cmd == "FIXSCORE" or upper_cmd.startswith("FIXSCORE "):
        if source not in {"ADMIN", "FACILITATOR"}:
            warn(module, "[LEADERBOARD] FIXSCORE is restricted to ADMIN or FACILITATOR sources.")
            return
        parts = cmd_str.strip().split(maxsplit=2)
        if len(parts) < 2:
            warn(module, "[LEADERBOARD] Usage: FIXSCORE <n> [optional reason]")
            return
        try:
            new_score = int(parts[1])
        except ValueError:
            warn(module, "[LEADERBOARD] FIXSCORE requires an integer score.")
            return
        if new_score < 0:
            warn(module, "[LEADERBOARD] FIXSCORE rejects negative scores.")
            return
        target_height = int(getattr(cfg, "TOWER_LEVELS", 7))
        if new_score > target_height:
            warn(module, f"[LEADERBOARD] FIXSCORE rejects scores above target height ({target_height}).")
            return
        if not last_finalized_run_id or not last_finalized_mode:
            warn(module, "[LEADERBOARD] No finalized run is available for FIXSCORE.")
            return
        reason = parts[2].strip() if len(parts) > 2 else ""
        try:
            result = lb.override_run_score(
                mode=last_finalized_mode,
                run_id=last_finalized_run_id,
                new_final_height=new_score,
            )
            original = result["original"]
            updated = result["updated"]
            lb.append_score_override_audit(
                mode=last_finalized_mode,
                run_id=last_finalized_run_id,
                session_id=original.get("session_id", last_finalized_session_id),
                participant_name=original.get("participant_name", last_finalized_participant_name),
                old_final_height=int(original.get("final_height", 0)),
                new_final_height=int(updated.get("final_height", 0)),
                reason=reason,
                source=source,
            )
            console_emit(
                f"[LEADERBOARD] FIXSCORE run_id={last_finalized_run_id} {int(original.get('final_height', 0))} -> {int(updated.get('final_height', 0))}",
                tag="FACILITATOR",
                level="INFO",
                module=module,
                allow_in_quiet=True,
            )
        except Exception as e:
            warn(module, f"[LEADERBOARD] FIXSCORE failed: {e}")
        return

    if upper_cmd == "TUMBLE":
        # Preempt any in-flight or queued stack continuation immediately.
        tumble_received_mono = time.monotonic()
        current_session_token = uuid4()

        waited = False
        while controller_busy:
            if not waited:
                info(module, "[TUMBLE] Controller busy; waiting to execute tumble...")
                waited = True
            time.sleep(0.05)

        if waited:
            info(module, "[TUMBLE] Controller free; executing queued tumble.")

        controller_busy = True
        # Run summary before any state/session reset
        official_score = _resolve_official_score(tumble_received_mono)
        blocks_placed = official_score
        run_quality_score, run_green_count, run_yellow_count, run_red_count = _snapshot_quality_tracking()
        run_time_s = lb.emit_run_summary(
            "TUMBLE",
            run_start_time=run_start_time,
            participant_name=participant_name,
            current_stack_level=official_score,
            quality_score=run_quality_score,
            end_time_mono=tumble_received_mono,
        )
        if lb.finalize_run(
            "TUMBLE",
            ctx=lb_ctx,
            run_id=run_id,
            session_id=session_id,
            participant_name=participant_name,
            current_stack_level=official_score,
            run_start_time=run_start_time,
            quality_score=run_quality_score,
            green_count=run_green_count,
            yellow_count=run_yellow_count,
            red_count=run_red_count,
            already_finalized=run_finalized,
            end_time_mono=tumble_received_mono,
        ):
            run_finalized = True
            last_finalized_run_id = run_id
            last_finalized_mode = normalize_leaderboard_mode(lb_ctx.mode)
            last_finalized_session_id = session_id
            last_finalized_participant_name = participant_name
        _send_line_to_unity("RUN_FAIL TUMBLE")

        log_event(
            "EVENT_RUN_SUMMARY",
            participant=participant_name,
            blocks_placed=blocks_placed,
            run_time_s=run_time_s,
            quality_score=run_quality_score,
            green_count=run_green_count,
            yellow_count=run_yellow_count,
            red_count=run_red_count,
            source=source,
        )

        try:
            info(module, f"[TUMBLE] enter state={STATE.name} holding_block_flag={holding_block}")
            detected_holding = actions.execute_tumble_sequence(handles, fallback_holding=holding_block)
            info(module, f"[TUMBLE] completed detected_holding={detected_holding}")
            log_event(
                "EVENT_TUMBLE",
                source=source,
                state_before=STATE.name,
                holding_block_flag=holding_block,
                holding_detected=detected_holding,
            )
        except Exception as e:
            warn(module, f"[{source}] TUMBLE failed: {e}")
            log_event("EVENT_TUMBLE_RESET_ERROR", source=source, error=str(e))
        finally:
            controller_busy = False

        STATE = State.IDLE
        current_pick_index = 0
        current_stack_level = 0
        _reset_pick_runtime_cache()
        holding_block = False
        participant_name = None
        tower_attempt_start_ts = None
        run_start_time = None
        block_attempt_start_ts = None
        drop_committed_this_window = False
        proposed_place_pose = None
        proposed_place_stack_level = None
        current_zone = "GREEN"
        current_zone_stack_level = None
        green_place_streak = 0
        if combo_active:
            send_boost_end()
        combo_active = False
        handles.combo_active = combo_active
        send_boost_state(0, False)
        run_id = None
        run_finalized = False
        current_run_seed = None
        cfg.DRIFT_RUNTIME_RUN_SEED = None
        cfg.DRIFT_RUNTIME_PARTICIPANT = ""
        _last_ready_level_printed = None
        _clear_score_commit_state()
        _reset_quality_tracking()
        _sync_json_log_context()
        if vr_connected:
            console_emit("Waiting for participant name...", tag="PROMPT", level="INFO", module="CONTROL", allow_in_quiet=True)
        return

    if upper_cmd == "MODE SHOW":
        console_emit(f"[LEADERBOARD] MODE={LEADERBOARD_MODE} EVENT={OFFICIAL_EVENT_ID}", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd == "MODE DEV":
        LEADERBOARD_MODE = "DEV"
        lb_ctx.mode = LEADERBOARD_MODE
        lb.save_leaderboard_mode(lb_ctx)
        _sync_json_log_context()
        console_emit(f"[LEADERBOARD] MODE set to DEV (event_id={OFFICIAL_EVENT_ID})", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd == "MODE OFFICIAL":
        LEADERBOARD_MODE = "OFFICIAL"
        lb_ctx.mode = LEADERBOARD_MODE
        lb.save_leaderboard_mode(lb_ctx)
        _sync_json_log_context()
        console_emit(f"[LEADERBOARD] MODE set to OFFICIAL (event_id={OFFICIAL_EVENT_ID})", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd == "DEBUG SHOW":
        state = "ON" if _is_debug_enabled() else "OFF"
        console_emit(f"[DEBUG] DEBUG={state}", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd == "DEBUG ON":
        _set_debug_enabled(True)
        console_emit("[DEBUG] DEBUG=ON", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd == "DEBUG OFF":
        _set_debug_enabled(False)
        console_emit("[DEBUG] DEBUG=OFF", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd.startswith("EVENT "):
        new_event_id = cmd_str[6:].strip()
        if not new_event_id:
            warn(module, "[LEADERBOARD] EVENT command requires a non-empty id (e.g., EVENT ARC2026).")
            return
        OFFICIAL_EVENT_ID = new_event_id
        lb_ctx.official_event_id = OFFICIAL_EVENT_ID
        lb.save_leaderboard_mode(lb_ctx)
        info(module, f"[LEADERBOARD] EVENT set to {OFFICIAL_EVENT_ID}")
        return

    # SAFE_RESET short-circuit (before parsing)
    if cmd_str == "SAFE_RESET":
        try:
            info(module, f"[{source}] SAFE_RESET requested.")
            actions.do_home(handles)
            actions.do_grip_open(handles)
            STATE = State.IDLE
            current_pick_index = 0
            current_stack_level = 0
            _reset_pick_runtime_cache()
            drop_committed_this_window = False
            proposed_place_pose = None
            proposed_place_stack_level = None
            current_zone = "GREEN"
            current_zone_stack_level = None
            _clear_score_commit_state()
        except Exception as e:
            warn(module, f"[{source}] SAFE_RESET failed: {e}")
        return

    # Parse command into event using state machine
    try:
        event, payload = parse_event(cmd_str)
    except ValueError as e:
        warn(module, f"[CMD] Reject: {e}")
        return

    drop_token = None
    fix_token = None
    if event == Event.DROP:
        parts = cmd_str.strip().split()
        if len(parts) < 2:
            send_nack("DROP", "BAD_FORMAT")
            info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=DROP missing decision token")
            return
        try:
            drop_token = int(parts[1])
        except ValueError:
            send_nack("DROP", "BAD_FORMAT")
            info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=DROP invalid decision token")
            return

    if event == Event.FIX:
        parts = cmd_str.strip().split()
        if len(parts) < 2:
            info(module, "[FIX][PY] send NACK FIX reason=BAD_FORMAT")
            send_nack("FIX", "BAD_FORMAT")
            info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=FIX missing decision token")
            return
        try:
            fix_token = int(parts[1])
        except ValueError:
            info(module, "[FIX][PY] send NACK FIX reason=BAD_FORMAT")
            send_nack("FIX", "BAD_FORMAT")
            info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=FIX invalid decision token")
            return

    if event == Event.FIX:
        info(module, f"[FIX][PY] recv FIX (STATE={STATE.name})")

    # Session participant gate
    if (participant_name is None or participant_name.strip() == "") and event in {Event.START_STACK, Event.VISION_RETRY, Event.DROP, Event.FIX, Event.NUDGE_XY, Event.NUDGE_YAW}:
        warn(module, f"[GATE] Participant name required. Rejecting: {cmd_str}")
        if event == Event.START_STACK:
            send_nack("START", "NO_NAME")
        if event == Event.DROP:
            send_nack("DROP", "NO_NAME")
        if event == Event.FIX:
            info(module, "[FIX][PY] send NACK FIX reason=NO_NAME")
            send_nack("FIX", "NO_NAME")
        log_event("EVENT_REJECT_NO_NAME", cmd=cmd_str, source=source)
        return

    if event == Event.FIX and STATE != State.WAITING_FOR_DECISION:
        info(module, "[FIX][PY] send NACK FIX reason=BAD_STATE")
        send_nack("FIX", "BAD_STATE")
        info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=FIX requires WAITING_FOR_DECISION")
        return

    if event == Event.FIX and fix_token != decision_seq:
        info(module, "[FIX][PY] send NACK FIX reason=STALE")
        send_nack("FIX", "STALE")
        info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=FIX stale token (got={fix_token}, expected={decision_seq})")
        return

    # Gate transition using state machine
    result = step(STATE, event)
    if not result.allowed:
        if event == Event.START_STACK:
            send_nack("START", "BAD_STATE")
        if event == Event.DROP:
            send_nack("DROP", "BAD_STATE")
        if event == Event.FIX:
            info(module, "[FIX][PY] send NACK FIX reason=BAD_STATE")
            send_nack("FIX", "BAD_STATE")
        info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason={result.reason}")
        return

    if event == Event.DROP and STATE not in {State.WAITING_FOR_DECISION, State.NUDGE}:
        send_nack("DROP", "BAD_STATE")
        info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=DROP requires WAITING_FOR_DECISION or NUDGE")
        return

    if event == Event.DROP and drop_token != decision_seq:
        send_nack("DROP", "STALE")
        info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=DROP stale token (got={drop_token}, expected={decision_seq})")
        return

    if event == Event.DROP and STATE in {State.WAITING_FOR_DECISION, State.NUDGE} and drop_committed_this_window:
        send_nack("DROP", "DUPLICATE")
        info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason=DROP duplicate (latch={drop_committed_this_window})")
        return

    # Safety gates BEFORE committing state transition
    # AUTO_RECOVER is exempt: it's the recovery path from FAULT and may be 
    # issued when disarmed, since it's responsible for re-enabling the robot.
    if event != Event.AUTO_RECOVER and event in {Event.HOME, Event.FIX, Event.NUDGE_XY, Event.NUDGE_YAW, Event.DROP, Event.START_STACK, Event.VISION_RETRY}:
        if not robot_armed:
            if event == Event.START_STACK:
                send_nack("START", "NOT_ARMED")
            if event == Event.DROP:
                send_nack("DROP", "NOT_ARMED")
            if event == Event.FIX:
                info(module, "[FIX][PY] send NACK FIX reason=NOT_ARMED")
                send_nack("FIX", "NOT_ARMED")
            warn(module, "[GATE] Robot not armed. Ignoring motion command.")
            return

    def _handle_vision_pick_unavailable(reason: str) -> None:
        global STATE
        warn(module, f"[VISION] pick unavailable: {reason}")
        warn(
            module,
            "[VISION] Unable to continue vision pickup. Switch to deterministic mode for remaining hardcoded P-slots. "
            "Use Robotics/perception/deterministic_remaining_pick_place.py to inspect or execute the remaining unvisited pickup points.",
        )
        _send_line_to_unity("VISION_STATUS FAIL")
        STATE = State.WAITING_FOR_REPOSITION
        info(module, "[SM] -> WAITING_FOR_REPOSITION (VISION pick unavailable)")

    if event in {Event.GRIP_OPEN, Event.GRIP_CLOSE, Event.GRIP_TOGGLE}:
        if not gripper_connected:
            warn(module, "[GATE] Gripper not connected. Ignoring gripper command.")
            return

    # Apply state transition only after passing gates
    # AUTO_RECOVER is special: state is decided by real recovery result, not transition table.
    prev = STATE
    if event != Event.AUTO_RECOVER:
        STATE = result.next_state
        if STATE != prev:
            info(module, f"[SM] {prev.name} -> {STATE.name} on {event.name}")

    # Execute side-effects based on event
    if event == Event.HOME:
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            # If recovering from FAULT, call recovery routine first
            if prev == State.FAULT:
                info(module, "[RECOVERY] HOME requested from FAULT. Starting recovery sequence...")
                actions.recover_from_fault(handles)
                mode_after = handles.robot.robot_mode()
                if mode_after not in (9, 11):
                    info(module, f"[RECOVERY] Recovery successful during HOME. Robot mode: {mode_after}")
                    robot_armed = True
                else:
                    warn(module, f"[RECOVERY] Recovery incomplete during HOME. Robot still in fault mode: {mode_after}")
                    robot_armed = False
            else:
                # Normal home
                actions.do_home(handles)
                # Immediate RobotMode check after motion
                m = handles.robot.robot_mode()
                if m in (9, 11):
                    console_emit(f"[FAULT] RobotMode={m} -> entering FAULT", tag="FAULT", level="WARN", module=module, allow_in_quiet=True)
                    fault_result = step(STATE, Event.FAULT)
                    if fault_result.allowed:
                        STATE = fault_result.next_state
                        info(module, f"[SM] -> {STATE.name} (FAULT)")
                    return
        finally:
            controller_busy = False

    elif event in {Event.START_STACK, Event.VISION_RETRY}:
        if controller_busy:
            if event == Event.START_STACK:
                send_nack("START", "BUSY")
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            # Bounds checking
            if current_pick_index >= len(cfg.PICK_SEQUENCE):
                if event == Event.START_STACK:
                    send_nack("START", "BAD_STATE")
                warn(module, "[STACK] No more blocks in PICK_SEQUENCE. Ignoring START.")
                return

            if current_stack_level >= target_stack_count:
                if event == Event.START_STACK:
                    send_nack("START", "BAD_STATE")
                warn(module, "[STACK] Tower full. Ignoring START.")
                return

            if event == Event.START_STACK and current_stack_level == 0:
                retrain_ok = _retrain_pick_ml_before_run()
                if not retrain_ok:
                    warn(
                        "VISION",
                        "[VISION] Pickup ML retraining did not complete cleanly. Starting anyway with the existing model.",
                    )

            # Run start timing (monotonic) when a new run actually begins
            if current_stack_level == 0:
                _last_ready_level_printed = None
                current_session_token = uuid4()
                tower_attempt_start_ts = time.monotonic()
                run_start_time = tower_attempt_start_ts
                run_id = uuid4().hex
                run_finalized = False
                _clear_score_commit_state()
                _reset_quality_tracking()
                green_place_streak = 0
                if combo_active:
                    send_boost_end()
                combo_active = False
                handles.combo_active = combo_active
                send_boost_state(0, False)
                current_run_seed = _generate_runtime_run_seed()
                cfg.DRIFT_RUNTIME_RUN_SEED = current_run_seed
                cfg.DRIFT_RUNTIME_PARTICIPANT = participant_name or ""
                _reset_drift_scale_for_run("START")
                if _is_debug_enabled():
                    info(
                        "DRIFT",
                        f"[RUN] START participant={participant_name or 'UNKNOWN'} RUN_SEED={current_run_seed}",
                    )
                log_event(
                    "EVENT_RUN_START_METADATA",
                    source=source,
                    participant_name=participant_name,
                    runtime_run_seed=current_run_seed,
                    drift_run_seed_baseline=int(getattr(cfg, "DRIFT_RUN_SEED", 0)),
                    drift_scale_default=float(getattr(cfg, "DRIFT_SCALE_DEFAULT", cfg.DRIFT_SCALE)),
                    drift_scale_at_start=float(cfg.DRIFT_SCALE),
                )
                _sync_json_log_context()
                actions.ensure_gripper_open_at_run_start(handles)

            if event == Event.START_STACK:
                send_ack("START")

            # Execute pick sequence
            my_token = current_session_token
            try:
                pick_target_id, pick_mode_override = _resolve_pick_target_for_cycle()
            except RuntimeError as e:
                _clear_active_pick_targets()
                if VISION_MODE_ENABLED:
                    _handle_vision_pick_unavailable(str(e))
                    return
                raise
            _set_active_pick_targets(pick_target_id, pick_mode_override)
            if str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower() == "vision":
                console_info(
                    "CONTROL",
                    (
                        f"[VISION] selected_source={pick_target_id} claimed_pick_slot={active_pick_claim_target_id}"
                        if pick_mode_override is None
                        else f"[VISION] deterministic_fallback_source={pick_target_id} claimed_pick_slot={active_pick_claim_target_id}"
                    ),
                    essential=_is_debug_enabled(),
                )
            handles.combo_active = combo_active
            vision_controller.log_pick_tracking(
                target_id=pick_target_id,
                stack_level=current_stack_level,
                debug_enabled=_is_debug_enabled(),
            )
            try:
                actions.execute_pick_sequence(
                    handles,
                    pick_target_id,
                    current_stack_level,
                    pick_mode_override=pick_mode_override,
                )
            except actions.PickPoseUnavailableError as e:
                _clear_active_pick_targets()
                if VISION_MODE_ENABLED:
                    fallback_target_id, pick_mode_override = _deterministic_remaining_pick_fallback_target(e.reason)
                    if fallback_target_id is None:
                        _handle_vision_pick_unavailable(e.reason)
                        return
                    pick_target_id = fallback_target_id
                    _set_active_pick_targets(pick_target_id, pick_mode_override)
                    console_info(
                        "CONTROL",
                        f"[VISION] deterministic_fallback_source={pick_target_id} claimed_pick_slot={active_pick_claim_target_id}",
                        essential=True,
                    )
                    handles.combo_active = combo_active
                    vision_controller.log_pick_tracking(
                        target_id=pick_target_id,
                        stack_level=current_stack_level,
                        debug_enabled=_is_debug_enabled(),
                    )
                    try:
                        actions.execute_pick_sequence(
                            handles,
                            pick_target_id,
                            current_stack_level,
                            pick_mode_override=pick_mode_override,
                        )
                    except actions.PickPoseUnavailableError as fallback_error:
                        _handle_vision_pick_unavailable(f"{e.reason}->deterministic_fallback:{fallback_error.reason}")
                        return
                else:
                    raise
            try:
                pick_pose = cfg.pick_target_pose(pick_target_id)
                hover_pose = cfg.pick_target_hover_pose(pick_target_id)
            except Exception:
                pick_pose = None
                hover_pose = None
            _log_inferred_block_state(
                "block_inferred_picked",
                target_id=pick_target_id,
                stack_level=int(current_stack_level),
                source="vision_assist_pick" if str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower() == "vision" else "deterministic_pick",
                pick_pose=pick_pose,
                hover_pose=hover_pose,
            )
            vision_controller.forget_pick_target(pick_target_id)
            # Immediate RobotMode check after motion
            m = handles.robot.robot_mode()
            if m in (9, 11):
                console_emit(f"[FAULT] RobotMode={m} -> entering FAULT", tag="FAULT", level="WARN", module=module, allow_in_quiet=True)
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return

            # Emit internal progression event
            if my_token != current_session_token:
                return
            result2 = step(STATE, Event.PICK_COMPLETE)
            if result2.allowed:
                STATE = result2.next_state
                info(module, f"[SM] -> {STATE.name} (PICK_COMPLETE)")

                # Immediately move to tower hover
                if STATE == State.MOVING_TO_TOWER_HOVER:
                    base_place = cfg.tower_place_pose(current_stack_level)
                    proposed_place_pose = drift_engine.inject_drift(
                        base_place,
                        current_stack_level,
                        run_seed=current_run_seed,
                        participant=participant_name,
                    )
                    proposed_place_stack_level = current_stack_level
                    log_event(
                        "EVENT_DRIFT_LEVEL",
                        source=source,
                        stack_level=current_stack_level,
                        drift_dx_mm=round(proposed_place_pose[0] - base_place[0], 3),
                        drift_dy_mm=round(proposed_place_pose[1] - base_place[1], 3),
                        runtime_run_seed=current_run_seed,
                        drift_run_seed_baseline=int(getattr(cfg, "DRIFT_RUN_SEED", 0)),
                    )
                    handles.combo_active = combo_active
                    actions.move_to_tower_hover(
                        handles,
                        current_stack_level,
                        target_xy=(proposed_place_pose[0], proposed_place_pose[1]),
                        target_id=pick_target_id,
                    )
                    if not _handle_post_tower_hover(module, my_token):
                        return
        finally:
            controller_busy = False

    elif event == Event.FIX:
        # no motion yet; just entering NUDGE
        info(module, "[FIX][PY] send ACK FIX")
        send_ack("FIX")
        pass

    elif event == Event.NUDGE_XY:
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        now = time.time()
        cooldown_s = getattr(cfg, "NUDGE_COOLDOWN_S", 0.20)
        if (now - _last_nudge_t) < cooldown_s:
            if _is_debug_enabled():
                info(module, f"[GATE] NUDGE_XY ignored: cooldown ({cooldown_s:.2f}s)")
            return
        _last_nudge_t = now
        controller_busy = True
        try:
            requested_dx = float(payload["dx"])
            requested_dy = float(payload["dy"])

            max_offset_mm = float(getattr(cfg, "NUDGE_MAX_OFFSET_MM", 10.0))
            nominal_place_pose = cfg.tower_place_pose(current_stack_level)

            if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                current_pose_for_clamp = proposed_place_pose
            else:
                current_pose_for_clamp = nominal_place_pose

            current_offset_x = current_pose_for_clamp[0] - nominal_place_pose[0]
            current_offset_y = current_pose_for_clamp[1] - nominal_place_pose[1]

            target_offset_x = current_offset_x + requested_dx
            target_offset_y = current_offset_y + requested_dy

            clamped_target_offset_x = max(-max_offset_mm, min(max_offset_mm, target_offset_x))
            clamped_target_offset_y = max(-max_offset_mm, min(max_offset_mm, target_offset_y))

            applied_dx = clamped_target_offset_x - current_offset_x
            applied_dy = clamped_target_offset_y - current_offset_y

            if _is_debug_enabled() and (applied_dx != requested_dx or applied_dy != requested_dy):
                warn(
                    module,
                    f"[GATE] NUDGE_XY clamped: req=({requested_dx:.3f},{requested_dy:.3f}) "
                    f"applied=({applied_dx:.3f},{applied_dy:.3f}) max=±{max_offset_mm:.1f}mm",
                )

            actions.do_nudge_xy(handles, applied_dx, applied_dy)
            if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                before_x = proposed_place_pose[0]
                before_y = proposed_place_pose[1]
                proposed_place_pose = (
                    proposed_place_pose[0] + applied_dx,
                    proposed_place_pose[1] + applied_dy,
                    proposed_place_pose[2],
                    proposed_place_pose[3],
                    proposed_place_pose[4],
                    proposed_place_pose[5],
                )
            else:
                before_x = nominal_place_pose[0]
                before_y = nominal_place_pose[1]
                proposed_place_pose = (
                    nominal_place_pose[0] + applied_dx,
                    nominal_place_pose[1] + applied_dy,
                    nominal_place_pose[2],
                    nominal_place_pose[3],
                    nominal_place_pose[4],
                    nominal_place_pose[5],
                )
                proposed_place_stack_level = current_stack_level

            if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                tcp_pose = handles.robot.get_tcp_pose()
                if tcp_pose is not None:
                    measured_pose = (
                        tcp_pose[0],
                        tcp_pose[1],
                        proposed_place_pose[2],
                        proposed_place_pose[3],
                        proposed_place_pose[4],
                        proposed_place_pose[5],
                    )
                    current_zone = tolerance_engine.classify_pose(
                        measured_pose,
                        center_xy=(nominal_place_pose[0], nominal_place_pose[1]),
                    )
                else:
                    current_zone = tolerance_engine.classify_pose(
                        proposed_place_pose,
                        center_xy=(nominal_place_pose[0], nominal_place_pose[1]),
                    )
                vision_controller.log_shadow_pose(
                    context=f"NUDGE_XY lvl={current_stack_level}",
                    pose=measured_pose if tcp_pose is not None else proposed_place_pose,
                    authoritative_zone=current_zone,
                    debug_enabled=_is_debug_enabled(),
                )
                ex = proposed_place_pose[0] - nominal_place_pose[0]
                ey = proposed_place_pose[1] - nominal_place_pose[1]
                if _is_debug_enabled():
                    info(
                        "CONTROL",
                        f"[NUDGE] lvl={current_stack_level} req=({requested_dx:.2f},{requested_dy:.2f}) "
                        f"appl=({applied_dx:.2f},{applied_dy:.2f}) before=({before_x:.2f},{before_y:.2f}) "
                        f"after=({proposed_place_pose[0]:.2f},{proposed_place_pose[1]:.2f}) err=({ex:.2f},{ey:.2f}) zone={current_zone}",
                    )
                    if tcp_pose is not None:
                        ex_tcp = tcp_pose[0] - nominal_place_pose[0]
                        ey_tcp = tcp_pose[1] - nominal_place_pose[1]
                        info(
                            "CONTROL",
                            f"[POSE] lvl={current_stack_level} tcp=({tcp_pose[0]:.2f},{tcp_pose[1]:.2f},{tcp_pose[2]:.2f}) "
                            f"err=({ex_tcp:.2f},{ey_tcp:.2f}) zone={current_zone}",
                        )
                    else:
                        info(
                            "CONTROL",
                            f"[POSE] lvl={current_stack_level} tcp=(nan,nan,nan) err=(nan,nan) zone={current_zone}",
                        )
                current_zone_stack_level = current_stack_level
                _push_zone_if_needed(force=False)
            # Immediate RobotMode check after motion
            m = handles.robot.robot_mode()
            if m in (9, 11):
                console_emit(f"[FAULT] RobotMode={m} -> entering FAULT", tag="FAULT", level="WARN", module=module, allow_in_quiet=True)
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return
            
        finally:
            controller_busy = False

    elif event == Event.NUDGE_YAW:
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        now = time.time()
        cooldown_s = getattr(cfg, "NUDGE_COOLDOWN_S", 0.20)
        if (now - _last_nudge_t) < cooldown_s:
            if _is_debug_enabled():
                info(module, f"[GATE] NUDGE_YAW ignored: cooldown ({cooldown_s:.2f}s)")
            return
        _last_nudge_t = now
        controller_busy = True
        try:
            actions.do_nudge_yaw(handles, payload["dtheta"])
            if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                proposed_place_pose = (
                    proposed_place_pose[0],
                    proposed_place_pose[1],
                    proposed_place_pose[2],
                    proposed_place_pose[3],
                    proposed_place_pose[4],
                    proposed_place_pose[5] + payload["dtheta"],
                )
                nominal_place_pose = cfg.tower_place_pose(current_stack_level)
                tcp_pose = handles.robot.get_tcp_pose()
                if tcp_pose is not None:
                    measured_pose = (
                        tcp_pose[0],
                        tcp_pose[1],
                        proposed_place_pose[2],
                        proposed_place_pose[3],
                        proposed_place_pose[4],
                        proposed_place_pose[5],
                    )
                    current_zone = tolerance_engine.classify_pose(
                        measured_pose,
                        center_xy=(nominal_place_pose[0], nominal_place_pose[1]),
                    )
                else:
                    current_zone = tolerance_engine.classify_pose(
                        proposed_place_pose,
                        center_xy=(nominal_place_pose[0], nominal_place_pose[1]),
                    )
                vision_controller.log_shadow_pose(
                    context=f"NUDGE_YAW lvl={current_stack_level}",
                    pose=measured_pose if tcp_pose is not None else proposed_place_pose,
                    authoritative_zone=current_zone,
                    debug_enabled=_is_debug_enabled(),
                )
                ex = proposed_place_pose[0] - nominal_place_pose[0]
                ey = proposed_place_pose[1] - nominal_place_pose[1]
                if _is_debug_enabled():
                    info(
                        "CONTROL",
                        f"[NUDGE_YAW] lvl={current_stack_level} yaw={payload['dtheta']:.2f} "
                        f"proposed=({proposed_place_pose[0]:.2f},{proposed_place_pose[1]:.2f}) err=({ex:.2f},{ey:.2f}) zone={current_zone}",
                    )
                current_zone_stack_level = current_stack_level
                _push_zone_if_needed(force=False)
            # Immediate RobotMode check after motion
            m = handles.robot.robot_mode()
            if m in (9, 11):
                console_emit(f"[FAULT] RobotMode={m} -> entering FAULT", tag="FAULT", level="WARN", module=module, allow_in_quiet=True)
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return
            _send_line_to_unity("NUDGE_DONE")
        finally:
            controller_busy = False

    elif event == Event.DROP:
        decision_time = None
        if block_attempt_start_ts is not None:
            decision_time = time.time() - block_attempt_start_ts
        log_event("EVENT_DROP_RECEIVED", source=source, decision_time=decision_time)
        block_attempt_start_ts = None

        if controller_busy:
            send_nack("DROP", "BUSY")
            warn(module, "[GATE] Controller busy.")
            return
        drop_committed_this_window = True
        controller_busy = True
        try:
            send_ack("DROP")
            my_token = current_session_token
            placed_target_id = _current_pick_target_id()
            zone_at_commit = "UNKNOWN"
            if current_stack_level is not None and current_stack_level >= 0:
                nominal_place_pose = cfg.tower_place_pose(current_stack_level)
                tcp_pose = handles.robot.get_tcp_pose()
                if _is_debug_enabled():
                    if tcp_pose is not None:
                        ex_tcp = tcp_pose[0] - nominal_place_pose[0]
                        ey_tcp = tcp_pose[1] - nominal_place_pose[1]
                        info(
                            "CONTROL",
                            f"[POSE] lvl={current_stack_level} tcp=({tcp_pose[0]:.2f},{tcp_pose[1]:.2f},{tcp_pose[2]:.2f}) "
                            f"err=({ex_tcp:.2f},{ey_tcp:.2f}) zone={current_zone}",
                        )
                    else:
                        info(
                            "CONTROL",
                            f"[POSE] lvl={current_stack_level} tcp=(nan,nan,nan) err=(nan,nan) zone={current_zone}",
                        )
            # Attempt placement with error handling
            try:
                handles.combo_active = combo_active
                if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                    actions.complete_place_sequence(
                        handles,
                        current_stack_level,
                        place_pose=proposed_place_pose,
                        perform_neutral_exit=False,
                        target_id=placed_target_id,
                    )
                else:
                    actions.complete_place_sequence(
                        handles,
                        current_stack_level,
                        perform_neutral_exit=False,
                        target_id=placed_target_id,
                    )
                # Immediate RobotMode check after motion
                m = handles.robot.robot_mode()
                if m in (9, 11):
                    green_place_streak = 0
                    if combo_active:
                        send_boost_end()
                        combo_active = False
                        handles.combo_active = combo_active
                        if _is_debug_enabled():
                            warn("COMBO", "combo ended")
                    send_boost_state(0, False)
                    console_emit(f"[FAULT] RobotMode={m} -> entering FAULT", tag="FAULT", level="WARN", module=module, allow_in_quiet=True)
                    fault_result = step(STATE, Event.FAULT)
                    if fault_result.allowed:
                        STATE = fault_result.next_state
                        info(module, f"[SM] -> {STATE.name} (FAULT)")
                    return

                if current_zone_stack_level == current_stack_level:
                    zone_at_commit = current_zone
                elif proposed_place_pose is not None:
                    nominal_place_pose = cfg.tower_place_pose(current_stack_level)
                    zone_at_commit = tolerance_engine.classify_pose(
                        proposed_place_pose,
                        center_xy=(nominal_place_pose[0], nominal_place_pose[1]),
                    )

                if getattr(cfg, "COMBO_ENABLED", True):
                    if zone_at_commit == "GREEN":
                        green_place_streak += 1
                    else:
                        green_place_streak = 0
                        if combo_active:
                            send_boost_end()
                            combo_active = False
                            handles.combo_active = combo_active
                            if _is_debug_enabled():
                                warn("COMBO", "combo ended")
                        send_boost_state(0, False)

                    combo_target = int(getattr(cfg, "COMBO_GREEN_PLACEMENTS_TARGET", 3))
                    if combo_target > 0 and green_place_streak >= combo_target:
                        participant = participant_name.strip() if isinstance(participant_name, str) and participant_name.strip() else "UNKNOWN"
                        if _is_debug_enabled():
                            warn("COMBO", f"{participant} combo achieved: {combo_target}x GREEN placements")
                        combo_active = True
                        handles.combo_active = combo_active
                        send_boost_state(combo_target, True)
                        green_place_streak = 0
                    elif zone_at_commit == "GREEN":
                        send_boost_state(green_place_streak, False)

                handles.combo_active = combo_active
                actions.complete_place_neutral_exit(handles, current_stack_level, target_id=placed_target_id)
                placement_quality_delta = _record_quality_placement(zone_at_commit)
                run_quality_score, run_green_count, run_yellow_count, run_red_count = _snapshot_quality_tracking()

                if current_zone_stack_level == current_stack_level and current_zone == "RED":
                    cfg.DRIFT_SCALE = min(3.0, float(cfg.DRIFT_SCALE) + float(cfg.DRIFT_RISK_INCREMENT))
                final_place_pose = (
                    proposed_place_pose
                    if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level
                    else cfg.tower_place_pose(current_stack_level)
                )
                _log_inferred_block_state(
                    "block_inferred_placed",
                    target_id=placed_target_id,
                    stack_level=int(current_stack_level),
                    tower_target_id=cfg.build_target_id_for_level(current_stack_level),
                    zone_at_commit=zone_at_commit,
                    quality_points_awarded=int(placement_quality_delta),
                    quality_score_after=int(run_quality_score),
                    green_count=int(run_green_count),
                    yellow_count=int(run_yellow_count),
                    red_count=int(run_red_count),
                    source="deterministic_place",
                    place_pose=final_place_pose,
                    retract_hover_pose=(
                        final_place_pose[0],
                        final_place_pose[1],
                        final_place_pose[2] + cfg.PLACE_CLEARANCE_MM,
                        final_place_pose[3],
                        final_place_pose[4],
                        final_place_pose[5],
                    ),
                )
                holding_block = False
                proposed_place_pose = None
                proposed_place_stack_level = None
                current_zone = "GREEN"
                current_zone_stack_level = None
            except Exception as e:
                green_place_streak = 0
                if combo_active:
                    send_boost_end()
                    combo_active = False
                    handles.combo_active = combo_active
                    if _is_debug_enabled():
                        warn("COMBO", "combo ended")
                send_boost_state(0, False)
                warn(module, f"[STACK] Place failed: {e}")
                proposed_place_pose = None
                proposed_place_stack_level = None
                current_zone = "GREEN"
                current_zone_stack_level = None
                # Emit fault event
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return

            # Update stack counters only on success
            if isinstance(placed_target_id, str) and placed_target_id.strip():
                if isinstance(active_pick_marker_target_id, str) and active_pick_marker_target_id.strip():
                    marker_target_id = str(active_pick_marker_target_id)
                    if marker_target_id not in picked_marker_target_ids:
                        picked_marker_target_ids.append(marker_target_id)
                claimed_target_id = (
                    str(active_pick_claim_target_id)
                    if isinstance(active_pick_claim_target_id, str) and active_pick_claim_target_id.strip()
                    else _nearest_pick_slot_claim(placed_target_id)
                )
                if claimed_target_id not in placed_pick_target_ids:
                    placed_pick_target_ids.append(claimed_target_id)
                write_jsonl_event(
                    "block_state",
                    {
                        "event": "pick_slot_claimed",
                        "module": "CONTROL",
                        "selected_target_id": str(placed_target_id),
                        "picked_marker_target_ids": list(picked_marker_target_ids),
                        "claimed_target_id": str(claimed_target_id),
                        "claim_radius_mm": float(getattr(cfg, "VISION_PICK_SLOT_CLAIM_RADIUS_MM", 70.0)),
                        "placed_pick_target_ids": list(placed_pick_target_ids),
                    },
                )
                _write_vision_assist_state()
            _clear_active_pick_targets()
            current_stack_level += 1
            current_pick_index += 1
            _set_pending_commit(current_stack_level, time.monotonic() + 5.0)

            tower_complete = current_stack_level >= target_stack_count

            # Emit internal progression event
            if my_token != current_session_token:
                return
            result4 = step(STATE, Event.PLACE_COMPLETE)
            if result4.allowed:
                STATE = result4.next_state
                info(module, f"[SM] -> {STATE.name} (PLACE_COMPLETE)")

                if tower_complete:
                    completion_end_mono = time.monotonic()
                    _send_line_to_unity("GAMEPLAY_COMPLETE")
                    with _score_state_lock:
                        if pending_commit_level is not None and pending_commit_deadline is not None and completion_end_mono < pending_commit_deadline:
                            completion_finalize_pending = True
                        else:
                            completion_finalize_pending = False
                    if completion_finalize_pending:
                        info(module, "[STACK] COMPLETE awaiting pending score commit window.")
                        return
                    run_time_s = emit_run_summary("COMPLETE")
                    finalize_run("COMPLETE")
                    run_quality_score, run_green_count, run_yellow_count, run_red_count = _snapshot_quality_tracking()
                    log_event(
                        "EVENT_RUN_SUMMARY",
                        participant=participant_name,
                        blocks_placed=current_stack_level,
                        run_time_s=run_time_s,
                        quality_score=run_quality_score,
                        green_count=run_green_count,
                        yellow_count=run_yellow_count,
                        red_count=run_red_count,
                        source="COMPLETE",
                    )
                    _send_line_to_unity(f"RUN_COMPLETE {current_stack_level}")
                    time.sleep(5.0)
                    console_emit("[STACK] COMPLETE summary emitted (post-wait)", tag="SUMMARY", level="INFO", module=module, allow_in_quiet=True)

                    current_pick_index = 0
                    current_stack_level = 0
                    _reset_pick_runtime_cache()
                    holding_block = False
                    proposed_place_pose = None
                    proposed_place_stack_level = None
                    block_attempt_start_ts = None
                    drop_committed_this_window = False
                    tower_attempt_start_ts = None
                    run_start_time = None
                    participant_name = None
                    run_id = None
                    current_run_seed = None
                    cfg.DRIFT_RUNTIME_RUN_SEED = None
                    cfg.DRIFT_RUNTIME_PARTICIPANT = ""
                    _last_ready_level_printed = None
                    _clear_score_commit_state()
                    _reset_quality_tracking()
                    _sync_json_log_context()
                    return

                # Auto-continue stacking if enabled and targets remain
                if stacking_enabled and current_stack_level < target_stack_count and current_pick_index < len(cfg.PICK_SEQUENCE):
                    info(module, "[STACK] Auto-continue to next block.")
                    if my_token != current_session_token:
                        return
                    auto_result = step(STATE, Event.START_STACK)
                    if auto_result.allowed:
                        STATE = auto_result.next_state
                        my_token = current_session_token
                        try:
                            pick_target_id, pick_mode_override = _resolve_pick_target_for_cycle()
                        except RuntimeError as e:
                            _clear_active_pick_targets()
                            if VISION_MODE_ENABLED:
                                warn(module, f"[VISION] pick target unavailable: {e}")
                                _send_line_to_unity("VISION_STATUS FAIL")
                                STATE = State.WAITING_FOR_REPOSITION
                                info(module, "[SM] -> WAITING_FOR_REPOSITION (VISION pick target unavailable)")
                                return
                            raise
                        _set_active_pick_targets(pick_target_id, pick_mode_override)
                        if str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower() == "vision":
                            console_info(
                                "CONTROL",
                                (
                                    f"[VISION] selected_source={pick_target_id} claimed_pick_slot={active_pick_claim_target_id}"
                                    if pick_mode_override is None
                                    else f"[VISION] deterministic_fallback_source={pick_target_id} claimed_pick_slot={active_pick_claim_target_id}"
                                ),
                                essential=_is_debug_enabled(),
                            )
                        handles.combo_active = combo_active
                        vision_controller.log_pick_tracking(
                            target_id=pick_target_id,
                            stack_level=current_stack_level,
                            debug_enabled=_is_debug_enabled(),
                        )
                        try:
                            actions.execute_pick_sequence(
                                handles,
                                pick_target_id,
                                current_stack_level,
                                pick_mode_override=pick_mode_override,
                            )
                        except actions.PickPoseUnavailableError as e:
                            _clear_active_pick_targets()
                            if VISION_MODE_ENABLED:
                                fallback_target_id, pick_mode_override = _deterministic_remaining_pick_fallback_target(e.reason)
                                if fallback_target_id is None:
                                    warn(module, f"[VISION] pick pose unavailable: {e.reason}")
                                    _send_line_to_unity("VISION_STATUS FAIL")
                                    STATE = State.WAITING_FOR_REPOSITION
                                    info(module, "[SM] -> WAITING_FOR_REPOSITION (VISION pick unavailable)")
                                    return
                                pick_target_id = fallback_target_id
                                _set_active_pick_targets(pick_target_id, pick_mode_override)
                                console_info(
                                    "CONTROL",
                                    f"[VISION] deterministic_fallback_source={pick_target_id} claimed_pick_slot={active_pick_claim_target_id}",
                                    essential=True,
                                )
                                handles.combo_active = combo_active
                                vision_controller.log_pick_tracking(
                                    target_id=pick_target_id,
                                    stack_level=current_stack_level,
                                    debug_enabled=_is_debug_enabled(),
                                )
                                try:
                                    actions.execute_pick_sequence(
                                        handles,
                                        pick_target_id,
                                        current_stack_level,
                                        pick_mode_override=pick_mode_override,
                                    )
                                except actions.PickPoseUnavailableError as fallback_error:
                                    warn(module, f"[VISION] pick pose unavailable: {e.reason}->deterministic_fallback:{fallback_error.reason}")
                                    _send_line_to_unity("VISION_STATUS FAIL")
                                    STATE = State.WAITING_FOR_REPOSITION
                                    info(module, "[SM] -> WAITING_FOR_REPOSITION (VISION pick unavailable)")
                                    return
                            else:
                                raise
                        try:
                            pick_pose = cfg.pick_target_pose(pick_target_id)
                            hover_pose = cfg.pick_target_hover_pose(pick_target_id)
                        except Exception:
                            pick_pose = None
                            hover_pose = None
                        _log_inferred_block_state(
                            "block_inferred_picked",
                            target_id=pick_target_id,
                            stack_level=int(current_stack_level),
                            source="vision_assist_pick" if str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower() == "vision" else "deterministic_pick",
                            pick_pose=pick_pose,
                            hover_pose=hover_pose,
                        )
                        vision_controller.forget_pick_target(pick_target_id)
                        # Immediate RobotMode check after motion (auto-continue)
                        m = handles.robot.robot_mode()
                        if m in (9, 11):
                            console_emit(f"[FAULT] RobotMode={m} -> entering FAULT", tag="FAULT", level="WARN", module=module, allow_in_quiet=True)
                            fault_result = step(STATE, Event.FAULT)
                            if fault_result.allowed:
                                STATE = fault_result.next_state
                                info(module, f"[SM] -> {STATE.name} (FAULT)")
                            return
                        if my_token != current_session_token:
                            return
                        result2 = step(STATE, Event.PICK_COMPLETE)
                        if result2.allowed:
                            STATE = result2.next_state
                            if STATE == State.MOVING_TO_TOWER_HOVER:
                                base_place = cfg.tower_place_pose(current_stack_level)
                                proposed_place_pose = drift_engine.inject_drift(
                                    base_place,
                                    current_stack_level,
                                    run_seed=current_run_seed,
                                    participant=participant_name,
                                )
                                proposed_place_stack_level = current_stack_level
                                log_event(
                                    "EVENT_DRIFT_LEVEL",
                                    source="AUTO",
                                    stack_level=current_stack_level,
                                    drift_dx_mm=round(proposed_place_pose[0] - base_place[0], 3),
                                    drift_dy_mm=round(proposed_place_pose[1] - base_place[1], 3),
                                    runtime_run_seed=current_run_seed,
                                    drift_run_seed_baseline=int(getattr(cfg, "DRIFT_RUN_SEED", 0)),
                                )
                                handles.combo_active = combo_active
                                actions.move_to_tower_hover(
                                    handles,
                                    current_stack_level,
                                    target_xy=(proposed_place_pose[0], proposed_place_pose[1]),
                                    target_id=pick_target_id,
                                )
                                if not _handle_post_tower_hover(module, my_token):
                                    return
                else:
                    info(module, "[STACK] Target reached or no more blocks.")
        finally:
            controller_busy = False

    elif event == Event.CANCEL:
        # returns to WAITING_FOR_DECISION by state machine; no motion needed
        pass

    elif event == Event.AUTO_RECOVER:
        # Recovery from FAULT state (or no-op from other states)
        # State is determined by actual recovery success, not transition table next_state.
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            info(module, "[RECOVERY] Auto-recover requested. Starting recovery sequence...")
            ok = actions.recover_from_fault(handles)
            if ok:
                info(module, "[RECOVERY] Recovery reported OK. Resetting state to IDLE.")
                STATE = State.IDLE
                current_pick_index = 0
                current_stack_level = 0
                _reset_pick_runtime_cache()
                proposed_place_pose = None
                proposed_place_stack_level = None
                current_zone = "GREEN"
                current_zone_stack_level = None
                robot_armed = True
                _clear_score_commit_state()
            else:
                warn(module, "[RECOVERY] Recovery reported FAILURE. Setting state to FAULT.")
                STATE = State.FAULT
                robot_armed = False
        except Exception as e:
            warn(module, f"[RECOVERY] Unexpected error during recovery: {e}")
            STATE = State.FAULT
            robot_armed = False
        finally:
            controller_busy = False

    elif event == Event.GRIP_OPEN:
        actions.do_grip_open(handles)
        holding_block = False

    elif event == Event.GRIP_CLOSE:
        actions.do_grip_close(handles)
        holding_block = True

    elif event == Event.GRIP_TOGGLE:
        actions.do_grip_toggle(handles)


# --- 4. HIGHWAY 2: COMMAND HUB (Logic Bridge) ---
def command_server():
    global STATE, robot_armed, gripper_connected, controller_busy, vr_connected
    global proposed_place_pose, proposed_place_stack_level, current_zone, current_zone_stack_level
    global _unity_command_conn, _last_sent_zone
    global current_pick_index, current_stack_level, stacking_enabled, target_stack_count
    global green_place_streak, combo_active
    global run_id, run_finalized, current_run_seed
    global last_finalized_run_id, last_finalized_mode, last_finalized_session_id, last_finalized_participant_name
    global _logged_raw_getpose_probe

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', UNITY_PORT_COMMANDS))
    server.listen(1)
    server.settimeout(1.0)
    _track_server_socket(server)

    info("CONTROL", f"[CONTROL] Hub ready on {UNITY_PORT_COMMANDS}")
    info("CONTROL", "[CONTROL] Commands: HOME | FIX | NUDGE dx dy | DROP | CANCEL | GRIP_TOGGLE | GRIP_OPEN | GRIP_CLOSE | (COMMIT->DROP)")
    info("CONTROL", "[CONTROL] NOTE: Robot arms once per VR connection (no per-command arming).")
    info("CONTROL", "[CONTROL] NOTE: Gripper is RS485 Modbus (no DH UI Init toggle required).")

    try:
        while not STOP_EVENT.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                if STOP_EVENT.is_set():
                    break
                raise
            _track_client_socket(conn)
            _unity_command_conn = conn
            _last_sent_zone = None
            vr_connected = True
            console_emit(f"[CONTROL] VR Connected: {addr}", tag="UNITY", level="INFO", module="CONTROL", allow_in_quiet=True)

            # Reset state machine for new session
            STATE = State.IDLE

            # Initialize fresh stacking session
            current_pick_index = 0
            current_stack_level = 0
            _reset_pick_runtime_cache()
            # Dev 8 autonomous stacking loop controls
            stacking_enabled = True
            target_stack_count = cfg.stack_target_count()
            controller_busy = False
            green_place_streak = 0
            if combo_active:
                send_boost_end()
            combo_active = False
            handles.combo_active = combo_active
            send_boost_state(0, False)
            run_id = None
            run_finalized = False
            current_run_seed = None
            cfg.DRIFT_RUNTIME_RUN_SEED = None
            cfg.DRIFT_RUNTIME_PARTICIPANT = ""
            _clear_score_commit_state()
            _sync_json_log_context()

            # Arm once for this VR session
            try:
                actions.arm_robot_once(handles)
                robot_armed = True
                if not _logged_raw_getpose_probe:
                    try:
                        raw_pose_resp = handles.robot.send("GetPose()")
                        if _is_debug_enabled():
                            info("CONTROL", f"[POSE_RAW] GetPose() -> {raw_pose_resp}")
                    except Exception as e:
                        if _is_debug_enabled():
                            warn("CONTROL", f"[POSE_RAW] GetPose() probe failed: {e}")
                    finally:
                        _logged_raw_getpose_probe = True
                console_emit("Waiting for participant name...", tag="PROMPT", level="INFO", module="CONTROL", allow_in_quiet=True)
            except Exception as e:
                robot_armed = False
                warn("CONTROL", f"[CONTROL] Robot arm FAILED: {e}")

            # Connect gripper once for this VR session
            try:
                actions.connect_gripper_once(handles)
                gripper_connected = True
            except Exception as e:
                gripper_connected = False
                warn("CONTROL", f"[CONTROL] Gripper connect FAILED: {e}")

            # Initialize stack session (home + open gripper)
            try:
                actions.initialize_stack_session(handles)
            except Exception as e:
                warn("CONTROL", f"[CONTROL] Session init FAILED: {e}")

            conn.settimeout(1.0)
            buf = ""

            try:
                while not STOP_EVENT.is_set():
                    try:
                        data = conn.recv(1024)
                    except socket.timeout:
                        continue
                    if not data:
                        break

                    buf += data.decode('utf-8', errors='ignore')

                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        cmd_str = line.strip()
                        if not cmd_str:
                            continue

                        # Dispatch to command handler
                        handle_command(cmd_str, "CONTROL")

            except Exception as e:
                if not STOP_EVENT.is_set():
                    warn("CONTROL", f"[CONTROL] Connection error: {e}")
            finally:
                _close_socket_quietly(conn)
                _untrack_client_socket(conn)
                if _unity_command_conn is conn:
                    _unity_command_conn = None

            # NOTE: handles.gripper.close() not called here because close() is used for
            # gripper actuation (closing the grip) and would cause unintended physical motion.
            # Gripper disconnection is handled by gripper_connected flag reset and eventual
            # reconnection on next VR session.

                robot_armed = False
                STATE = State.IDLE
                console_emit("[CONTROL] VR disconnected. Robot disarmed. STATE reset to IDLE.", tag="UNITY", level="INFO", module="CONTROL", allow_in_quiet=True)

                # Keep the gripper connection status conservative:
                # If Unity reconnects, we'll re-attempt connect.
                gripper_connected = False
                vr_connected = False
                proposed_place_pose = None
                proposed_place_stack_level = None
                current_zone = "GREEN"
                current_zone_stack_level = None
                current_run_seed = None
                cfg.DRIFT_RUNTIME_RUN_SEED = None
                cfg.DRIFT_RUNTIME_PARTICIPANT = ""
                _clear_score_commit_state()
    finally:
        _close_socket_quietly(server)
        _untrack_server_socket(server)


# --- 4.5. ADMIN SERVER ---
def admin_server():
    """
    Administrative TCP server (localhost only, port 8089).
    Accepts commands and processes them via handle_command with source="ADMIN".
    Does NOT reset session variables or arm/disarm on connect.
    Shares state with command_server (STATE, robot_armed, etc.).
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 8089))
    server.listen(1)
    server.settimeout(1.0)
    _track_server_socket(server)

    info("ADMIN", "[ADMIN] Hub ready on 8089")

    try:
        while not STOP_EVENT.is_set():
            try:
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if STOP_EVENT.is_set():
                        break
                    raise
                _track_client_socket(conn)
                info("ADMIN", f"[ADMIN] Client connected: {addr}")
                
                conn.settimeout(1.0)
                buf = ""

                try:
                    while not STOP_EVENT.is_set():
                        try:
                            data = conn.recv(1024)
                        except socket.timeout:
                            continue
                        if not data:
                            break

                        buf += data.decode('utf-8', errors='ignore')

                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            cmd_str = line.strip()
                            if not cmd_str:
                                continue

                            # Dispatch to command handler with ADMIN source
                            handle_command(cmd_str, "ADMIN")

                except Exception as e:
                    if not STOP_EVENT.is_set():
                        warn("ADMIN", f"[ADMIN] Client error: {e}")
                finally:
                    _close_socket_quietly(conn)
                    _untrack_client_socket(conn)
                    if not STOP_EVENT.is_set():
                        info("ADMIN", f"[ADMIN] Client disconnected: {addr}")

            except Exception as e:
                if not STOP_EVENT.is_set():
                    warn("ADMIN", f"[ADMIN] Server error: {e}")
    finally:
        _close_socket_quietly(server)
        _untrack_server_socket(server)


def facilitator_hotkey_loop():
    """
    Facilitator keyboard hotkeys from stdin.
    Runs in a daemon thread and dispatches commands via handle_command(..., "FACILITATOR").
    """
    if _is_debug_enabled():
        info("CONTROL", "[FACILITATOR] Console ready: t|tumble, r|recover, n <name>|name <name>, h|help")
    while not STOP_EVENT.is_set():
        try:
            with _TERMINAL_INPUT_LOCK:
                if _is_debug_enabled():
                    raw = input("FACILITATOR> ")
                else:
                    raw = input()
            line = raw.strip()
            if not line:
                continue

            parts = line.split(maxsplit=1)
            head = parts[0].lower() if parts else ""
            tail = parts[1].strip() if len(parts) > 1 else ""
            tokens = line.split()
            lowered_tokens = [t.lower() for t in tokens]

            if head in {"h", "help"}:
                info("CONTROL", "[FACILITATOR] Commands:")
                info("CONTROL", "  t | tumble           -> TUMBLE")
                info("CONTROL", "  r | recover          -> AUTO_RECOVER")
                info("CONTROL", "  n <name>             -> NAME <name>")
                info("CONTROL", "  name <name>          -> NAME <name>")
                info("CONTROL", "  mode show            -> MODE SHOW")
                info("CONTROL", "  mode dev             -> MODE DEV")
                info("CONTROL", "  mode official        -> MODE OFFICIAL")
                info("CONTROL", "  event <id>           -> EVENT <id>")
                info("CONTROL", "  fixscore <n> [why]   -> FIXSCORE <n> [why]")
                info("CONTROL", "  debug show           -> DEBUG SHOW")
                info("CONTROL", "  debug on             -> DEBUG ON")
                info("CONTROL", "  debug off            -> DEBUG OFF")
                info("CONTROL", "  h | help             -> this help")
                continue

            facilitator_mode_cmd = None

            if head in {"t", "tumble"}:
                cmd = "TUMBLE"
            elif head in {"r", "recover"}:
                cmd = "AUTO_RECOVER"
            elif head in {"n", "name"}:
                name = tail
                if not name:
                    warn("CONTROL", "[FACILITATOR] Usage: n <name>  (or: name <name>)")
                    continue
                cmd = f"NAME {name}"
            elif len(lowered_tokens) >= 2 and lowered_tokens[0] == "mode":
                mode_arg = lowered_tokens[1]
                if mode_arg == "show":
                    cmd = "MODE SHOW"
                    facilitator_mode_cmd = "MODE_SHOW"
                elif mode_arg == "dev":
                    cmd = "MODE DEV"
                    facilitator_mode_cmd = "MODE_DEV"
                elif mode_arg == "official":
                    cmd = "MODE OFFICIAL"
                    facilitator_mode_cmd = "MODE_OFFICIAL"
                else:
                    warn("CONTROL", "[FACILITATOR] Usage: mode show|dev|official")
                    continue
            elif len(lowered_tokens) >= 2 and lowered_tokens[0] == "debug":
                debug_arg = lowered_tokens[1]
                if debug_arg == "show":
                    cmd = "DEBUG SHOW"
                    facilitator_mode_cmd = "DEBUG_SHOW"
                elif debug_arg == "on":
                    cmd = "DEBUG ON"
                    facilitator_mode_cmd = "DEBUG_ON"
                elif debug_arg == "off":
                    cmd = "DEBUG OFF"
                    facilitator_mode_cmd = "DEBUG_OFF"
                else:
                    warn("CONTROL", "[FACILITATOR] Usage: debug show|on|off")
                    continue
            elif head == "event":
                event_id = tail
                if not event_id:
                    warn("CONTROL", "[FACILITATOR] Usage: event <id>")
                    continue
                cmd = f"EVENT {event_id}"
                facilitator_mode_cmd = "EVENT_SET"
            elif head == "fixscore":
                if not tail:
                    warn("CONTROL", "[FACILITATOR] Usage: fixscore <n> [optional reason]")
                    continue
                cmd = f"FIXSCORE {tail}"
            else:
                warn("CONTROL", f"[FACILITATOR] Unknown command: {line} (type 'h' or 'help')")
                continue

            try:
                handle_command(cmd, "FACILITATOR")

                if facilitator_mode_cmd is not None:
                    if facilitator_mode_cmd == "MODE_SHOW":
                        pass
                    elif facilitator_mode_cmd == "MODE_DEV":
                        pass
                    elif facilitator_mode_cmd == "MODE_OFFICIAL":
                        pass
                    elif facilitator_mode_cmd == "EVENT_SET":
                        ack = f"EVENT set to {OFFICIAL_EVENT_ID} (MODE={LEADERBOARD_MODE})"
                        show = f"MODE={LEADERBOARD_MODE} EVENT={OFFICIAL_EVENT_ID}"
                        console_emit(f"[FACILITATOR] {ack}", tag="FACILITATOR", level="INFO", module="CONTROL")
                        console_emit(f"[FACILITATOR] {show}", tag="FACILITATOR", level="INFO", module="CONTROL")
                    elif facilitator_mode_cmd == "DEBUG_SHOW":
                        pass
                    elif facilitator_mode_cmd == "DEBUG_ON":
                        pass
                    elif facilitator_mode_cmd == "DEBUG_OFF":
                        pass
                else:
                    upper_cmd = cmd.upper()
                    if upper_cmd == "TUMBLE" or upper_cmd.startswith("NAME "):
                        pass
                    else:
                        ack = f"OK: {cmd}"
                        console_emit(f"[FACILITATOR] {ack}", tag="FACILITATOR", level="INFO", module="CONTROL")
            except Exception as e:
                warn("CONTROL", f"[FACILITATOR] Command error: {e}")
        except EOFError:
            time.sleep(0.2)
        except Exception as e:
            warn("CONTROL", f"[FACILITATOR] Input loop error: {e}")


def _start_worker_thread(name: str, target, args=(), daemon: bool = False) -> None:
    thread = threading.Thread(target=target, args=args, name=name, daemon=daemon)
    _WORKER_THREADS.append(thread)
    thread.start()


# --- 5. START SYSTEM ---
try:
    _start_worker_thread("command-server", command_server)
    _start_worker_thread("admin-server", admin_server)
    _start_worker_thread("leaderboard-http", lb.leaderboard_http_server, args=(lb_ctx, STOP_EVENT, LEADERBOARD_PORT))
    if CAMERA_STREAM_RUNTIME_ENABLED and (dai is not None) and (camera_streamer is not None):
        _start_worker_thread("camera-inspector", camera_server_wrapper, args=(MXID_INSPECTOR, UNITY_PORT_INSPECTOR, "INSPECTOR"))
        time.sleep(10)
        _start_worker_thread("camera-site-manager", camera_server_wrapper, args=(MXID_MANAGER, UNITY_PORT_MANAGER, "SITE_MANAGER"))
    if _is_official_mode():
        console_emit("Start Unity and press Play...", tag="UNITY", level="INFO", module="CONTROL", allow_in_quiet=True)
    _start_worker_thread("facilitator-hotkey", facilitator_hotkey_loop, daemon=True)

    info("CONTROL", "TASK CONTROLLER ACTIVE. Press Ctrl+C to stop.")
    while True:
        _promote_pending_commit_if_ready()
        
        # Check for 5-minute hard timeout
        if _check_run_timeout():
            _handle_run_timeout()
        
        time.sleep(0.1)
except KeyboardInterrupt:
    info("CONTROL", "[MAIN] Ctrl+C received. Initiating graceful shutdown...")
    try:
        # Ignore repeated Ctrl+C while Python tears down worker threads.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except Exception:
        pass
    STOP_EVENT.set()
    vision_controller.shutdown_perception()
    cleanup_errors = _close_all_tracked_sockets()
    alive_threads = _join_worker_threads(timeout_s=3.0)

    if alive_threads:
        if _is_debug_enabled():
            warn("CONTROL", f"[MAIN] Threads still alive after join timeout: {alive_threads}")

    if cleanup_errors:
        if _is_debug_enabled():
            warn("CONTROL", f"[MAIN] Shutdown encountered {len(cleanup_errors)} cleanup issue(s).")

    info("CONTROL", "[MAIN] Shutdown complete.")
    # Exit deterministically after best-effort cleanup, avoiding interpreter
    # thread teardown races when Ctrl+C is pressed repeatedly.
    os._exit(0)
