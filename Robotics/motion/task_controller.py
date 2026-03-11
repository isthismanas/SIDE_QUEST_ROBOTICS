import time, socket, struct, os, threading, json, sys
import warnings
import secrets
from datetime import timedelta
import hashlib
from uuid import uuid4
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from state_machine import State, Event, step, parse_event

import actions
from actions import SystemHandles
import drift_engine
import tolerance_engine
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE  # NEW: RS485 Modbus gripper driver
import robot_config as cfg
from logger import info, warn, error

# --- Add perception module ---
PICK_POSE_MODE = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
VISION_MODE_ENABLED = PICK_POSE_MODE in {"vision", "perception"}
CAMERA_STREAM_ENABLED = bool(getattr(cfg, "CAMERA_STREAM_ENABLED", True))

dai = None
if VISION_MODE_ENABLED or CAMERA_STREAM_ENABLED:
    try:
        import depthai as dai
    except Exception:
        dai = None

PERC_AVAILABLE = False
perc_engine = None
if not VISION_MODE_ENABLED:
    if bool(getattr(cfg, "DEBUG_ENABLED", False)):
        info("PERC", "PERC bypassed (deterministic mode)")
else:
    try:
        perc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "perception")
        if perc_path not in sys.path:
            sys.path.append(perc_path)
        from perception_engine import engine as perc_engine  # type: ignore[reportMissingImports]
        PERC_AVAILABLE = True
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("PERC", "Perception module enabled")
    except Exception as e:
        warn("PERC", f"Perception module disabled: {e}")

warnings.filterwarnings("ignore", category=DeprecationWarning)

if bool(getattr(cfg, "DEBUG_ENABLED", False)):
    info("CONTROL", f"USING ACTIONS FROM: {actions.__file__}")


# --- 0. STABILITY & PORT CONFIG ---
os.environ["DEPTHAI_WATCHDOG_TIMEOUT"] = "5000"

UNITY_PORT_INSPECTOR = 8085
UNITY_PORT_MANAGER   = 8086
UNITY_PORT_COMMANDS  = 8088

MXID_INSPECTOR = "19443010B14C872F00"
MXID_MANAGER   = "194430108183F12E00"

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
green_place_streak = 0
combo_active = False
_unity_command_conn = None
_last_sent_zone = None
run_id = None
run_finalized = False
_logged_raw_getpose_probe = False
current_run_seed = None
DEBUG_ENABLED = bool(getattr(cfg, "DEBUG_ENABLED", False))
CONSOLE_QUIET = not DEBUG_ENABLED
_DEFAULT_LOG_MODULE_LEVELS = dict(getattr(cfg, "LOG_MODULES", {}))
_last_ready_level_printed = None
QUIET_ALLOWLIST = {"PROMPT", "SUMMARY", "FAULT", "ERROR", "FATAL"}
LEADERBOARD_MODE = "DEV"
OFFICIAL_EVENT_ID = "ARC2026"

LEADERBOARD_PORT = 8090


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


def log_event(event: str, **fields) -> None:
    """Append one JSONL event record to cfg.LOG_DIR."""
    if _normalize_leaderboard_mode(LEADERBOARD_MODE) != "OFFICIAL":
        return

    try:
        log_dir = _current_log_subdir()

        sid = session_id or "no_session"
        log_path = os.path.join(log_dir, f"session_{sid}.jsonl")

        record = {
            "timestamp": time.time(),
            "event": event,
            "session_id": session_id,
            "participant_name": participant_name,
            "state": STATE.name if hasattr(STATE, "name") else str(STATE),
            "current_stack_level": globals().get("current_stack_level", None),
            "current_pick_index": globals().get("current_pick_index", None),
        }
        if hasattr(cfg, "DRIFT_SCALE"):
            record["drift_scale"] = cfg.DRIFT_SCALE
        if fields:
            record.update(fields)

        with open(log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except Exception as e:
        warn("CONTROL", f"[LOG] Failed to write event {event}: {e}")


def emit_run_summary(reason: str) -> float:
    now_mono = time.monotonic()
    start_ts = run_start_time if run_start_time is not None else now_mono
    duration_s = max(0.0, now_mono - start_ts)
    participant = participant_name.strip() if isinstance(participant_name, str) and participant_name.strip() else "UNKNOWN"
    placed = int(current_stack_level) if current_stack_level is not None else 0
    summary_reason = reason.strip().upper() if isinstance(reason, str) and reason.strip() else "UNKNOWN"
    warn("STACK", f"{participant} successfully placed {placed} blocks in {duration_s:.1f} seconds ({summary_reason})")
    return duration_s


def _normalize_leaderboard_mode(value) -> str:
    return "OFFICIAL" if str(value).strip().upper() == "OFFICIAL" else "DEV"


def _is_official_mode() -> bool:
    return _normalize_leaderboard_mode(LEADERBOARD_MODE) == "OFFICIAL"


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
        allow = (level_u in {"WARN", "ERROR", "FATAL"}) or (tag_u in QUIET_ALLOWLIST) or (allow_in_quiet and tag_u in QUIET_ALLOWLIST)
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
    if _is_console_quiet():
        return
    if level == _last_ready_level_printed:
        return
    _last_ready_level_printed = level
    console_info("CONTROL", f"READY: waiting for DROP/FIX (stack_level={level})", essential=True)


def _set_debug_enabled(enabled: bool) -> None:
    global DEBUG_ENABLED, CONSOLE_QUIET
    DEBUG_ENABLED = bool(enabled)
    CONSOLE_QUIET = not DEBUG_ENABLED
    cfg.DEBUG_ENABLED = DEBUG_ENABLED
    _apply_console_verbosity()


def _current_log_subdir() -> str:
    mode = _normalize_leaderboard_mode(LEADERBOARD_MODE)
    base_log_dir = getattr(cfg, "LOG_DIR", "logs")
    subdir = "official" if mode == "OFFICIAL" else "dev"
    path = os.path.join(base_log_dir, subdir)
    os.makedirs(path, exist_ok=True)
    return path


def _leaderboard_mode_path() -> str:
    log_dir = getattr(cfg, "LOG_DIR", "logs")
    os.makedirs(log_dir, exist_ok=True)
    return os.path.join(log_dir, "leaderboard_mode.json")


def _save_leaderboard_mode() -> None:
    payload = {
        "mode": LEADERBOARD_MODE,
        "official_event_id": OFFICIAL_EVENT_ID,
        "updated_at_unix": time.time(),
    }
    path = _leaderboard_mode_path()
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
    except Exception as e:
        warn("STACK", f"[LEADERBOARD] Failed to persist mode config: {e}")


def _load_leaderboard_mode() -> None:
    global LEADERBOARD_MODE, OFFICIAL_EVENT_ID

    path = _leaderboard_mode_path()
    if not os.path.exists(path):
        _save_leaderboard_mode()
        return

    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except Exception as e:
        warn("STACK", f"[LEADERBOARD] Failed to read mode config, using defaults: {e}")
        return

    if isinstance(raw, dict):
        LEADERBOARD_MODE = _normalize_leaderboard_mode(raw.get("mode", LEADERBOARD_MODE))
        event_value = str(raw.get("official_event_id", OFFICIAL_EVENT_ID)).strip()
        OFFICIAL_EVENT_ID = event_value if event_value else OFFICIAL_EVENT_ID

    info("STACK", f"[LEADERBOARD] Mode loaded: mode={LEADERBOARD_MODE} event_id={OFFICIAL_EVENT_ID}")


def _leaderboard_jsonl_path() -> str:
    return os.path.join(_current_log_subdir(), "leaderboard.jsonl")


def _compute_completion_time_s() -> float:
    now_mono = time.monotonic()
    start_ts = run_start_time if run_start_time is not None else now_mono
    return max(0.0, now_mono - start_ts)


def finalize_run(end_state: str) -> None:
    global run_finalized
    if run_finalized:
        return

    ended_state = str(end_state).strip().upper() if isinstance(end_state, str) else "UNKNOWN"
    target_height = int(getattr(cfg, "TOWER_LEVELS", 7))
    stable_height = int(globals().get("current_stack_level", 0) or 0)
    final_height = max(0, min(stable_height, target_height))
    completion_time_s = _compute_completion_time_s()
    finalized_run_id = run_id or uuid4().hex
    mode = _normalize_leaderboard_mode(LEADERBOARD_MODE)
    event_id = OFFICIAL_EVENT_ID if mode == "OFFICIAL" else "DEV"

    record = {
        "run_id": finalized_run_id,
        "session_id": session_id,
        "participant_name": participant_name,
        "end_state": ended_state,
        "mode": mode,
        "event_id": event_id,
        "final_height": final_height,
        "target_height": target_height,
        "completion_time_s": round(completion_time_s, 3),
        "ended_at_unix": time.time(),
    }

    path = _leaderboard_jsonl_path()
    try:
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        run_finalized = True
        info("STACK", f"[LEADERBOARD] Finalized run_id={finalized_run_id} state={ended_state} height={final_height}/{target_height} time={completion_time_s:.3f}s")
    except Exception as e:
        warn("STACK", f"[LEADERBOARD] Failed to append leaderboard record: {e}")


def _load_leaderboard_records() -> list[dict]:
    path = _leaderboard_jsonl_path()
    if not os.path.exists(path):
        return []

    rows: list[dict] = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                    if isinstance(row, dict):
                        rows.append(row)
                except Exception:
                    continue
    except Exception as e:
        warn("STACK", f"[LEADERBOARD] Failed to read leaderboard: {e}")
    return rows


class _LeaderboardHandler(BaseHTTPRequestHandler):
    def _send_bytes(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, status: int, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _kiosk_html(self) -> str:
        return """<!doctype html>
<html lang=\"en\">
<head>
    <meta charset=\"utf-8\" />
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
    <title>SIDE QUEST LEADERBOARD</title>
    <style>
        :root {
            --bg: #07090f;
            --panel: #0f1420;
            --line: #253047;
            --text: #e7edf9;
            --muted: #9ab0d0;
            --gold: #f2c94c;
            --silver: #c0cad6;
            --bronze: #cd8c5d;
            --accent: #4db7ff;
        }
        * { box-sizing: border-box; }
        html, body { width: 100%; height: 100%; margin: 0; }
        body {
            font-family: Inter, Segoe UI, Roboto, Arial, sans-serif;
            background: #00E5FF;
            color: var(--text);
            display: flex;
            align-items: stretch;
            justify-content: center;
        }
        .wrap {
            width: min(1200px, 100%);
            padding: 0 16px 14px 16px;
            display: grid;
            grid-template-rows: auto auto auto auto;
            gap: 4px;
        }
        .logo-band {
            width: 100vw;
            margin-left: calc(50% - 50vw);
            margin-right: calc(50% - 50vw);
            background: linear-gradient(180deg, #16243f 0%, #0f1420 100%);
            border-bottom: 1px solid #253047;
            padding: 72px 0;
            margin-bottom: 4px;
        }
        .logo-wrap {
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 50px;
            margin: 0;
            padding: 0;
        }
        .logo {
            height: 100px;
            max-width: 26%;
            object-fit: contain;
        }
        h1 {
            margin: 6px 0 2px 0;
            letter-spacing: 1px;
            font-size: clamp(36px, 6vw, 63px);
            text-transform: uppercase;
            text-align: center;
            font-family: Futura, "Futura PT", "Trebuchet MS", Inter, sans-serif;
            color: #0f1420;
        }
        .meta {
            display: flex;
            justify-content: center;
            gap: 18px;
            color: var(--muted);
            font-weight: 500;
            flex-wrap: wrap;
            margin: 26px 0 0 0;
            font-size: clamp(12px, 1.1vw, 14px);
            opacity: 0.8;
        }
        .panel {
            border: 1px solid var(--line);
            border-radius: 14px;
            background: color-mix(in oklab, var(--panel) 92%, black);
            overflow: hidden;
            min-height: 0;
        }
        table {
            width: 100%;
            border-collapse: collapse;
            font-size: clamp(13px, 1.35vw, 18px);
            font-family: "Encode Sans", "Encode Sans SemiExpanded", Inter, system-ui, sans-serif;
        }
        thead th {
            text-align: left;
            color: var(--muted);
            padding: 9px 12px;
            border-bottom: 1px solid var(--line);
            letter-spacing: 0.5px;
            text-transform: uppercase;
            font-size: 0.75em;
        }
        tbody td {
            padding: 8px 12px;
            border-bottom: 1px solid #1a2233;
        }
        tbody tr { height: 42px; }
        tbody tr:nth-child(1) { background: linear-gradient(90deg, rgba(242,201,76,0.22), transparent 70%); }
        tbody tr:nth-child(2) { background: linear-gradient(90deg, rgba(192,202,214,0.20), transparent 70%); }
        tbody tr:nth-child(3) { background: linear-gradient(90deg, rgba(205,140,93,0.20), transparent 70%); }
        tbody tr:nth-child(1) td:first-child { color: var(--gold); font-weight: 800; }
        tbody tr:nth-child(2) td:first-child { color: var(--silver); font-weight: 800; }
        tbody tr:nth-child(3) td:first-child { color: var(--bronze); font-weight: 800; }
        tbody tr.empty { background: transparent !important; }
        tbody tr.empty td:first-child { color: var(--text); font-weight: 400; }
        .status {
            min-height: 24px;
            color: var(--accent);
            text-align: center;
            font-weight: 600;
        }
    </style>
</head>
<body>
    <div class=\"wrap\">
        <div class=\"logo-band\">
            <div class=\"logo-wrap\">
                <img src=\"/assets/logo1.png\" alt=\"Side Quest Logo\" class=\"logo\">
                <img src=\"/assets/logo2.png\" alt=\"Side Quest Logo 2\" class=\"logo\">
                <img src=\"/assets/logo3.png\" alt=\"Side Quest Logo 3\" class=\"logo\">
                <img src=\"/assets/logo4.png\" alt=\"Side Quest Logo 4\" class=\"logo\">
                <img src=\"/assets/logo5.png\" alt=\"Side Quest Logo 5\" class=\"logo\">
            </div>
        </div>
        <h1>SIDE QUEST LEADERBOARD</h1>
        <div class=\"panel\">
            <table>
                <thead>
                    <tr>
                        <th style=\"width: 14%\">Rank</th>
                        <th style=\"width: 46%\">Pilot Name</th>
                        <th style=\"width: 20%\">Height</th>
                        <th style=\"width: 20%\">Time</th>
                    </tr>
                </thead>
                <tbody id=\"rows\"></tbody>
            </table>
        </div>
        <div class=\"meta\">
            <div id=\"mode\">Mode: --</div>
            <div id=\"event\">Event: --</div>
        </div>
        <div class=\"status\" id=\"status\">Waiting for data…</div>
    </div>

    <script>
        const qs = new URLSearchParams(window.location.search);
        if (!qs.has('limit')) qs.set('limit', '10');
        const targetRows = Math.max(1, Number.parseInt(qs.get('limit') || '10', 10) || 10);

        const modeEl = document.getElementById('mode');
        const eventEl = document.getElementById('event');
        const rowsEl = document.getElementById('rows');
        const statusEl = document.getElementById('status');

        function fmtTime(v) {
            const n = Number(v);
            if (!Number.isFinite(n)) return '--';
            return n.toFixed(3) + 's';
        }

        function renderRows(entries) {
            const realEntries = Array.isArray(entries) ? entries : [];
            let html = '';

            for (let i = 0; i < targetRows; i += 1) {
                if (i < realEntries.length) {
                    const row = realEntries[i];
                    const rank = row.rank ?? '--';
                    const name = (row.participant_name && String(row.participant_name).trim()) || 'UNKNOWN';
                    const height = row.final_height ?? '--';
                    const time = fmtTime(row.completion_time_s);
                    html += `<tr><td>${rank}</td><td>${name}</td><td>${height}</td><td>${time}</td></tr>`;
                } else {
                    html += '<tr class="empty"><td>—</td><td></td><td></td><td></td></tr>';
                }
            }

            rowsEl.innerHTML = html;
        }

        async function tick() {
            try {
                const resp = await fetch('/leaderboard?' + qs.toString(), { cache: 'no-store' });
                if (!resp.ok) throw new Error('HTTP ' + resp.status);
                const data = await resp.json();
                modeEl.textContent = 'Mode: ' + (data.mode ?? '--');
                eventEl.textContent = 'Event: ' + (data.current_official_event_id ?? '--');
                renderRows(data.entries || []);
                statusEl.textContent = '';
            } catch (_err) {
                modeEl.textContent = 'Mode: --';
                eventEl.textContent = 'Event: --';
                rowsEl.innerHTML = '<tr><td colspan=\"4\" style=\"color:#9ab0d0\">Waiting for data…</td></tr>';
                statusEl.textContent = 'Waiting for data…';
            }
        }

        tick();
        setInterval(tick, 1000);
    </script>
</body>
</html>
"""

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path.startswith("/assets/"):
            assets_root = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "..", "assets")
            )
            rel_path = parsed.path[len("/assets/"):]
            requested_path = os.path.abspath(os.path.join(assets_root, rel_path))

            if not requested_path.startswith(assets_root + os.sep):
                self._send_json(404, {"error": "not_found"})
                return

            if not os.path.isfile(requested_path):
                self._send_json(404, {"error": "not_found"})
                return

            ext = os.path.splitext(requested_path)[1].lower()
            if ext == ".png":
                content_type = "image/png"
            elif ext in {".jpg", ".jpeg"}:
                content_type = "image/jpeg"
            elif ext == ".svg":
                content_type = "image/svg+xml"
            else:
                content_type = "application/octet-stream"

            try:
                with open(requested_path, "rb") as f:
                    data = f.read()
                self._send_bytes(200, data, content_type)
            except Exception:
                self._send_json(404, {"error": "not_found"})
            return

        if parsed.path == "/":
            self._send_html(200, self._kiosk_html())
            return

        if parsed.path != "/leaderboard":
            self._send_json(404, {"error": "not_found"})
            return

        qs = parse_qs(parsed.query)
        try:
            limit = int(qs.get("limit", ["10"])[0])
        except Exception:
            limit = 10
        limit = max(1, min(200, limit))

        mode_override = qs.get("mode", [None])[0]
        effective_mode = _normalize_leaderboard_mode(mode_override) if mode_override else _normalize_leaderboard_mode(LEADERBOARD_MODE)
        event_override_raw = qs.get("event_id", [None])[0]
        event_override = str(event_override_raw).strip() if event_override_raw is not None else None
        if event_override == "":
            event_override = None

        rows = _load_leaderboard_records()
        rows = [r for r in rows if _normalize_leaderboard_mode(r.get("mode", "DEV")) == effective_mode]
        if event_override is not None:
            rows = [r for r in rows if str(r.get("event_id", "")).strip() == event_override]
        rows.sort(
            key=lambda r: (
                -int(r.get("final_height", 0)),
                float(r.get("completion_time_s", 1e12)),
                float(r.get("ended_at_unix", 0.0)),
            )
        )

        top = rows[:limit]
        ranked = []
        for idx, row in enumerate(top, start=1):
            item = dict(row)
            item["rank"] = idx
            ranked.append(item)

        self._send_json(
            200,
            {
                "mode": effective_mode,
                "event_id": event_override,
                "current_official_event_id": OFFICIAL_EVENT_ID,
                "limit": limit,
                "count": len(ranked),
                "entries": ranked,
            },
        )

    def log_message(self, format, *args):
        return


def leaderboard_http_server() -> None:
    server = ThreadingHTTPServer(("0.0.0.0", LEADERBOARD_PORT), _LeaderboardHandler)
    info("STACK", f"[LEADERBOARD] HTTP server listening on {LEADERBOARD_PORT}")
    try:
        while not STOP_EVENT.is_set():
            server.handle_request()
    finally:
        try:
            server.server_close()
        except Exception:
            pass


_load_leaderboard_mode()
_apply_console_verbosity()


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
import camera_streamer

def camera_server_wrapper(mxid, port, label):
    if (not CAMERA_STREAM_ENABLED) or (dai is None):
        return
    enable_rawL = (label == "INSPECTOR" and VISION_MODE_ENABLED and PERC_AVAILABLE)
    
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
    global LEADERBOARD_MODE, OFFICIAL_EVENT_ID
    global _last_nudge_t

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
        _reset_drift_scale_for_run("NAME")
        _send_line_to_unity("NAME_SET")
        log_event("EVENT_NAME_SET", participant=participant_name, source=source)
        console_info("CONTROL", f"Participant set: {participant_name}. Ready to START.", essential=True)
        return

    if upper_cmd == "TUMBLE":
        # Preempt any in-flight or queued stack continuation immediately.
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
        blocks_placed = current_stack_level
        run_time_s = emit_run_summary("TUMBLE")
        finalize_run("TUMBLE")
        _send_line_to_unity("RUN_FAIL TUMBLE")

        log_event(
            "EVENT_RUN_SUMMARY",
            participant=participant_name,
            blocks_placed=blocks_placed,
            run_time_s=run_time_s,
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
        if _is_official_mode() and vr_connected:
            console_emit("Waiting for participant name...", tag="PROMPT", level="INFO", module="CONTROL", allow_in_quiet=True)
        return

    if upper_cmd == "MODE SHOW":
        console_emit(f"[LEADERBOARD] MODE={LEADERBOARD_MODE} EVENT={OFFICIAL_EVENT_ID}", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd == "MODE DEV":
        LEADERBOARD_MODE = "DEV"
        _save_leaderboard_mode()
        console_emit(f"[LEADERBOARD] MODE set to DEV (event_id={OFFICIAL_EVENT_ID})", tag="PROMPT", level="INFO", module=module, allow_in_quiet=True)
        return

    if upper_cmd == "MODE OFFICIAL":
        LEADERBOARD_MODE = "OFFICIAL"
        _save_leaderboard_mode()
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
        _save_leaderboard_mode()
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
            drop_committed_this_window = False
            proposed_place_pose = None
            proposed_place_stack_level = None
            current_zone = "GREEN"
            current_zone_stack_level = None
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
        warn(module, f"[VISION] pick pose unavailable: {reason}")
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

            if current_stack_level >= 7:
                if event == Event.START_STACK:
                    send_nack("START", "BAD_STATE")
                warn(module, "[STACK] Tower full. Ignoring START.")
                return

            # Run start timing (monotonic) when a new run actually begins
            if current_stack_level == 0:
                _last_ready_level_printed = None
                current_session_token = uuid4()
                tower_attempt_start_ts = time.monotonic()
                run_start_time = tower_attempt_start_ts
                run_id = uuid4().hex
                run_finalized = False
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

            if event == Event.START_STACK:
                send_ack("START")

            # Execute pick sequence
            my_token = current_session_token
            side, level = cfg.PICK_SEQUENCE[current_pick_index]
            handles.combo_active = combo_active
            try:
                actions.execute_pick_sequence(handles, side, level)
            except actions.PickPoseUnavailableError as e:
                if VISION_MODE_ENABLED:
                    _handle_vision_pick_unavailable(e.reason)
                    return
                raise
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
                    )
                else:
                    actions.complete_place_sequence(
                        handles,
                        current_stack_level,
                        perform_neutral_exit=False,
                    )
                # Immediate RobotMode check after motion
                m = handles.robot.robot_mode()
                if m in (9, 11):
                    green_place_streak = 0
                    if combo_active:
                        send_boost_end()
                        combo_active = False
                        handles.combo_active = combo_active
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
                            warn("COMBO", "combo ended")
                        send_boost_state(0, False)

                    combo_target = int(getattr(cfg, "COMBO_GREEN_PLACEMENTS_TARGET", 3))
                    if combo_target > 0 and green_place_streak >= combo_target:
                        participant = participant_name.strip() if isinstance(participant_name, str) and participant_name.strip() else "UNKNOWN"
                        warn("COMBO", f"{participant} combo achieved: {combo_target}x GREEN placements")
                        combo_active = True
                        handles.combo_active = combo_active
                        send_boost_state(combo_target, True)
                        green_place_streak = 0
                    elif zone_at_commit == "GREEN":
                        send_boost_state(green_place_streak, False)

                handles.combo_active = combo_active
                actions.complete_place_neutral_exit(handles, current_stack_level)

                if current_zone_stack_level == current_stack_level and current_zone == "RED":
                    cfg.DRIFT_SCALE = min(3.0, float(cfg.DRIFT_SCALE) + float(cfg.DRIFT_RISK_INCREMENT))
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
            current_stack_level += 1
            current_pick_index += 1

            tower_complete = current_stack_level >= target_stack_count

            # Emit internal progression event
            if my_token != current_session_token:
                return
            result4 = step(STATE, Event.PLACE_COMPLETE)
            if result4.allowed:
                STATE = result4.next_state
                info(module, f"[SM] -> {STATE.name} (PLACE_COMPLETE)")

                if tower_complete:
                    emit_run_summary("COMPLETE")
                    finalize_run("COMPLETE")
                    _send_line_to_unity(f"RUN_COMPLETE {current_stack_level}")
                    time.sleep(5.0)
                    console_emit("[STACK] COMPLETE summary emitted (post-wait)", tag="SUMMARY", level="INFO", module=module, allow_in_quiet=True)

                    current_pick_index = 0
                    current_stack_level = 0
                    holding_block = False
                    proposed_place_pose = None
                    proposed_place_stack_level = None
                    block_attempt_start_ts = None
                    drop_committed_this_window = False
                    tower_attempt_start_ts = None
                    run_start_time = None
                    participant_name = None
                    current_run_seed = None
                    cfg.DRIFT_RUNTIME_RUN_SEED = None
                    cfg.DRIFT_RUNTIME_PARTICIPANT = ""
                    _last_ready_level_printed = None
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
                        side, level = cfg.PICK_SEQUENCE[current_pick_index]
                        handles.combo_active = combo_active
                        try:
                            actions.execute_pick_sequence(handles, side, level)
                        except actions.PickPoseUnavailableError as e:
                            if VISION_MODE_ENABLED:
                                warn(module, f"[VISION] pick pose unavailable: {e.reason}")
                                _send_line_to_unity("VISION_STATUS FAIL")
                                STATE = State.WAITING_FOR_REPOSITION
                                info(module, "[SM] -> WAITING_FOR_REPOSITION (VISION pick unavailable)")
                                return
                            raise
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
                proposed_place_pose = None
                proposed_place_stack_level = None
                current_zone = "GREEN"
                current_zone_stack_level = None
                robot_armed = True
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
            # Dev 8 autonomous stacking loop controls
            stacking_enabled = True
            target_stack_count = min(7, len(cfg.PICK_SEQUENCE))
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
                if _is_official_mode():
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
_start_worker_thread("command-server", command_server)
_start_worker_thread("admin-server", admin_server)
_start_worker_thread("leaderboard-http", leaderboard_http_server)
if CAMERA_STREAM_ENABLED and (dai is not None):
    _start_worker_thread("camera-inspector", camera_server_wrapper, args=(MXID_INSPECTOR, UNITY_PORT_INSPECTOR, "INSPECTOR"))
    time.sleep(10)
    _start_worker_thread("camera-site-manager", camera_server_wrapper, args=(MXID_MANAGER, UNITY_PORT_MANAGER, "SITE_MANAGER"))
if _is_official_mode():
    console_emit("Start Unity and press Play...", tag="UNITY", level="INFO", module="CONTROL", allow_in_quiet=True)
_start_worker_thread("facilitator-hotkey", facilitator_hotkey_loop, daemon=True)

info("CONTROL", "TASK CONTROLLER ACTIVE. Press Ctrl+C to stop.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    info("CONTROL", "[MAIN] Ctrl+C received. Initiating graceful shutdown...")
    STOP_EVENT.set()
    cleanup_errors = _close_all_tracked_sockets()
    alive_threads = _join_worker_threads(timeout_s=1.0)

    if alive_threads:
        if _is_debug_enabled():
            warn("CONTROL", f"[MAIN] Threads still alive after join timeout: {alive_threads}")
        cleanup_errors.append(RuntimeError("Worker threads still alive after shutdown timeout"))

    if cleanup_errors:
        if _is_debug_enabled():
            warn("CONTROL", f"[MAIN] Shutdown encountered {len(cleanup_errors)} issue(s). Exiting with code 1.")
        sys.exit(1)

    info("CONTROL", "[MAIN] Shutdown complete.")
    if _is_official_mode():
        os._exit(0)
    sys.exit(0)
