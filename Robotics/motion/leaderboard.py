"""
leaderboard.py — Side Quest leaderboard subsystem.

Contains:
  - LeaderboardContext: small shared-state object (mode + event id)
  - Path/mode helpers
  - log_event, emit_run_summary, finalize_run
  - _LeaderboardHandler, leaderboard_http_server

Imports: robot_config (for LOG_DIR, TOWER_LEVELS, DRIFT_SCALE), logger, stdlib only.
No import of task_controller — no circular dependency.
"""

import time
import os
import json
import tempfile
from uuid import uuid4
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import robot_config as cfg
from logger import info, warn


# ---------------------------------------------------------------------------
# Shared mutable state between task_controller and the HTTP server
# ---------------------------------------------------------------------------

@dataclass
class LeaderboardContext:
    """
    Tiny mutable container for leaderboard mode and event id.

    task_controller holds the instance and updates mode / official_event_id
    whenever MODE or EVENT commands change them.  _LeaderboardHandler reads
    these fields at request time, so the HTTP kiosk always reflects the
    latest values without any extra IPC.
    """
    mode: str = "DEV"
    official_event_id: str = "ARC2026"


# ---------------------------------------------------------------------------
# Mode helpers
# ---------------------------------------------------------------------------

def normalize_leaderboard_mode(value) -> str:
    return "OFFICIAL" if str(value).strip().upper() == "OFFICIAL" else "DEV"


# ---------------------------------------------------------------------------
# Disk path helpers
# ---------------------------------------------------------------------------

def _current_log_subdir(mode: str) -> str:
    base_log_dir = getattr(cfg, "LOG_DIR", "logs")
    subdir = "official" if normalize_leaderboard_mode(mode) == "OFFICIAL" else "dev"
    path = os.path.join(base_log_dir, subdir)
    os.makedirs(path, exist_ok=True)
    return path


def _leaderboard_mode_path() -> str:
    log_dir = getattr(cfg, "LOG_DIR", "logs")
    os.makedirs(log_dir, exist_ok=True)
    return os.path.join(log_dir, "leaderboard_mode.json")


def _leaderboard_jsonl_path(mode: str) -> str:
    return os.path.join(_current_log_subdir(mode), "leaderboard.jsonl")


def _leaderboard_audit_jsonl_path(mode: str) -> str:
    return os.path.join(_current_log_subdir(mode), "leaderboard_audit.jsonl")


# ---------------------------------------------------------------------------
# Mode persistence
# ---------------------------------------------------------------------------

def save_leaderboard_mode(ctx: LeaderboardContext) -> None:
    payload = {
        "mode": ctx.mode,
        "official_event_id": ctx.official_event_id,
        "updated_at_unix": time.time(),
    }
    path = _leaderboard_mode_path()
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
    except Exception as e:
        warn("STACK", f"[LEADERBOARD] Failed to persist mode config: {e}")


def load_leaderboard_mode(ctx: LeaderboardContext) -> None:
    """Read saved mode/event from disk into ctx.  Creates the file if missing."""
    path = _leaderboard_mode_path()
    if not os.path.exists(path):
        save_leaderboard_mode(ctx)
        return

    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except Exception as e:
        warn("STACK", f"[LEADERBOARD] Failed to read mode config, using defaults: {e}")
        return

    if isinstance(raw, dict):
        ctx.mode = normalize_leaderboard_mode(raw.get("mode", ctx.mode))
        event_value = str(raw.get("official_event_id", ctx.official_event_id)).strip()
        ctx.official_event_id = event_value if event_value else ctx.official_event_id

    info("STACK", f"[LEADERBOARD] Mode loaded: mode={ctx.mode} event_id={ctx.official_event_id}")


# ---------------------------------------------------------------------------
# Record loading
# ---------------------------------------------------------------------------

def _load_leaderboard_records(mode: str) -> list:
    path = _leaderboard_jsonl_path(mode)
    if not os.path.exists(path):
        return []

    rows = []
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


def _atomic_write_jsonl(path: str, rows: list[dict]) -> None:
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix="leaderboard_", suffix=".jsonl.tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            for row in rows:
                f.write(json.dumps(row, ensure_ascii=False) + "\n")
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except Exception:
            pass
        raise


def override_run_score(
    *,
    mode: str,
    run_id: str,
    new_final_height: int,
) -> dict:
    path = _leaderboard_jsonl_path(mode)
    rows = _load_leaderboard_records(mode)
    updated = False
    original_row = None
    updated_row = None

    for row in rows:
        if str(row.get("run_id", "")).strip() != str(run_id).strip():
            continue
        original_row = dict(row)
        row["final_height"] = int(new_final_height)
        row["score_overridden_at_unix"] = time.time()
        updated_row = dict(row)
        updated = True
        break

    if not updated or original_row is None or updated_row is None:
        raise ValueError(f"Run not found for override: {run_id}")

    _atomic_write_jsonl(path, rows)
    return {
        "path": path,
        "original": original_row,
        "updated": updated_row,
    }


def append_score_override_audit(
    *,
    mode: str,
    run_id: str,
    session_id,
    participant_name,
    old_final_height: int,
    new_final_height: int,
    reason: str,
    source: str,
) -> str:
    path = _leaderboard_audit_jsonl_path(mode)
    record = {
        "timestamp": time.time(),
        "mode": normalize_leaderboard_mode(mode),
        "run_id": run_id,
        "session_id": session_id,
        "participant_name": participant_name,
        "old_final_height": int(old_final_height),
        "new_final_height": int(new_final_height),
        "reason": reason,
        "source": source,
    }
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")
    return path


# ---------------------------------------------------------------------------
# Scoring / logging functions (parameterized — no module-global reads)
# ---------------------------------------------------------------------------

def log_event(
    *,
    event_type: str,
    leaderboard_mode: str,
    session_id,
    participant_name,
    state,
    current_stack_level,
    current_pick_index,
    **fields,
) -> None:
    """Append one JSONL event record.  No-op in DEV mode."""
    if normalize_leaderboard_mode(leaderboard_mode) != "OFFICIAL":
        return

    try:
        log_dir = _current_log_subdir(leaderboard_mode)
        sid = session_id or "no_session"
        log_path = os.path.join(log_dir, f"session_{sid}.jsonl")

        record = {
            "timestamp": time.time(),
            "event": event_type,
            "session_id": session_id,
            "participant_name": participant_name,
            "state": state.name if hasattr(state, "name") else str(state),
            "current_stack_level": current_stack_level,
            "current_pick_index": current_pick_index,
        }
        if hasattr(cfg, "DRIFT_SCALE"):
            record["drift_scale"] = cfg.DRIFT_SCALE
        if fields:
            record.update(fields)

        with open(log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except Exception as e:
        warn("CONTROL", f"[LOG] Failed to write event {event_type}: {e}")


def emit_run_summary(
    reason: str,
    *,
    run_start_time,
    participant_name,
    current_stack_level,
    end_time_mono=None,
) -> float:
    now_mono = end_time_mono if end_time_mono is not None else time.monotonic()
    start_ts = run_start_time if run_start_time is not None else now_mono
    duration_s = max(0.0, now_mono - start_ts)
    participant = participant_name.strip() if isinstance(participant_name, str) and participant_name.strip() else "UNKNOWN"
    placed = int(current_stack_level) if current_stack_level is not None else 0
    summary_reason = reason.strip().upper() if isinstance(reason, str) and reason.strip() else "UNKNOWN"
    warn("STACK", f"{participant} successfully placed {placed} blocks in {duration_s:.1f} seconds ({summary_reason})")
    return duration_s


def finalize_run(
    end_state: str,
    *,
    ctx: LeaderboardContext,
    run_id,
    session_id,
    participant_name,
    current_stack_level,
    run_start_time,
    already_finalized: bool,
    end_time_mono=None,
) -> bool:
    """
    Write a finalized run record to leaderboard.jsonl.

    Returns True if the record was written (caller should set run_finalized=True).
    Returns False if already finalized or if the write failed.
    """
    if already_finalized:
        return False

    ended_state = str(end_state).strip().upper() if isinstance(end_state, str) else "UNKNOWN"
    target_height = int(getattr(cfg, "TOWER_LEVELS", 7))
    stable_height = int(current_stack_level or 0)
    final_height = max(0, min(stable_height, target_height))

    now_mono = end_time_mono if end_time_mono is not None else time.monotonic()
    start_ts = run_start_time if run_start_time is not None else now_mono
    completion_time_s = max(0.0, now_mono - start_ts)

    finalized_run_id = run_id or uuid4().hex
    mode = normalize_leaderboard_mode(ctx.mode)
    event_id = ctx.official_event_id if mode == "OFFICIAL" else "DEV"

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

    path = _leaderboard_jsonl_path(ctx.mode)
    try:
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        info("STACK", f"[LEADERBOARD] Finalized run_id={finalized_run_id} state={ended_state} height={final_height}/{target_height} time={completion_time_s:.3f}s")
        return True
    except Exception as e:
        warn("STACK", f"[LEADERBOARD] Failed to append leaderboard record: {e}")
        return False


# ---------------------------------------------------------------------------
# HTTP kiosk handler
# ---------------------------------------------------------------------------

class _LeaderboardHandler(BaseHTTPRequestHandler):
    # Set by leaderboard_http_server via the BoundHandler subclass.
    _ctx: LeaderboardContext = None  # type: ignore[assignment]

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
        @font-face {
            font-family: "Futura Custom";
            src: local("Futura"), local("Futura PT"), url("/assets/Futura.ttf") format("truetype");
            font-style: normal;
            font-weight: 400;
            font-display: swap;
        }

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
            background: #425ba6;
            color: var(--text);
            display: flex;
            align-items: stretch;
            justify-content: center;
        }
        .wrap {
            width: 100%;
            padding: 0 0 14px 0;
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
            margin-bottom: 0px;
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
            height: 75px;
            max-width: 26%;
            object-fit: contain;
        }
        h1 {
            margin: 6px 0 2px 0;
            letter-spacing: 1px;
            font-size: clamp(48px, 6.5vw, 96px);
            text-transform: uppercase;
            text-align: center;
            white-space: nowrap;
            font-family: "Futura Custom", Futura, "Futura PT", "Trebuchet MS", Inter, sans-serif;
            color: #ffffff;
        }
        .subhead {
            margin: 0 0 10px 0;
            text-align: center;
            text-transform: uppercase;
            letter-spacing: 0.6px;
            font-size: clamp(24px, 3.25vw, 48px);
            white-space: nowrap;
            font-family: "Futura Custom", Futura, "Futura PT", "Trebuchet MS", Inter, sans-serif;
            color: #ffffff;
            line-height: 1;
        }
        .countdown {
            text-align: center;
            color: #d7e6ff;
            font-weight: 700;
            letter-spacing: 0.4px;
            font-size: clamp(14px, 1.5vw, 24px);
            margin: 2px 0 8px 0;
            min-height: 1.4em;
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
            width: min(1200px, 100%);
            justify-self: center;
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
        <div class=\"subhead\">HIGH SCORE</div>
        <div class="countdown" id="countdown">Challenge finishes in: --</div>
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
        const countdownEl = document.getElementById('countdown');
        const rowsEl = document.getElementById('rows');
        const statusEl = document.getElementById('status');
        const countdownTarget = new Date(2026, 2, 29, 15, 0, 0); // Mar 29, 2026 15:00 local time

        function updateCountdown() {
            const now = new Date();
            const diffMs = countdownTarget.getTime() - now.getTime();

            if (diffMs <= 0) {
                countdownEl.textContent = 'Challenge finishes in: Event started';
                return;
            }

            const totalSeconds = Math.floor(diffMs / 1000);
            const days = Math.floor(totalSeconds / 86400);
            const hours = Math.floor((totalSeconds % 86400) / 3600);
            const minutes = Math.floor((totalSeconds % 3600) / 60);
            const seconds = totalSeconds % 60;

            const hh = String(hours).padStart(2, '0');
            const mm = String(minutes).padStart(2, '0');
            const ss = String(seconds).padStart(2, '0');
            countdownEl.textContent = `Challenge finishes in: ${days}d ${hh}h ${mm}m ${ss}s`;
        }

        function fmtTime(v) {
            const n = Number(v);
            if (!Number.isFinite(n)) return '--';
            const clamped = Math.max(0, n);
            const minutes = Math.floor(clamped / 60);
            const seconds = clamped - (minutes * 60);
            return `${String(minutes).padStart(2, '0')}:${seconds.toFixed(2).padStart(5, '0')}`;
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

        updateCountdown();
        tick();
        setInterval(updateCountdown, 1000);
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
            elif ext == ".ttf":
                content_type = "font/ttf"
            elif ext == ".otf":
                content_type = "font/otf"
            elif ext == ".woff":
                content_type = "font/woff"
            elif ext == ".woff2":
                content_type = "font/woff2"
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
        effective_mode = normalize_leaderboard_mode(mode_override) if mode_override else normalize_leaderboard_mode(self._ctx.mode)
        event_override_raw = qs.get("event_id", [None])[0]
        event_override = str(event_override_raw).strip() if event_override_raw is not None else None
        if event_override == "":
            event_override = None

        rows = _load_leaderboard_records(effective_mode)
        rows = [r for r in rows if normalize_leaderboard_mode(r.get("mode", "DEV")) == effective_mode]
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
                "current_official_event_id": self._ctx.official_event_id,
                "limit": limit,
                "count": len(ranked),
                "entries": ranked,
            },
        )

    def log_message(self, format, *args):
        return


# ---------------------------------------------------------------------------
# HTTP server entry point
# ---------------------------------------------------------------------------

def leaderboard_http_server(ctx: LeaderboardContext, stop_event, port: int) -> None:
    """Start the leaderboard kiosk HTTP server.  Runs until stop_event is set."""

    # Bind ctx to the handler class via a local subclass so the ThreadingHTTPServer
    # can instantiate handlers for each request with the correct context reference.
    class BoundHandler(_LeaderboardHandler):
        _ctx = ctx

    server = ThreadingHTTPServer(("0.0.0.0", port), BoundHandler)
    info("STACK", f"[LEADERBOARD] HTTP server listening on {port}")
    try:
        while not stop_event.is_set():
            server.handle_request()
    finally:
        try:
            server.server_close()
        except Exception:
            pass
