#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from uuid import uuid4

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MOTION_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "motion"))

if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)
if MOTION_DIR not in sys.path:
    sys.path.append(MOTION_DIR)

import actions
import robot_config as cfg
from logger import set_jsonl_context, write_jsonl_event
from vision_pick_place_once import _close_system_handles, _create_system_handles, _initialize_cycle_session


DEFAULT_STATE_JSON = os.path.join(THIS_DIR, "logs", "vision_pick_assist_state.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Standalone deterministic fallback for the remaining hardcoded pickup slots "
            "after a vision-assist run."
        )
    )
    parser.add_argument(
        "--state-json",
        default=DEFAULT_STATE_JSON,
        help="Path to the persisted vision-assist state JSON.",
    )
    parser.add_argument(
        "--place-level-start",
        type=int,
        default=None,
        help="Optional explicit 0-indexed tower level to start placing at. Defaults to the claimed-slot count from state.",
    )
    parser.add_argument(
        "--max-count",
        type=int,
        default=None,
        help="Optional cap on how many remaining deterministic pickups to execute.",
    )
    parser.add_argument(
        "--home-after",
        action="store_true",
        help="Return to safe home after completion.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually execute the deterministic fallback. Without this flag the tool only prints the plan.",
    )
    return parser.parse_args()


def _load_state(path: str) -> dict[str, object]:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _remaining_targets_from_state(state: dict[str, object]) -> list[str]:
    claimed = {
        str(target).strip().upper()
        for target in state.get("placed_pick_target_ids", [])
        if str(target).strip()
    }
    ordered_targets = [str(target).strip().upper() for target in getattr(cfg, "PICK_SEQUENCE", [])]
    remaining = [target for target in ordered_targets if target not in claimed]

    expected = state.get("expected_workbench_brick_count")
    if expected is None:
        return remaining

    try:
        remaining_needed = max(0, int(expected) - len(claimed))
    except Exception:
        return remaining
    return remaining[:remaining_needed]


def _write_event(event_name: str, **fields) -> None:
    payload = {
        "event": event_name,
        "module": "PERCEPTION",
        "tool": "deterministic_remaining_pick_place",
    }
    payload.update(fields)
    write_jsonl_event("grab_pick", payload)


def main() -> int:
    args = parse_args()
    state = _load_state(str(args.state_json))
    remaining_targets = _remaining_targets_from_state(state)
    if args.max_count is not None:
        remaining_targets = remaining_targets[: max(0, int(args.max_count))]

    claimed_count = len(state.get("placed_pick_target_ids", []))
    place_level_start = int(args.place_level_start) if args.place_level_start is not None else claimed_count

    participant_name = str(state.get("participant_name") or "deterministic_remaining_pick_place").strip()
    session_id = f"detfallback-{int(time.time())}-{uuid4().hex[:8]}"
    run_id = uuid4().hex
    set_jsonl_context(
        participant_name=participant_name,
        session_id=session_id,
        run_id=run_id,
        leaderboard_mode="LAB",
    )

    print(f"[DET_FALLBACK] state_json={args.state_json}")
    print(f"[DET_FALLBACK] claimed_slots={state.get('placed_pick_target_ids', [])}")
    print(f"[DET_FALLBACK] remaining_targets={remaining_targets}")
    print(f"[DET_FALLBACK] place_level_start={place_level_start} execute={bool(args.execute)}")

    _write_event(
        "deterministic_remaining_plan",
        state_json=os.path.abspath(str(args.state_json)),
        claimed_slots=state.get("placed_pick_target_ids", []),
        remaining_targets=remaining_targets,
        place_level_start=place_level_start,
        execute=bool(args.execute),
    )

    if not args.execute:
        print("[DET_FALLBACK] dry-run only. Re-run with --execute to move the robot.")
        return 0

    if not remaining_targets:
        print("[DET_FALLBACK] no remaining targets to execute.")
        return 0

    original_pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic"))
    cfg.PICK_POSE_MODE = "deterministic"
    handles = _create_system_handles()
    executed_targets: list[str] = []
    try:
        _initialize_cycle_session(handles)
        for offset, target_id in enumerate(remaining_targets):
            stack_level = int(place_level_start) + int(offset)
            if stack_level >= int(cfg.stack_target_count()):
                print(f"[DET_FALLBACK] tower capacity reached at stack_level={stack_level}; stopping.")
                break
            print(f"[DET_FALLBACK] executing target={target_id} stack_level={stack_level}")
            actions.execute_pick_sequence(handles, target_id, stack_level)
            actions.move_to_tower_hover(handles, stack_level, target_id=target_id)
            actions.complete_place_sequence(handles, stack_level, target_id=target_id)
            executed_targets.append(str(target_id))
        if bool(args.home_after):
            actions.do_home(handles)
    finally:
        cfg.PICK_POSE_MODE = original_pick_mode
        _close_system_handles(handles)

    _write_event(
        "deterministic_remaining_complete",
        executed_targets=executed_targets,
        place_level_start=place_level_start,
    )
    print(f"[DET_FALLBACK] complete executed_targets={executed_targets}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
