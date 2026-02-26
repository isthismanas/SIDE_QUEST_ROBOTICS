"""
state_machine.py

A minimal, explicit state machine for Side Quest Dev 7+.

Goals:
- No robot/gripper code here.
- Only: states, events, transition rules, and gating logic.
- task_controller.py remains orchestration + TCP.

Design notes:
- Unity sends string commands. task_controller maps those to Event.
- State machine decides whether the event is legal, and what the next state is.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Tuple


class State(Enum):
    IDLE = auto()
    MOVING_TO_PICK = auto()
    MOVING_TO_TOWER_HOVER = auto()
    WAITING_FOR_DECISION = auto()
    NUDGE = auto()
    PLACING = auto()
    FAULT = auto()


class Event(Enum):
    # Session / safety boundary
    CONNECT = auto()
    DISCONNECT = auto()

    # Supervisor intents
    HOME = auto()
    START_STACK = auto()
    FIX = auto()
    DROP = auto()
    CANCEL = auto()

    # Progression events (from task controller)
    PICK_COMPLETE = auto()
    AT_TOWER_HOVER = auto()
    PLACE_COMPLETE = auto()

    # Nudges (only legal in NUDGE)
    NUDGE_XY = auto()
    NUDGE_YAW = auto()

    # Gripper commands (allowed broadly; controller still safety-gates)
    GRIP_OPEN = auto()
    GRIP_CLOSE = auto()
    GRIP_TOGGLE = auto()

    # Optional future hooks
    FAULT = auto()
    CLEAR_FAULT = auto()
    AUTO_RECOVER = auto()


@dataclass(frozen=True)
class TransitionResult:
    allowed: bool
    next_state: State
    reason: str = ""


# Transition table: (current_state, event) -> next_state
# If a pair isn't in the table, it is considered NOT allowed (except "always allowed" events below).
_TRANSITIONS: Dict[Tuple[State, Event], State] = {
    # Start stacking
    (State.IDLE, Event.START_STACK): State.MOVING_TO_PICK,

    # After pick sequence completes
    (State.MOVING_TO_PICK, Event.PICK_COMPLETE): State.MOVING_TO_TOWER_HOVER,

    # After robot reaches tower hover
    (State.MOVING_TO_TOWER_HOVER, Event.AT_TOWER_HOVER): State.WAITING_FOR_DECISION,

    # Decision state: supervisor chooses to adjust or place
    (State.WAITING_FOR_DECISION, Event.FIX): State.NUDGE,
    (State.WAITING_FOR_DECISION, Event.DROP): State.PLACING,

    # Nudge exits
    (State.NUDGE, Event.CANCEL): State.WAITING_FOR_DECISION,
    (State.NUDGE, Event.DROP): State.PLACING,

    # After placement completes
    (State.PLACING, Event.PLACE_COMPLETE): State.IDLE,

    # HOME resets safely
    (State.IDLE, Event.HOME): State.IDLE,
    (State.WAITING_FOR_DECISION, Event.HOME): State.IDLE,
    (State.NUDGE, Event.HOME): State.IDLE,
    (State.MOVING_TO_PICK, Event.HOME): State.IDLE,
    (State.MOVING_TO_TOWER_HOVER, Event.HOME): State.IDLE,
    (State.PLACING, Event.HOME): State.IDLE,

    # Fault handling
    (State.IDLE, Event.FAULT): State.FAULT,
    (State.WAITING_FOR_DECISION, Event.FAULT): State.FAULT,
    (State.NUDGE, Event.FAULT): State.FAULT,
    (State.PLACING, Event.FAULT): State.FAULT,
    (State.MOVING_TO_PICK, Event.FAULT): State.FAULT,
    (State.MOVING_TO_TOWER_HOVER, Event.FAULT): State.FAULT,
    (State.FAULT, Event.CLEAR_FAULT): State.IDLE,
    (State.FAULT, Event.AUTO_RECOVER): State.IDLE,
    (State.FAULT, Event.HOME): State.IDLE,
}

# Events that are *state-invariant* (allowed; state doesn't change here)
_ALWAYS_ALLOWED = {
    Event.GRIP_OPEN,
    Event.GRIP_CLOSE,
    Event.GRIP_TOGGLE,
    Event.CONNECT,
    Event.DISCONNECT,
    Event.AUTO_RECOVER,
}

# Events that are only meaningful in certain states but don't necessarily change state
# (we still gate them here)
_STATE_GATED_ONLY = {
    Event.NUDGE_XY: {State.NUDGE},
    Event.NUDGE_YAW: {State.NUDGE},
}


def step(state: State, event: Event) -> TransitionResult:
    """
    Returns whether 'event' is allowed in 'state', and what the next state should be.
    If allowed and no explicit transition exists, next_state==state.
    
    Explicit transitions take priority, allowing overrides like AUTO_RECOVER from FAULT.
    """
    # Check explicit state transitions first (highest priority)
    key = (state, event)
    if key in _TRANSITIONS:
        return TransitionResult(True, _TRANSITIONS[key])

    # Check state-gated events (allowed only in specific states)
    if event in _STATE_GATED_ONLY:
        allowed_states = _STATE_GATED_ONLY[event]
        if state in allowed_states:
            return TransitionResult(True, state)
        return TransitionResult(False, state, reason=f"{event.name} not allowed in {state.name}")

    # Check events allowed from any state (preserve state)
    if event in _ALWAYS_ALLOWED:
        return TransitionResult(True, state)

    # Provide contextual error message for FAULT state
    if state == State.FAULT:
        return TransitionResult(
            False, 
            state, 
            reason="Robot in FAULT state. Allowed exits: AUTO_RECOVER, HOME (both trigger recovery), or CLEAR_FAULT."
        )

    return TransitionResult(False, state, reason=f"No transition for {state.name} + {event.name}")


def parse_event(cmd: str) -> Tuple[Event, dict]:
    """
    Parse Unity command strings into (Event, payload).
    payload is a dict with extracted parameters (dx/dy/etc).
    """
    s = cmd.strip()
    if not s:
        raise ValueError("Empty command")

    parts = s.split()

    # Normalize some synonyms
    if parts[0].upper() == "COMMIT":
        parts[0] = "DROP"

    head = parts[0].upper()

    if head == "HOME":
        return Event.HOME, {}
    if head == "FIX":
        return Event.FIX, {}
    if head == "DROP":
        return Event.DROP, {}
    if head == "CANCEL":
        return Event.CANCEL, {}

    if head == "NUDGE":
        if len(parts) != 3:
            raise ValueError("NUDGE requires: NUDGE <dx> <dy>")
        dx = float(parts[1])
        dy = float(parts[2])
        return Event.NUDGE_XY, {"dx": dx, "dy": dy}

    if head == "NUDGE_YAW":
        if len(parts) != 2:
            raise ValueError("NUDGE_YAW requires: NUDGE_YAW <dtheta>")
        dtheta = float(parts[1])
        return Event.NUDGE_YAW, {"dtheta": dtheta}

    if head == "GRIP_OPEN":
        return Event.GRIP_OPEN, {}
    if head == "GRIP_CLOSE":
        return Event.GRIP_CLOSE, {}
    if head == "GRIP_TOGGLE":
        return Event.GRIP_TOGGLE, {}

    if head == "START":
        return Event.START_STACK, {}

    if head == "AUTO_RECOVER":
        return Event.AUTO_RECOVER, None

    raise ValueError(f"Unknown command: {cmd}")