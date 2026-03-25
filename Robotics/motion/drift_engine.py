"""
drift_engine.py

Deterministic XY drift injection for placement poses.

Design constraints:
- Pure function: no hardware calls.
- Deterministic per (seed, stack_level).
- Modifies only X and Y.
- Never modifies Z or orientation.
- Returns new pose tuple.
"""

from __future__ import annotations
import hashlib
import random
from typing import Optional, Tuple
import robot_config as cfg
from logger import info

Pose = Tuple[float, float, float, float, float, float]


def _stable_seed(
    baseline_seed: int,
    runtime_run_seed: int,
    participant: str,
    stack_level: int,
) -> Tuple[int, str]:
    """
    Create deterministic integer seed from baseline seed + runtime seed + participant + stack level.
    """
    key = f"{baseline_seed}:{runtime_run_seed}:{participant}:{stack_level}".encode()
    digest = hashlib.sha256(key).hexdigest()
    return int(digest[:16], 16), digest


def compute_drift(
    stack_level: int,
    run_seed: Optional[int] = None,
    participant: Optional[str] = None,
) -> Tuple[float, float]:
    """
    Compute deterministic XY drift for a given stack level.
    """
    if not cfg.DRIFT_ENABLED:
        return 0.0, 0.0

    # Keep the first base block centered; drift starts from level 1.
    if int(stack_level) == 0:
        return 0.0, 0.0

    effective_max = cfg.DRIFT_MAX_XY_MM * cfg.DRIFT_SCALE

    if effective_max <= 0:
        return 0.0, 0.0

    if getattr(cfg, "DRIFT_USE_RUNTIME_SEED", True):
        if run_seed is None:
            run_seed = getattr(cfg, "DRIFT_RUNTIME_RUN_SEED", None)
        if participant is None:
            participant = getattr(cfg, "DRIFT_RUNTIME_PARTICIPANT", "")

    runtime_run_seed = int(run_seed) if run_seed is not None else 0
    participant_value = str(participant or "")

    seed, seed_hex = _stable_seed(
        int(cfg.DRIFT_RUN_SEED),
        runtime_run_seed,
        participant_value,
        stack_level,
    )
    rng = random.Random(seed)

    if cfg.DRIFT_MODE == "fixed":
        dx = effective_max
        dy = 0.0

    elif cfg.DRIFT_MODE == "grid":
        # deterministic discrete steps
        step = effective_max / 2.0
        dx = step if (stack_level % 2 == 0) else -step
        dy = step if (stack_level % 3 == 0) else -step

    else:  # default "uniform"
        dx = rng.uniform(-effective_max, effective_max)
        dy = rng.uniform(-effective_max, effective_max)

    axis_mode = getattr(cfg, "DRIFT_AXIS_MODE", "XY").upper()

    if axis_mode == "X":
        dy = 0.0
    elif axis_mode == "Y":
        dx = 0.0
    # else: XY = no change

    if bool(getattr(cfg, "DEBUG_ENABLED", False)):
        info(
            "DRIFT",
            f"level={stack_level} seed={seed_hex[:8]} runtime_seed={runtime_run_seed} participant={participant_value or 'UNKNOWN'} dx={dx:.3f} dy={dy:.3f}",
        )

    return dx, dy


def inject_drift(
    base_pose: Pose,
    stack_level: int,
    run_seed: Optional[int] = None,
    participant: Optional[str] = None,
) -> Pose:
    """
    Return a new pose with XY drift applied.
    Z and orientation remain untouched.
    """
    dx, dy = compute_drift(stack_level, run_seed=run_seed, participant=participant)

    x, y, z, rx, ry, rz = base_pose

    drifted_pose = (
        x + dx,
        y + dy,
        z,
        rx,
        ry,
        rz,
    )

    return drifted_pose