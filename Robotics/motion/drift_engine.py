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
from typing import Tuple
import robot_config as cfg

Pose = Tuple[float, float, float, float, float, float]


def _stable_seed(run_seed: int, stack_level: int) -> int:
    """
    Create deterministic integer seed from run_seed + stack_level.
    """
    key = f"{run_seed}:{stack_level}".encode()
    digest = hashlib.sha256(key).hexdigest()
    return int(digest[:16], 16)


def compute_drift(stack_level: int) -> Tuple[float, float]:
    """
    Compute deterministic XY drift for a given stack level.
    """
    if not cfg.DRIFT_ENABLED:
        return 0.0, 0.0

    effective_max = cfg.DRIFT_MAX_XY_MM * cfg.DRIFT_SCALE

    if effective_max <= 0:
        return 0.0, 0.0

    seed = _stable_seed(cfg.DRIFT_RUN_SEED, stack_level)
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

    return dx, dy


def inject_drift(base_pose: Pose, stack_level: int) -> Pose:
    """
    Return a new pose with XY drift applied.
    Z and orientation remain untouched.
    """
    dx, dy = compute_drift(stack_level)

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