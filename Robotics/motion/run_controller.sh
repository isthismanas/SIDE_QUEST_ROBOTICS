#!/usr/bin/env bash
set -euo pipefail

# Always run from repo root (two levels up from motion/)
cd "$(dirname "$0")/../.." || exit

echo "Using python from: $(which python3)"

# Activate known-good environment
source ~/regolith-robotics-env/bin/activate

readarray -t RUN_CONFIG < <(python3 - <<'PY'
import os
import sys

repo_root = os.getcwd()
motion_dir = os.path.join(repo_root, "Robotics", "motion")
if motion_dir not in sys.path:
    sys.path.append(motion_dir)

import robot_config as cfg

pick_mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
manager_ids = tuple(str(value) for value in getattr(cfg, "MANAGER_CAMERA_IDS", ()) if str(value).strip())
default_device_id = manager_ids[0] if manager_ids else "194430108183F12E00"
print(pick_mode)
print(default_device_id)
PY
)

PICK_MODE="${RUN_CONFIG[0]:-deterministic}"
DEFAULT_VISION_DEVICE_ID="${RUN_CONFIG[1]:-194430108183F12E00}"

VISION_DEVICE_ID="${SIDE_QUEST_VISION_DEVICE_ID:-$DEFAULT_VISION_DEVICE_ID}"
VISION_PLACE_LEVEL="${SIDE_QUEST_VISION_PLACE_LEVEL:-0}"
VISION_MAX_COUNT="${SIDE_QUEST_VISION_MAX_COUNT:-}"
VISION_REMAINING_TARGETS="${SIDE_QUEST_VISION_REMAINING_TARGETS:-}"
VISION_ENABLE_PICK_ML="${SIDE_QUEST_VISION_ENABLE_PICK_ML:-0}"
VISION_USE_DEPTH_MODULE="${SIDE_QUEST_VISION_USE_DEPTH_MODULE:-0}"
VISION_PICK_ML_MODEL_JSON="${SIDE_QUEST_VISION_PICK_ML_MODEL_JSON:-}"

if [[ "$PICK_MODE" == "vision-autonomous" || "$PICK_MODE" == "experimental" ]]; then
  echo "[RUN_CONTROLLER] PICK_POSE_MODE=$PICK_MODE"
  echo "[RUN_CONTROLLER] Launching guarded standalone vision-autonomous pick-place runtime."
  echo "[RUN_CONTROLLER] device_id=$VISION_DEVICE_ID place_level=$VISION_PLACE_LEVEL"

  cmd=(
    python3
    Robotics/perception/vision_pick_place_once.py
    --device-id "$VISION_DEVICE_ID"
    --place-level "$VISION_PLACE_LEVEL"
    --execute
  )

  if [[ -n "$VISION_MAX_COUNT" ]]; then
    cmd+=(--max-count "$VISION_MAX_COUNT")
  fi

  if [[ -n "$VISION_REMAINING_TARGETS" ]]; then
    read -r -a remaining_targets <<< "$VISION_REMAINING_TARGETS"
    if [[ ${#remaining_targets[@]} -gt 0 ]]; then
      cmd+=(--remaining-targets "${remaining_targets[@]}")
    fi
  fi

  if [[ "$VISION_ENABLE_PICK_ML" == "1" ]]; then
    cmd+=(--enable-pick-ml)
    if [[ -n "$VISION_PICK_ML_MODEL_JSON" ]]; then
      cmd+=(--pick-ml-model-json "$VISION_PICK_ML_MODEL_JSON")
    fi
  fi

  if [[ "$VISION_USE_DEPTH_MODULE" == "1" ]]; then
    cmd+=(--use-depth-module)
  fi

  printf '[RUN_CONTROLLER] command='
  printf '%q ' "${cmd[@]}"
  printf '\n'
  exec "${cmd[@]}"
fi

echo "[RUN_CONTROLLER] PICK_POSE_MODE=$PICK_MODE"
echo "[RUN_CONTROLLER] Launching main task controller."
exec python3 Robotics/motion/task_controller.py
