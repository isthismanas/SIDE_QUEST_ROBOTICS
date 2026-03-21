#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"

DEFAULT_DEVICE_ID="194430108183F12E00"

active_lineage_paths() {
  python3 - <<'PY'
import json
import os
import sys

repo_root = os.getcwd()
motion_dir = os.path.join(repo_root, "Robotics", "motion")
if motion_dir not in sys.path:
    sys.path.append(motion_dir)

import robot_config as cfg

solution = str(getattr(cfg, "VISION_CALIBRATION_JSON", "")).strip()
if not os.path.isabs(solution):
    solution = os.path.abspath(os.path.join(motion_dir, solution))

with open(solution, "r", encoding="utf-8") as handle:
    payload = json.load(handle)

input_jsonl = str(payload.get("input_jsonl", "")).strip()
if not input_jsonl:
    raise SystemExit("Active calibration JSON does not declare input_jsonl")

base_no_ext, _ = os.path.splitext(input_jsonl)
print(solution)
print(input_jsonl)
print(base_no_ext + "_ml_residual.json")
print(base_no_ext + "_ml_residual_runtime.json")
PY
}

readarray -t LINEAGE_PATHS < <(active_lineage_paths)
SOLUTION_JSON="${LINEAGE_PATHS[0]}"
INPUT_JSONL="${LINEAGE_PATHS[1]}"
BASELINE_MODEL_JSON="${LINEAGE_PATHS[2]}"
RUNTIME_MODEL_JSON="${LINEAGE_PATHS[3]}"

COMMAND="${1:-}"
if [[ -z "$COMMAND" ]]; then
  echo "usage: bash Robotics/perception/run_pick_ml_lineage_loop.sh <paths|train-baseline|train-runtime|run-affine|run-ml|cycle> [args]"
  exit 1
fi
shift || true

DEVICE_ID="$DEFAULT_DEVICE_ID"
MAX_COUNT=""
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device-id)
      DEVICE_ID="$2"
      shift 2
      ;;
    --max-count)
      MAX_COUNT="$2"
      shift 2
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

max_count_args=()
if [[ -n "$MAX_COUNT" ]]; then
  max_count_args=(--max-count "$MAX_COUNT")
fi

case "$COMMAND" in
  paths)
    printf 'solution_json=%s\n' "$SOLUTION_JSON"
    printf 'input_jsonl=%s\n' "$INPUT_JSONL"
    printf 'baseline_model_json=%s\n' "$BASELINE_MODEL_JSON"
    printf 'runtime_model_json=%s\n' "$RUNTIME_MODEL_JSON"
    ;;
  train-baseline)
    python3 Robotics/perception/train_pick_ml_residual.py \
      --input-jsonl "$INPUT_JSONL" \
      --output-json "$BASELINE_MODEL_JSON"
    ;;
  train-runtime)
    python3 Robotics/perception/train_pick_ml_residual.py \
      --input-jsonl "$INPUT_JSONL" \
      --use-log-confirmation-samples \
      --use-runtime-residual-samples \
      --output-json "$RUNTIME_MODEL_JSON"
    ;;
  run-affine)
    python3 Robotics/perception/vision_pick_place_once.py \
      --device-id "$DEVICE_ID" \
      --execute \
      "${max_count_args[@]}" \
      "${EXTRA_ARGS[@]}"
    ;;
  run-ml)
    python3 Robotics/perception/vision_pick_place_once.py \
      --device-id "$DEVICE_ID" \
      --execute \
      --enable-pick-ml \
      --pick-ml-model-json "$RUNTIME_MODEL_JSON" \
      "${max_count_args[@]}" \
      "${EXTRA_ARGS[@]}"
    ;;
  cycle)
    python3 Robotics/perception/train_pick_ml_residual.py \
      --input-jsonl "$INPUT_JSONL" \
      --use-log-confirmation-samples \
      --use-runtime-residual-samples \
      --output-json "$RUNTIME_MODEL_JSON"
    python3 Robotics/perception/vision_pick_place_once.py \
      --device-id "$DEVICE_ID" \
      --execute \
      --enable-pick-ml \
      --pick-ml-model-json "$RUNTIME_MODEL_JSON" \
      "${max_count_args[@]}" \
      "${EXTRA_ARGS[@]}"
    ;;
  *)
    echo "unknown command: $COMMAND"
    exit 1
    ;;
esac
