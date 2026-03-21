# Active Tasks

## 2026-03-21 Pickup Recalibration And Runtime ML Session

### What Changed

- A new pickup-plane calibration lineage was created for the moved top-down camera:
  - `Robotics/perception/calibration_data/pick_plane_marker13_20260321_retry.jsonl`
  - `Robotics/perception/calibration_data/pick_plane_marker13_20260321_retry_phase3_solution.json`
- `Robotics/motion/robot_config.py` now points `VISION_CALIBRATION_JSON` at the new `20260321_retry` solution.
- Legacy pickup-only trims were removed:
  - `VISION_PICK_X_OFFSET_MM = 0.0`
  - `VISION_PICK_Y_OFFSET_MM = 0.0`
- Guarded affine-only execute runs improved materially and successfully picked blocks outside exact `P1..P7` template locations.

### Calibration Result

- New planar XY affine fit:
  - `rmse_x_mm = 1.645`
  - `rmse_y_mm = 3.056`
  - `rmse_total_mm = 3.470`
- This fit is good enough for guarded runtime validation.
- Deterministic controller authority remains unchanged.

### Important Runtime Findings

- The old pickup-only `-32.0 mm` Y correction was stale for the new camera geometry and caused guarded workspace rejection.
- Removing the old pickup offsets fixed the first guarded dry-run sanity check.
- A guarded full-stack execute run then completed much better than the prior lineage.
- Current affine-only guarded pickup is now the trusted path.

### Runtime ML Status

- Runtime ML training is now lineage-aware.
- Guarded plan logs and pickup runtime residual logs now carry:
  - active calibration JSON path
  - active calibration input JSONL path
  - calibration generation timestamp
  - active pickup offset state
- `train_pick_ml_residual.py` now rejects stale mixed-lineage rows by default.
- Explicit `--pick-ml-model-json ...` path handling was fixed so repo-relative model JSON paths resolve correctly from the Pi working directory.

### Current ML Verdict

- The first usable new-lineage runtime retrain produced:
  - `sample_count = 26`
  - `calibration_sample_count = 18`
  - `log_confirmation_sample_count = 7`
  - `runtime_residual_sample_count = 1`
  - `base_rmse_total_mm = 3.626`
  - `loo_rmse_total_mm = 3.579`
- This is acceptable and no longer poisoned by stale data, but it is still a weak model.
- The affine solve is still doing most of the work.
- ML should still be validated conservatively with `--max-count 2` before any larger ML-enabled run.

### Current Safe Loop

From the Pi repo root:

```bash
source ~/regolith-robotics-env/bin/activate
bash Robotics/perception/run_pick_ml_lineage_loop.sh paths
```

Affine-only guarded execute:

```bash
bash Robotics/perception/run_pick_ml_lineage_loop.sh run-affine --device-id 194430108183F12E00 --max-count 2
```

Retrain current-lineage baseline and runtime models:

```bash
bash Robotics/perception/run_pick_ml_lineage_loop.sh train-baseline
bash Robotics/perception/run_pick_ml_lineage_loop.sh train-runtime
```

ML-enabled conservative validation:

```bash
bash Robotics/perception/run_pick_ml_lineage_loop.sh run-ml --device-id 194430108183F12E00 --max-count 2
```

Combined retrain-and-run loop:

```bash
bash Robotics/perception/run_pick_ml_lineage_loop.sh cycle --device-id 194430108183F12E00 --max-count 2
```

### Next Practical Work

- Keep collecting current-lineage guarded execute logs.
- Retrain after each bounded run.
- Only increase ML-enabled run count after repeated `--max-count 2` runs remain stable.
- Continue pushing Pi logs through the Pi logs branch flow so the Mac repo stays current.
- Do not reuse old-camera residual models.
