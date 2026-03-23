STATUS: DRAFT v0 – To be aligned with Manas before implementation.

1. Vision → Robot controller
- timestamp
- frame_id (camera frame name)
- markers[]:
   - id
   - pose_cam (position + quaternion)
   - confidence
   - pixel_corners (optional)
- best_candidate (id or pose)
- status (OK | FAIL | NO_MARKERS | LOW_CONF)

2. Robot controller → Unity
- VISION_STATUS (OK/FAIL) + reason
- current_pick_pose_mode (deterministic/vision/vision-autonomous)
- retry_available (bool)

3. Unity → Robot controller
- VISION_RETRY
- (later) VISION_DEBUG_OVERLAY_ON/OFF, SELECT_MARKER_ID
