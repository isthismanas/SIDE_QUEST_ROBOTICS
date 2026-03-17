from __future__ import annotations

import argparse
import time
from typing import Optional

import robot_config as cfg
from dobot_driver import DobotDriver
from logger import write_jsonl_event


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Poll Dobot GetPose() and write timestamped TCP poses for calibration/debugging.",
    )
    parser.add_argument(
        "--interval-s",
        type=float,
        default=0.25,
        help="Polling interval in seconds.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=0,
        help="Optional sample limit. Use 0 to run until interrupted.",
    )
    parser.add_argument(
        "--stream-name",
        default="robot_tcp_pose",
        help="JSONL stream name written under Robotics/perception/logs/.",
    )
    parser.add_argument(
        "--label",
        default="manual_probe",
        help="Free-form label stored with each pose sample.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Disable console pose prints and only write JSONL output.",
    )
    return parser.parse_args()


def emit_pose(stream_name: str, label: str, pose: tuple[float, float, float, float, float, float], sample_index: int) -> None:
    write_jsonl_event(
        stream_name,
        {
            "event": "robot_tcp_pose",
            "module": "DOBOT",
            "label": label,
            "sample_index": int(sample_index),
            "pose_mm_deg": {
                "x": round(float(pose[0]), 3),
                "y": round(float(pose[1]), 3),
                "z": round(float(pose[2]), 3),
                "rx": round(float(pose[3]), 3),
                "ry": round(float(pose[4]), 3),
                "rz": round(float(pose[5]), 3),
            },
        },
    )


def main() -> int:
    args = parse_args()
    driver = DobotDriver(robot_ip=cfg.ROBOT_IP, dashboard_port=cfg.DASHBOARD_PORT, timeout_s=cfg.SOCKET_TIMEOUT_S)

    sample_index = 0
    try:
        driver.connect()
        while True:
            pose: Optional[tuple[float, float, float, float, float, float]] = driver.get_tcp_pose()
            if pose is not None:
                emit_pose(args.stream_name, args.label, pose, sample_index)
                if not args.quiet:
                    print(
                        "[POSE_LOG] "
                        f"sample={sample_index} "
                        f"x={pose[0]:.3f} y={pose[1]:.3f} z={pose[2]:.3f} "
                        f"rx={pose[3]:.3f} ry={pose[4]:.3f} rz={pose[5]:.3f}"
                    )
                sample_index += 1

            if args.samples > 0 and sample_index >= int(args.samples):
                break

            time.sleep(max(0.01, float(args.interval_s)))
    except KeyboardInterrupt:
        pass
    finally:
        driver.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
