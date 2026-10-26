"""
rerun_trajectory.py
-------------------
CLI tool to replay recorded trajectories on the EMIO cable-driven robot.

Usage:
    python rerun_trajectory.py <trajectory.json> [--port PORT] [--speed 1.0] [--loop] [--dry-run]
"""

import argparse
import json
import math
import os
import sys
import time

# Ensure workspace root is in path
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from cabledriven import CableDrivenRobotController, MAX_ANGLE_DEG


def load_trajectory(filepath):
    if not os.path.exists(filepath):
        print(f"[ERROR] Trajectory file not found: {filepath}")
        sys.exit(1)

    with open(filepath, "r") as f:
        data = json.load(f)

    if isinstance(data, dict) and "trajectory" in data:
        points = data["trajectory"]
        meta = {k: v for k, v in data.items() if k != "trajectory"}
    elif isinstance(data, list):
        points = data
        meta = {}
    else:
        print("[ERROR] Unrecognized trajectory format: expected dict with 'trajectory' or list of points.")
        sys.exit(1)

    if not points:
        print("[ERROR] Trajectory file contains 0 waypoints.")
        sys.exit(1)

    return points, meta


def replay_trajectory(robot, points, speed=1.0, loop=False, dry_run=False):
    total_duration = points[-1]["t"]
    scaled_duration = total_duration / speed
    n_pts = len(points)

    print("\n==========================================")
    print(f" Replaying Trajectory: {n_pts} waypoints")
    print(f" Original Duration:   {total_duration:.2f}s")
    print(f" Replay Speed:        {speed:g}x ({scaled_duration:.2f}s)")
    print(f" Mode:                {'Looping' if loop else 'Single Run'}")
    print("==========================================")
    print("Press Ctrl+C at any time to safely stop and relax cables.\n")

    iteration = 0
    try:
        while True:
            iteration += 1
            if loop:
                print(f"\n--- Starting Cycle {iteration} ---")

            # 1. Smoothly move to trajectory start position
            start_angles_deg = points[0]["angles"]
            start_rads = [math.radians(a) for a in start_angles_deg]
            cur_deg = [math.degrees(a) for a in robot.current_angles]
            max_diff = max(abs(cur_deg[i] - start_angles_deg[i]) for i in range(4))

            if max_diff > 1.0:
                print(f"[ROBOT] Moving smoothly to start position {start_angles_deg}...")
                if not dry_run:
                    robot.move_smooth(start_rads, duration=1.0)
                else:
                    time.sleep(0.5)

            # 2. Replay points in real time
            replay_start = time.perf_counter()
            for i, pt in enumerate(points):
                target_t = pt["t"] / speed
                target_angles_deg = pt["angles"]
                target_rads = [math.radians(a) for a in target_angles_deg]

                # Drift-compensated sleep
                elapsed = time.perf_counter() - replay_start
                sleep_sec = target_t - elapsed
                if sleep_sec > 0:
                    time.sleep(sleep_sec)

                if not dry_run:
                    robot.send_angles(target_rads)

                # Progress readout
                pct = int((i + 1) / n_pts * 100)
                deg_str = " ".join(f"M{m+1}:{a:5.1f}°" for m, a in enumerate(target_angles_deg))
                print(f"\r [{pct:3d}%] t={pt['t']:5.2f}s | {deg_str}", end="", flush=True)

            print("\n[OK] Cycle completed successfully.")
            if not loop:
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Replay interrupted by user.")
    finally:
        print("\n[ROBOT] Safely returning to neutral home (0.0°)...")
        if not dry_run:
            robot.home()
        print("[OK] Finished.")


def main():
    parser = argparse.ArgumentParser(description="Replay recorded trajectory on EMIO cable-driven robot.")
    parser.add_argument("trajectory", type=str, help="Path to JSON trajectory file")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. COM13 or /dev/ttyUSB0)")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (default: 1.0)")
    parser.add_argument("--loop", action="store_true", help="Repeat trajectory in an infinite loop")
    parser.add_argument("--dry-run", action="store_true", help="Run in mock/dry-run mode without sending hardware commands")
    args = parser.parse_args()

    points, meta = load_trajectory(args.trajectory)
    if meta.get("recorded_at"):
        print(f"[INFO] Trajectory recorded on: {meta['recorded_at']}")

    # Connect to robot
    limit_rad = math.radians(MAX_ANGLE_DEG)
    robot = CableDrivenRobotController(min_angle=0.0, max_angle=limit_rad, num_motors=4)
    if not args.dry_run:
        robot.connect(port=args.port)
    else:
        robot.is_mock = True
        print("[INFO] Running in DRY-RUN mode.")

    replay_trajectory(robot, points, speed=args.speed, loop=args.loop, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
