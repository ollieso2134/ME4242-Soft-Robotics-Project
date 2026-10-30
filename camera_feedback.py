"""
camera_feedback.py
------------------
Replay a recorded trajectory (from cabledriven_dpad_v2.py) with RealSense camera feedback,
so the run ends up where the recording did even if the robot drifts (cable slack,
friction, a slightly different start).

Three steps:
1. Record the motion with cabledriven_dpad_v2.py and save it (trajectories/*.json).
2. Teach, once, with the setup as it should be:
       python camera_feedback.py teach trajectories/config_one.json
   The trajectory is played slowly. At each checkpoint (every pause in the recording,
   and the end) the tip's 3D position is measured with the camera (orange marker + depth),
   and each motor is nudged by PROBE_DEG to measure how the tip responds there.
   This is saved into the same JSON under "checkpoints".
3. Run, as often as needed:
       python camera_feedback.py run trajectories/config_one.json
   The trajectory is played; at each checkpoint the live tip is compared with the taught
   one and the motors are corrected until it is within TOLERANCE_MM. The correction is
   kept for the rest of the run. If the camera cannot see the tip, the run carries on
   with the recorded angles only.

3D: the tip position is the marker's pixel position plus its depth, turned into
millimetres in the camera's frame. Teach and run use the same fixed camera, so no
camera-to-robot calibration is needed. If depth is missing, only the image directions
are corrected.

Options: --port COM3, --speed 0.5 (teach) / 1.0 (run), --tolerance 3, --no-camera
(plays the recording only), --measure-only (replays without correcting and logs how far
the tip is from the taught checkpoints: use it to see if a plain replay is repeatable
enough), --fx/--fy (camera focal length in pixels).
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

PROBE_DEG = 3.0          # motor nudge used to measure the tip's response during teach
TOLERANCE_MM = 3.0       # stop correcting a checkpoint when the tip is this close
MAX_ITERATIONS = 6       # correction steps per checkpoint
MAX_STEP_DEG = 4.0       # largest change of one motor in one correction step
MAX_OFFSET_DEG = 25.0    # largest total correction of one motor
SETTLE_S = 0.4           # wait after a move before measuring
HOLD_S = 0.3             # a recording pause at least this long becomes a checkpoint
MAX_GAP_S = 2.0          # add a checkpoint if two are further apart than this
TICK_S = 0.05            # replay command period
DAMPING = 0.5            # damped least squares (mm): smaller = bolder corrections
MEASURE_ONLY = False     # --measure-only: log the errors, do not correct
DEFAULT_DEPTH_MM = 250.0 # depth assumed for the image directions if the camera gives no depth


# ---------------------------------------------------------------------------
# Trajectory file (format written by cabledriven_dpad_v2.py, angles in degrees)
# ---------------------------------------------------------------------------

def load(path):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    traj = data["trajectory"] if isinstance(data, dict) else data
    points = [(float(p["t"]), [float(a) for a in p["angles"][:4]]) for p in traj]
    return data if isinstance(data, dict) else {"trajectory": traj}, points


def angles_at(points, t):
    if t <= points[0][0]:
        return list(points[0][1])
    for (t0, a0), (t1, a1) in zip(points, points[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            return [x + f * (y - x) for x, y in zip(a0, a1)]
    return list(points[-1][1])


def checkpoint_times(points):
    """Ends of pauses in the recording, extra ones in long stretches, and the end."""
    times = []
    hold_start = None
    for (t0, a0), (t1, a1) in zip(points, points[1:]):
        if max(abs(x - y) for x, y in zip(a0, a1)) < 0.5:
            hold_start = t0 if hold_start is None else hold_start
            if t1 - hold_start >= HOLD_S and (not times or t1 - times[-1] > 0.2):
                times.append(t1)
        else:
            hold_start = None
    end = points[-1][0]
    if not times or abs(times[-1] - end) > 1e-6:
        times.append(end)
    filled, last = [], points[0][0]
    for t in times:
        while t - last > MAX_GAP_S:
            last += MAX_GAP_S
            filled.append(round(last, 3))
        filled.append(t)
        last = t
    return sorted(set(filled))


# ---------------------------------------------------------------------------
# Tip tracking in 3D with the RealSense
# ---------------------------------------------------------------------------

class TipTracker:
    """
    Finds the orange tip marker in the colour image and its depth, and returns its
    position in mm in the camera frame (X right, Y down, Z away from the camera).
    """

    def __init__(self, camera, color="orange", fx=None, fy=None, depth_scale_mm=None):
        from visual_servoing import ColorDetector
        self.camera = camera
        self.detector = ColorDetector(min_contour_area=60)
        self.color = color
        self.fx, self.fy = fx, fy
        self.depth_scale_mm = depth_scale_mm  # mm per depth unit; guessed on first use if None

    def _intrinsics(self, w, h):
        # Approximate D405 focal length for 640x480 when nothing better is given.
        fx = self.fx or 0.61 * w
        fy = self.fy or fx
        return fx, fy, w / 2.0, h / 2.0

    def measure_once(self):
        """(u, v, depth in mm or nan) of the marker in one frame, or None if not seen."""
        frame = getattr(self.camera, "frame", None)
        if frame is None:
            return None
        det = self.detector.detect_single(frame, self.color)
        if not det:
            return None
        u, v = det["centroid"]
        self._shape = frame.shape[:2]
        depth = getattr(self.camera, "depth_frame", None)
        z = np.nan
        if depth is not None:
            import cv2
            mask = np.zeros(depth.shape[:2], np.uint8)
            cv2.drawContours(mask, [det["contour"]], -1, 255, -1)
            vals = depth[(mask > 0) & (depth > 0)]
            if vals.size:
                raw = float(np.median(vals))
                if self.depth_scale_mm is None:
                    # D405 depth is in 0.1 mm or 1 mm units; the rod is 50-600 mm away.
                    self.depth_scale_mm = 0.1 if raw > 1500 else 1.0
                    print(f"[camera] depth units taken as {self.depth_scale_mm} mm (raw {raw:.0f})")
                z = raw * self.depth_scale_mm
        return float(u), float(v), z

    def measure(self, depth_fallback_mm=DEFAULT_DEPTH_MM, samples=5, interval=0.03):
        """
        Tip position (X, Y, Z) in mm in the camera frame, median of a few frames; None if
        the tip is not seen. Without depth, X and Y are computed at depth_fallback_mm
        (the taught depth during a run) and Z is nan, so only the image directions count.
        """
        found = []
        for _ in range(samples):
            m = self.measure_once()
            if m is not None:
                found.append(m)
            time.sleep(interval)
        if not found:
            return None
        u = float(np.median([f[0] for f in found]))
        v = float(np.median([f[1] for f in found]))
        zs = [f[2] for f in found if not np.isnan(f[2])]
        z = float(np.median(zs)) if zs else np.nan
        h, w = self._shape
        fx, fy, cx, cy = self._intrinsics(w, h)
        zz = z if not np.isnan(z) else depth_fallback_mm
        return np.array([(u - cx) * zz / fx, (v - cy) * zz / fy, z])


# ---------------------------------------------------------------------------
# Robot helpers
# ---------------------------------------------------------------------------

class Arm:
    """Thin wrapper over CableDrivenRobotController, in degrees."""

    def __init__(self, robot, max_deg, lo=None, hi=None):
        self.robot = robot
        self.max_deg = max_deg
        self.lo = lo if lo is not None else [0.0] * 4       # per-motor limits (deg)
        self.hi = hi if hi is not None else [max_deg] * 4

    def _limit(self, deg):
        return [math.radians(min(max(a, lo), hi)) for a, lo, hi in zip(deg, self.lo, self.hi)]

    def send(self, deg):
        self.robot.send_angles(self._limit(deg))

    def move(self, deg, duration=0.4):
        self.robot.move_smooth(self._limit(deg), duration=duration, steps=max(2, int(duration / TICK_S)))

    def current(self):
        return [math.degrees(a) for a in self.robot.current_angles]


def play_segment(arm, points, t_from, t_to, speed, offset):
    """Feed the recorded angles (+ offset) from t_from to t_to."""
    t = t_from
    while t < t_to - 1e-9:
        t = min(t + TICK_S * speed, t_to)
        arm.send([a + o for a, o in zip(angles_at(points, t), offset)])
        time.sleep(TICK_S)


def _valid(e):
    return e is not None and not np.any(np.isnan(e))


def _rows(vec):
    """Use all 3 axes if depth is valid, else only the 2 image axes."""
    return [0, 1, 2] if not np.isnan(vec[2]) else [0, 1]


# ---------------------------------------------------------------------------
# Teach and run
# ---------------------------------------------------------------------------

def teach(arm, tracker, points, speed=0.5):
    """Play the recording slowly, store the tip and its response to each motor at every checkpoint."""
    checkpoints = []
    arm.move(points[0][1], duration=2.0)
    time.sleep(SETTLE_S)
    t_prev = points[0][0]
    for k, t in enumerate(checkpoint_times(points)):
        play_segment(arm, points, t_prev, t, speed, [0.0] * 4)
        t_prev = t
        nominal = angles_at(points, t)
        arm.move(nominal, 0.3)
        time.sleep(SETTLE_S)
        tip = tracker.measure()
        if tip is None:
            print(f"[teach] checkpoint {k + 1} t={t:.2f}s: tip not visible, skipped")
            continue
        jac = np.zeros((3, 4))
        for m in range(4):
            probe = list(nominal)
            probe[m] += PROBE_DEG
            arm.move(probe, 0.3)
            time.sleep(SETTLE_S)
            moved = tracker.measure()
            arm.move(nominal, 0.3)
            if moved is not None:
                jac[:, m] = np.nan_to_num((moved - tip) / PROBE_DEG)
        time.sleep(SETTLE_S)
        checkpoints.append({"t": t, "angles": [round(a, 3) for a in nominal],
                            "tip_mm": [None if np.isnan(v) else round(float(v), 2) for v in tip],
                            "jacobian_mm_per_deg": np.round(jac, 4).tolist()})
        print(f"[teach] checkpoint {k + 1} t={t:.2f}s: tip {np.round(tip, 1)} mm, "
              f"strongest motor M{int(np.argmax(np.linalg.norm(jac, axis=0))) + 1}")
    return checkpoints


def correct(arm, tracker, nominal, offset, target, jac):
    """Correct the motors at one checkpoint. Returns (new offset, error before, error after)."""
    fallback = target[2] if not np.isnan(target[2]) else DEFAULT_DEPTH_MM
    tip = tracker.measure(fallback)
    if tip is None:
        return offset, None, None
    rows = [r for r in _rows(tip) if not np.isnan(target[r])]
    err = (target - tip)[rows]
    first = float(np.linalg.norm(err))
    if MEASURE_ONLY:
        return offset, first, first
    for _ in range(MAX_ITERATIONS):
        if np.linalg.norm(err) <= TOLERANCE_MM:
            break
        J = jac[rows]
        step = J.T @ np.linalg.solve(J @ J.T + DAMPING ** 2 * np.eye(len(rows)), err)
        step = np.clip(step, -MAX_STEP_DEG, MAX_STEP_DEG)
        new_offset = np.clip(np.array(offset) + step, -MAX_OFFSET_DEG, MAX_OFFSET_DEG)
        new_offset = np.clip(new_offset, np.array(arm.lo) - nominal, np.array(arm.hi) - nominal)  # motor limits
        applied = new_offset - np.array(offset)
        arm.move([a + o for a, o in zip(nominal, new_offset)], 0.25)
        time.sleep(SETTLE_S)
        new_tip = tracker.measure(fallback)
        if new_tip is None:
            break
        # Broyden update: learn how the tip really responded to this step
        if np.dot(applied, applied) > 1e-6:
            dy = (new_tip - tip)[rows]
            jac[rows] += np.outer(dy - jac[rows] @ applied, applied) / np.dot(applied, applied)
        offset, tip = [float(x) for x in new_offset], new_tip
        err = (target - tip)[rows]
    return offset, first, float(np.linalg.norm(err))


def run(arm, tracker, points, checkpoints, speed=1.0):
    """Play the recording, correcting at each taught checkpoint."""
    offset = [0.0] * 4
    arm.move(points[0][1], duration=2.0)
    time.sleep(SETTLE_S)
    t_prev, log = points[0][0], []
    t_start = time.perf_counter()
    for k, cp in enumerate(checkpoints):
        play_segment(arm, points, t_prev, cp["t"], speed, offset)
        t_prev = cp["t"]
        if tracker is None:
            continue
        nominal = angles_at(points, cp["t"])
        target = np.array([np.nan if v is None else v for v in cp["tip_mm"]], dtype=float)
        jac = np.array(cp["jacobian_mm_per_deg"], dtype=float)
        time.sleep(SETTLE_S)
        offset, before, after = correct(arm, tracker, nominal, offset, target, jac)
        if before is None:
            print(f"[run] checkpoint {k + 1}: tip not visible, using the recorded angles")
        else:
            if MEASURE_ONLY:
                print(f"[run] checkpoint {k + 1}: tip is {before:5.1f} mm from the taught position (not corrected)")
            else:
                print(f"[run] checkpoint {k + 1}: error {before:5.1f} mm -> {after:5.1f} mm, "
                      f"correction {[round(o, 1) for o in offset]} deg")
        log.append({"checkpoint": k + 1, "t": cp["t"], "error_before_mm": before, "error_after_mm": after})
    play_segment(arm, points, t_prev, points[-1][0], speed, offset)
    total = time.perf_counter() - t_start
    print(f"[run] done in {total:.1f} s")
    return log, total


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("mode", choices=["teach", "run"])
    p.add_argument("trajectory")
    p.add_argument("--port", default=None)
    p.add_argument("--speed", type=float, default=None, help="replay speed (default 0.5 teach, 1.0 run)")
    p.add_argument("--tolerance", type=float, default=TOLERANCE_MM)
    p.add_argument("--no-camera", action="store_true", help="replay the recorded angles only")
    p.add_argument("--measure-only", action="store_true",
                   help="replay without correcting; log the tip error at each checkpoint")
    p.add_argument("--fx", type=float, default=None)
    p.add_argument("--fy", type=float, default=None)
    args = p.parse_args()

    globals()["TOLERANCE_MM"] = args.tolerance
    globals()["MEASURE_ONLY"] = args.measure_only
    data, points = load(args.trajectory)
    from cabledriven import CableDrivenRobotController, MAX_ANGLE_DEG
    limit = 2 * math.radians(MAX_ANGLE_DEG)  # the Arm applies the real per-motor limits
    robot = CableDrivenRobotController(min_angle=-limit, max_angle=limit, num_motors=4)
    robot.connect(args.port)
    if robot.is_mock:
        print("[warn] the robot is not connected: running in simulation / mock mode")
    if "start_angles_deg" in data:
        # Recorded with cabledriven_sliders.py / dpad_v3: play the changes from the pose the robot is in now.
        from cabledriven import trajectory_changes
        from cabledriven_sliders import REL_MIN_DEG
        now = [math.degrees(a) for a in robot.current_angles]
        points = [(t, [n + c for n, c in zip(now, ch)]) for t, ch in trajectory_changes(data)]
        arm = Arm(robot, MAX_ANGLE_DEG, lo=[n + REL_MIN_DEG for n in now], hi=[n + MAX_ANGLE_DEG for n in now])
        print(f"[info] replaying the recorded changes from the current pose {[round(n, 1) for n in now]} deg")
    else:
        arm = Arm(robot, MAX_ANGLE_DEG)

    tracker = None
    if not args.no_camera:
        ok, msg = robot.connect_camera(allow_simulation_fallback=False)
        if not ok:
            sys.exit(f"[error] camera: {msg}  (use --no-camera to replay without feedback)")
        tracker = TipTracker(robot.camera, fx=args.fx, fy=args.fy)
        time.sleep(1.0)  # let the camera start streaming

    try:
        if args.mode == "teach":
            if tracker is None:
                sys.exit("[error] teach needs the camera")
            data["checkpoints"] = teach(arm, tracker, points, args.speed or 0.5)
            data["taught_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(args.trajectory, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            print(f"[teach] saved {len(data['checkpoints'])} checkpoints to {args.trajectory}")
        else:
            checkpoints = data.get("checkpoints")
            if tracker is not None and not checkpoints:
                sys.exit("[error] no checkpoints in this file: run 'teach' first (or use --no-camera)")
            if tracker is None:
                checkpoints = [{"t": t} for t in checkpoint_times(points)]
            log, total = run(arm, tracker, points, checkpoints, args.speed or 1.0)
            os.makedirs("trajectories/logs", exist_ok=True)
            name = os.path.splitext(os.path.basename(args.trajectory))[0]
            out = os.path.join("trajectories", "logs", f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.json")
            with open(out, "w", encoding="utf-8") as f:
                json.dump({"trajectory": args.trajectory, "total_s": round(total, 2), "checkpoints": log}, f, indent=2)
            print(f"[run] log saved to {out}")
    except KeyboardInterrupt:
        print("\n[stop] interrupted: motors hold their last position")
    finally:
        robot.disconnect_camera()


if __name__ == "__main__":
    main()
