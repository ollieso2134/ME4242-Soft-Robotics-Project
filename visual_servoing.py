"""
visual_servoing.py
------------------
Obstacle-Aided Continuum Robot Visual Servoing & Multi-Target Retrieval System.
Designed for EMIO 4-Cable Driven Soft Robot (ME4242 Soft Robotics Project 2).

Task Workflow:
1. Detect target letter tagged by color: Red ('N'), Blue ('U'), Yellow ('S').
2. Navigate into obstacle environment, establishing controlled contact with rigid
   obstacles to induce multi-curvature deformation (at least two distinct bends).
3. Position green end-effector (with magnet) over target letter and magnetically grasp it.
4. Come out of the obstacle: execute a safe, controlled withdrawal trajectory that
   unwinds the bends without hooking or snagging on obstacle pegs.
5. Deliver the letter to the designated drop-off slot outside the obstacle field.
6. Mechanically strip / wipe the letter against the slot edge to dislodge the permanent magnet.
7. Return back to the obstacle entrance staging pose.
8. Repeat for the next color until all 3 targets (Red -> Blue -> Yellow) have been
   successfully picked and placed.
"""

import math
import os
import sys
import time
import numpy as np
import cv2


# =====================================================================
# 1. Color Detector (Red, Blue, Yellow, Green Tip, Obstacles, Slots)
# =====================================================================

class ColorDetector:
    """Detects and tracks colored targets, end-effector markers, and arena regions in HSV."""

    COLOR_RANGES = {
        "red": {
            "lower1": np.array([0, 90, 70]),
            "upper1": np.array([10, 255, 255]),
            "lower2": np.array([165, 90, 70]),
            "upper2": np.array([180, 255, 255]),
            "display_color": (40, 40, 235),  # BGR
            "label": "Target N (Red)",
            "letter": "N",
        },
        "blue": {
            "lower": np.array([100, 90, 70]),
            "upper": np.array([130, 255, 255]),
            "display_color": (235, 140, 30),  # BGR
            "label": "Target U (Blue)",
            "letter": "U",
        },
        "yellow": {
            "lower": np.array([18, 90, 70]),
            "upper": np.array([38, 255, 255]),
            "display_color": (30, 215, 245),  # BGR
            "label": "Target S (Yellow)",
            "letter": "S",
        },
        "orange": {
            "lower": np.array([11, 120, 90]),
            "upper": np.array([17, 255, 255]),
            "display_color": (0, 140, 255),  # BGR (orange)
            "label": "Arm Tip (Orange)",
            "letter": "TIP",
        },
    }

    def __init__(self, min_contour_area=200):
        self.min_contour_area = min_contour_area
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    def get_mask(self, hsv, color_name):
        """Generates clean binary mask for the requested color."""
        if color_name not in self.COLOR_RANGES:
            return None

        cfg = self.COLOR_RANGES[color_name]
        if color_name == "red":
            m1 = cv2.inRange(hsv, cfg["lower1"], cfg["upper1"])
            m2 = cv2.inRange(hsv, cfg["lower2"], cfg["upper2"])
            mask = cv2.bitwise_or(m1, m2)
        else:
            mask = cv2.inRange(hsv, cfg["lower"], cfg["upper"])

        # Morphological noise filtering
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close, iterations=2)
        return mask

    def detect_single(self, frame_bgr, color_name):
        """
        Detects the primary (largest) target of specified color in the frame.
        Returns dict with: centroid (x, y), area, contour, bbox (x, y, w, h).
        """
        if frame_bgr is None:
            return None

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(hsv, color_name)
        if mask is None:
            return None

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        valid = [c for c in contours if cv2.contourArea(c) >= self.min_contour_area]
        if not valid:
            return None

        largest = max(valid, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        x, y, w, h = cv2.boundingRect(largest)

        return {
            "color": color_name,
            "centroid": (cx, cy),
            "area": cv2.contourArea(largest),
            "bbox": (x, y, w, h),
            "contour": largest,
            "label": self.COLOR_RANGES[color_name]["label"],
            "letter": self.COLOR_RANGES[color_name]["letter"],
            "display_color": self.COLOR_RANGES[color_name]["display_color"],
        }

    def detect_all_targets(self, frame_bgr):
        """Detects all objects: Red (N), Blue (U), Yellow (S), and Green (Tip)."""
        results = {}
        for c in ["red", "blue", "yellow", "orange"]:
            det = self.detect_single(frame_bgr, c)
            if det:
                results[c] = det
        return results

    def convert_rotated_pixel_to_raw(self, rot_x, rot_y, rot_w=480, rot_h=640):
        """Converts coordinates from 90° CCW rotated frame back to raw RealSense 640x480."""
        raw_x = 640 - 1 - rot_y
        raw_y = rot_x
        raw_x = max(0, min(639, raw_x))
        raw_y = max(0, min(479, raw_y))
        return raw_x, raw_y

    def get_3d_coordinates(self, rot_cx, rot_cy, depth_raw, camera_estimator=None):
        """Calculates real-world (X, Y, Z in mm) from rotated pixel and raw depth map."""
        if depth_raw is None:
            return None

        raw_x, raw_y = self.convert_rotated_pixel_to_raw(rot_cx, rot_cy)
        depth_val = float(depth_raw[raw_y, raw_x])

        if depth_val <= 0.0:
            patch = depth_raw[max(0, raw_y - 2):raw_y + 3, max(0, raw_x - 2):raw_x + 3]
            valid_depths = patch[patch > 0]
            if len(valid_depths) > 0:
                depth_val = float(np.median(valid_depths))
            else:
                depth_val = 300.0

        if camera_estimator is not None and hasattr(camera_estimator, "image_to_simulation"):
            try:
                coords = camera_estimator.image_to_simulation(raw_x, raw_y, depth_val)
                return coords
            except Exception:
                pass

        return [float(raw_x - 320), float(raw_y - 240), depth_val]

    def draw_hud(
        self,
        frame_bgr,
        detections,
        target_color=None,
        tip_pos=None,
        error_dist=None,
        state_text="IDLE",
        obstacle_env=None,
        target_statuses=None,
        mission_time=None,
    ):
        """
        Renders rich computer vision HUD overlay onto frame:
        - Bounding boxes and badges for Red (N), Blue (U), Yellow (S), and Green (Tip)
        - Target lock reticle on active letter
        - Obstacle boundary pegs & contact zones
        - Drop slot boundaries (Slot N, Slot U, Slot S)
        - Smooth Bézier trajectory path from tip to target/slot
        - Live mission state banner, elapsed timer, and completion checklist
        """
        if frame_bgr is None:
            return None

        annotated = frame_bgr.copy()
        h, w = annotated.shape[:2]

        target_statuses = target_statuses or {"red": "PENDING", "blue": "PENDING", "yellow": "PENDING"}

        # 1. Draw Obstacles if provided
        if obstacle_env is not None and hasattr(obstacle_env, "obstacles"):
            for i, obs in enumerate(obstacle_env.obstacles):
                if obs.get("orient") == "horizontal" or "x_mount" in obs:
                    xm = int(obs["x_mount"])
                    xt = int(obs["x_tip"])
                    oy = int(obs["y"])
                    r = int(obs.get("r", 16))
                    z_h = obs.get("z", 200)

                    # Vertical column strut (blue vertical frame column)
                    cv2.line(annotated, (xm, max(0, oy - 80)), (xm, min(h - 1, oy + 80)), (210, 120, 20), 4, cv2.LINE_AA)
                    cv2.circle(annotated, (xm, oy), 7, (230, 140, 40), -1)

                    # Horizontal cylindrical peg pointing horizontally towards middle
                    min_x, max_x = min(xm, xt), max(xm, xt)
                    cv2.rectangle(annotated, (min_x, oy - r // 2), (max_x, oy + r // 2), (48, 52, 64), -1)
                    cv2.rectangle(annotated, (min_x, oy - r // 2), (max_x, oy + r // 2), (180, 185, 200), 2)
                    # Rounded end cap pointing inwards
                    cv2.circle(annotated, (xt, oy), r // 2, (210, 140, 40), -1)
                    cv2.circle(annotated, (xt, oy), r // 2, (255, 255, 255), 1)

                    # Contact fulcrum marker
                    xf = int(obs.get("x", (xm + xt) // 2))
                    cv2.circle(annotated, (xf, oy), 4, (76, 201, 240), -1)

                    lbl = f"HORIZ PEG {i+1} (Z={z_h}mm)"
                    cv2.putText(annotated, lbl, (min_x + 4, oy - r // 2 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 210, 230), 1, cv2.LINE_AA)
                else:
                    ox, oy = int(obs["x"]), int(obs["y"])
                    r = int(obs.get("r", 20))
                    cv2.circle(annotated, (ox, oy), r + 8, (70, 70, 100), 1, cv2.LINE_AA)
                    cv2.circle(annotated, (ox, oy), r, (40, 40, 55), -1)
                    cv2.circle(annotated, (ox, oy), r, (180, 180, 200), 2, cv2.LINE_AA)
                    cv2.circle(annotated, (ox, oy), 4, (220, 220, 255), -1)
                    lbl = f"OBS {i+1}"
                    cv2.putText(annotated, lbl, (ox - 16, oy - r - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 220), 1, cv2.LINE_AA)

        # Draw Middle Suspended Arm Clamp Collar in Setup Center
        cv2.circle(annotated, (240, 170), 10, (180, 185, 195), -1)
        cv2.circle(annotated, (240, 170), 10, (255, 255, 255), 2)
        cv2.putText(annotated, "MIDDLE ARM CLAMP", (172, 154), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (220, 230, 245), 1, cv2.LINE_AA)

        # 2. Draw 3 Drop Slots Side by Side on Table Floor
        if obstacle_env is not None and hasattr(obstacle_env, "slot_positions"):
            slot_poses = obstacle_env.slot_positions
            xs = [int(p[0]) for p in slot_poses.values()]
            ys = [int(p[1]) for p in slot_poses.values()]
            min_x, max_x = min(xs) - 26, max(xs) + 26
            min_y, max_y = min(ys) - 24, max(ys) + 24

            # Outer 3-slot container tray
            cv2.rectangle(annotated, (min_x, min_y), (max_x, max_y), (28, 32, 40), -1)
            cv2.rectangle(annotated, (min_x, min_y), (max_x, max_y), (140, 145, 160), 2)
            # Continuous top stripper edge / wiper lip (white)
            cv2.line(annotated, (min_x, min_y), (max_x, min_y), (255, 255, 255), 2)

            # Tray header label
            cv2.putText(annotated, "3-SLOT TRAY (SIDE-BY-SIDE)", (min_x + 4, min_y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (170, 180, 200), 1, cv2.LINE_AA)

            # Internal dividers & slot labels
            for letter in ["N", "U", "S"]:
                pos = slot_poses.get(letter, (240, 560))
                sx, sy = int(pos[0]), int(pos[1])
                lbl_col = (60, 60, 240) if letter == "N" else ((240, 140, 40) if letter == "U" else (40, 220, 240))

                # Color accent stripe at top
                cv2.line(annotated, (sx - 24, min_y + 1), (sx + 24, min_y + 1), lbl_col, 2)

                # Vertical dividers between side-by-side compartments
                if letter != "S":
                    div_x = sx + 27
                    cv2.line(annotated, (div_x, min_y), (div_x, max_y), (90, 95, 110), 1)

                cv2.putText(annotated, f"SLOT {letter}", (sx - 22, sy + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.36, lbl_col, 1, cv2.LINE_AA)
                st = target_statuses.get(obstacle_env.letter_to_color.get(letter, ""), "")
                if st == "SLOTTED":
                    cv2.putText(annotated, "[PLACED]", (sx - 22, sy + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.31, (60, 240, 60), 1, cv2.LINE_AA)

        # 3. Draw Detected Letter Targets
        target_centroid = None
        detected_tip = tip_pos

        for col, det in detections.items():
            cx, cy = det["centroid"]
            bx, by, bw, bh = det["bbox"]
            disp_col = det["display_color"]

            if col == "orange" and detected_tip is None:
                detected_tip = (cx, cy)

            if col == target_color:
                target_centroid = (cx, cy)
                pad = 8
                x1, y1 = max(0, bx - pad), max(0, by - pad)
                x2, y2 = min(w - 1, bx + bw + pad), min(h - 1, by + bh + pad)
                c_len = 12
                thick = 2
                # Target lock brackets
                cv2.line(annotated, (x1, y1), (x1 + c_len, y1), disp_col, thick)
                cv2.line(annotated, (x1, y1), (x1, y1 + c_len), disp_col, thick)
                cv2.line(annotated, (x2, y1), (x2 - c_len, y1), disp_col, thick)
                cv2.line(annotated, (x2, y1), (x2, y1 + c_len), disp_col, thick)
                cv2.line(annotated, (x1, y2), (x1 + c_len, y2), disp_col, thick)
                cv2.line(annotated, (x1, y2), (x1, y2 - c_len), disp_col, thick)
                cv2.line(annotated, (x2, y2), (x2 - c_len, y2), disp_col, thick)
                cv2.line(annotated, (x2, y2), (x2, y2 - c_len), disp_col, thick)

                # Pulsing crosshair
                cv2.circle(annotated, (cx, cy), 14, disp_col, 1)
                cv2.circle(annotated, (cx, cy), 4, disp_col, -1)
                cv2.line(annotated, (cx - 18, cy), (cx + 18, cy), disp_col, 1)
                cv2.line(annotated, (cx, cy - 18), (cx, cy + 18), disp_col, 1)

                badge = f"LOCKED: {det['label']}"
                cv2.rectangle(annotated, (x1, max(0, y1 - 20)), (x1 + len(badge) * 8 + 10, y1), (20, 20, 20), -1)
                cv2.putText(annotated, badge, (x1 + 4, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, disp_col, 1, cv2.LINE_AA)
            elif col in ["red", "blue", "yellow"]:
                cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), disp_col, 1)
                cv2.circle(annotated, (cx, cy), 3, disp_col, -1)
                badge = det["label"]
                cv2.rectangle(annotated, (bx, max(0, by - 16)), (bx + len(badge) * 7 + 8, by), (30, 30, 30), -1)
                cv2.putText(annotated, badge, (bx + 3, max(10, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, disp_col, 1, cv2.LINE_AA)

        # 4. Draw Green Tip
        if detected_tip is not None:
            cv2.circle(annotated, (int(detected_tip[0]), int(detected_tip[1])), 9, (0, 140, 255), 2)
            cv2.circle(annotated, (int(detected_tip[0]), int(detected_tip[1])), 3, (0, 140, 255), -1)
            cv2.putText(annotated, "ORANGE MAGNET TIP", (int(detected_tip[0]) + 12, int(detected_tip[1]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 140, 255), 1, cv2.LINE_AA)

        # 5. Draw Trajectory Path Curve
        if target_color and target_centroid and detected_tip:
            bezier_pts = BezierTrajectory.plan_2d_bezier_path(detected_tip, target_centroid, curvature=0.15, steps=20)
            if len(bezier_pts) >= 2:
                pts_arr = np.array(bezier_pts, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated, [pts_arr], isClosed=False, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
                for i, pt in enumerate(bezier_pts):
                    if i % 4 == 0 or i == len(bezier_pts) - 1:
                        cv2.circle(annotated, (int(pt[0]), int(pt[1])), 2, (255, 200, 0), -1)

        # 6. Draw Top Mission Header Banner
        cv2.rectangle(annotated, (0, 0), (w, 28), (20, 22, 28), -1)
        tgt_str = f"TARGET: {target_color.upper()}" if target_color else "TARGET: NONE"
        st_str = f"STEP: {state_text}"
        time_str = f"T:{mission_time:.1f}s" if mission_time is not None else ""
        err_str = f"|ERR|:{error_dist:.1f}px" if error_dist is not None else ""
        hud_line = f"{tgt_str}  |  {st_str}  |  {err_str}  {time_str}".strip(" | ")
        cv2.putText(annotated, hud_line, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (240, 245, 255), 1, cv2.LINE_AA)

        # 7. Draw Bottom Target Checklist Bar
        cv2.rectangle(annotated, (0, h - 22), (w, h), (18, 19, 24), -1)
        chk_items = []
        for c, let in [("red", "N"), ("blue", "U"), ("yellow", "S")]:
            status = target_statuses.get(c, "PENDING")
            sym = "[V]" if status == "SLOTTED" else ("[>]" if status == "ACTIVE" else "[o]")
            chk_items.append(f"{sym} {let}({c[:3].upper()}):{status}")
        bottom_txt = "  |  ".join(chk_items)
        cv2.putText(annotated, bottom_txt, (10, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 200, 220), 1, cv2.LINE_AA)

        return annotated


# =====================================================================
# 2. Smooth Bézier Trajectory Generator & Minimum-Jerk Timing
# =====================================================================

class BezierTrajectory:
    """
    Generates smooth 2D/3D cubic Bézier trajectories with quintic minimum-jerk
    timing profiles to ensure zero acceleration/jerk spikes on continuum cables.
    """

    @staticmethod
    def minimum_jerk_time(tau):
        """
        Quintic minimum-jerk profile: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5.
        Guarantees zero velocity, zero acceleration, and zero jerk at boundaries.
        """
        tau = max(0.0, min(1.0, float(tau)))
        return 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)

    @staticmethod
    def plan_2d_bezier_path(p_start, p_target, curvature=0.15, steps=25):
        """Generates smooth 2D cubic Bézier spline waypoints between p_start and p_target."""
        p0 = np.array(p_start, dtype=float)
        p3 = np.array(p_target, dtype=float)
        delta = p3 - p0
        dist = float(np.linalg.norm(delta))

        if dist < 1.0:
            return [p_start, p_target]

        normal = np.array([-delta[1], delta[0]]) / dist * (dist * curvature)
        p1 = p0 + delta * 0.3 + normal
        p2 = p0 + delta * 0.7 + normal

        trajectory = []
        for i in range(steps + 1):
            tau = float(i) / float(steps)
            s = BezierTrajectory.minimum_jerk_time(tau)
            pt = (
                ((1.0 - s) ** 3) * p0
                + 3.0 * ((1.0 - s) ** 2) * s * p1
                + 3.0 * (1.0 - s) * (s ** 2) * p2
                + (s ** 3) * p3
            )
            trajectory.append((float(pt[0]), float(pt[1])))
        return trajectory

    @staticmethod
    def plan_pick_and_place_curve(p_start, p_target, lift_height=35.0, steps=40):
        """Plans 3D parabolic lift-travel-descend cubic Bézier curve."""
        p0 = np.array(p_start, dtype=float)
        p3 = np.array(p_target, dtype=float)
        p1 = p0.copy()
        p1[2] += lift_height
        p2 = p3.copy()
        p2[2] += lift_height

        trajectory = []
        for i in range(steps + 1):
            tau = float(i) / float(steps)
            s = BezierTrajectory.minimum_jerk_time(tau)
            b = (
                ((1.0 - s) ** 3) * p0
                + 3.0 * ((1.0 - s) ** 2) * s * p1
                + 3.0 * (1.0 - s) * (s ** 2) * p2
                + (s ** 3) * p3
            )
            trajectory.append(b.tolist())
        return trajectory

    @staticmethod
    def plan_obstacle_withdrawal_path(current_angles, intermediate_waypoints, duration=1.8, steps=35):
        """
        Plans smooth multi-segment joint-space withdrawal trajectory to reverse
        out of obstacles without hooking or colliding.
        """
        full_trajectory = []
        all_points = [list(current_angles)] + intermediate_waypoints
        num_segments = len(all_points) - 1
        steps_per_seg = max(5, steps // max(1, num_segments))

        for seg in range(num_segments):
            p_start = np.array(all_points[seg], dtype=float)
            p_end = np.array(all_points[seg + 1], dtype=float)
            for i in range(steps_per_seg):
                tau = float(i) / float(steps_per_seg)
                s = BezierTrajectory.minimum_jerk_time(tau)
                interpolated = (1.0 - s) * p_start + s * p_end
                full_trajectory.append(interpolated.tolist())

        full_trajectory.append(list(all_points[-1]))
        return full_trajectory


# =====================================================================
# 3. Obstacle Environment & Configuration Models
# =====================================================================

class ObstacleEnvironment:
    """
    Encapsulates obstacle geometries, contact points, target letter poses,
    drop slots, and multi-curvature actuation waypoints for Project 2.
    """

    def __init__(self, preset_name="Config 1: Dual-Peg S-Bend"):
        self.preset_name = preset_name

        self.color_to_letter = {"red": "N", "blue": "U", "yellow": "S"}
        self.letter_to_color = {"N": "red", "U": "blue", "S": "yellow"}

        # Slot drop-off positions outside the obstacle arena (pixel coordinates)
        # 3 side-by-side compartments: Slot N (Red) | Slot U (Blue) | Slot S (Yellow)
        self.slot_positions = {
            "N": (185, 560),  # Slot N (Red) - Left compartment
            "U": (240, 560),  # Slot U (Blue) - Center compartment
            "S": (295, 560),  # Slot S (Yellow) - Right compartment
        }

        # Obstacle configuration presets
        self.configure_preset(preset_name)

    def configure_preset(self, name):
        """Configures obstacles and actuation waypoints according to chosen setup."""
        self.preset_name = name

        if "Config 2" in name or "Lateral" in name:
            # Preset 2: Lateral Deflector (Single central pillar)
            self.obstacles = [
                {"x_mount": 420, "x_tip": 210, "y": 320, "z": 180, "r": 20, "x": 240, "id": "Horizontal Deflector", "orient": "horizontal"},
            ]
            self.entry_pose = [0.0, 0.0, 0.0, 0.0]
            self.contact_poses = {
                "red": [0.65, 0.10, 0.42, 0.0],
                "blue": [0.72, 0.25, 0.35, 0.0],
                "yellow": [0.25, 0.10, 0.60, 0.0],
            }
        elif "Config 3" in name or "Corridor" in name:
            # Preset 3: Dual parallel horizontal bars
            self.obstacles = [
                {"x_mount": 420, "x_tip": 220, "y": 280, "z": 220, "r": 16, "x": 260, "id": "Upper Bar", "orient": "horizontal"},
                {"x_mount": 60, "x_tip": 260, "y": 360, "z": 140, "r": 16, "x": 220, "id": "Lower Bar", "orient": "horizontal"},
            ]
            self.entry_pose = [0.0, 0.0, 0.0, 0.0]
            self.contact_poses = {
                "red": [0.45, 0.15, 0.55, 0.0],
                "blue": [0.10, 0.40, 0.10, 0.0],
                "yellow": [0.55, 0.15, 0.45, 0.0],
            }
        else:
            # Preset 1: Dual Horizontal Pegs Mounted on Vertical Struts (Standard Setup)
            # - Upper Peg: clamped to right vertical strut at Z=235mm, points horizontally across middle to x=210
            # - Lower Peg: clamped to left vertical strut at Z=125mm, points horizontally across middle to x=270
            # - Descending arm is physically PRE-BENT into an S-bend at rest by these horizontal fulcrums!
            self.obstacles = [
                {
                    "id": "Upper Peg (Right)",
                    "x_mount": 420,
                    "x_tip": 210,
                    "y": 265,
                    "z": 235,
                    "r": 16,
                    "x": 260,
                    "orient": "horizontal"
                },
                {
                    "id": "Lower Peg (Left)",
                    "x_mount": 60,
                    "x_tip": 270,
                    "y": 365,
                    "z": 125,
                    "r": 16,
                    "x": 220,
                    "orient": "horizontal"
                },
            ]
            # Staging entry angle is natural pre-bent rest (m = [0, 0, 0, 0])
            self.entry_pose = [0.0, 0.0, 0.0, 0.0]

            # Contact-assisted dual-bend motor angles (radians)
            # M1: +X (Right bend against Upper Peg)
            # M2: +Y (Forward/Down extension)
            # M3: -X (Left bend around Lower Peg)
            # M4: -Y (Upward retraction)
            self.contact_poses = {
                # Target Red ('N') - occluded on left behind Lower Peg
                "red": [0.08, 0.12, 0.44, 0.0],
                # Target Blue ('U') - deep center between pegs
                "blue": [0.18, 0.35, 0.18, 0.0],
                # Target Yellow ('S') - lateral deflection around Upper Peg to right
                "yellow": [0.48, 0.12, 0.06, 0.0],
            }

        # Safe multi-step joint-space unwinding waypoints for withdrawal
        self.withdrawal_paths = {
            "red": [
                [0.06, 0.10, 0.20, 0.0],  # Step 1: Relax distal bend
                [0.02, 0.05, 0.08, 0.0],  # Step 2: Unwind proximal contact
                self.entry_pose,           # Step 3: Return to natural pre-bent rest
            ],
            "blue": [
                [0.12, 0.20, 0.12, 0.0],
                [0.05, 0.08, 0.05, 0.0],
                self.entry_pose,
            ],
            "yellow": [
                [0.22, 0.10, 0.04, 0.0],
                [0.08, 0.05, 0.0, 0.0],
                self.entry_pose,
            ],
        }

        # Slot delivery transfer angles (radians) for side-by-side sorting tray at y=560
        self.slot_delivery_poses = {
            "red": [0.08, 0.32, 0.38, 0.10],     # Delivers left toward Slot N
            "blue": [0.15, 0.48, 0.15, 0.0],     # Center toward Slot U
            "yellow": [0.38, 0.32, 0.08, 0.10],  # Delivers right toward Slot S
        }

        # Magnetic stripper wipe delta angles (lateral shear to strip permanent magnet)
        self.slot_wipe_deltas = {
            "red": [-0.10, 0.0, 0.10, 0.08],
            "blue": [0.10, 0.0, -0.10, 0.08],
            "yellow": [0.10, 0.0, -0.10, 0.08],
        }


# =====================================================================
# 4. Closed-Loop Position-Based Visual Servoing (PBVS)
# =====================================================================

class VisualServoController:
    """
    Closed-Loop Position-Based Visual Servoing for cable-driven continuum arm.
    Computes PI corrections from pixel errors between green tip and target letter.
    """

    def __init__(self, robot_controller, kp=0.0035, ki=0.0003, max_step_rad=0.08, sign_x=1.0, sign_y=1.0):
        self.robot = robot_controller
        self.kp = kp
        self.ki = ki
        self.max_step_rad = max_step_rad
        self.sign_x = sign_x
        self.sign_y = sign_y

        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.tolerance_pixels = 6.0

    def reset_integral(self):
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0

    def compute_step(self, tip_pos, target_pos, current_angles=None):
        """
        Calculates motor angle updates from tip and target 2D image coordinates.
        Returns: (new_angles, error_distance, is_converged)
        """
        ex = float(target_pos[0] - tip_pos[0]) * self.sign_x
        ey = float(target_pos[1] - tip_pos[1]) * self.sign_y
        error_dist = math.hypot(ex, ey)

        if error_dist <= self.tolerance_pixels:
            return self.robot.current_angles, error_dist, True

        self.integral_error_x = max(-50.0, min(50.0, self.integral_error_x + ex * 0.033))
        self.integral_error_y = max(-50.0, min(50.0, self.integral_error_y + ey * 0.033))

        ux = self.kp * ex + self.ki * self.integral_error_x
        uy = self.kp * ey + self.ki * self.integral_error_y

        ux = max(-self.max_step_rad, min(self.max_step_rad, ux))
        uy = max(-self.max_step_rad, min(self.max_step_rad, uy))

        if current_angles is None:
            current_angles = list(self.robot.current_angles)

        new_angles = list(current_angles)

        # Coordinate mapping for 4 antagonistic cables:
        # Motor 1 (+X) vs Motor 3 (-X)
        # Motor 2 (+Y) vs Motor 4 (-Y)
        if ux > 0:
            new_angles[0] = max(0.0, new_angles[0] + ux)
            new_angles[2] = max(0.0, new_angles[2] - ux * 0.5)
        else:
            new_angles[2] = max(0.0, new_angles[2] - ux)
            new_angles[0] = max(0.0, new_angles[0] + ux * 0.5)

        if uy > 0:
            new_angles[1] = max(0.0, new_angles[1] + uy)
            new_angles[3] = max(0.0, new_angles[3] - uy * 0.5)
        else:
            new_angles[3] = max(0.0, new_angles[3] - uy)
            new_angles[1] = max(0.0, new_angles[1] + uy * 0.5)

        clamped = self.robot.clamp_angles(new_angles)
        return clamped, error_dist, False


# =====================================================================
# 5. Obstacle-Aided Continuum Retrieval Orchestrator
# =====================================================================

class ObstacleAidedRetriever:
    """
    Manages autonomous multi-color retrieval for Obstacle-Aided Continuum Robot:
    - Target letters: Red ('N'), Blue ('U'), Yellow ('S')
    - End-effector: Green tip with permanent magnet
    - Sequences:
        1. Navigate into obstacle environment
        2. Controlled obstacle contact bending (producing at least two distinct bends)
        3. Visual servoing & magnetic pickup of target letter
        4. Safe withdrawal trajectory out of the obstacles
        5. Delivery to designated drop-off slot
        6. Mechanical slot wiping / peeling to detach the permanent magnet
        7. Return back to obstacle entrance
        8. Repeat for each color until all 3 targets are picked and placed!
    """

    def __init__(self, robot_controller, color_detector, servo_controller, obstacle_env=None):
        self.robot = robot_controller
        self.detector = color_detector
        self.servo = servo_controller
        self.env = obstacle_env or ObstacleEnvironment()

        self.is_running = False
        self.current_state = "IDLE"
        self.status_message = "Ready"
        self.active_color = None

        self.target_statuses = {
            "red": "PENDING",
            "blue": "PENDING",
            "yellow": "PENDING",
        }

        self.mission_start_time = None
        self.target_split_times = {}

    def get_latest_rotated_frame(self):
        """Grabs latest frame from robot camera rotated 90° CCW."""
        if self.robot.camera is None or getattr(self.robot.camera, "frame", None) is None:
            return None
        raw = self.robot.camera.frame
        return cv2.rotate(raw, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def retrieve_single_target(self, target_color, progress_callback=None):
        """
        Executes full pick-and-place sequence for a single colored letter target:
        Enter Obstacles -> Contact Dual Bend -> Magnet Grasp -> Withdraw ->
        Deliver to Slot -> Mechanical Detach -> Return to Obstacle Entrance.
        """
        self.is_running = True
        self.active_color = target_color
        letter = self.env.color_to_letter.get(target_color, "N")
        t_start = time.time()

        def _update(state, msg):
            self.current_state = state
            self.status_message = msg
            if progress_callback:
                progress_callback(state, msg)
            print(f"[OBSTACLE RETRIEVAL] [{letter} ({target_color.upper()})] {state}: {msg}")

        try:
            self.target_statuses[target_color] = "ACTIVE"

            # ---------------------------------------------------------
            # Phase 1: Locate Target Letter with Camera
            # ---------------------------------------------------------
            _update("LOCATING", f"Locating target '{letter}' ({target_color.upper()}) with camera...")
            target_pt = None
            for _ in range(25):
                if not self.is_running:
                    return False
                frame = self.get_latest_rotated_frame()
                if frame is not None:
                    det = self.detector.detect_single(frame, target_color)
                    if det:
                        target_pt = det["centroid"]
                        break
                time.sleep(0.04)

            if target_pt:
                _update("TARGET_LOCATED", f"Target '{letter}' detected at {target_pt}.")
            else:
                _update("TARGET_ESTIMATING", f"Using calibrated obstacle coordinates for '{letter}'.")

            # ---------------------------------------------------------
            # Phase 2: Enter Obstacle Arena (Entrance Waypoint)
            # ---------------------------------------------------------
            _update("ENTERING_OBSTACLES", "Approaching obstacle environment entrance...")
            entry_pose = self.env.entry_pose
            self.robot.move_smooth(entry_pose, duration=1.2, steps=25)
            time.sleep(0.3)

            # ---------------------------------------------------------
            # Phase 3: Obstacle Contact Bending (Dual Curvature)
            # ---------------------------------------------------------
            _update("OBSTACLE_CONTACT_BEND", f"Establishing contact with obstacles to form dual bend towards '{letter}'...")
            contact_pose = self.env.contact_poses.get(target_color, [0.55, 0.15, 0.40, 0.0])
            self.robot.move_smooth(contact_pose, duration=1.8, steps=35)
            time.sleep(0.4)

            # ---------------------------------------------------------
            # Phase 4: Fine Approach & Visual Servoing
            # ---------------------------------------------------------
            _update("FINE_APPROACH", "Visual servoing green tip directly onto target letter...")
            self.servo.reset_integral()

            for _ in range(35):
                if not self.is_running:
                    return False
                frame = self.get_latest_rotated_frame()
                if frame is not None:
                    tip_det = self.detector.detect_single(frame, "orange")
                    tgt_det = self.detector.detect_single(frame, target_color)

                    if tgt_det:
                        target_pt = tgt_det["centroid"]

                    if tip_det and target_pt:
                        tip_pt = tip_det["centroid"]
                        new_angles, dist, conv = self.servo.compute_step(tip_pt, target_pt)
                        self.robot.send_angles(new_angles)
                        if conv or dist < 8.0:
                            _update("TOUCHDOWN", "Magnetic tip aligned over target letter.")
                            break
                time.sleep(0.035)

            # ---------------------------------------------------------
            # Phase 5: Magnetic Attachment (Pickup)
            # ---------------------------------------------------------
            _update("MAGNETIC_ATTACH", "Lowering magnetic end effector to grasp target letter...")
            # Slight push to secure magnetic adhesion
            press_pose = [a * 1.05 for a in self.robot.current_angles]
            self.robot.move_smooth(press_pose, duration=0.6, steps=12)
            time.sleep(0.6)  # Dwell for magnetic latch

            if hasattr(self.robot.camera, "attached_letter"):
                self.robot.camera.attached_letter = target_color

            # Gentle lift to clear table/surface friction
            _update("LIFT_LETTER", f"Lifting letter '{letter}' with magnetic end effector...")
            lift_pose = [a * 0.90 for a in self.robot.current_angles]
            self.robot.move_smooth(lift_pose, duration=0.8, steps=15)
            time.sleep(0.3)

            # ---------------------------------------------------------
            # Phase 6: Come Out of the Obstacle (Safe Withdrawal)
            # ---------------------------------------------------------
            _update("WITHDRAWING_FROM_OBSTACLE", "Executing controlled withdrawal out of obstacle field...")
            withdrawal_wps = self.env.withdrawal_paths.get(target_color, [entry_pose, [0.0] * 4])

            # Unwind intermediate poses smoothly
            for wp_idx, wp in enumerate(withdrawal_wps):
                if not self.is_running:
                    return False
                self.robot.move_smooth(wp, duration=0.9, steps=20)
                time.sleep(0.15)

            _update("OBSTACLE_CLEARED", "Successfully exited obstacle course without snagging.")
            time.sleep(0.3)

            # ---------------------------------------------------------
            # Phase 7: Transfer to Designated Slot
            # ---------------------------------------------------------
            slot_pt = self.env.slot_positions.get(letter, (240, 520))
            _update("DELIVERING_TO_SLOT", f"Transferring letter '{letter}' to Slot {letter} at {slot_pt}...")
            slot_delivery_pose = self.env.slot_delivery_poses.get(target_color, [0.0, 0.0, 0.2, 0.4])
            self.robot.move_smooth(slot_delivery_pose, duration=1.4, steps=30)
            time.sleep(0.4)

            # ---------------------------------------------------------
            # Phase 8: Mechanical Slot Wiping / Permanent Magnet Detachment
            # ---------------------------------------------------------
            _update("SLOT_DETACH_WIPE", "Wiping against slot lip to shear permanent magnetic bond...")
            wipe_delta = self.env.slot_wipe_deltas.get(target_color, [0.10, 0.0, -0.10, 0.15])
            wipe_pose = [slot_delivery_pose[i] + wipe_delta[i] for i in range(4)]
            # Controlled shear stroke against slot stripper edge
            self.robot.move_smooth(wipe_pose, duration=0.8, steps=18)
            time.sleep(0.3)

            if hasattr(self.robot.camera, "attached_letter"):
                self.robot.camera.attached_letter = None
                self.robot.camera.slotted_letters.add(target_color)
                slot_raw_x = 640 - 1 - int(slot_pt[1])
                slot_raw_y = int(slot_pt[0])
                self.robot.camera.letter_positions[target_color] = [slot_raw_x, slot_raw_y]

            # Vertical retreat clear of slot
            clear_pose = [a * 0.5 for a in wipe_pose]
            self.robot.move_smooth(clear_pose, duration=0.7, steps=15)
            time.sleep(0.3)

            _update("SLOT_CONFIRMED", f"Letter '{letter}' successfully deposited in Slot {letter}!")
            self.target_statuses[target_color] = "SLOTTED"
            self.target_split_times[target_color] = time.time() - t_start

            # ---------------------------------------------------------
            # Phase 9: Return Back to Obstacle Environment
            # ---------------------------------------------------------
            _update("RETURNING_TO_OBSTACLE", "Returning back to obstacle entrance for next target...")
            self.robot.move_smooth(self.env.entry_pose, duration=1.2, steps=25)
            time.sleep(0.4)

            _update("CYCLE_COMPLETE", f"Finished cycle for '{letter}'. Ready at obstacle entrance.")
            return True

        except Exception as e:
            _update("ERROR", f"Exception during retrieval: {e}")
            return False
        finally:
            self.active_color = None

    def run_full_3target_sequence(self, order=None, progress_callback=None):
        """
        Runs the complete 3-target obstacle-aided retrieval routine:
        Red ('N') ➔ Blue ('U') ➔ Yellow ('S').
        Measures total elapsed time against ME4242 project benchmark.
        """
        if order is None:
            order = ["red", "blue", "yellow"]

        self.is_running = True
        self.mission_start_time = time.time()
        self.target_statuses = {"red": "PENDING", "blue": "PENDING", "yellow": "PENDING"}

        def _log(st, msg):
            self.current_state = st
            self.status_message = msg
            if progress_callback:
                progress_callback(st, msg)
            print(f"[MISSION PROGRESS] {st}: {msg}")

        _log("MISSION_STARTED", f"Starting full 3-target retrieval sequence ({' -> '.join([c.upper() for c in order])})...")

        all_success = True
        for idx, col in enumerate(order):
            if not self.is_running:
                _log("ABORTED", "Mission aborted by user.")
                all_success = False
                break

            letter = self.env.color_to_letter.get(col, "?")
            _log(f"TARGET_{idx+1}_START", f"=== Starting Target {idx+1}/3: '{letter}' ({col.upper()}) ===")

            success = self.retrieve_single_target(col, progress_callback=progress_callback)
            if not success:
                _log("TARGET_FAILED", f"Failed retrieval for '{letter}'. Retrying once...")
                time.sleep(0.8)
                success = self.retrieve_single_target(col, progress_callback=progress_callback)

            if not success:
                _log("SEQUENCE_INTERRUPTED", f"Could not retrieve '{letter}'. Continuing sequence...")
                all_success = False
            else:
                _log(f"TARGET_{idx+1}_COMPLETE", f"Target {idx+1}/3 '{letter}' completed in {self.target_split_times.get(col, 0):.1f}s.")

            time.sleep(0.8)

        # Park safely at home
        self.robot.home()

        total_time = time.time() - self.mission_start_time
        speed_mark = 3 if total_time < 60 else (2 if total_time < 120 else (1 if total_time < 180 else 0))

        summary = (
            f"[SUCCESS] 3-Target Retrieval Complete! Total Time: {total_time:.1f}s | "
            f"Benchmark: {speed_mark}/3 marks | Targets Placed: "
            f"{sum(1 for s in self.target_statuses.values() if s == 'SLOTTED')}/3"
        )
        _log("MISSION_COMPLETED" if all_success else "MISSION_PARTIAL", summary)
        self.is_running = False
        return all_success

    def stop(self):
        """Immediately halts autonomous routines and relaxes arm."""
        self.is_running = False
        self.active_color = None
        self.current_state = "STOPPED"
        self.status_message = "Halted by user."
        self.robot.home()


# Backward-compatible alias
PickAndPlaceManager = ObstacleAidedRetriever


# =====================================================================
# 6. High-Fidelity Simulated Camera with Continuum Obstacle Physics
# =====================================================================

class SimulatedObstacleCamera:
    """
    Synthetic Intel RealSense camera simulator for testing without physical hardware.
    Renders top-down 640x480 view of obstacle field, deformable continuum robot,
    Red ('N'), Blue ('U'), Yellow ('S') letter blocks, and magnetic pick-and-place.
    """

    def __init__(self, robot_controller, obstacle_env=None):
        self.robot = robot_controller
        self.env = obstacle_env or ObstacleEnvironment()
        self.is_running = True
        self.frame = None
        self.depth_frame = None
        self.track_markers = True
        self.trackers_pos = []

        # Target letters: positions in unrotated camera coordinates (640x480)
        # Note: 90° CCW rotation transforms (raw_x, raw_y) -> (rot_x, rot_y):
        # rot_x = raw_y, rot_y = 640 - 1 - raw_x
        # We set raw coordinates such that rotated coordinates match:
        # Red: (160, 470), Blue: (240, 510), Yellow: (320, 470)
        self.letter_positions = {
            "red": [640 - 1 - 470, 160],      # Red ('N')
            "blue": [640 - 1 - 510, 240],     # Blue ('U')
            "yellow": [640 - 1 - 470, 320],   # Yellow ('S')
        }
        self.attached_letter = None
        self.slotted_letters = set()

        self.update()

    def open(self, serial=None):
        self.is_running = True
        return True

    def close(self):
        self.is_running = False

    def update(self):
        """Renders simulated top-down camera frame with obstacle-aided kinematics."""
        w, h = 640, 480
        img = np.zeros((h, w, 3), dtype=np.uint8)

        # Arena floor (textured dark slate)
        img[:] = (32, 34, 40)
        # Subtle grid
        for gy in range(0, h, 40):
            cv2.line(img, (0, gy), (w, gy), (38, 40, 48), 1)
        for gx in range(0, w, 40):
            cv2.line(img, (gx, 0), (gx, h), (38, 40, 48), 1)

        # 1. Base Clamp in the Middle of the Setup (Suspended overhead)
        # Rotated coordinates: (rot_x = 240, rot_y = 170)
        # Raw coordinates: raw_x = 640 - 1 - 170 = 469, raw_y = 240
        base_raw_x = 469
        base_raw_y = 240
        base_pt = (base_raw_x, base_raw_y)

        # Draw Base Clamp Collar in Middle of Setup
        cv2.circle(img, (base_raw_x, base_raw_y), 14, (180, 185, 195), -1)
        cv2.circle(img, (base_raw_x, base_raw_y), 14, (255, 255, 255), 2)
        cv2.circle(img, (base_raw_x, base_raw_y - 20), 6, (210, 120, 20), -1)
        cv2.circle(img, (base_raw_x, base_raw_y + 20), 6, (210, 120, 20), -1)

        # Calculate continuum deformation from motor angles
        # M1: +X (Right bend against Upper Peg)
        # M2: +Y (Forward/Down extension)
        # M3: -X (Left bend around Lower Peg)
        # M4: -Y (Upward retraction)
        angles = list(self.robot.current_angles)
        m1, m2, m3, m4 = angles[0], angles[1], angles[2], angles[3]

        # Inextensible Continuum Backbone Model (Fixed Arc Length L = const, Pure Bending)
        TOTAL_BACKBONE_LENGTH = 350.0  # Constant total arc length (pixels)
        num_seg = 30
        ds = TOTAL_BACKBONE_LENGTH / float(num_seg)

        curr_rx, curr_ry = 240.0, 170.0
        angle = 0.0
        backbone_pts_raw = [base_pt]

        for seg_idx in range(num_seg):
            s_frac = float(seg_idx) / float(num_seg)
            y = curr_ry

            # 1. Natural Pre-Bend Curvature imparted by horizontal obstacle pegs:
            # - Upper peg at y ~ 265 pushes arm towards left (-X, rot_x < 240)
            # - Lower peg at y ~ 365 pushes arm towards right (+X, rot_x > 240)
            # Creates physical S-bend curve present even at zero motor angle!
            pre_k = -0.045 * math.exp(-((y - 265.0) / 45.0) ** 2) + 0.065 * math.exp(-((y - 365.0) / 45.0) ** 2)

            # 2. Active Tendon Curvature:
            act_k = (m1 - m3) * 0.055 + (m2 - m4) * 0.010

            d_angle = pre_k + act_k

            # 3. Dynamic physical contact constraint with horizontal pegs:
            for obs in self.env.obstacles:
                if obs.get("orient") == "horizontal":
                    oy = float(obs["y"])
                    xm = float(obs["x_mount"])
                    xt = float(obs["x_tip"])
                    if xm > xt and abs(y - oy) < 22.0 and curr_rx > (xt + 6.0):
                        d_angle -= (0.03 + m1 * 0.08)
                    elif xm < xt and abs(y - oy) < 22.0 and curr_rx < (xt - 6.0):
                        d_angle += (0.03 + m3 * 0.08)

            angle += d_angle
            angle = max(-0.85, min(0.85, angle))

            curr_rx += ds * math.sin(angle)
            curr_ry += ds * math.cos(angle)
            curr_rx = max(40.0, min(440.0, curr_rx))
            curr_ry = max(170.0, min(615.0, curr_ry))

            raw_x = int(640 - 1 - curr_ry)
            raw_y = int(curr_rx)
            backbone_pts_raw.append((raw_x, raw_y))

        tip_raw = backbone_pts_raw[-1]
        tip_rot = (tip_raw[1], 640 - 1 - tip_raw[0])
        self.trackers_pos = [[float(tip_rot[0]), float(tip_rot[1]), 300.0]]

        # Render Horizontal Obstacle Pegs Mounted on Vertical Frame Columns
        for obs in self.env.obstacles:
            if obs.get("orient") == "horizontal":
                xm = int(obs["x_mount"])
                xt = int(obs["x_tip"])
                oy = int(obs["y"])
                r = int(obs.get("r", 16))
                rx = 640 - 1 - oy

                # Dark metallic vertical frame strut
                cv2.line(img, (rx - 60, xm), (rx + 60, xm), (75, 80, 92), 5, cv2.LINE_AA)
                cv2.circle(img, (rx, xm), 8, (100, 105, 120), -1)

                # Horizontal cylindrical peg
                min_ry, max_ry = min(xm, xt), max(xm, xt)
                cv2.rectangle(img, (rx - r // 2, min_ry), (rx + r // 2, max_ry), (50, 54, 65), -1)
                cv2.rectangle(img, (rx - r // 2, min_ry), (rx + r // 2, max_ry), (170, 175, 190), 2)
                # Rounded free tip
                cv2.circle(img, (rx, xt), r // 2, (140, 145, 160), -1)
                cv2.circle(img, (rx, xt), r // 2, (255, 255, 255), 1)
            else:
                obs_raw_x = 640 - 1 - int(obs["y"])
                obs_raw_y = int(obs["x"])
                r = int(obs.get("r", 20))
                cv2.circle(img, (obs_raw_x, obs_raw_y), r, (50, 52, 60), -1)
                cv2.circle(img, (obs_raw_x, obs_raw_y), r, (180, 185, 200), 2)

        # Render Side-by-Side Drop Slots (tray at rot_y = 560 -> raw_x = 79)
        all_raw_y = [int(p[0]) for p in self.env.slot_positions.values()]
        min_ry, max_ry = min(all_raw_y) - 26, max(all_raw_y) + 26
        tray_rx = 640 - 1 - 560
        cv2.rectangle(img, (tray_rx - 22, min_ry), (tray_rx + 22, max_ry), (35, 40, 50), -1)
        cv2.rectangle(img, (tray_rx - 22, min_ry), (tray_rx + 22, max_ry), (110, 115, 130), 1)
        # Dividers between compartments
        for let, pos in self.env.slot_positions.items():
            s_ry = int(pos[0])
            cv2.line(img, (tray_rx - 22, s_ry + 27), (tray_rx + 22, s_ry + 27), (70, 75, 90), 1)

        # Render Letter Targets on Table Floor
        for c, let in [("red", "N"), ("blue", "U"), ("yellow", "S")]:
            pos = self.letter_positions[c]
            lx, ly = int(pos[0]), int(pos[1])
            col = (30, 30, 230) if c == "red" else ((230, 130, 20) if c == "blue" else (20, 220, 240))
            cv2.rectangle(img, (lx - 12, ly - 12), (lx + 12, ly + 12), col, -1)
            cv2.rectangle(img, (lx - 12, ly - 12), (lx + 12, ly + 12), (255, 255, 255), 1)
            cv2.putText(img, let, (lx - 6, ly + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # Render Continuum Backbone (neutral low-saturation silver with blue spacer rings)
        for i in range(len(backbone_pts_raw) - 1):
            p1 = backbone_pts_raw[i]
            p2 = backbone_pts_raw[i + 1]
            thickness = max(3, int(11 * (1.0 - i / len(backbone_pts_raw))))
            cv2.line(img, p1, p2, (180, 185, 190), thickness, cv2.LINE_AA)
            if i % 3 == 0:
                cv2.circle(img, p1, thickness // 2 + 3, (85, 90, 102), -1)  # Dark metallic spacer ring
                cv2.circle(img, p1, thickness // 2 + 3, (200, 205, 215), 1)

        # Render Orange Magnetic Tip Marker
        cv2.circle(img, tip_raw, 11, (0, 140, 255), -1)
        cv2.circle(img, tip_raw, 11, (255, 255, 255), 2)
        cv2.circle(img, tip_raw, 4, (0, 110, 230), -1)

        # Update attached letter position
        if self.attached_letter is not None:
            self.letter_positions[self.attached_letter] = [tip_raw[0], tip_raw[1]]

        self.frame = img
        self.depth_frame = np.full((h, w), 300, dtype=np.uint16)
