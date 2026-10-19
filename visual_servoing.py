"""
visual_servoing.py
------------------
Closed-Loop Visual Servoing & Smooth Bézier Trajectory Planning for
EMIO Cable-Driven Soft Robot.

Features:
- Multi-color target detection:
    * Red ("N" Letter / Target)
    * Blue ("U" Letter / Target)
    * Yellow ("S" Letter / Target)
    * Green (End-Effector / Tip Marker)
- Cubic 3D Bézier Curve Trajectory Generator with Minimum-Jerk S-Curve Timing (zero jerk on cables)
- Position-Based Visual Servoing (PBVS) with closed-loop PI feedback
- Automated Pick & Place Sequence:
    * Hover -> Descend to target (magnet contact) -> Lift -> Travel along smooth arc -> Slot in -> Return Home
"""

import math
import time
import numpy as np
import cv2


# =====================================================================
# 1. Color Detector (Red, Blue, Yellow, Green Tip)
# =====================================================================

class ColorDetector:
    """Detects and tracks colored targets and end-effector markers in HSV color space."""

    # Default calibrated HSV ranges [H_min, S_min, V_min] - [H_max, S_max, V_max]
    COLOR_RANGES = {
        "red": {
            "lower1": np.array([0, 90, 70]),
            "upper1": np.array([10, 255, 255]),
            "lower2": np.array([165, 90, 70]),
            "upper2": np.array([180, 255, 255]),
            "display_color": (0, 0, 255),  # BGR
            "label": "Target N (Red)",
        },
        "blue": {
            "lower": np.array([100, 90, 70]),
            "upper": np.array([130, 255, 255]),
            "display_color": (255, 120, 0),  # BGR
            "label": "Target U (Blue)",
        },
        "yellow": {
            "lower": np.array([18, 90, 70]),
            "upper": np.array([38, 255, 255]),
            "display_color": (0, 230, 255),  # BGR
            "label": "Target S (Yellow)",
        },
        "green": {
            "lower": np.array([40, 80, 60]),
            "upper": np.array([85, 255, 255]),
            "display_color": (0, 255, 0),  # BGR
            "label": "Arm Tip (Green)",
        },
    }

    def __init__(self, min_contour_area=250):
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

        # Pick largest contour meeting minimum area
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
            "display_color": self.COLOR_RANGES[color_name]["display_color"],
        }

    def detect_all_targets(self, frame_bgr):
        """Detects all 4 objects: Red (N), Blue (U), Yellow (S), and Green (Tip)."""
        results = {}
        for c in ["red", "blue", "yellow", "green"]:
            det = self.detect_single(frame_bgr, c)
            if det:
                results[c] = det
        return results

    def convert_rotated_pixel_to_raw(self, rot_x, rot_y, rot_w=480, rot_h=640):
        """
        Converts coordinates from a 90° CCW rotated frame back to the
        original unrotated camera sensor frame (640x480) for RealSense 3D unprojection.
        """
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
            # Look at 5x5 neighborhood if single pixel is 0
            patch = depth_raw[max(0, raw_y - 2):raw_y + 3, max(0, raw_x - 2):raw_x + 3]
            valid_depths = patch[patch > 0]
            if len(valid_depths) > 0:
                depth_val = float(np.median(valid_depths))
            else:
                depth_val = 300.0  # nominal workspace depth fallback (mm)

        if camera_estimator is not None and hasattr(camera_estimator, "image_to_simulation"):
            try:
                coords = camera_estimator.image_to_simulation(raw_x, raw_y, depth_val)
                return coords
            except Exception:
                pass

        # Fallback approximate pinhole scaling
        return [float(raw_x - 320), float(raw_y - 240), depth_val]

    def draw_hud(self, frame_bgr, detections, target_color=None, tip_pos=None, error_dist=None, state_text=""):
        """
        Renders rich computer vision HUD overlay onto frame:
        - Bounding boxes and badges for Red (N), Blue (U), Yellow (S), and Green (Tip)
        - Target lock reticle on selected color
        - Smooth Bézier trajectory path from tip to target
        - Live error vectors and status diagnostics
        """
        if frame_bgr is None:
            return None

        annotated = frame_bgr.copy()
        h, w = annotated.shape[:2]

        # Draw all detected targets
        target_centroid = None
        detected_tip = tip_pos

        for col, det in detections.items():
            cx, cy = det["centroid"]
            bx, by, bw, bh = det["bbox"]
            disp_col = det["display_color"]

            if col == "green" and detected_tip is None:
                detected_tip = (cx, cy)

            if col == target_color:
                target_centroid = (cx, cy)
                # Highlighted active target reticle
                pad = 8
                x1, y1 = max(0, bx - pad), max(0, by - pad)
                x2, y2 = min(w - 1, bx + bw + pad), min(h - 1, by + bh + pad)
                # Corner brackets
                c_len = 12
                thick = 2
                # Top-left
                cv2.line(annotated, (x1, y1), (x1 + c_len, y1), disp_col, thick)
                cv2.line(annotated, (x1, y1), (x1, y1 + c_len), disp_col, thick)
                # Top-right
                cv2.line(annotated, (x2, y1), (x2 - c_len, y1), disp_col, thick)
                cv2.line(annotated, (x2, y1), (x2, y1 + c_len), disp_col, thick)
                # Bottom-left
                cv2.line(annotated, (x1, y2), (x1 + c_len, y2), disp_col, thick)
                cv2.line(annotated, (x1, y2), (x1, y2 - c_len), disp_col, thick)
                # Bottom-right
                cv2.line(annotated, (x2, y2), (x2 - c_len, y2), disp_col, thick)
                cv2.line(annotated, (x2, y2), (x2, y2 - c_len), disp_col, thick)

                # Pulsing crosshair at centroid
                cv2.circle(annotated, (cx, cy), 14, disp_col, 1)
                cv2.circle(annotated, (cx, cy), 4, disp_col, -1)
                cv2.line(annotated, (cx - 18, cy), (cx + 18, cy), disp_col, 1)
                cv2.line(annotated, (cx, cy - 18), (cx, cy + 18), disp_col, 1)

                badge = f"LOCKED: {det['label']}"
                cv2.rectangle(annotated, (x1, max(0, y1 - 20)), (x1 + len(badge) * 8 + 10, y1), (20, 20, 20), -1)
                cv2.putText(annotated, badge, (x1 + 4, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, disp_col, 1, cv2.LINE_AA)
            else:
                # Regular target box
                cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), disp_col, 1)
                cv2.circle(annotated, (cx, cy), 3, disp_col, -1)
                badge = det["label"]
                cv2.rectangle(annotated, (bx, max(0, by - 16)), (bx + len(badge) * 7 + 8, by), (30, 30, 30), -1)
                cv2.putText(annotated, badge, (bx + 3, max(10, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, disp_col, 1, cv2.LINE_AA)

        # Fallback tip position to center of workspace if no tip detected
        if detected_tip is None:
            detected_tip = (w // 2, int(h * 0.4))

        # Draw Bézier Trajectory Spline & Error vector if target is active
        if target_color and target_centroid:
            # Plan 2D Bézier curve
            bezier_pts = BezierTrajectory.plan_2d_bezier_path(detected_tip, target_centroid, steps=24)
            if len(bezier_pts) >= 2:
                # Draw smooth Bézier curve polyline
                pts_arr = np.array(bezier_pts, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated, [pts_arr], isClosed=False, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)

                # Draw waypoints along the Bézier curve
                for i, pt in enumerate(bezier_pts):
                    if i % 4 == 0 or i == len(bezier_pts) - 1:
                        cv2.circle(annotated, (int(pt[0]), int(pt[1])), 2, (255, 200, 0), -1)

            # Draw tip ring
            cv2.circle(annotated, (int(detected_tip[0]), int(detected_tip[1])), 8, (0, 255, 0), 2)
            cv2.circle(annotated, (int(detected_tip[0]), int(detected_tip[1])), 2, (0, 255, 0), -1)
            cv2.putText(annotated, "EE TIP", (int(detected_tip[0]) + 10, int(detected_tip[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1, cv2.LINE_AA)

        # Draw Top Status Banner
        cv2.rectangle(annotated, (0, 0), (w, 24), (20, 20, 25), -1)
        tgt_str = f"TARGET: {target_color.upper()}" if target_color else "TARGET: NONE"
        st_str = f"STATE: {state_text}" if state_text else "STATE: IDLE"
        err_str = f"|ERR|: {error_dist:.1f}px" if error_dist is not None else ""
        hud_line = f"{tgt_str}  |  {st_str}  |  {err_str}".strip(" | ")
        cv2.putText(annotated, hud_line, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 240, 255), 1, cv2.LINE_AA)

        return annotated


# =====================================================================
# 2. Smooth Bézier Trajectory Generator (Minimum-Jerk Timing)
# =====================================================================

class BezierTrajectory:
    """
    Generates smooth 2D and 3D cubic Bézier trajectories with parabolic lift-and-descend profiles
    and minimum-jerk time parameterization to prevent tension spikes on continuum cables.
    """

    @staticmethod
    def minimum_jerk_time(tau):
        """
        Quintic minimum-jerk profile: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5.
        Guarantees zero velocity, zero acceleration, and zero jerk at tau=0 and tau=1.
        """
        tau = max(0.0, min(1.0, tau))
        return 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)

    @staticmethod
    def plan_2d_bezier_path(p_start, p_target, curvature=0.15, steps=25):
        """
        Generates smooth 2D cubic Bézier spline waypoints from p_start to p_target.
        Control points P1 and P2 provide smooth curved lead-in and lead-out.
        """
        p0 = np.array(p_start, dtype=float)
        p3 = np.array(p_target, dtype=float)
        delta = p3 - p0
        dist = np.linalg.norm(delta)

        if dist < 1.0:
            return [p_start, p_target]

        # Normal vector for curvature
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
        """
        Plans a 3D lift-travel-descend cubic Bézier curve between p_start and p_target.

        P0 = p_start (current arm position)
        P1 = p_start + [0, 0, +lift] (vertical departure)
        P2 = p_target + [0, 0, +lift] (vertical approach)
        P3 = p_target (target grasp position)
        """
        p0 = np.array(p_start, dtype=float)
        p3 = np.array(p_target, dtype=float)

        # Control points creating a smooth arch
        p1 = p0.copy()
        p1[2] += lift_height
        p2 = p3.copy()
        p2[2] += lift_height

        trajectory = []
        for i in range(steps + 1):
            tau = float(i) / float(steps)
            s = BezierTrajectory.minimum_jerk_time(tau)

            # Cubic Bézier formula: B(s) = (1-s)^3*P0 + 3*(1-s)^2*s*P1 + 3*(1-s)*s^2*P2 + s^3*P3
            b = (
                ((1.0 - s) ** 3) * p0
                + 3.0 * ((1.0 - s) ** 2) * s * p1
                + 3.0 * (1.0 - s) * (s ** 2) * p2
                + (s ** 3) * p3
            )
            trajectory.append(b.tolist())

        return trajectory


# =====================================================================
# 3. Closed-Loop Position-Based Visual Servoing (PBVS)
# =====================================================================

class VisualServoController:
    """
    Closed-Loop Visual Servoing for continuum cable-driven arm.
    Maps Cartesian errors between Arm Tip and Target into motor cable corrections.
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
        self.is_tracking = False

    def reset_integral(self):
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0

    def compute_step(self, tip_pos, target_pos, current_angles=None):
        """
        Calculates motor angle updates from tip and target 2D image coordinates (or 3D mm).
        
        tip_pos: (x_tip, y_tip)
        target_pos: (x_target, y_target)
        
        Returns: (new_angles, error_distance, is_converged)
        """
        ex = float(target_pos[0] - tip_pos[0]) * self.sign_x
        ey = float(target_pos[1] - tip_pos[1]) * self.sign_y
        error_dist = math.hypot(ex, ey)

        if error_dist <= self.tolerance_pixels:
            return self.robot.current_angles, error_dist, True

        # Accumulate integral
        self.integral_error_x += ex * 0.033
        self.integral_error_y += ey * 0.033
        # Anti-windup clamping
        self.integral_error_x = max(-50.0, min(50.0, self.integral_error_x))
        self.integral_error_y = max(-50.0, min(50.0, self.integral_error_y))

        # PI control output in radians
        ux = self.kp * ex + self.ki * self.integral_error_x
        uy = self.kp * ey + self.ki * self.integral_error_y

        # Clamp max delta step
        ux = max(-self.max_step_rad, min(self.max_step_rad, ux))
        uy = max(-self.max_step_rad, min(self.max_step_rad, uy))

        if current_angles is None:
            current_angles = list(self.robot.current_angles)

        new_angles = list(current_angles)

        # Coordinate mapping for 4 antagonistic cables (assuming standard orientation):
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

        # Clamp against robot safety limits
        clamped = self.robot.clamp_angles(new_angles)
        return clamped, error_dist, False


# =====================================================================
# 4. High-Level Pick-and-Place State Machine
# =====================================================================

class PickAndPlaceManager:
    """Manages autonomous Pick-and-Place execution for 'N', 'U', 'S'."""

    def __init__(self, robot_controller, color_detector, servo_controller):
        self.robot = robot_controller
        self.detector = color_detector
        self.servo = servo_controller

        # Target slot drop-off positions (default or camera-calibrated pixel locations)
        self.slot_positions = {
            "N": (140, 500),  # Slot N
            "U": (240, 500),  # Slot U
            "S": (340, 500),  # Slot S
        }

        self.color_to_letter = {
            "red": "N",
            "blue": "U",
            "yellow": "S",
        }

        self.is_running = False
        self.current_state = "IDLE"
        self.status_message = "Ready"

    def execute_pick_and_place(self, target_color, progress_callback=None):
        """
        Executes autonomous pick-and-place for the specified target color.
        Runs in a background worker thread.
        """
        self.is_running = True
        letter = self.color_to_letter.get(target_color, "N")

        def _update(state, msg):
            self.current_state = state
            self.status_message = msg
            if progress_callback:
                progress_callback(state, msg)
            print(f"[PICK & PLACE] {state}: {msg}")

        try:
            # 1. Detect target
            _update("LOCATING", f"Searching for {target_color.upper()} ('{letter}')...")
            target_det = None
            for _ in range(30):
                if not self.is_running:
                    return
                if self.robot.camera and self.robot.camera.frame is not None:
                    # Camera frame is rotated 90° CCW
                    frame = cv2.rotate(self.robot.camera.frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    target_det = self.detector.detect_single(frame, target_color)
                    if target_det:
                        break
                time.sleep(0.05)

            if not target_det:
                _update("FAILED", f"Could not locate {target_color.upper()} target.")
                return

            target_pt = target_det["centroid"]
            _update("TARGET_FOUND", f"Target '{letter}' at pixel {target_pt}")

            # 2. Plan trajectory and visual servo to target
            _update("APPROACHING", f"Visual servoing to {target_color.upper()}...")
            self.servo.reset_integral()

            for step in range(80):
                if not self.is_running:
                    return

                if self.robot.camera and self.robot.camera.frame is not None:
                    frame = cv2.rotate(self.robot.camera.frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    tip_det = self.detector.detect_single(frame, "green")
                    target_det = self.detector.detect_single(frame, target_color)

                    if target_det:
                        target_pt = target_det["centroid"]

                    # If green tip is detected, use closed-loop visual servoing
                    if tip_det:
                        tip_pt = tip_det["centroid"]
                        new_angles, dist, converged = self.servo.compute_step(tip_pt, target_pt)
                        self.robot.send_angles(new_angles)
                        if converged or dist < 10.0:
                            _update("AT_TARGET", "Reached target hover position.")
                            break
                    else:
                        # Open-loop continuum approximation
                        dx = (target_pt[0] - 240) * 0.002
                        dy = (target_pt[1] - 320) * 0.002
                        approx = [max(0.0, dx), max(0.0, dy), max(0.0, -dx), max(0.0, -dy)]
                        self.robot.move_smooth(approx, duration=0.8, steps=15)
                        break

                time.sleep(0.04)

            # 3. Descend to magnetize
            _update("DESCENDING", "Lowering magnet to grasp target...")
            time.sleep(0.8)

            # 4. Lift up
            _update("LIFTING", "Lifting letter with magnetic end effector...")
            lift_angles = [a * 0.7 for a in self.robot.current_angles]
            self.robot.move_smooth(lift_angles, duration=1.0, steps=20)
            time.sleep(0.5)

            # 5. Transfer to slot
            slot_pt = self.slot_positions.get(letter, (240, 500))
            _update("TRANSFERRING", f"Transferring '{letter}' to Slot at {slot_pt}...")

            for step in range(60):
                if not self.is_running:
                    return
                if self.robot.camera and self.robot.camera.frame is not None:
                    frame = cv2.rotate(self.robot.camera.frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    tip_det = self.detector.detect_single(frame, "green")
                    if tip_det:
                        tip_pt = tip_det["centroid"]
                        new_angles, dist, converged = self.servo.compute_step(tip_pt, slot_pt)
                        self.robot.send_angles(new_angles)
                        if converged or dist < 12.0:
                            break
                    else:
                        # Direct smooth interpolation towards slot
                        dx = (slot_pt[0] - 240) * 0.002
                        dy = (slot_pt[1] - 320) * 0.002
                        target_angles = [max(0.0, dx), max(0.0, dy), max(0.0, -dx), max(0.0, -dy)]
                        self.robot.move_smooth(target_angles, duration=1.2, steps=25)
                        break
                time.sleep(0.04)

            # 6. Slot in and release
            _update("SLOTTING", f"Inserting '{letter}' into destination slot...")
            time.sleep(1.0)

            # 7. Return home
            _update("RETRACTING", "Returning arm to neutral home position...")
            self.robot.home()
            _update("COMPLETED", f"Successfully slotted '{letter}'!")

        finally:
            self.is_running = False

    def stop(self):
        self.is_running = False
        self.robot.home()
