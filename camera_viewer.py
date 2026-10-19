"""
camera_viewer.py
----------------
Live viewer and marker tracking utility for EMIO RealSense D405 Depth Camera.

Usage:
  python camera_viewer.py
  python camera_viewer.py --track
  python camera_viewer.py --depth
"""

import argparse
import sys
import time
import numpy as np

try:
    import cv2
except ImportError:
    print("[ERROR] OpenCV (cv2) is required. Install with: pip install opencv-python")
    sys.exit(1)

try:
    from emioapi import EmioCamera
    import pyrealsense2 as rs
    HAS_EMIO_CAMERA = True
except ImportError as e:
    HAS_EMIO_CAMERA = False
    print(f"[ERROR] Could not import emioapi or pyrealsense2: {e}")
    sys.exit(1)


from visual_servoing import ColorDetector

def apply_rotation(img, mode):
    if img is None:
        return None
    if mode == "90_ccw":
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif mode == "180":
        return cv2.rotate(img, cv2.ROTATE_180)
    elif mode == "90_cw":
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return img


def main():
    parser = argparse.ArgumentParser(description="EMIO Camera Viewer & Multi-Color Bézier Visual Servoing HUD")
    parser.add_argument("--serial", type=str, default=None, help="Camera serial number")
    parser.add_argument("--track", action="store_true", help="Enable soft robot marker tracking")
    parser.add_argument("--depth", action="store_true", help="Display depth feed alongside RGB")
    parser.add_argument(
        "--target",
        type=str,
        default=None,
        choices=["red", "blue", "yellow", None],
        help="Initial target color (red/blue/yellow)",
    )
    parser.add_argument(
        "--rotate",
        type=str,
        default="90_ccw",
        choices=["90_ccw", "180", "90_cw", "0"],
        help="Image rotation (default: 90_ccw for 90 degrees counter-clockwise)",
    )
    args = parser.parse_args()

    cameras = EmioCamera.listCameras()
    if not cameras:
        print("[FAIL] No Intel RealSense camera detected. Is it plugged into a USB 3.0 port?")
        return

    serial = args.serial or cameras[0]
    print(f"[INFO] Connecting to RealSense camera: {serial}")

    cam = EmioCamera(
        camera_serial=serial,
        track_markers=args.track,
        show=False,
    )

    if not cam.open(serial):
        print(f"[FAIL] Could not open camera {serial}.")
        return

    detector = ColorDetector(min_contour_area=200)
    target_color = args.target

    print("[OK]   Camera stream started!")
    print("       Orientation: 90° CCW (Counter-Clockwise)")
    print("       ----------------------------------------------------")
    print("       [1] / [n] : Target Red (Letter 'N')")
    print("       [2] / [u] : Target Blue (Letter 'U')")
    print("       [3] / [s] : Target Yellow (Letter 'S')")
    print("       [c]       : Clear target lock")
    print("       [r]       : Cycle rotation (90° CCW -> 180° -> 90° CW -> None)")
    print("       [t]       : Toggle marker tracking")
    print("       [d]       : Toggle depth view")
    print("       [space]   : Save snapshot")
    print("       [q] / ESC : Quit")
    print("       ----------------------------------------------------")

    tracking_enabled = args.track
    depth_view = args.depth
    rotation_mode = args.rotate
    rotations = ["90_ccw", "180", "90_cw", "0"]

    try:
        while True:
            cam.update()
            frame = cam.frame
            if frame is None:
                time.sleep(0.01)
                continue

            # Apply 90° CCW rotation
            display_frame = apply_rotation(frame.copy(), rotation_mode)

            # Detect Red, Blue, Yellow, and Green markers
            detections = detector.detect_all_targets(display_frame)

            # Calculate error if target and tip are present
            err_dist = None
            tip_pos = None
            if "green" in detections:
                tip_pos = detections["green"]["centroid"]
            elif cam.trackers_pos:
                # Use first EMIO tracker if available
                tip_pos = (display_frame.shape[1] // 2, int(display_frame.shape[0] * 0.45))

            if target_color and target_color in detections and tip_pos:
                tx, ty = detections[target_color]["centroid"]
                import math
                err_dist = math.hypot(tx - tip_pos[0], ty - tip_pos[1])

            state_msg = "TRACKING LOCKED" if (target_color and err_dist is not None and err_dist < 10) else ("ACQUIRING" if target_color else "SEARCHING")

            # Render HUD overlay
            annotated = detector.draw_hud(
                display_frame,
                detections=detections,
                target_color=target_color,
                tip_pos=tip_pos,
                error_dist=err_dist,
                state_text=state_msg,
            )

            if depth_view and cam.depth_frame is not None:
                depth_rot = apply_rotation(cam.depth_frame, rotation_mode)
                depth_norm = cv2.normalize(depth_rot, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                display = np.hstack((annotated, depth_color))
            else:
                display = annotated

            cv2.imshow("EMIO RealSense - Red/Blue/Yellow Bézier Visual Servoing", display)
            key = cv2.waitKey(1) & 0xFF

            if key in [ord('q'), 27]:  # q or ESC
                break
            elif key in [ord('1'), ord('n'), ord('N')]:
                target_color = "red"
                print("[TARGET] Selected RED (Letter 'N')")
            elif key in [ord('2'), ord('u'), ord('U')]:
                target_color = "blue"
                print("[TARGET] Selected BLUE (Letter 'U')")
            elif key in [ord('3'), ord('s'), ord('S')]:
                target_color = "yellow"
                print("[TARGET] Selected YELLOW (Letter 'S')")
            elif key in [ord('c'), ord('C')]:
                target_color = None
                print("[TARGET] Cleared target selection.")
            elif key == ord('r'):
                idx = (rotations.index(rotation_mode) + 1) % len(rotations)
                rotation_mode = rotations[idx]
                print(f"[INFO] Camera rotation changed to: {rotation_mode}")
            elif key == ord('t'):
                tracking_enabled = not tracking_enabled
                cam.track_markers = tracking_enabled
                print(f"[INFO] Marker tracking: {'ON' if tracking_enabled else 'OFF'}")
            elif key == ord('d'):
                depth_view = not depth_view
                print(f"[INFO] Depth view: {'ON' if depth_view else 'OFF'}")
            elif key in [ord(' '), ord('p')]:
                filename = f"snapshot_{int(time.time())}.png"
                cv2.imwrite(filename, display_frame)
                print(f"[INFO] Snapshot saved to {filename}")

    finally:
        cam.close()
        cv2.destroyAllWindows()
        print("[INFO] Camera closed cleanly.")


if __name__ == "__main__":
    main()
