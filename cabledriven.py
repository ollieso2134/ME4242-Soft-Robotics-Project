"""
cabledriven.py
--------------
Soft Robotics Cable-Driven Robot Test Script for EMIO Lab

This script allows you to test and actuate a cable-driven continuum/soft robot
using the EMIO hardware (Dynamixel motor interface via emioapi).

Features:
- Safe homing & graceful shutdown (releases cable tension on exit / Ctrl+C)
- Smooth trajectory interpolation (avoids abrupt tension spikes on cables)
- Motion tests:
    1. Individual Cable / Motor Calibration & Tension Test
    2. Antagonistic Bending (Bending along X and Y axes)
    3. Coordinated Circular / Orbital Bending (Sinusoidal trajectory)
    4. Full automated demo routine
    5. Interactive manual control (set specific motor angles)
- Hardware auto-detect with Mock/Simulation fallback if robot is not plugged in.
"""

import math
import os
import sys
import time

# Try importing the official EMIO API
try:
    from emioapi import EmioAPI
    HAS_EMIO_API = True
except Exception:
    HAS_EMIO_API = False
    EmioAPI = None


# Software limit for motor angles. Defaulting to 360.0 deg (multi-turn rotation).
MAX_ANGLE_DEG = 360.0

# Zero-based motor indices whose rotation direction is flipped at the hardware boundary.
REVERSED_MOTORS = (1, 3)


def _apply_direction(angles):
    """Negate angles of reversed motors (self-inverse: used for both send and read)."""
    return [-a if i in REVERSED_MOTORS else a for i, a in enumerate(angles)]


class CableDrivenRobotController:
    def __init__(self, min_angle=0.0, max_angle=math.radians(MAX_ANGLE_DEG), num_motors=4):
        """
        Initialize the cable-driven robot controller.
        
        :param min_angle: Minimum safe motor angle in radians (typically 0.0, cable fully released/slack)
        :param max_angle: Maximum safe motor angle in radians (default 2*pi = 360 deg)
        :param num_motors: Number of actuation cables/motors (standard EMIO has 4)
        """
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.num_motors = num_motors
        self.current_angles = [0.0] * num_motors
        self.is_connected = False
        self.is_mock = False
        self.emio = None
        self.port = None
        self.camera = None
        self.camera_serial = None
        self.last_connection_error = None

    def connect_camera(self, camera_serial=None, track_markers=False, allow_simulation_fallback=True):
        """
        Connect to Intel RealSense D405 depth camera via emioapi,
        or fallback to SimulatedObstacleCamera if no hardware camera is plugged in.
        
        :param camera_serial: Camera serial number (str). If None, auto-selects first detected camera.
        :param track_markers: Enable automatic soft robot marker tracking.
        :param allow_simulation_fallback: Fall back to high-fidelity simulation camera if no hardware is detected.
        :return: (bool success, str message)
        """
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass

        if HAS_EMIO_API:
            try:
                from emioapi import EmioCamera
                cameras = EmioCamera.listCameras()
                target_serial = camera_serial or (cameras[0] if cameras else None)
                if target_serial:
                    cam = EmioCamera(camera_serial=target_serial, track_markers=track_markers, show=False)
                    if cam.open(target_serial):
                        self.camera = cam
                        self.camera_serial = target_serial
                        print(f"[INFO] Successfully connected to RealSense Camera: {target_serial}")
                        return True, f"Connected to RealSense ({target_serial})"
            except Exception as e:
                print(f"[WARN] Error connecting to physical RealSense camera: {e}")

        if allow_simulation_fallback:
            try:
                from visual_servoing import SimulatedObstacleCamera
                self.camera = SimulatedObstacleCamera(self)
                self.camera_serial = "SIM_D405"
                print("[INFO] No hardware camera found. Connected to Simulated Obstacle Camera (Dual-Bend Obstacles Active).")
                return True, "Connected to Simulation Camera (Obstacles Active)"
            except Exception as e:
                return False, f"Simulation camera error: {e}"

        return False, "No RealSense camera detected. Is it plugged in?"

    def disconnect_camera(self):
        """Disconnect and release the RealSense camera."""
        if self.camera is not None:
            try:
                self.camera.close()
                print("[INFO] Camera disconnected.")
            except Exception as e:
                print(f"[WARN] Error closing camera: {e}")
            self.camera = None
            self.camera_serial = None

    def connect(self, port=None):
        """Establish connection to EMIO hardware or fallback to mock mode."""
        print("\n==========================================")
        print(" Connecting to EMIO Cable-Driven Robot...")
        print("==========================================")

        if self.emio is not None:
            print("[INFO] Reconnecting: closing existing connection first...")
            try:
                self.disconnect()
            except Exception:
                pass

        if HAS_EMIO_API:
            try:
                target_port = port
                if not target_port:
                    devices = EmioAPI.listEmioDevices()
                    if devices:
                        target_port = devices[0]
                    else:
                        raise RuntimeError("no EMIO motor board found (is it plugged in?)")

                # Check read/write permission only on POSIX/Linux devices (not Windows COM ports)
                if os.name != 'nt' and isinstance(target_port, str) and target_port.startswith('/'):
                    if not os.access(target_port, os.R_OK | os.W_OK):
                        raise PermissionError(
                            f"no read/write access to {target_port}. "
                            "Add your user to the 'dialout' group and log in again."
                        )

                # Only the motors are needed here, so skip the camera.
                emio = EmioAPI()
                if not emio.motors.open(target_port):
                    raise RuntimeError(f"could not open the EMIO motors on port {target_port}")
                self.emio = emio
                self.port = target_port

                # Enable Extended Position Mode (Mode 4) on all motors for multi-turn (> 180 deg) rotation
                try:
                    mg = emio.motors._mg
                    params = mg.parameters
                    for dxl_id in params.DXL_IDs:
                        mg.packetHandler.write1ByteTxRx(mg.portHandler, dxl_id, params.ADDR_TORQUE_ENABLE, 0)
                    mg.setOperatingMode(4)
                    mg.enableTorque()
                    print("[INFO] Extended Position Mode (Mode 4) enabled: motors can rotate past 180° up to 360°.")
                except Exception as e:
                    print(f"[WARN] Could not enable Extended Position Mode: {e}")

                # Allow bus to settle after torque enable, then read initial positions with retry.
                read_ok = False
                for attempt in range(3):
                    try:
                        time.sleep(0.05)
                        pulses = emio.motors._mg.getCurrentPosition()
                        if pulses:
                            signed_pulses = [(p - 0x100000000 if p >= 0x80000000 else p) for p in pulses]
                            self.current_angles = _apply_direction(
                                [float(emio.motors.pulseToRad([p])[0]) for p in signed_pulses]
                            )
                            read_ok = True
                            break
                    except Exception as ex:
                        if attempt < 2:
                            time.sleep(0.05)
                        else:
                            print(f"[WARN] Could not read initial motor pulses ({ex}), defaulting to neutral 0.0 rad.")
                if not read_ok:
                    self.current_angles = [0.0] * self.num_motors

                self.is_connected = True
                self.is_mock = False
                self.last_connection_error = None
                print(f"[INFO] Successfully connected to EMIO hardware on {emio.motors.device_name} ({target_port})!")
                deg_str = ", ".join(f"M{i+1}:{math.degrees(a):6.1f}°" for i, a in enumerate(self.current_angles))
                print(f"[INFO] Current motor angles: {deg_str}")
                print("[INFO] Motors were NOT moved. Use 'Home' (menu 5) to send them to 0.")
            except Exception as e:
                self.last_connection_error = str(e)
                print(f"[WARN] Could not connect to physical EMIO device ({e}).")
                print("[INFO] Falling back to SIMULATION / MOCK mode for dry testing.")
                self.is_connected = True
                self.is_mock = True
        else:
            self.last_connection_error = "emioapi is not installed"
            print("[WARN] 'emioapi' is not installed in current Python environment.")
            print("[INFO] Running in SIMULATION / MOCK mode.")
            print("[TIP]  To connect to real hardware, install emioapi: pip install emioapi")
            print("[TIP]  If it is installed, the import failed: check that its dependencies load.")
            self.is_connected = True
            self.is_mock = True

        # Only home automatically in simulation. On real hardware, 0 rad is the
        # motor centre position, which is not necessarily a slack cable, so the
        # user must ask for it explicitly.
        if self.is_mock:
            self.home()

    def clamp_angles(self, angles):
        """Clamps angle commands to safe operating range [min_angle, max_angle]."""
        clamped = []
        for a in angles:
            val = max(self.min_angle, min(self.max_angle, a))
            clamped.append(val)
        return clamped

    def send_angles(self, angles, clamp=True):
        """Send target angles to motors."""
        safe_angles = self.clamp_angles(angles) if clamp else list(angles)
        self.current_angles = list(safe_angles)

        if not self.is_mock and self.emio is not None:
            try:
                self.emio.motors.angles = _apply_direction(safe_angles)
            except Exception as e:
                print(f"[ERROR] Failed to send angles to EMIO: {e}")
        else:
            # Print compact mock representation
            deg_str = ", ".join([f"M{i+1}:{math.degrees(a):5.1f}°" for i, a in enumerate(safe_angles)])
            print(f" [SIM MOTOR OUTPUT] -> {deg_str}", end="\r", flush=True)

    def move_smooth(self, target_angles, duration=1.5, steps=30):
        """
        Smoothly interpolates motor angles from current positions to target_angles
        using minimum-jerk / cosine easing to protect soft cables and structures.
        """
        start_angles = list(self.current_angles)
        target_angles = self.clamp_angles(target_angles)
        dt = duration / float(steps)

        for s in range(1, steps + 1):
            # Cosine S-curve interpolation (0.0 to 1.0)
            alpha = (1.0 - math.cos(math.pi * (s / float(steps)))) / 2.0
            interpolated = [
                start_angles[i] + alpha * (target_angles[i] - start_angles[i])
                for i in range(self.num_motors)
            ]
            self.send_angles(interpolated, clamp=False)
            time.sleep(dt)

        self.current_angles = list(target_angles)
        if self.is_mock:
            print()  # newline after step prints

    def home(self):
        """Returns all motors to the neutral / fully relaxed home position (0 radians)."""
        print("\n[ROBOT] Homing: Releasing tension on all cables...")
        self.move_smooth([0.0] * self.num_motors, duration=1.0)
        print("[ROBOT] Robot is in neutral home position.")

    # -------------------------------------------------------------
    # Motion Test Routines
    # -------------------------------------------------------------

    def test_single_cable_actuation(self, pull_angle=0.9):
        """
        Movement 1: Sequentially pulls and releases each cable one by one.
        Helps verify motor numbering, cable routing, and tension mapping.
        """
        print("\n>>> TEST 1: Sequential Single-Cable Pull & Release <<<")
        self.home()
        time.sleep(0.5)

        for i in range(self.num_motors):
            print(f"\n -> Pulling Cable {i+1} (Motor {i+1}) to {math.degrees(pull_angle):.1f}°...")
            target = [0.0] * self.num_motors
            target[i] = pull_angle
            self.move_smooth(target, duration=1.2)
            time.sleep(0.8)

            print(f" -> Releasing Cable {i+1} back to neutral...")
            self.move_smooth([0.0] * self.num_motors, duration=1.0)
            time.sleep(0.4)

        print("\n[OK] Single cable test complete.")

    def test_antagonistic_bending(self, pull_angle=0.5):
        """
        Movement 2: Antagonistic pair bending.
        Assumes 4-cable setup spaced at 90-degree intervals:
        - Pair X: Motor 1 vs Motor 3 (+X and -X bending)
        - Pair Y: Motor 2 vs Motor 4 (+Y and -Y bending)
        """
        print("\n>>> TEST 2: Antagonistic Bending (4 Quadrants) <<<")
        self.home()
        time.sleep(0.5)

        directions = [
            ("+X Bending", [pull_angle, 0.0, 0.0, 0.0]),
            ("-X Bending", [0.0, 0.0, pull_angle, 0.0]),
            ("+Y Bending", [0.0, pull_angle, 0.0, 0.0]),
            ("-Y Bending", [0.0, 0.0, 0.0, pull_angle]),
        ]

        for name, pose in directions:
            print(f"\n -> Bending towards: {name}")
            self.move_smooth(pose, duration=1.5)
            time.sleep(1.0)
            print(" -> Returning to neutral...")
            self.move_smooth([0.0] * self.num_motors, duration=1.0)
            time.sleep(0.3)

        print("\n[OK] Antagonistic bending test complete.")

    def test_circular_orbit(self, max_pull=0.45, cycles=2, duration_per_cycle=4.0):
        """
        Movement 3: Coordinated circular/orbital motion (continuously rotating bending angle).
        Actuates adjacent cable pairs sinusoidally to produce a continuous 360-degree sweep.
        """
        print(f"\n>>> TEST 3: Coordinated Circular/Orbital Sweep ({cycles} cycles) <<<")
        self.home()
        time.sleep(0.5)

        steps_per_cycle = 40
        total_steps = cycles * steps_per_cycle
        dt = duration_per_cycle / float(steps_per_cycle)

        print(" -> Sweeping continuously...")
        for s in range(total_steps + 1):
            theta = 2.0 * math.pi * (s / float(steps_per_cycle))
            # Direction vector in polar coordinates
            # Motor 1: 0 rad (X+), Motor 2: pi/2 (Y+), Motor 3: pi (X-), Motor 4: 3pi/2 (Y-)
            motor_phases = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]

            angles = []
            for phase in motor_phases:
                # Cable pulls when projected component along its direction is positive
                val = max(0.0, math.cos(theta - phase)) * max_pull
                angles.append(val)

            self.send_angles(angles)
            time.sleep(dt)

        if self.is_mock:
            print()

        print(" -> Returning to home position...")
        self.move_smooth([0.0] * self.num_motors, duration=1.5)
        print("\n[OK] Circular sweep test complete.")

    def run_all_tests(self):
        """Runs the complete sequence of tests."""
        print("\n==========================================")
        print(" STARTING FULL CABLE-DRIVEN DEMO ROUTINE")
        print("==========================================")
        self.test_single_cable_actuation(pull_angle=0.8)
        time.sleep(1.0)
        self.test_antagonistic_bending(pull_angle=0.45)
        time.sleep(1.0)
        self.test_circular_orbit(max_pull=0.4, cycles=2)
        time.sleep(0.5)
        self.home()
        print("\n==========================================")
        print(" ALL TESTS COMPLETED SUCCESSFULLY!")
        print("==========================================")

    def disconnect(self):
        """Safely release tension and close connection."""
        if self.is_mock:
            print("\n[ROBOT] Safely relaxing tension before disconnecting...")
            try:
                self.home()
            except Exception:
                pass

        if not self.is_mock and self.emio is not None:
            try:
                print("\n[ROBOT] Disabling motor torque and closing the port (motors are not moved).")
                self.emio.motors.close()
                print("[ROBOT] Disconnected from EMIO.")
            except Exception as e:
                print(f"[WARN] Error while disconnecting: {e}")
            self.emio = None
        self.disconnect_camera()
        self.is_connected = False


def check_hardware(port=None):
    """
    Read-only hardware check: reports the port, access rights and motor state.
    It only pings and reads registers, it never writes to the motors.
    """
    print("\n=== EMIO hardware check (read-only) ===")
    if not HAS_EMIO_API:
        print("[FAIL] 'emioapi' could not be imported.")
        return

    devices = EmioAPI.listEmioDevices()
    target_port = port or (devices[0] if devices else None)
    if not target_port:
        print("[FAIL] No EMIO motor board found. Is it plugged in?")
        return
    print(f"[OK]   Motor board found on port: {target_port}")

    if os.name != 'nt' and str(target_port).startswith('/'):
        if not os.access(target_port, os.R_OK | os.W_OK):
            print(f"[FAIL] No read/write access to {target_port}. Add your user to 'dialout' and log in again.")
            return
        print(f"[OK]   Read/write access to {target_port}")
    else:
        print(f"[OK]   Port access ready on {target_port}")

    from dynamixel_sdk import PortHandler, PacketHandler
    from emioapi import _emiomotorsparameters as params

    ph = PortHandler(target_port)
    pk = PacketHandler(params.PROTOCOL_VERSION)
    if not ph.openPort():
        print(f"[FAIL] Could not open {target_port} (is another program using it?).")
        return
    try:
        ph.setBaudRate(params.BAUDRATE)
        # Register map of the XM430-W210 (model 1030), as used by emioapi.
        addr_torque, addr_present = 64, 132
        for motor_id in params.DXL_IDs:
            model, comm, _ = pk.ping(ph, motor_id)
            if comm != 0:
                print(f"[FAIL] Motor ID {motor_id}: no response ({pk.getTxRxResult(comm)})")
                continue
            line = f"[OK]   Motor ID {motor_id}: model {model}"
            if model == 1030:
                torque, _, _ = pk.read1ByteTxRx(ph, motor_id, addr_torque)
                pulse, _, _ = pk.read4ByteTxRx(ph, motor_id, addr_present)
                deg = (2048 - pulse) * 360.0 / 4096.0
                line += f" | torque={'on' if torque else 'off'} | position={pulse} pulses = {deg:6.1f}° (0° = pulse 2048)"
            else:
                line += " | unexpected model, position not read"
            print(line)
    finally:
        ph.closePort()
    print("=== Nothing was moved. ===")


def print_menu():
    print("\n----------------- EMIO Cable-Driven Menu -----------------")
    print(" 1. Run Full Automated Demo (All Movements)")
    print(" 2. Test 1: Single Cable Actuation (Motors 1 to 4)")
    print(" 3. Test 2: Antagonistic Bending (+X, -X, +Y, -Y)")
    print(" 4. Test 3: Continuous Circular Orbit / Sweep")
    print(" 5. Home / Relax All Cables (0.0 rad)")
    print(" 6. Manual Angle Input (Set 4 angles in degrees)")
    print(" 7. Launch Graphical User Interface (GUI)")
    print(" 8. Launch RealSense Camera Viewer & Tracker")
    print(" 9. Exit")
    print("---------------------------------------------------------")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="EMIO Cable-Driven Robot Controller")
    parser.add_argument("--port", type=str, default=None, help="Serial port to connect (e.g. COM12 or /dev/ttyUSB0)")
    parser.add_argument("--check", action="store_true", help="Run read-only hardware check")
    parser.add_argument("--gui", "-g", action="store_true", help="Launch Graphical User Interface (GUI)")
    parser.add_argument("--camera", "-c", action="store_true", help="Launch RealSense Camera Viewer")
    args = parser.parse_args()

    if args.check:
        check_hardware(port=args.port)
        return

    if args.camera:
        try:
            import camera_viewer
            camera_viewer.main()
            return
        except Exception as e:
            print(f"[ERROR] Failed to start Camera Viewer: {e}")
            return

    if args.gui:
        try:
            import cabledriven_gui
            cabledriven_gui.main(port=args.port)
            return
        except Exception as e:
            print(f"[ERROR] Failed to start GUI: {e}")
            return

    robot = CableDrivenRobotController(min_angle=0.0, max_angle=math.radians(MAX_ANGLE_DEG), num_motors=4)

    try:
        robot.connect(port=args.port)

        while True:
            print_menu()
            choice = input("Enter choice (1-9) [or 'q' to quit]: ").strip()

            if choice in ["1", "demo"]:
                robot.run_all_tests()
            elif choice == "2":
                robot.test_single_cable_actuation()
            elif choice == "3":
                robot.test_antagonistic_bending()
            elif choice == "4":
                robot.test_circular_orbit()
            elif choice == "5":
                robot.home()
            elif choice == "6":
                print("\nEnter 4 angles in degrees (e.g. '30 0 0 0' or '25 25 0 0'):")
                inp = input(f"Angles (deg, 0 to {MAX_ANGLE_DEG:g}): ").strip()
                try:
                    degs = [float(x) for x in inp.replace(",", " ").split()]
                    if len(degs) != 4:
                        print("[ERROR] Please provide exactly 4 values separated by space.")
                        continue
                    rads = [math.radians(d) for d in degs]
                    print(f"Moving to: {degs}°...")
                    robot.move_smooth(rads, duration=1.5)
                except ValueError:
                    print("[ERROR] Invalid numeric input.")
            elif choice in ["7", "gui"]:
                robot.disconnect()
                import cabledriven_gui
                cabledriven_gui.main(port=args.port)
                break
            elif choice in ["8", "cam", "camera"]:
                import camera_viewer
                camera_viewer.main()
            elif choice in ["9", "q", "exit", "quit"]:
                print("Exiting...")
                break
            else:
                print("[WARN] Invalid option, please try again.")

    except KeyboardInterrupt:
        print("\n\n[INFO] Keyboard interrupt detected (Ctrl+C). Stopping immediately.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()

