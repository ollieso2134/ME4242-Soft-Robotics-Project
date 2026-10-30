"""
home_zero.py
------------
Turn all 4 motors to their absolute 0 deg (pulse 2048), smoothly, after confirmation.
Use it with the cables / mounting removed, so the motors can turn freely.

    .\\emio.bat home_zero.py            (finds the board automatically)
    .\\emio.bat home_zero.py --port COM9 --speed 20 --yes

1. Motors in hardware error (e.g. overload) are rebooted first, or they would not move.
2. The motors are connected without moving (cabledriven.py holds them where they are).
3. The current angles and how far each motor will turn are shown; type y to go.
4. All motors move together at --speed deg/s and arrive at the same time.
5. They hold 0 until you press Enter; then the torque is turned off and the program ends.
"""

import argparse
import math
import time

from cabledriven import CableDrivenRobotController, MAX_ANGLE_DEG


def main():
    p = argparse.ArgumentParser(description="Turn all motors to absolute 0 deg.")
    p.add_argument("--port", default=None)
    p.add_argument("--speed", type=float, default=30.0, help="deg/s of the motor that moves furthest (default 30)")
    p.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    args = p.parse_args()

    limit = 4 * math.radians(MAX_ANGLE_DEG)  # wide: after hand turning, a motor can read past one turn
    robot = CableDrivenRobotController(min_angle=-limit, max_angle=limit, num_motors=4)
    robot.connect(args.port)
    if robot.is_mock:
        print("\n[STOP] The robot is not connected (mock mode), nothing to do.")
        if robot.last_connection_error:
            print(f"       Reason: {robot.last_connection_error}")
        return

    cleared = robot.clear_motor_errors()
    if cleared:
        print(f"[INFO] {', '.join(cleared)} were in hardware error and have been rebooted.")
    angles = [math.degrees(a) for a in robot.hold_here()]  # take control exactly where they are now

    print("\n Motor   now (deg)   will turn (deg)")
    for i, a in enumerate(angles):
        print(f"   M{i + 1}   {a:9.1f}   {-a:+9.1f}")
    furthest = max(abs(a) for a in angles)
    duration = max(1.0, furthest / args.speed)
    if furthest < 0.5:
        print("\nAll motors are already at 0.")
    else:
        print(f"\nAll motors will turn together for {duration:.1f} s.")
        if not args.yes and input("Cables / mounting removed and safe to turn? Type y to go: ").strip().lower() != "y":
            print("Cancelled: nothing moved. The motors hold where they are until the program ends.")
            robot.disconnect()
            return
        try:
            robot.move_smooth([0.0] * 4, duration=duration, steps=max(2, int(duration / 0.02)))
        except KeyboardInterrupt:
            print("\n[STOP] Interrupted: the motors hold where they are.")
        time.sleep(0.5)
        pulses = robot.emio.motors._mg.getCurrentPosition()
        signed = [(q - 0x100000000 if q >= 0x80000000 else q) for q in pulses]
        now = [math.degrees(robot.emio.motors.pulseToRad([q])[0]) for q in signed]
        print("\nReached: " + "  ".join(f"M{i + 1} {a:+.1f}" for i, a in enumerate(now)) + " deg")
        far = [f"M{i + 1}" for i, a in enumerate(now) if abs(a) > 2.0]
        if far:
            print(f"[WARN] {', '.join(far)} did not reach 0: blocked, or in error again (run: cabledriven.py --check)")

    input("\nHolding 0. Press Enter to turn the torque off and exit... ")
    robot.disconnect()


if __name__ == "__main__":
    main()
