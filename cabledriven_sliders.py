"""
cabledriven_sliders.py
----------------------
Simple slider GUI for the EMIO cable-driven robot, made for recording trajectories.

- One slider per motor, plus -1 / +1 deg buttons for fine moves. The motors follow the
  sliders at SPEED_DEG_PER_S, so a fast drag is still smooth.
- 0 on every slider is the pose the robot is in when the program starts: + pulls that
  motor's cable in, - lets it out (REL_MIN_DEG to MAX_ANGLE_DEG). So nothing moves at start,
  and motors reading odd absolute angles (e.g. negative) never jump.
- Nothing is sent to the motors until you move something.
- Freeze: stop where the motors are now (the sliders jump to the current angles).
- Reset: go back to 0 (the starting pose, or the last "Set zero here").
- Release motors: torque off, so the spools can be turned by hand to adjust the cables.
  Set zero here: take control again exactly where they are (no jump) and make that the
  new 0. Do this after any adjustment by hand or power cycle: the motors lose their
  multi-turn count then, so their absolute angles cannot be trusted.
- Record / Stop / Save: records what the motors do (including pauses, which become
  checkpoints for camera_feedback.py) and saves to trajectories/*.json in the same
  format as cabledriven_dpad_v2.py: "angles" are the real motor angles, and
  "start_angles_deg" is the pose the recording started from.
- Load / Replay: plays a saved trajectory relative to the pose the robot starts in now
  (a file without start_angles_deg, e.g. from the D-pad, is played as recorded).

Run:  python cabledriven_sliders.py            (finds the board automatically)
      python cabledriven_sliders.py --port COM3
"""

import json
import math
import os
import time
import tkinter as tk
from tkinter import filedialog, messagebox

from cabledriven import CableDrivenRobotController, MAX_ANGLE_DEG

SPEED_DEG_PER_S = 60.0   # how fast the motors follow the sliders
TICK_MS = 50
HOLD_S = 0.15            # a pause longer than this is kept in the recording
REL_MIN_DEG = -180.0     # how far a cable can be let out below the starting pose
TRAJ_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectories")  # always the project's folder
NAMES = ["M1", "M2", "M3", "M4"]
COLORS = ["#38bdf8", "#a855f7", "#f97316", "#10b981"]


class SliderRecorder:
    def __init__(self, root, port=None):
        self.root = root
        root.title("Cable-Driven Sliders (recording)")
        # Wide limits: the sliders keep the motion within REL_MIN_DEG..MAX_ANGLE_DEG of the start pose.
        limit = 2 * math.radians(MAX_ANGLE_DEG)
        self.robot = CableDrivenRobotController(min_angle=-limit, max_angle=limit, num_motors=4)
        self.robot.connect(port)

        # All angles below are relative to this starting pose (real motor angle = start + relative)
        self.start = [math.degrees(a) for a in self.robot.current_angles]
        self.angles = [0.0] * 4   # where the motors are (relative)
        self.targets = [0.0] * 4  # what the sliders ask for (relative)
        self.holding = True       # send nothing until the user moves something
        self.recording = False
        self.points = []          # recorded points, real motor angles
        self.rec_t0 = 0.0
        self.last_change = 0.0
        self.trajectory = []      # loaded or just recorded, relative: [(t, [deg x4])]
        self.replay = None        # dict(t0, speed) while replaying
        self.info_extra = ""
        self.released = False     # torque off, motors turned by hand

        status = "Connected" if not self.robot.is_mock else f"Simulation / mock ({self.robot.last_connection_error})"
        tk.Label(root, text=status, fg="#16a34a" if not self.robot.is_mock else "#d97706").pack(pady=(8, 2))
        tk.Label(root, text="0 = the pose when this program started.  + pulls the cable in,  - lets it out.",
                 font=("Segoe UI", 9)).pack()

        self.scales = []
        self._ignore_slide = False
        for i in range(4):
            row = tk.Frame(root)
            row.pack(fill="x", padx=10, pady=2)
            tk.Label(row, text=NAMES[i], width=4, fg=COLORS[i], font=("Segoe UI", 10, "bold")).pack(side="left")
            tk.Button(row, text="-1", width=3, command=lambda i=i: self.nudge(i, -1)).pack(side="left")
            sc = tk.Scale(row, from_=REL_MIN_DEG, to=MAX_ANGLE_DEG, resolution=1, orient="horizontal", length=420,
                          command=lambda v, i=i: self.on_slide(i, v))
            sc.pack(side="left")
            tk.Button(row, text="+1", width=3, command=lambda i=i: self.nudge(i, +1)).pack(side="left")
            self.scales.append(sc)
            self._set_scale(i, 0)

        bar = tk.Frame(root)
        bar.pack(pady=6)
        tk.Button(bar, text="Freeze", width=8, command=self.freeze).pack(side="left", padx=3)
        tk.Button(bar, text="Reset", width=8, command=self.reset).pack(side="left", padx=3)

        zero_bar = tk.Frame(root)
        zero_bar.pack(pady=(0, 4))
        self.release_btn = tk.Button(zero_bar, text="Release motors (adjust by hand)", width=28,
                                     command=self.toggle_release)
        self.release_btn.pack(side="left", padx=3)
        tk.Button(zero_bar, text="Set zero here", width=14, command=self.set_zero_here).pack(side="left", padx=3)
        tk.Button(zero_bar, text="Motor status", width=12, command=self.show_status).pack(side="left", padx=3)
        tk.Button(zero_bar, text="Clear motor errors", width=16, command=self.clear_errors).pack(side="left", padx=3)
        self.rec_btn = tk.Button(bar, text="● Record", width=10, fg="#dc2626", command=self.toggle_record)
        self.rec_btn.pack(side="left", padx=3)
        self.save_btn = tk.Button(bar, text="Save", width=6, command=self.save, state=tk.DISABLED)
        self.save_btn.pack(side="left", padx=3)
        tk.Button(bar, text="Load", width=6, command=self.load).pack(side="left", padx=3)
        self.play_btn = tk.Button(bar, text="Replay", width=7, command=self.start_replay, state=tk.DISABLED)
        self.play_btn.pack(side="left", padx=3)
        self.speed = tk.StringVar(value="1.0x")
        tk.OptionMenu(bar, self.speed, "0.5x", "1.0x", "1.5x", "2.0x").pack(side="left")

        self.info = tk.Label(root, text="", font=("Consolas", 9), justify="left")
        self.info.pack(padx=10, pady=(0, 8), anchor="w")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    # -- input --------------------------------------------------------------
    def on_slide(self, i, value):
        if self._ignore_slide or self.replay:
            return
        if self.released:  # motors are being turned by hand: ignore the sliders
            self._set_scale(i, self.targets[i])
            return
        self.targets[i] = float(value)
        self.holding = False

    def nudge(self, i, step):
        if self.replay or self.released:
            return
        self.targets[i] = min(max(self.targets[i] + step, REL_MIN_DEG), MAX_ANGLE_DEG)
        self._set_scale(i, self.targets[i])
        self.holding = False

    def _set_scale(self, i, value):
        self._ignore_slide = True
        self.scales[i].set(value)
        self._ignore_slide = False

    def toggle_release(self):
        """Torque off so the spools can be turned by hand; press again (or Set zero here) to take control."""
        if self.recording:
            return
        if not self.released:
            self.replay = None
            self.robot.release_torque()
            self.released = True
            self.holding = True
            self.release_btn.config(text="Released: press Set zero here", fg="#dc2626")
            self.info_extra = "Motors released: turn the spools by hand, then press Set zero here."
        else:
            self.set_zero_here()

    def set_zero_here(self):
        """Hold the motors where they are now (no jump) and make this pose the new 0 on all sliders."""
        if self.recording:
            return
        self.replay = None
        try:
            self.robot.hold_here()
        except Exception as e:
            messagebox.showerror("Error", f"Could not take control of the motors: {e}")
            return
        self.start = [math.degrees(a) for a in self.robot.current_angles]
        self.angles = [0.0] * 4
        self.targets = [0.0] * 4
        for i in range(4):
            self._set_scale(i, 0)
        self.released = False
        self.holding = True
        self.release_btn.config(text="Release motors (adjust by hand)", fg="black")
        self.info_extra = "New zero set here: " + ", ".join(f"{n} {a:.0f}" for n, a in zip(NAMES, self.start))

    def show_status(self):
        """Read each motor's mode, temperature, voltage, current and hardware error (read-only)."""
        lines = []
        for i, (name, status) in enumerate(self.robot.motor_status()):
            lines.append(f"{name}  (slider {self.angles[i]:+.1f} deg from zero)")
            lines += [("   !! " if err else "   ") + text for text, err in status]
        text = "\n".join(lines)
        print(text)
        messagebox.showinfo("Motor status", text)

    def clear_errors(self):
        """Reboot motors in hardware error (e.g. overload), then hold all motors where they are as the new zero."""
        if self.recording:
            return
        cleared = self.robot.clear_motor_errors()
        if not cleared:
            messagebox.showinfo("Clear motor errors", "No motor reports a hardware error.")
            return
        self.set_zero_here()  # rebooted motors lost their multi-turn count: take control where they are
        messagebox.showinfo("Clear motor errors",
                            f"Rebooted {', '.join(cleared)}. All motors now hold where they are; that pose is the new zero.")

    def reset(self):
        """Move all motors back to 0, the pose the program started in (smoothly, at the slider speed)."""
        if self.released:
            return
        self.replay = None
        self.targets = [0.0] * 4
        for i in range(4):
            self._set_scale(i, 0)
        self.holding = False

    def freeze(self):
        self.replay = None
        self.targets = list(self.angles)
        for i, a in enumerate(self.angles):
            self._set_scale(i, a)

    def _absolute(self, rel):
        return [round(s + a, 2) for s, a in zip(self.start, rel)]

    @staticmethod
    def _relative(angles, base):
        return [float(a) - b for a, b in zip(angles, base)]

    # -- recording ------------------------------------------------------------
    def toggle_record(self):
        if self.replay or self.released:
            return
        if not self.recording:
            self.recording = True
            self.rec_t0 = time.perf_counter()
            self.points = [{"t": 0.0, "angles": self._absolute(self.angles)}]
            self.last_change = 0.0
            self.rec_btn.config(text="■ Stop")
            self.save_btn.config(state=tk.DISABLED)
        else:
            self.recording = False
            t = round(time.perf_counter() - self.rec_t0, 3)
            if t > self.points[-1]["t"]:
                self.points.append({"t": t, "angles": self._absolute(self.angles)})
            self.rec_btn.config(text="● Record")
            self.save_btn.config(state=tk.NORMAL)
            self.trajectory = [(p["t"], self._relative(p["angles"], self.start)) for p in self.points]
            self.play_btn.config(state=tk.NORMAL)

    def _record_tick(self):
        t = round(time.perf_counter() - self.rec_t0, 3)
        cur = self._absolute(self.angles)
        last = self.points[-1]
        if any(abs(c - p) >= 0.05 for c, p in zip(cur, last["angles"])):
            # keep the end of a pause, so replay waits there too
            if t - self.last_change > HOLD_S and t - last["t"] > HOLD_S:
                self.points.append({"t": round(t - TICK_MS / 1000.0, 3), "angles": list(last["angles"])})
            self.points.append({"t": t, "angles": cur})
            self.last_change = t

    def save(self):
        if not self.points:
            return
        os.makedirs(TRAJ_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(initialdir=TRAJ_DIR, defaultextension=".json",
                                            initialfile=time.strftime("sliders_%Y%m%d_%H%M%S.json"),
                                            filetypes=[("JSON Trajectory", "*.json")])
        if not path:
            return
        data = {"name": os.path.basename(path), "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "duration_s": self.points[-1]["t"], "num_points": len(self.points),
                "start_angles_deg": [round(a, 2) for a in self.start], "trajectory": self.points}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self.info_extra = f"Saved {path}"
        print(f"[saved] {path}")

    # -- replay ---------------------------------------------------------------
    def load(self):
        if self.replay or self.recording:
            return
        path = filedialog.askopenfilename(initialdir=TRAJ_DIR, filetypes=[("JSON Trajectory", "*.json")])
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            traj = data["trajectory"] if isinstance(data, dict) else data
            # Relative to the recording's start pose if it has one, else to ours (= play as recorded)
            base = data.get("start_angles_deg", self.start) if isinstance(data, dict) else self.start
            self.trajectory = [(float(p["t"]), self._relative(p["angles"][:4], base)) for p in traj]
        except Exception as e:
            messagebox.showerror("Error", f"Could not load trajectory: {e}")
            return
        self.play_btn.config(state=tk.NORMAL)
        self.info_extra = f"Loaded {os.path.basename(path)} ({self.trajectory[-1][0]:.1f} s)"

    def start_replay(self):
        if not self.trajectory or self.recording or self.released:
            return
        # Ease to the first pose at the normal speed, then play
        self.targets = list(self.trajectory[0][1])
        self.holding = False
        self.replay = {"t0": None, "speed": float(self.speed.get().rstrip("x"))}

    def _replay_targets(self):
        r = self.replay
        if r["t0"] is None:  # still easing to the start pose
            if max(abs(a - b) for a, b in zip(self.angles, self.targets)) < 0.5:
                r["t0"] = time.perf_counter()
            return self.targets
        t = (time.perf_counter() - r["t0"]) * r["speed"]
        if t >= self.trajectory[-1][0]:
            self.replay = None
            return list(self.trajectory[-1][1])
        for (t0, a0), (t1, a1) in zip(self.trajectory, self.trajectory[1:]):
            if t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return [x + f * (y - x) for x, y in zip(a0, a1)]
        return list(self.trajectory[-1][1])

    # -- loop -------------------------------------------------------------------
    def tick(self):
        if not self.holding:
            step = SPEED_DEG_PER_S * TICK_MS / 1000.0
            if self.replay:
                easing = self.replay["t0"] is None
                self.targets = self._replay_targets()
                for i, a in enumerate(self.targets):
                    self._set_scale(i, a)
                if not easing:
                    step *= 10  # follow the replay closely (it was recorded at the normal speed)
            for i in range(4):
                d = self.targets[i] - self.angles[i]
                self.angles[i] += max(-step, min(step, d))
            self.robot.send_angles([math.radians(a) for a in self._absolute(self.angles)])
            if self.recording:
                self._record_tick()
        state = ("Released: turn the spools by hand, then press Set zero here" if self.released else
                 "Holding: move a slider to start" if self.holding else
                 "Replaying" if self.replay else
                 f"Recording {time.perf_counter() - self.rec_t0:5.1f} s, {len(self.points)} points" if self.recording else
                 "Ready")
        self.info.config(text="  ".join(f"{n}: {a:+6.1f}" for n, a in zip(NAMES, self.angles))
                         + " deg from start   (motor: " + "  ".join(f"{a:.0f}" for a in self._absolute(self.angles))
                         + ")\n" + state + ("\n" + self.info_extra if self.info_extra else ""))
        self.root.after(TICK_MS, self.tick)

    def close(self):
        if self.recording:
            self.toggle_record()
        self.robot.disconnect()
        self.root.destroy()


def main(port=None):
    root = tk.Tk()
    SliderRecorder(root, port)
    root.mainloop()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    main(p.parse_args().port)
