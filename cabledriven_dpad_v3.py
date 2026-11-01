"""
cabledriven_dpad_v3.py
----------------------
D-pad + drag pad + sliders for the EMIO cable-driven robot, with antagonistic control:
when one cable is pulled in, the opposite one is let out, so the motors do not fight
each other (that fight is what overloaded M2 / M4).

Motors (same layout as v2, seen from above, Front up):
    M2 (Front-Left)    M1 (Front-Right)
    M3 (Back-Left)     M4 (Back-Right)
Pairs on opposite sides: M1 <-> M3 and M2 <-> M4.

Controls, all in "bend" degrees (how far the pulling motor turns):
- Drag the pink rod tip on the pad: towards a motor pulls that motor's cable.
- Sliders "M1 <-> M3" and "M2 <-> M4": + pulls M1 / M2, - pulls M3 / M4.
- D-pad buttons (hold to repeat) or keys W A S D: step the bend; centre = Straight.
- Let-out ratio: how much the opposite cable is let out per degree pulled
  (0.9 = 90 %, keeps it slightly taut; 0 = hold the opposite, like v2 co-tension).
- Stiffness (co-tension): extra pull on all 4 cables, to hold a bend against a bar.
  Keep it small (0-30 deg).
Each motor stays within REL_MIN_DEG .. PULL_MAX_DEG of zero.

Zero:
- 0 is the pose when the program starts (nothing moves at start).
- Pretension: pulls all 4 cables in by the given amount, then makes that the new zero.
  Do it once with the rod straight, so the cables are taut but the rod is not bent.
- Release motors / Set zero here / Motor status / Clear motor errors: as in cabledriven_sliders.py.

Record / Save / Load / Replay: same file format as cabledriven_sliders.py (motor angles +
"delta", the change from start_angles_deg), so recordings from either play in both and in
camera_feedback.py. Replay plays the changes from the current zero.

Run:  emio.bat cabledriven_dpad_v3.py            (finds the board automatically)
      emio.bat cabledriven_dpad_v3.py --port COM9
"""

import json
import math
import os
import time
import tkinter as tk
from tkinter import filedialog, messagebox

from cabledriven import CableDrivenRobotController, MAX_ANGLE_DEG, trajectory_changes

SPEED_DEG_PER_S = 60.0   # how fast the motors follow the controls
TICK_MS = 50
HOLD_S = 0.15            # a pause longer than this is kept in the recording
PULL_MAX_DEG = 150.0     # max pull of one motor from zero (more only winds against a fully bent rod)
REL_MIN_DEG = -180.0     # max let-out of one motor below zero
STIFF_MAX_DEG = 60.0
LET_OUT = 0.9
PRETENSION_DEG = 30.0
STEP_DEG = 5.0           # D-pad step
TRAJ_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectories")
NAMES = ["M1", "M2", "M3", "M4"]
COLORS = ["#38bdf8", "#a855f7", "#f97316", "#10b981"]

PAD = 300                # drag pad size (px)
R = 110                  # pad radius = PULL_MAX_DEG of bend
S = math.sqrt(0.5)
# Unit vector from the centre towards each motor on the pad (canvas y is down, Front is up)
DIRS = [(S, -S), (-S, -S), (-S, S), (S, S)]
# D-pad: name -> (row, col, (step on M1<->M3 axis, step on M2<->M4 axis))
BUTTONS = {
    "M2": (0, 0, (0, 1)), "Front": (0, 1, (1, 1)), "M1": (0, 2, (1, 0)),
    "Left": (1, 0, (-1, 1)), "Right": (1, 2, (1, -1)),
    "M3": (2, 0, (-1, 0)), "Back": (2, 1, (-1, -1)), "M4": (2, 2, (0, -1)),
}
KEYS = {"w": "Front", "s": "Back", "a": "Left", "d": "Right"}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DPadV3:
    def __init__(self, root, port=None):
        self.root = root
        root.title("Cable-Driven D-Pad v3 (antagonistic)")
        # Wide limits: this GUI keeps each motor within REL_MIN_DEG..PULL_MAX_DEG of zero.
        limit = 2 * math.radians(MAX_ANGLE_DEG)
        self.robot = CableDrivenRobotController(min_angle=-limit, max_angle=limit, num_motors=4)
        self.robot.connect(port)

        # Motor angles are relative to this pose (real motor angle = start + relative)
        self.start = [math.degrees(a) for a in self.robot.current_angles]
        self.angles = [0.0] * 4   # where the motors are (relative)
        self.bend = [0.0, 0.0]    # commanded bend on the M1<->M3 and M2<->M4 axes
        self.base = [0.0] * 4     # pose the bend is added to (0, or where a replay ended)
        self.override = None      # direct relative motor targets (replay, pretension)
        self.on_reached = None    # called once the override is reached
        self.holding = True       # send nothing until the user moves something
        self.released = False
        self.recording = False
        self.points = []
        self.rec_t0 = 0.0
        self.last_change = 0.0
        self.trajectory = []
        self.replay = None
        self.info_extra = ""
        self._ignore = False

        status = "Connected" if not self.robot.is_mock else f"Simulation / mock ({self.robot.last_connection_error})"
        tk.Label(root, text=status, fg="#16a34a" if not self.robot.is_mock else "#d97706").pack(pady=(8, 2))

        top = tk.Frame(root)
        top.pack(padx=10)
        self.canvas = tk.Canvas(top, width=PAD, height=PAD, bg="#111217", highlightthickness=0)
        self.canvas.pack(side="left", padx=(0, 12))
        self.canvas.bind("<Button-1>", self.on_drag)
        self.canvas.bind("<B1-Motion>", self.on_drag)

        right = tk.Frame(top)
        right.pack(side="left", fill="y")
        grid = tk.Frame(right)
        grid.pack(pady=(4, 8))
        for name, (r, c, _) in BUTTONS.items():
            color = COLORS[NAMES.index(name)] if name in NAMES else "black"
            tk.Button(grid, text=name, width=7, height=2, fg=color, repeatdelay=300, repeatinterval=80,
                      command=lambda n=name: self.step(n)).grid(row=r, column=c, padx=1, pady=1)
        tk.Button(grid, text="Straight", width=7, height=2, command=self.straight).grid(row=1, column=1)

        self.axis_scales = []
        for k, label in enumerate(["M3  ⟵  ⟶  M1", "M4  ⟵  ⟶  M2"]):
            tk.Label(right, text=label, font=("Segoe UI", 9)).pack()
            sc = tk.Scale(right, from_=-PULL_MAX_DEG, to=PULL_MAX_DEG, resolution=1, orient="horizontal",
                          length=260, command=lambda v, k=k: self.on_axis(k, v))
            sc.pack()
            self.axis_scales.append(sc)

        opts = tk.Frame(right)
        opts.pack(pady=(6, 0), fill="x")
        tk.Label(opts, text="Stiffness (co-tension) deg").grid(row=0, column=0, sticky="w")
        self.stiff = tk.Scale(opts, from_=0, to=STIFF_MAX_DEG, resolution=1, orient="horizontal", length=130,
                              command=lambda v: self.on_option())
        self.stiff.grid(row=0, column=1)
        tk.Label(opts, text="Let-out ratio").grid(row=1, column=0, sticky="w")
        self.let_out = tk.Scale(opts, from_=0.0, to=1.0, resolution=0.05, orient="horizontal", length=130,
                                command=lambda v: self.on_option())
        self.let_out.set(LET_OUT)
        self.let_out.grid(row=1, column=1)
        tk.Label(opts, text="Step / pretension deg").grid(row=2, column=0, sticky="w")
        ent = tk.Frame(opts)
        ent.grid(row=2, column=1, sticky="w")
        self.step_var = tk.StringVar(value=f"{STEP_DEG:g}")
        tk.Entry(ent, textvariable=self.step_var, width=5).pack(side="left", padx=2)
        self.pre_var = tk.StringVar(value=f"{PRETENSION_DEG:g}")
        tk.Entry(ent, textvariable=self.pre_var, width=5).pack(side="left", padx=2)

        zero_bar = tk.Frame(root)
        zero_bar.pack(pady=(8, 2))
        tk.Button(zero_bar, text="Pretension + set zero", command=self.pretension).pack(side="left", padx=3)
        self.release_btn = tk.Button(zero_bar, text="Release motors (adjust by hand)", width=28,
                                     command=self.toggle_release)
        self.release_btn.pack(side="left", padx=3)
        tk.Button(zero_bar, text="Set zero here", command=self.set_zero_here).pack(side="left", padx=3)
        tk.Button(zero_bar, text="Motor status", command=self.show_status).pack(side="left", padx=3)
        tk.Button(zero_bar, text="Clear motor errors", command=self.clear_errors).pack(side="left", padx=3)

        bar = tk.Frame(root)
        bar.pack(pady=4)
        self.rec_btn = tk.Button(bar, text="● Record", width=10, fg="#dc2626", command=self.toggle_record)
        self.rec_btn.pack(side="left", padx=3)
        self.save_btn = tk.Button(bar, text="Save", width=6, command=self.save, state=tk.DISABLED)
        self.save_btn.pack(side="left", padx=3)
        tk.Button(bar, text="Load", width=6, command=self.load).pack(side="left", padx=3)
        self.play_btn = tk.Button(bar, text="Replay", width=7, command=self.start_replay, state=tk.DISABLED)
        self.play_btn.pack(side="left", padx=3)
        tk.Button(bar, text="Stop", width=6, command=self.stop).pack(side="left", padx=3)
        self.speed = tk.StringVar(value="1.0x")
        tk.OptionMenu(bar, self.speed, "0.5x", "1.0x", "1.5x", "2.0x").pack(side="left")

        self.info = tk.Label(root, text="", font=("Consolas", 9), justify="left")
        self.info.pack(padx=10, pady=(0, 8), anchor="w")
        root.bind("<KeyPress>", self.on_key)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    # -- bend -> motors -------------------------------------------------------
    def bend_targets(self):
        """Relative motor targets for the commanded bend: pull one side, let the opposite out."""
        r = float(self.let_out.get())
        s = float(self.stiff.get())
        t = [0.0] * 4
        for k, b in enumerate(self.bend):
            p, q = k, k + 2  # M1/M3, M2/M4
            t[p] = s + max(b, 0.0) - r * max(-b, 0.0)
            t[q] = s + max(-b, 0.0) - r * max(b, 0.0)
        return [clamp(base + v, REL_MIN_DEG, PULL_MAX_DEG) for base, v in zip(self.base, t)]

    def _bend_from_angles(self):
        """Bend shown on the pad for the motors' actual angles (the inverse of bend_targets)."""
        r = float(self.let_out.get())
        return [((self.angles[k] - self.base[k]) - (self.angles[k + 2] - self.base[k + 2])) / (1.0 + r)
                for k in range(2)]

    # -- input ------------------------------------------------------------------
    def _can_move(self):
        return not (self.released or self.replay or self.override)

    def _set_bend(self, a, b):
        # Keep the bend inside the pad circle (each motor then stays within PULL_MAX_DEG)
        n = math.hypot(a, b)
        if n > PULL_MAX_DEG:
            a, b = a * PULL_MAX_DEG / n, b * PULL_MAX_DEG / n
        self.bend = [a, b]
        self._ignore = True
        for sc, v in zip(self.axis_scales, self.bend):
            sc.set(round(v))
        self._ignore = False
        self.holding = False

    def on_drag(self, ev):
        if not self._can_move():
            return
        vx, vy = (ev.x - PAD / 2) * PULL_MAX_DEG / R, (ev.y - PAD / 2) * PULL_MAX_DEG / R
        # Project onto the M1 and M2 directions (M3 / M4 are the negatives)
        self._set_bend(vx * DIRS[0][0] + vy * DIRS[0][1], vx * DIRS[1][0] + vy * DIRS[1][1])

    def on_axis(self, k, value):
        if self._ignore:
            return
        if not self._can_move():
            self.axis_scales[k].set(round(self.bend[k]))
            return
        b = list(self.bend)
        b[k] = float(value)
        self._set_bend(*b)

    def on_option(self):
        # Stiffness / let-out take effect through bend_targets on the next tick
        if self._can_move():
            self.holding = False

    def step(self, name):
        if not self._can_move():
            return
        try:
            d = abs(float(self.step_var.get()))
        except ValueError:
            d = STEP_DEG
        sa, sb = BUTTONS[name][2]
        self._set_bend(self.bend[0] + sa * d, self.bend[1] + sb * d)

    def on_key(self, ev):
        if isinstance(ev.widget, tk.Entry):
            return
        name = KEYS.get(ev.keysym.lower())
        if name:
            self.step(name)

    def straight(self):
        """Back to the zero pose (no bend)."""
        if self.released:
            return
        self.replay = None
        self.override = None
        self.base = [0.0] * 4
        self._set_bend(0.0, 0.0)

    def stop(self):
        """Stop a replay / pretension and hold where the motors are now."""
        self.replay = None
        self.override = None
        self.on_reached = None
        self.base = list(self.angles)
        if float(self.stiff.get()) > 0:
            self.stiff.set(0)  # base already contains it
        self._set_bend(0.0, 0.0)

    # -- zero -------------------------------------------------------------------
    def pretension(self):
        """Pull all 4 cables in by the pretension amount (from where they are now), then set zero there."""
        if self.released or self.recording or self.replay:
            return
        try:
            amount = clamp(float(self.pre_var.get()), 0.0, 90.0)
        except ValueError:
            amount = PRETENSION_DEG
        if not messagebox.askyesno("Pretension",
                                   f"Pull all 4 cables in by {amount:g} deg from where they are now, "
                                   "then make that the new zero?\n\nThe rod should be straight first."):
            return
        self.override = [a + amount for a in self.angles]
        self.on_reached = self.set_zero_here
        self.holding = False

    def toggle_release(self):
        if self.recording:
            return
        if not self.released:
            self.replay = None
            self.override = None
            self.robot.release_torque()
            self.released = True
            self.holding = True
            self.release_btn.config(text="Released: press Set zero here", fg="#dc2626")
            self.info_extra = "Motors released: turn the spools by hand, then press Set zero here."
        else:
            self.set_zero_here()

    def set_zero_here(self):
        """Hold the motors where they are now (no jump) and make this pose the new zero."""
        if self.recording:
            return
        self.replay = None
        self.override = None
        self.on_reached = None
        try:
            self.robot.hold_here()
        except Exception as e:
            messagebox.showerror("Error", f"Could not take control of the motors: {e}")
            return
        self.start = [math.degrees(a) for a in self.robot.current_angles]
        self.angles = [0.0] * 4
        self.base = [0.0] * 4
        self.stiff.set(0)
        self._set_bend(0.0, 0.0)
        self.released = False
        self.holding = True
        self.release_btn.config(text="Release motors (adjust by hand)", fg="black")
        self.info_extra = "New zero set here: " + ", ".join(f"{n} {a:.0f}" for n, a in zip(NAMES, self.start))

    def show_status(self):
        lines = []
        for i, (name, status) in enumerate(self.robot.motor_status()):
            lines.append(f"{name}  ({self.angles[i]:+.1f} deg from zero)")
            lines += [("   !! " if err else "   ") + text for text, err in status]
        text = "\n".join(lines)
        print(text)
        messagebox.showinfo("Motor status", text)

    def clear_errors(self):
        if self.recording:
            return
        cleared = self.robot.clear_motor_errors()
        if not cleared:
            messagebox.showinfo("Clear motor errors", "No motor reports a hardware error.")
            return
        self.set_zero_here()  # rebooted motors lost their multi-turn count: take control where they are
        messagebox.showinfo("Clear motor errors",
                            f"Rebooted {', '.join(cleared)}. All motors now hold where they are; that pose is the new zero.")

    def _absolute(self, rel):
        return [round(s + a, 2) for s, a in zip(self.start, rel)]

    def _point(self, t, rel):
        return {"t": t, "angles": self._absolute(rel), "delta": [round(a, 2) for a in rel]}

    # -- recording --------------------------------------------------------------
    def toggle_record(self):
        if self.replay or self.released:
            return
        if not self.recording:
            self.recording = True
            self.rec_t0 = time.perf_counter()
            self.points = [self._point(0.0, self.angles)]
            self.last_change = 0.0
            self.rec_btn.config(text="■ Stop rec")
            self.save_btn.config(state=tk.DISABLED)
        else:
            self.recording = False
            t = round(time.perf_counter() - self.rec_t0, 3)
            if t > self.points[-1]["t"]:
                self.points.append(self._point(t, self.angles))
            self.rec_btn.config(text="● Record")
            self.save_btn.config(state=tk.NORMAL)
            self.trajectory = trajectory_changes(self.points)
            self.play_btn.config(state=tk.NORMAL)

    def _record_tick(self):
        t = round(time.perf_counter() - self.rec_t0, 3)
        last = self.points[-1]
        if any(abs(c - p) >= 0.05 for c, p in zip(self.angles, last["delta"])):
            # keep the end of a pause, so replay waits there too
            if t - self.last_change > HOLD_S and t - last["t"] > HOLD_S:
                self.points.append(dict(last, t=round(t - TICK_MS / 1000.0, 3)))
            self.points.append(self._point(t, self.angles))
            self.last_change = t

    def save(self):
        if not self.points:
            return
        os.makedirs(TRAJ_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(initialdir=TRAJ_DIR, defaultextension=".json",
                                            initialfile=time.strftime("dpad3_%Y%m%d_%H%M%S.json"),
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

    # -- replay -----------------------------------------------------------------
    def load(self):
        if self.replay or self.recording:
            return
        path = filedialog.askopenfilename(initialdir=TRAJ_DIR, filetypes=[("JSON Trajectory", "*.json")])
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            # Changes from the recording's start pose, played from our zero (a D-pad file: as recorded)
            self.trajectory = trajectory_changes(data, self.start)
        except Exception as e:
            messagebox.showerror("Error", f"Could not load trajectory: {e}")
            return
        self.play_btn.config(state=tk.NORMAL)
        self.info_extra = f"Loaded {os.path.basename(path)} ({self.trajectory[-1][0]:.1f} s)"

    def start_replay(self):
        if not self.trajectory or self.recording or self.released:
            return
        self.override = None
        self.holding = False
        self.replay = {"t0": None, "speed": float(self.speed.get().rstrip("x"))}

    def _replay_targets(self):
        """Replay targets for this tick, and whether the motors are still easing to the start pose."""
        r = self.replay
        if r["t0"] is None:
            first = self.trajectory[0][1]
            if max(abs(a - b) for a, b in zip(self.angles, first)) < 0.5:
                r["t0"] = time.perf_counter()
            return list(first), True
        t = (time.perf_counter() - r["t0"]) * r["speed"]
        if t >= self.trajectory[-1][0]:
            end = list(self.trajectory[-1][1])
            self.stop()
            self.info_extra = "Replay finished: the controls now start from this pose (Straight = back to zero)."
            return end, False
        for (t0, a0), (t1, a1) in zip(self.trajectory, self.trajectory[1:]):
            if t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return [x + f * (y - x) for x, y in zip(a0, a1)], False
        return list(self.trajectory[-1][1]), False

    # -- loop -------------------------------------------------------------------
    def tick(self):
        if not self.holding:
            step = SPEED_DEG_PER_S * TICK_MS / 1000.0
            if self.replay:
                targets, easing = self._replay_targets()
                if not easing:
                    step *= 10  # follow the replay closely (it was recorded at the normal speed)
            elif self.override:
                targets = self.override
            else:
                targets = self.bend_targets()
            for i in range(4):
                self.angles[i] += clamp(targets[i] - self.angles[i], -step, step)
            self.robot.send_angles([math.radians(a) for a in self._absolute(self.angles)])
            if self.recording:
                self._record_tick()
            if self.override and max(abs(a - b) for a, b in zip(self.angles, self.override)) < 0.5:
                done, self.on_reached = self.on_reached, None
                self.override = None
                if done:
                    done()
        self.draw()
        state = ("Released: turn the spools by hand, then press Set zero here" if self.released else
                 "Holding: drag the rod, use a slider or the D-pad to start" if self.holding else
                 "Replaying" if self.replay else
                 "Pretensioning..." if self.override else
                 f"Recording {time.perf_counter() - self.rec_t0:5.1f} s, {len(self.points)} points" if self.recording else
                 "Ready")
        self.info.config(text="  ".join(f"{n}: {a:+6.1f}" for n, a in zip(NAMES, self.angles))
                         + " deg from zero   (motor: " + "  ".join(f"{a:.0f}" for a in self._absolute(self.angles))
                         + ")\n" + state + ("\n" + self.info_extra if self.info_extra else ""))
        self.root.after(TICK_MS, self.tick)

    def _pad_xy(self, a, b):
        x = a * DIRS[0][0] + b * DIRS[1][0]
        y = a * DIRS[0][1] + b * DIRS[1][1]
        return PAD / 2 + x * R / PULL_MAX_DEG, PAD / 2 + y * R / PULL_MAX_DEG

    def draw(self):
        c = self.canvas
        c.delete("all")
        cx = cy = PAD / 2
        c.create_oval(cx - R, cy - R, cx + R, cy + R, outline="#333", width=2)
        c.create_oval(cx - R / 2, cy - R / 2, cx + R / 2, cy + R / 2, outline="#222", dash=(2, 2))
        c.create_text(cx, 10, text="Front", fill="#888", font=("Segoe UI", 8))
        for i, ((dx, dy), name, color) in enumerate(zip(DIRS, NAMES, COLORS)):
            ax, ay = cx + dx * (R + 18), cy + dy * (R + 18)
            rel = self.angles[i]
            # cable: thick = pulled in, dashed grey = let out
            if rel > 0.5:
                c.create_line(ax, ay, cx, cy, fill=color, width=1 + 5 * min(rel, PULL_MAX_DEG) / PULL_MAX_DEG)
            else:
                c.create_line(ax, ay, cx, cy, fill="#444", dash=(3, 3))
            c.create_oval(ax - 9, ay - 9, ax + 9, ay + 9, fill=color, outline="")
            c.create_text(ax, ay + (20 if dy > 0 else -20), text=f"{name} {rel:+.0f}", fill=color,
                          font=("Segoe UI", 8, "bold"))
        tx, ty = self._pad_xy(*self.bend)  # where the controls ask the tip to go
        c.create_oval(tx - 9, ty - 9, tx + 9, ty + 9, outline="#f72585", dash=(2, 2))
        ax, ay = self._pad_xy(*self._bend_from_angles())  # where the motors actually are
        c.create_line(cx, cy, ax, ay, fill="#f72585", width=4)
        c.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#555", outline="")
        c.create_oval(ax - 7, ay - 7, ax + 7, ay + 7, fill="#f72585", outline="white")

    def close(self):
        if self.recording:
            self.toggle_record()
        self.robot.disconnect()
        self.root.destroy()


def main(port=None):
    root = tk.Tk()
    DPadV3(root, port)
    root.mainloop()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    main(p.parse_args().port)
