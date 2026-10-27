"""
cabledriven_dpad.py
-------------------
Toggle D-pad (Front / Back / Left / Right / Relax) for the EMIO cable-driven robot,
rotated 45 deg relative to the motors.

Motors: M1 (+X), M2 (+Y), M3 (-X), M4 (-Y).
    Front = M1+M2    Left = M2+M3    Back = M3+M4    Right = M4+M1

Each active button contracts its two motors (+angle) and relaxes the other two
(0 deg), where angle is set in the input field (default 90 deg, max 180) and takes
effect on Apply / Enter. Contributions of active buttons are summed and clamped to
0..angle, so two adjacent buttons give a diagonal: Front+Left -> M2 +angle,
M1/M3/M4 at 0.
Pressing a button toggles it and disengages its opposite (Front<->Back, Left<->Right).
Angles ramp linearly toward the target.

A second, unrotated D-pad (same layout as cabledriven_gui.py) drives one motor per button:
    +Y = M2    -X = M3    +X = M1    -Y = M4
It toggles the same way (the opposite button disengages, the opposite motor is relaxed to 0 deg).
Pressing a button on one pad disengages every active button on the other pad.
"""

import json
import math
import os
import time
import tkinter as tk
from tkinter import filedialog, messagebox

from cabledriven import CableDrivenRobotController, MAX_ANGLE_DEG

SPEED_DEG_PER_S = 90.0
TICK_MS = 50
INCREMENT_DEG = 10.0
CANVAS_SIZE = 200
R_OUTER = 80
R_INNER = 15
D = R_OUTER * math.sqrt(0.5)

# Motor anchors rotated 45 deg: (dx, dy, label, color); canvas y is down, so Front is up.
ANCHORS = [
    (D, -D, "M1 (Front-Right)", "#38bdf8"),
    (-D, -D, "M2 (Front-Left)", "#a855f7"),
    (-D, D, "M3 (Back-Left)", "#f97316"),
    (D, D, "M4 (Back-Right)", "#10b981"),
]

# name -> (grid row, grid col, contracted motor indices, opposite button, pad)
BUTTONS = {
    "Front": (0, 1, (0, 1), "Back", 0),
    "Left": (1, 0, (1, 2), "Right", 0),
    "Right": (1, 2, (3, 0), "Left", 0),
    "Back": (2, 1, (2, 3), "Front", 0),
    "+Y": (0, 1, (1,), "-Y", 1),
    "-X": (1, 0, (2,), "+X", 1),
    "+X": (1, 2, (0,), "-X", 1),
    "-Y": (2, 1, (3,), "+Y", 1),
}


class ToggleDPad:
    def __init__(self, root, port=None):
        self.root = root
        root.title("Cable-Driven Toggle D-Pad")

        limit = math.radians(MAX_ANGLE_DEG)
        self.robot = CableDrivenRobotController(min_angle=0.0, max_angle=limit, num_motors=4)
        self.robot.connect(port)
        self.angles = [0.0] * 4
        self.amplitude = 90.0
        self.active = set()
        self.inc_targets = [0.0] * 4
        self.buttons = {}
        self.trajectory = []
        self.replay_state = None  # None, or dict(phase, t0)

        self.status = tk.Label(root, text="")
        self.status.pack(pady=(10, 0))
        self.update_status()

        self.canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="#111217", highlightthickness=0)
        self.canvas.pack(pady=5)

        amp = tk.Frame(root)
        amp.pack()
        tk.Label(amp, text=f"Angle (deg, max {MAX_ANGLE_DEG:g}):").pack(side=tk.LEFT)
        self.amp_var = tk.StringVar(value=f"{self.amplitude:g}")
        entry = tk.Entry(amp, textvariable=self.amp_var, width=6)
        entry.pack(side=tk.LEFT, padx=4)
        entry.bind("<Return>", lambda e: self.apply_amplitude())
        tk.Button(amp, text="Apply", command=self.apply_amplitude).pack(side=tk.LEFT)
        self.incremental_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            amp, text=f"Incremental ({INCREMENT_DEG:g}°)", variable=self.incremental_var, command=self.clear
        ).pack(side=tk.LEFT, padx=(8, 0))

        pads = tk.Frame(root)
        pads.pack(padx=10, pady=10)
        grids = [tk.Frame(pads), tk.Frame(pads)]
        for g in grids:
            g.pack(side=tk.LEFT, padx=10)
            tk.Button(g, text="Relax", width=11, height=3, command=self.clear).grid(row=1, column=1)
        for name, (r, c, _, _, pad) in BUTTONS.items():
            b = tk.Button(grids[pad], text=name, width=11, height=3, command=lambda n=name: self.press(n))
            b.grid(row=r, column=c)
            self.buttons[name] = b

        rep = tk.Frame(root)
        rep.pack()
        tk.Button(rep, text="Load", command=self.load_trajectory).pack(side=tk.LEFT, padx=2)
        self.replay_btn = tk.Button(rep, text="Replay", command=self.start_replay, state=tk.DISABLED)
        self.replay_btn.pack(side=tk.LEFT, padx=2)
        self.abort_btn = tk.Button(rep, text="Abort", command=self.abort_replay, state=tk.DISABLED)
        self.abort_btn.pack(side=tk.LEFT, padx=2)
        tk.Label(rep, text="Speed:").pack(side=tk.LEFT, padx=(8, 2))
        self.speed_var = tk.StringVar(value="1.0x")
        tk.OptionMenu(rep, self.speed_var, "0.5x", "1.0x", "1.5x", "2.0x").pack(side=tk.LEFT)
        self.loop_var = tk.BooleanVar(value=False)
        tk.Checkbutton(rep, text="Loop", variable=self.loop_var).pack(side=tk.LEFT)
        self.backtrack_var = tk.BooleanVar(value=False)
        tk.Checkbutton(rep, text="Backtrack", variable=self.backtrack_var).pack(side=tk.LEFT)
        self.play_traj = []
        self.traj_label = tk.Label(root, text="No trajectory loaded")
        self.traj_label.pack()

        self.readout = tk.Label(root, text="")
        self.readout.pack(pady=(0, 10))

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    def update_status(self):
        if not self.robot.is_connected:
            self.status.config(text="● Disconnected", fg="#ef4444")
        elif self.robot.is_mock:
            self.status.config(text="● Simulation / Mock Mode", fg="#fbbf24")
        else:
            port = getattr(self.robot, "port", None) or ""
            self.status.config(text=f"● Connected {port}".strip(), fg="#22c55e")

    def draw(self):
        c = self.canvas
        c.delete("all")
        cx = cy = CANVAS_SIZE // 2
        c.create_oval(cx - R_OUTER, cy - R_OUTER, cx + R_OUTER, cy + R_OUTER, outline="#333", width=2)
        c.create_oval(cx - R_OUTER // 2, cy - R_OUTER // 2, cx + R_OUTER // 2, cy + R_OUTER // 2, outline="#222", dash=(2, 2))
        c.create_text(cx, cy - R_OUTER - 8, text="Front", fill="#888", font=("TkDefaultFont", 7))

        net_x = net_y = 0.0
        for (dx, dy, label, color), deg in zip(ANCHORS, self.angles):
            ax, ay = cx + dx, cy + dy
            norm = deg / MAX_ANGLE_DEG  # 0 (relaxed) .. +1 (contracted)
            c.create_oval(ax - 7, ay - 7, ax + 7, ay + 7, fill=color, outline="")
            c.create_text(ax, ay + (14 if dy > 0 else -14), text=label, fill=color, font=("TkDefaultFont", 7))
            if norm > 0.01:
                c.create_line(cx, cy, cx + dx * norm, cy + dy * norm, fill=color, width=int(1 + norm * 4))
            net_x += dx / R_OUTER * norm
            net_y += dy / R_OUTER * norm

        tip_x = cx + net_x * R_OUTER * 0.5
        tip_y = cy + net_y * R_OUTER * 0.5
        c.create_line(cx, cy, tip_x, tip_y, fill="#f72585", width=3)
        c.create_oval(tip_x - 6, tip_y - 6, tip_x + 6, tip_y + 6, fill="#f72585", outline="white")
        c.create_oval(cx - R_INNER, cy - R_INNER, cx + R_INNER, cy + R_INNER, outline="#555", width=1.5)

    def load_trajectory(self):
        if self.replay_state:
            return
        os.makedirs("trajectories", exist_ok=True)
        path = filedialog.askopenfilename(
            initialdir="trajectories",
            filetypes=[("JSON Trajectory", "*.json"), ("All Files", "*.*")],
        )
        if not path:
            return
        try:
            with open(path, "r") as f:
                data = json.load(f)
            traj = data["trajectory"] if isinstance(data, dict) else data
            if not traj:
                raise ValueError("empty trajectory")
            self.trajectory = [(float(p["t"]), [float(a) for a in p["angles"][:4]]) for p in traj]
        except Exception as e:
            messagebox.showerror("Error", f"Could not load trajectory: {e}")
            return
        self.traj_label.config(
            text=f"Loaded: {os.path.basename(path)} ({len(self.trajectory)} pts, {self.trajectory[-1][0]:.1f}s)"
        )
        self.replay_btn.config(state=tk.NORMAL)

    def start_replay(self):
        if not self.trajectory or self.replay_state:
            return
        self.active.clear()
        self.refresh()
        self.play_traj = list(self.trajectory)
        if self.backtrack_var.get():
            total = self.play_traj[-1][0]
            self.play_traj += [(2 * total - t, list(a)) for t, a in reversed(self.trajectory[:-1])]
        self.replay_state = {"phase": "lead", "t0": 0.0}
        self.replay_btn.config(state=tk.DISABLED)
        self.abort_btn.config(state=tk.NORMAL)

    def abort_replay(self):
        self.end_replay("Replay aborted")

    def end_replay(self, msg):
        if not self.replay_state:
            return
        self.replay_state = None
        self.replay_btn.config(state=tk.NORMAL)
        self.abort_btn.config(state=tk.DISABLED)
        self.traj_label.config(text=msg)

    def replay_angles(self):
        """Replay-mode angles for this tick (None = fall back to the normal ramp)."""
        st = self.replay_state
        traj = self.play_traj
        if st["phase"] == "lead":
            if max(abs(a - b) for a, b in zip(self.angles, traj[0][1])) > 1.0:
                return None, traj[0][1]
            st["phase"], st["t0"] = "play", time.perf_counter()
        speed = float(self.speed_var.get().rstrip("x"))
        t = (time.perf_counter() - st["t0"]) * speed
        total = traj[-1][0]
        if t >= total:
            if self.loop_var.get():
                st["phase"] = "lead"
                return None, traj[0][1]
            self.end_replay(f"Replay complete ({len(traj)} pts)")
            return traj[-1][1], None
        # linear interpolation between the two surrounding waypoints
        for (t0, a0), (t1, a1) in zip(traj, traj[1:]):
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                self.traj_label.config(text=f"Replaying ({speed:g}x): {t:.1f}s / {total:.1f}s")
                return [x + f * (y - x) for x, y in zip(a0, a1)], None
        return traj[-1][1], None

    def press(self, name):
        if self.replay_state:
            return
        if self.incremental_var.get():
            for m in BUTTONS[name][2]:
                self.inc_targets[m] += INCREMENT_DEG
                self.inc_targets[(m + 2) % 4] -= INCREMENT_DEG
            self.inc_targets = [max(0.0, min(self.amplitude, v)) for v in self.inc_targets]
            return
        if name in self.active:
            self.active.discard(name)
        else:
            self.active = {n for n in self.active if BUTTONS[n][4] == BUTTONS[name][4]}
            self.active.discard(BUTTONS[name][3])
            self.active.add(name)
        self.refresh()

    def apply_amplitude(self):
        try:
            value = float(self.amp_var.get())
        except ValueError:
            value = self.amplitude
        self.amplitude = max(0.0, min(MAX_ANGLE_DEG, value))
        self.amp_var.set(f"{self.amplitude:g}")

    def clear(self):
        self.active.clear()
        self.inc_targets = [0.0] * 4
        self.refresh()

    def refresh(self):
        for name, b in self.buttons.items():
            b.config(relief=tk.SUNKEN if name in self.active else tk.RAISED)

    def targets_deg(self):
        if self.incremental_var.get():
            return [min(v, self.amplitude) for v in self.inc_targets]
        t = [0.0] * 4
        for name in self.active:
            motors = BUTTONS[name][2]
            for m in motors:
                t[m] += self.amplitude
                t[(m + 2) % 4] -= self.amplitude
        return [max(0.0, min(self.amplitude, v)) for v in t]

    def tick(self):
        max_step = SPEED_DEG_PER_S * TICK_MS / 1000.0
        targets = self.targets_deg()
        if self.replay_state:
            direct, lead_target = self.replay_angles()
            if direct is not None:
                self.angles = list(direct)
                targets = None
            elif lead_target is not None:
                targets = lead_target
        if targets is not None:
            for i in range(4):
                diff = targets[i] - self.angles[i]
                self.angles[i] += max(-max_step, min(max_step, diff))
        self.robot.send_angles([math.radians(a) for a in self.angles])
        self.readout.config(text="  ".join(f"M{i + 1}: {a:6.1f}°" for i, a in enumerate(self.angles)))
        self.draw()
        self.root.after(TICK_MS, self.tick)

    def close(self):
        self.robot.disconnect()
        self.root.destroy()


def main(port=None):
    root = tk.Tk()
    ToggleDPad(root, port)
    root.mainloop()


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    main(p.parse_args().port)
