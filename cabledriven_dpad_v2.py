"""
cabledriven_dpad_v2.py
----------------------
Toggle D-pad (Front / Back / Left / Right / Relax) for the EMIO cable-driven robot,
rotated 45 deg relative to the motors.

Motors: M1 (+X), M2 (+Y), M3 (-X), M4 (-Y).
    Front = M1+M2    Left = M2+M3    Back = M3+M4    Right = M4+M1

NUS ME4242 Project 2 (Obstacle-Assisted Multi-Curved Magnetic Retrieval):
------------------------------------------------------------------------
- Axis A: M1 (Top-Left) <-> M3 (Bottom-Right)
- Axis B: M2 (Top-Right) <-> M4 (Bottom-Left)
To achieve an S-curve using rigid pegboard obstacles, Bend 1 is established against Peg 1
and holding tension must be MAINTAINED on Bend 1 motors while pulling opposing motors to
curl Bend 2 around Peg 2.

Features in v2:
1. Hardware Port Selector & Reconnect:
   - Dynamic port scanning with COM port dropdown and Connect/Disconnect toggle.
   - Shows live connection status (Green = Connected, Amber = Simulation with error reason, Red = Disconnected).
2. "Obstacle Co-Tension (Hold S-Bend)" mode (default: Enabled):
   - Opposing buttons and cross-pad buttons remain active simultaneously.
   - Opposing motor tension is preserved (no antagonistic relaxation subtraction).
   - Motor angles clamped independently to [0.0, MAX_ANGLE_DEG].
   - When unchecked, returns to free-air antagonistic relaxation mode.
3. Live Trajectory Recording:
   - "🔴 Record" / "⏹ Stop Rec" live jog recording sampled at 50 ms tick resolution
     with a ~100 ms minimum time delta to capture dynamic ramps and static anchor holds.
   - "💾 Save" trajectory to JSON standard format with metadata.
4. Autonomous Replay:
   - Interpolated waypoint execution with speed selection (0.5x, 1.0x, 1.5x, 2.0x),
     looping, and backtrack (reverse trajectory for obstacle egress).
"""

import json
import math
import os
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

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
    # Pad 0: Directional Tip Bending (+ Shape)
    "Front": (0, 1, (0, 1), "Back", 0),
    "Left": (1, 0, (1, 2), "Right", 0),
    "Right": (1, 2, (3, 0), "Left", 0),
    "Back": (2, 1, (2, 3), "Front", 0),

    # Pad 1: Individual Motors (X Shape matching physical corner positions)
    # M2 is Top-Left (row 0, col 0)
    "M2 (+Y)": (0, 0, (1,), "M4 (-Y)", 1),
    # M1 is Top-Right (row 0, col 2)
    "M1 (+X)": (0, 2, (0,), "M3 (-X)", 1),
    # M3 is Bottom-Left (row 2, col 0)
    "M3 (-X)": (2, 0, (2,), "M1 (+X)", 1),
    # M4 is Bottom-Right (row 2, col 2)
    "M4 (-Y)": (2, 2, (3,), "M2 (+Y)", 1),
}


class ToggleDPad:
    def __init__(self, root, port=None):
        self.root = root
        root.title("Cable-Driven Toggle D-Pad v2 (ME4242 Co-Tension & Trajectory Tool)")

        limit = math.radians(MAX_ANGLE_DEG)
        self.robot = CableDrivenRobotController(min_angle=0.0, max_angle=limit, num_motors=4)

        # Port scanning
        self.available_ports = self.scan_ports()
        default_port = port or (self.available_ports[0] if self.available_ports else None)

        self.angles = [0.0] * 4
        self.amplitude = 90.0
        self.active = set()
        self.inc_targets = [0.0] * 4
        self.buttons = {}
        self.trajectory = []
        self.play_traj = []
        self.replay_state = None  # None, or dict(phase, t0)

        # Recording state
        self.recording = False
        self.record_t0 = 0.0
        self.recorded_points = []
        self.last_record_time = 0.0
        self.last_recorded_angles = None

        # Connection Bar (Port selection & Connect/Disconnect)
        conn_frame = tk.Frame(root)
        conn_frame.pack(pady=(8, 2), padx=10, fill=tk.X)

        tk.Label(conn_frame, text="Port:").pack(side=tk.LEFT, padx=(4, 2))
        self.port_var = tk.StringVar(value=default_port or "")
        self.port_menu = ttk.Combobox(conn_frame, textvariable=self.port_var, values=self.available_ports, width=10)
        self.port_menu.pack(side=tk.LEFT, padx=2)

        tk.Button(conn_frame, text="🔄", width=3, command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        self.conn_btn = tk.Button(
            conn_frame, text="Connect", command=self.toggle_connection, bg="#0284c7", fg="white", width=9
        )
        self.conn_btn.pack(side=tk.LEFT, padx=4)

        self.status = tk.Label(conn_frame, text="", font=("TkDefaultFont", 9, "bold"))
        self.status.pack(side=tk.LEFT, padx=(8, 4))

        # Perform initial connection attempt
        self.connect_hardware(port=default_port)

        self.canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="#111217", highlightthickness=0)
        self.canvas.pack(pady=5)

        # Amplitude and Kinematic Mode Controls
        amp = tk.Frame(root)
        amp.pack(pady=4)
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

        # Obstacle Co-Tension (Hold S-Bend) toggle checkbutton
        self.co_tension_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            amp,
            text="Obstacle Co-Tension (Hold S-Bend)",
            variable=self.co_tension_var,
            font=("TkDefaultFont", 9, "bold"),
            fg="#0284c7",
        ).pack(side=tk.LEFT, padx=(12, 0))

        # D-pad grids: Pad 0 (Directional +) and Pad 1 (Motors X)
        pads = tk.Frame(root)
        pads.pack(padx=10, pady=10)

        pad_frames = [tk.Frame(pads), tk.Frame(pads)]
        pad_titles = ["Directional Tip (+)", "Individual Motors (X)"]
        grids = []
        for pf, title in zip(pad_frames, pad_titles):
            pf.pack(side=tk.LEFT, padx=14)
            tk.Label(pf, text=title, font=("TkDefaultFont", 8, "bold"), fg="#64748b").pack(pady=(0, 4))
            g = tk.Frame(pf)
            g.pack()
            grids.append(g)
            tk.Button(g, text="Relax", width=11, height=3, command=self.clear).grid(row=1, column=1)

        motor_colors = {
            "M1 (+X)": "#0284c7",
            "M2 (+Y)": "#9333ea",
            "M3 (-X)": "#ea580c",
            "M4 (-Y)": "#16a34a",
        }
        for name, (r, c, _, _, pad) in BUTTONS.items():
            fg_color = motor_colors.get(name, "black")
            b = tk.Button(
                grids[pad],
                text=name,
                width=11,
                height=3,
                fg=fg_color,
                font=("TkDefaultFont", 8, "bold" if name in motor_colors else "normal"),
                command=lambda n=name: self.press(n),
            )
            b.grid(row=r, column=c)
            self.buttons[name] = b

        # Trajectory Replay & Recording Bar
        rep = tk.Frame(root)
        rep.pack(pady=(6, 2))
        self.load_btn = tk.Button(rep, text="Load", command=self.load_trajectory)
        self.load_btn.pack(side=tk.LEFT, padx=2)
        self.replay_btn = tk.Button(rep, text="Replay", command=self.start_replay, state=tk.DISABLED)
        self.replay_btn.pack(side=tk.LEFT, padx=2)
        self.abort_btn = tk.Button(rep, text="Abort", command=self.abort_replay, state=tk.DISABLED)
        self.abort_btn.pack(side=tk.LEFT, padx=2)
        tk.Label(rep, text="Speed:").pack(side=tk.LEFT, padx=(8, 2))
        self.speed_var = tk.StringVar(value="1.0x")
        tk.OptionMenu(rep, self.speed_var, "0.5x", "1.0x", "1.5x", "2.0x").pack(side=tk.LEFT)
        self.loop_var = tk.BooleanVar(value=False)
        tk.Checkbutton(rep, text="Loop", variable=self.loop_var).pack(side=tk.LEFT, padx=2)
        self.backtrack_var = tk.BooleanVar(value=False)
        tk.Checkbutton(rep, text="Backtrack", variable=self.backtrack_var).pack(side=tk.LEFT, padx=2)

        tk.Label(rep, text=" │ ", fg="#94a3b8").pack(side=tk.LEFT, padx=4)

        # Recording buttons
        self.record_btn = tk.Button(rep, text="🔴 Record", command=self.toggle_record, width=10)
        self.record_btn.pack(side=tk.LEFT, padx=2)
        self.save_btn = tk.Button(rep, text="💾 Save", command=self.save_trajectory, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=2)

        self.traj_label = tk.Label(root, text="No trajectory loaded", fg="#64748b")
        self.traj_label.pack(pady=2)

        self.readout = tk.Label(root, text="", font=("Consolas", 10))
        self.readout.pack(pady=(2, 10))

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    def scan_ports(self):
        ports = []
        try:
            from emioapi import EmioAPI
            devs = EmioAPI.listEmioDevices()
            if devs:
                for d in devs:
                    if d not in ports:
                        ports.append(d)
        except Exception:
            pass
        try:
            import serial.tools.list_ports
            for p in serial.tools.list_ports.comports():
                if p.device not in ports:
                    ports.append(p.device)
        except Exception:
            pass
        return ports

    def refresh_ports(self):
        self.available_ports = self.scan_ports()
        self.port_menu.config(values=self.available_ports)
        if self.available_ports and not self.port_var.get():
            self.port_var.set(self.available_ports[0])

    def toggle_connection(self):
        if self.robot.is_connected and not self.robot.is_mock:
            self.robot.disconnect()
            self.update_status()
        else:
            p = self.port_var.get().strip() or None
            self.connect_hardware(port=p)

    def connect_hardware(self, port=None):
        if self.robot.is_connected:
            try:
                self.robot.disconnect()
            except Exception:
                pass
        self.robot.connect(port=port)
        if self.robot.is_connected and not self.robot.is_mock:
            self.angles = [math.degrees(a) for a in self.robot.current_angles]
            actual_port = getattr(self.robot, "port", None) or port
            if actual_port:
                if actual_port not in self.available_ports:
                    self.available_ports.append(actual_port)
                    self.port_menu.config(values=self.available_ports)
                self.port_var.set(actual_port)
        self.update_status()

    def update_status(self):
        if not self.robot.is_connected:
            self.status.config(text="● Disconnected", fg="#ef4444")
            self.conn_btn.config(text="Connect", bg="#0284c7")
        elif self.robot.is_mock:
            err = getattr(self.robot, "last_connection_error", "") or "No hardware response"
            short_err = (err[:30] + "...") if len(err) > 30 else err
            self.status.config(text=f"● Simulation ({short_err})", fg="#fbbf24")
            self.conn_btn.config(text="Retry", bg="#d97706")
        else:
            port = getattr(self.robot, "port", None) or self.port_var.get() or ""
            self.status.config(text=f"● Connected {port}".strip(), fg="#22c55e")
            self.conn_btn.config(text="Disconnect", bg="#b91c1c")

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

    def toggle_record(self):
        if self.replay_state:
            return
        if not self.recording:
            self.recording = True
            self.record_t0 = time.perf_counter()
            self.last_record_time = 0.0
            cur_angles = [round(a, 2) for a in self.angles]
            self.recorded_points = [{"t": 0.0, "angles": cur_angles}]
            self.last_recorded_angles = list(cur_angles)
            self.record_btn.config(text="⏹ Stop Rec", relief=tk.SUNKEN)
            self.save_btn.config(state=tk.DISABLED)
            self.load_btn.config(state=tk.DISABLED)
            self.replay_btn.config(state=tk.DISABLED)
            self.traj_label.config(text="🔴 Recording: 0.0s (1 pts)")
        else:
            self.recording = False
            elapsed = round(time.perf_counter() - self.record_t0, 3)
            cur_angles = [round(a, 2) for a in self.angles]
            if not self.recorded_points or self.recorded_points[-1]["angles"] != cur_angles or elapsed > self.recorded_points[-1]["t"]:
                self.recorded_points.append({"t": elapsed, "angles": cur_angles})
            self.record_btn.config(text="🔴 Record", relief=tk.RAISED)
            self.save_btn.config(state=tk.NORMAL if self.recorded_points else tk.DISABLED)
            self.load_btn.config(state=tk.NORMAL)
            self.replay_btn.config(state=tk.NORMAL if self.trajectory else tk.DISABLED)
            total_dur = self.recorded_points[-1]["t"] if self.recorded_points else 0.0
            self.traj_label.config(
                text=f"Recording stopped: {len(self.recorded_points)} pts ({total_dur:.1f}s). Ready to save."
            )

    def save_trajectory(self):
        if self.recording:
            self.toggle_record()
        if not self.recorded_points:
            messagebox.showwarning("Warning", "No trajectory points to save.")
            return

        os.makedirs("trajectories", exist_ok=True)
        default_name = f"traj_{time.strftime('%Y%m%d_%H%M%S')}.json"
        path = filedialog.asksaveasfilename(
            initialdir="trajectories",
            initialfile=default_name,
            defaultextension=".json",
            filetypes=[("JSON Trajectory", "*.json"), ("All Files", "*.*")],
        )
        if not path:
            return

        total_time = round(self.recorded_points[-1]["t"], 2)
        fname = os.path.basename(path)
        data = {
            "name": fname,
            "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": total_time,
            "num_points": len(self.recorded_points),
            "trajectory": self.recorded_points,
        }
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            messagebox.showerror("Save Error", f"Could not save trajectory: {e}")
            return

        # Automatically make available for immediate replay
        self.trajectory = [(float(p["t"]), [float(a) for a in p["angles"][:4]]) for p in self.recorded_points]
        self.traj_label.config(
            text=f"Saved & Loaded: {fname} ({len(self.trajectory)} pts, {total_time:.1f}s)"
        )
        self.replay_btn.config(state=tk.NORMAL)

    def load_trajectory(self):
        if self.replay_state or self.recording:
            return
        os.makedirs("trajectories", exist_ok=True)
        path = filedialog.askopenfilename(
            initialdir="trajectories",
            filetypes=[("JSON Trajectory", "*.json"), ("All Files", "*.*")],
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
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
        if not self.trajectory or self.replay_state or self.recording:
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
        self.load_btn.config(state=tk.DISABLED)
        self.record_btn.config(state=tk.DISABLED)
        self.save_btn.config(state=tk.DISABLED)

    def abort_replay(self):
        self.end_replay("Replay aborted")

    def end_replay(self, msg):
        if not self.replay_state:
            return
        self.replay_state = None
        self.replay_btn.config(state=tk.NORMAL)
        self.abort_btn.config(state=tk.DISABLED)
        self.load_btn.config(state=tk.NORMAL)
        self.record_btn.config(state=tk.NORMAL)
        self.save_btn.config(state=tk.NORMAL if self.recorded_points else tk.DISABLED)
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
        co_tension = self.co_tension_var.get()
        if self.incremental_var.get():
            for m in BUTTONS[name][2]:
                self.inc_targets[m] += INCREMENT_DEG
                if not co_tension:
                    self.inc_targets[(m + 2) % 4] -= INCREMENT_DEG
            limit = MAX_ANGLE_DEG if co_tension else self.amplitude
            self.inc_targets = [max(0.0, min(limit, v)) for v in self.inc_targets]
            return

        if name in self.active:
            self.active.discard(name)
        else:
            if not co_tension:
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
        co_tension = self.co_tension_var.get()
        if self.incremental_var.get():
            limit = MAX_ANGLE_DEG if co_tension else self.amplitude
            return [max(0.0, min(limit, v)) for v in self.inc_targets]
        t = [0.0] * 4
        for name in self.active:
            motors = BUTTONS[name][2]
            for m in motors:
                t[m] += self.amplitude
                if not co_tension:
                    t[(m + 2) % 4] -= self.amplitude
        limit = MAX_ANGLE_DEG if co_tension else self.amplitude
        return [max(0.0, min(limit, v)) for v in t]

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

        # Live trajectory recording
        if self.recording:
            elapsed = round(time.perf_counter() - self.record_t0, 3)
            cur_angles = [round(a, 2) for a in self.angles]
            time_delta = elapsed - self.last_record_time

            angles_changed = (
                self.last_recorded_angles is None or
                any(abs(c - p) >= 0.05 for c, p in zip(cur_angles, self.last_recorded_angles))
            )

            if angles_changed:
                if time_delta >= 0.10:
                    # If previously holding stationary for >250 ms, lock the hold waypoint before motion
                    if self.last_recorded_angles is not None and time_delta > 0.25:
                        self.recorded_points.append({
                            "t": round(elapsed - TICK_MS / 1000.0, 3),
                            "angles": list(self.last_recorded_angles),
                        })
                    self.recorded_points.append({"t": elapsed, "angles": list(cur_angles)})
                    self.last_recorded_angles = list(cur_angles)
                    self.last_record_time = elapsed
                    self.traj_label.config(text=f"🔴 Recording: {elapsed:.1f}s ({len(self.recorded_points)} pts)")
            elif time_delta >= 1.0:
                self.recorded_points.append({"t": elapsed, "angles": list(cur_angles)})
                self.last_recorded_angles = list(cur_angles)
                self.last_record_time = elapsed
                self.traj_label.config(text=f"🔴 Recording: {elapsed:.1f}s ({len(self.recorded_points)} pts)")

        self.robot.send_angles([math.radians(a) for a in self.angles])
        self.readout.config(text="  ".join(f"M{i + 1}: {a:6.1f}°" for i, a in enumerate(self.angles)))
        self.draw()
        self.root.after(TICK_MS, self.tick)

    def close(self):
        if self.recording:
            self.toggle_record()
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
