"""
cabledriven_gui.py
------------------
Interactive Graphical User Interface (GUI) for EMIO Cable-Driven Soft Robot.

Features:
- Individual motor sliders with real-time degree/radian readout & fine nudge buttons (+/-1°, +/-5°)
- 2D Cross-section / Polar Tension Visualization
- Directional D-Pad for intuitive bending (+X, -X, +Y, -Y, Diagonal)
- Automated motion test triggers (Single-cable, Antagonistic, 360° Orbit)
- Emergency Stop / Relax All Cables button
- Hardware status & COM port selector
"""

import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

# Import controller logic from cabledriven.py
try:
    from cabledriven import CableDrivenRobotController, HAS_EMIO_API, MAX_ANGLE_DEG
except ImportError:
    # Fallback minimal import if moved
    HAS_EMIO_API = False
    MAX_ANGLE_DEG = 180.0
    class CableDrivenRobotController:
        def __init__(self, **kwargs):
            self.current_angles = [0.0] * 4
            self.is_connected = False
            self.is_mock = True
            self.max_angle = math.radians(MAX_ANGLE_DEG)
            self.min_angle = 0.0
            self.num_motors = 4
        def connect(self, port=None): self.is_connected = True
        def disconnect(self): self.is_connected = False
        def send_angles(self, a): self.current_angles = a
        def move_smooth(self, a, duration=1.0): self.current_angles = a
        def home(self): self.current_angles = [0.0] * 4


class CableRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("EMIO Cable-Driven Soft Robot Controller")
        self.root.geometry("980x740")
        self.root.minsize(920, 680)

        # Set theme styling
        self.style = ttk.Style()
        self.style.theme_use("clam")
        
        # Color Palette
        self.bg_dark = "#1e1e24"
        self.panel_bg = "#2b2d42"
        self.accent_color = "#4cc9f0"
        self.alert_color = "#f72585"
        self.text_color = "#edf2f4"
        self.green_color = "#4ade80"

        self.root.configure(bg=self.bg_dark)

        # Hardware Controller
        self.robot = CableDrivenRobotController(min_angle=0.0, max_angle=math.radians(MAX_ANGLE_DEG), num_motors=4)
        self.is_moving_thread = False
        self.motor_vars = [tk.DoubleVar(value=0.0) for _ in range(4)]
        self.max_limit_var = tk.DoubleVar(value=MAX_ANGLE_DEG)  # Max angle limit in degrees

        self.setup_ui()
        self.connect_hardware()

    def setup_ui(self):
        # Top Header & Connection Bar
        top_frame = tk.Frame(self.root, bg=self.panel_bg, padx=15, pady=10)
        top_frame.pack(fill=tk.X, padx=10, pady=(10, 5))

        title_lbl = tk.Label(
            top_frame,
            text="EMIO Cable-Driven Robot Controller",
            font=("Segoe UI", 14, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
        )
        title_lbl.pack(side=tk.LEFT)

        # Status Indicator
        self.status_lbl = tk.Label(
            top_frame,
            text="● Disconnected",
            font=("Segoe UI", 10, "bold"),
            fg="#ef4444",
            bg=self.panel_bg,
            padx=10,
        )
        self.status_lbl.pack(side=tk.RIGHT)

        btn_reconnect = tk.Button(
            top_frame,
            text="Reconnect",
            command=self.connect_hardware,
            bg="#3a0ca3",
            fg="white",
            relief=tk.FLAT,
            padx=8,
            pady=2,
            font=("Segoe UI", 9),
            cursor="hand2",
        )
        btn_reconnect.pack(side=tk.RIGHT, padx=5)

        # Main Layout: 2 Columns (Left: Sliders & Controls, Right: Visualization & Direction Pad & Routines)
        main_pane = tk.Frame(self.root, bg=self.bg_dark)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        left_frame = tk.Frame(main_pane, bg=self.bg_dark)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        right_frame = tk.Frame(main_pane, bg=self.bg_dark, width=380)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))

        self.create_slider_controls(left_frame)
        self.create_visualization_panel(right_frame)
        self.create_dpad_panel(right_frame)
        self.create_motion_routines_panel(left_frame)
        self.create_bottom_bar()

    def create_slider_controls(self, parent):
        group = tk.LabelFrame(
            parent,
            text=f" Individual Motor / Cable Tension (0° - {MAX_ANGLE_DEG:g}°) ",
            font=("Segoe UI", 11, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            padx=15,
            pady=10,
        )
        group.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        cable_names = [
            ("Motor 1 (Cable +X / 0°)", "#38bdf8"),
            ("Motor 2 (Cable +Y / 90°)", "#a855f7"),
            ("Motor 3 (Cable -X / 180°)", "#f97316"),
            ("Motor 4 (Cable -Y / 270°)", "#10b981"),
        ]

        self.slider_widgets = []
        self.val_labels = []

        for i in range(4):
            m_frame = tk.Frame(group, bg=self.panel_bg)
            m_frame.pack(fill=tk.X, pady=6)

            # Label
            name, color = cable_names[i]
            lbl = tk.Label(
                m_frame,
                text=name,
                font=("Segoe UI", 9, "bold"),
                fg=color,
                bg=self.panel_bg,
                width=24,
                anchor="w",
            )
            lbl.pack(side=tk.LEFT)

            # Slider
            slider = tk.Scale(
                m_frame,
                from_=0.0,
                to=MAX_ANGLE_DEG,
                orient=tk.HORIZONTAL,
                resolution=0.5,
                variable=self.motor_vars[i],
                command=lambda val, idx=i: self.on_slider_change(idx, val),
                bg="#3f37c9",
                fg=self.text_color,
                troughcolor="#1e1e24",
                highlightthickness=0,
                showvalue=0,
                length=220,
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
            self.slider_widgets.append(slider)

            # Value label
            val_lbl = tk.Label(
                m_frame,
                text=" 0.0° (0.00 rad)",
                font=("Consolas", 9),
                fg=self.text_color,
                bg=self.panel_bg,
                width=16,
            )
            val_lbl.pack(side=tk.LEFT, padx=4)
            self.val_labels.append(val_lbl)

            # Fine adjustment nudge buttons
            btn_minus5 = tk.Button(
                m_frame,
                text="-5°",
                command=lambda idx=i: self.nudge_motor(idx, -5.0),
                bg="#495057",
                fg="white",
                relief=tk.FLAT,
                width=3,
                font=("Segoe UI", 8),
            )
            btn_minus5.pack(side=tk.LEFT, padx=1)

            btn_minus1 = tk.Button(
                m_frame,
                text="-1°",
                command=lambda idx=i: self.nudge_motor(idx, -1.0),
                bg="#495057",
                fg="white",
                relief=tk.FLAT,
                width=3,
                font=("Segoe UI", 8),
            )
            btn_minus1.pack(side=tk.LEFT, padx=1)

            btn_plus1 = tk.Button(
                m_frame,
                text="+1°",
                command=lambda idx=i: self.nudge_motor(idx, 1.0),
                bg="#495057",
                fg="white",
                relief=tk.FLAT,
                width=3,
                font=("Segoe UI", 8),
            )
            btn_plus1.pack(side=tk.LEFT, padx=1)

            btn_plus5 = tk.Button(
                m_frame,
                text="+5°",
                command=lambda idx=i: self.nudge_motor(idx, 5.0),
                bg="#495057",
                fg="white",
                relief=tk.FLAT,
                width=3,
                font=("Segoe UI", 8),
            )
            btn_plus5.pack(side=tk.LEFT, padx=1)

            # Zero individual button
            btn_zero = tk.Button(
                m_frame,
                text="Zero",
                command=lambda idx=i: self.zero_motor(idx),
                bg="#212529",
                fg="#fca311",
                relief=tk.FLAT,
                width=4,
                font=("Segoe UI", 8, "bold"),
            )
            btn_zero.pack(side=tk.LEFT, padx=(3, 0))

    def create_visualization_panel(self, parent):
        group = tk.LabelFrame(
            parent,
            text=" Real-time Cable Tension Visualizer ",
            font=("Segoe UI", 11, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            padx=10,
            pady=10,
        )
        group.pack(fill=tk.X, pady=(0, 10))

        self.canvas_size = 200
        self.canvas = tk.Canvas(
            group,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="#111217",
            highlightthickness=0,
        )
        self.canvas.pack(pady=5)
        self.draw_canvas_visualizer()

    def draw_canvas_visualizer(self):
        self.canvas.delete("all")
        cx, cy = self.canvas_size // 2, self.canvas_size // 2
        r_outer = 80
        r_inner = 15

        # Background reference rings
        self.canvas.create_oval(cx - r_outer, cy - r_outer, cx + r_outer, cy + r_outer, outline="#333", width=2)
        self.canvas.create_oval(cx - r_outer//2, cy - r_outer//2, cx + r_outer//2, cy + r_outer//2, outline="#222", width=1, dash=(2, 2))
        self.canvas.create_line(cx - r_outer - 10, cy, cx + r_outer + 10, cy, fill="#2c2c34", dash=(2, 2))
        self.canvas.create_line(cx, cy - r_outer - 10, cx, cy + r_outer + 10, fill="#2c2c34", dash=(2, 2))

        # Cable anchor positions (4 quadrants)
        # M1 (+X), M2 (+Y), M3 (-X), M4 (-Y) in Cartesian (note canvas Y is inverted)
        anchors = [
            (cx + r_outer, cy, "M1 (+X)", "#38bdf8"),
            (cx, cy - r_outer, "M2 (+Y)", "#a855f7"),
            (cx - r_outer, cy, "M3 (-X)", "#f97316"),
            (cx, cy + r_outer, "M4 (-Y)", "#10b981"),
        ]

        net_fx = 0.0
        net_fy = 0.0

        for i, (ax, ay, label, color) in enumerate(anchors):
            deg = self.motor_vars[i].get()
            tension_norm = deg / MAX_ANGLE_DEG  # 0.0 to 1.0

            # Draw anchor circle
            self.canvas.create_oval(ax - 7, ay - 7, ax + 7, ay + 7, fill=color, outline="")
            self.canvas.create_text(ax, ay + (14 if ay >= cy else -14), text=label, fill=color, font=("Segoe UI", 7))

            # Draw tension vector from center to anchor
            if tension_norm > 0.01:
                t_x = cx + (ax - cx) * tension_norm
                t_y = cy + (ay - cy) * tension_norm
                self.canvas.create_line(cx, cy, t_x, t_y, fill=color, width=int(1 + tension_norm * 4))

            # Accumulate net bending vector
            if i == 0: net_fx += tension_norm
            elif i == 1: net_fy -= tension_norm
            elif i == 2: net_fx -= tension_norm
            elif i == 3: net_fy += tension_norm

        # Tip bending indicator dot
        tip_x = cx + net_fx * (r_outer * 0.7)
        tip_y = cy + net_fy * (r_outer * 0.7)

        # Draw soft center backbone
        self.canvas.create_line(cx, cy, tip_x, tip_y, fill="#f72585", width=3)
        self.canvas.create_oval(tip_x - 6, tip_y - 6, tip_x + 6, tip_y + 6, fill="#f72585", outline="white", width=1.5)
        self.canvas.create_oval(cx - r_inner, cy - r_inner, cx + r_inner, cy + r_inner, outline="#555", width=1.5)

    def create_dpad_panel(self, parent):
        group = tk.LabelFrame(
            parent,
            text=" Directional Bending D-Pad ",
            font=("Segoe UI", 11, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            padx=10,
            pady=10,
        )
        group.pack(fill=tk.BOTH, expand=True)

        dpad_frame = tk.Frame(group, bg=self.panel_bg)
        dpad_frame.pack(pady=5)

        # Bending angle preset for D-Pad
        dpad_pull_deg = 30.0

        btn_up = tk.Button(
            dpad_frame,
            text="▲\n+Y",
            command=lambda: self.bend_direction(0, dpad_pull_deg, 0, 0),
            bg="#3a0ca3",
            fg="white",
            width=6,
            height=2,
            relief=tk.FLAT,
            font=("Segoe UI", 9, "bold"),
        )
        btn_up.grid(row=0, column=1, padx=4, pady=4)

        btn_left = tk.Button(
            dpad_frame,
            text="◀ -X",
            command=lambda: self.bend_direction(0, 0, dpad_pull_deg, 0),
            bg="#3a0ca3",
            fg="white",
            width=6,
            height=2,
            relief=tk.FLAT,
            font=("Segoe UI", 9, "bold"),
        )
        btn_left.grid(row=1, column=0, padx=4, pady=4)

        btn_center = tk.Button(
            dpad_frame,
            text="●\nCenter",
            command=self.home_all,
            bg="#f72585",
            fg="white",
            width=6,
            height=2,
            relief=tk.FLAT,
            font=("Segoe UI", 9, "bold"),
        )
        btn_center.grid(row=1, column=1, padx=4, pady=4)

        btn_right = tk.Button(
            dpad_frame,
            text="+X ▶",
            command=lambda: self.bend_direction(dpad_pull_deg, 0, 0, 0),
            bg="#3a0ca3",
            fg="white",
            width=6,
            height=2,
            relief=tk.FLAT,
            font=("Segoe UI", 9, "bold"),
        )
        btn_right.grid(row=1, column=2, padx=4, pady=4)

        btn_down = tk.Button(
            dpad_frame,
            text="-Y\n▼",
            command=lambda: self.bend_direction(0, 0, 0, dpad_pull_deg),
            bg="#3a0ca3",
            fg="white",
            width=6,
            height=2,
            relief=tk.FLAT,
            font=("Segoe UI", 9, "bold"),
        )
        btn_down.grid(row=2, column=1, padx=4, pady=4)

    def create_motion_routines_panel(self, parent):
        group = tk.LabelFrame(
            parent,
            text=" Automated Motion Routines & Tests ",
            font=("Segoe UI", 11, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            padx=15,
            pady=10,
        )
        group.pack(fill=tk.X, pady=(0, 5))

        btn_frame = tk.Frame(group, bg=self.panel_bg)
        btn_frame.pack(fill=tk.X)

        btn_t1 = tk.Button(
            btn_frame,
            text="1. Single Cable Test",
            command=lambda: self.run_async_routine(self.robot.test_single_cable_actuation),
            bg="#4361ee",
            fg="white",
            relief=tk.FLAT,
            padx=8,
            pady=6,
            font=("Segoe UI", 9),
            cursor="hand2",
        )
        btn_t1.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        btn_t2 = tk.Button(
            btn_frame,
            text="2. Antagonistic Test",
            command=lambda: self.run_async_routine(self.robot.test_antagonistic_bending),
            bg="#4361ee",
            fg="white",
            relief=tk.FLAT,
            padx=8,
            pady=6,
            font=("Segoe UI", 9),
            cursor="hand2",
        )
        btn_t2.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        btn_t3 = tk.Button(
            btn_frame,
            text="3. Circular Orbit Sweep",
            command=lambda: self.run_async_routine(self.robot.test_circular_orbit),
            bg="#4361ee",
            fg="white",
            relief=tk.FLAT,
            padx=8,
            pady=6,
            font=("Segoe UI", 9),
            cursor="hand2",
        )
        btn_t3.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        btn_all = tk.Button(
            btn_frame,
            text="★ Full Auto Demo",
            command=lambda: self.run_async_routine(self.robot.run_all_tests),
            bg="#7209b7",
            fg="white",
            relief=tk.FLAT,
            padx=8,
            pady=6,
            font=("Segoe UI", 9, "bold"),
            cursor="hand2",
        )
        btn_all.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

    def create_bottom_bar(self):
        bottom_frame = tk.Frame(self.root, bg=self.panel_bg, padx=15, pady=8)
        bottom_frame.pack(fill=tk.X, padx=10, pady=(5, 10))

        # Emergency Stop / Relax All
        btn_estop = tk.Button(
            bottom_frame,
            text="🛑 RELAX ALL / EMERGENCY STOP",
            command=self.home_all,
            bg=self.alert_color,
            fg="white",
            relief=tk.FLAT,
            padx=15,
            pady=6,
            font=("Segoe UI", 11, "bold"),
            cursor="hand2",
        )
        btn_estop.pack(side=tk.LEFT)

        # Safety Max Angle Limit
        limit_frame = tk.Frame(bottom_frame, bg=self.panel_bg)
        limit_frame.pack(side=tk.RIGHT)

        lbl_limit = tk.Label(
            limit_frame,
            text="Max Safe Angle Limit: ",
            fg=self.text_color,
            bg=self.panel_bg,
            font=("Segoe UI", 9),
        )
        lbl_limit.pack(side=tk.LEFT)

        limit_spin = tk.Spinbox(
            limit_frame,
            from_=15.0,
            to=MAX_ANGLE_DEG,
            increment=5.0,
            textvariable=self.max_limit_var,
            width=5,
            command=self.update_max_limit,
            font=("Segoe UI", 9),
        )
        limit_spin.pack(side=tk.LEFT, padx=3)

    # -------------------------------------------------------------
    # Action Handlers
    # -------------------------------------------------------------

    def connect_hardware(self):
        def _connect():
            self.robot.connect()
            if self.robot.is_mock:
                self.status_lbl.config(
                    text="● Running in Simulation / Mock Mode",
                    fg="#fbbf24",
                )
            else:
                self.status_lbl.config(
                    text="● Connected to EMIO Hardware",
                    fg=self.green_color,
                )
        threading.Thread(target=_connect, daemon=True).start()

    def update_max_limit(self):
        max_deg = self.max_limit_var.get()
        self.robot.max_angle = math.radians(max_deg)
        for slider in self.slider_widgets:
            slider.config(to=max_deg)

    def on_slider_change(self, index, val_str):
        if self.is_moving_thread:
            return
        deg = float(val_str)
        rad = math.radians(deg)
        self.val_labels[index].config(text=f"{deg:5.1f}° ({rad:.2f} rad)")
        self.draw_canvas_visualizer()

        # Send target angles directly
        all_rads = [math.radians(v.get()) for v in self.motor_vars]
        self.robot.send_angles(all_rads)

    def nudge_motor(self, index, delta_deg):
        current = self.motor_vars[index].get()
        max_lim = self.max_limit_var.get()
        new_val = max(0.0, min(max_lim, current + delta_deg))
        self.motor_vars[index].set(new_val)
        self.on_slider_change(index, new_val)

    def zero_motor(self, index):
        self.motor_vars[index].set(0.0)
        self.on_slider_change(index, 0.0)

    def bend_direction(self, m1_deg, m2_deg, m3_deg, m4_deg):
        """Smoothly moves robot to specific quadrant bending angles."""
        targets = [m1_deg, m2_deg, m3_deg, m4_deg]
        for i, val in enumerate(targets):
            self.motor_vars[i].set(val)
            self.val_labels[i].config(text=f"{val:5.1f}° ({math.radians(val):.2f} rad)")

        self.draw_canvas_visualizer()
        rads = [math.radians(t) for t in targets]

        def _move():
            self.is_moving_thread = True
            self.robot.move_smooth(rads, duration=1.0)
            self.is_moving_thread = False

        threading.Thread(target=_move, daemon=True).start()

    def home_all(self):
        """Immediately relaxes all motors to zero."""
        for i in range(4):
            self.motor_vars[i].set(0.0)
            self.val_labels[i].config(text=" 0.0° (0.00 rad)")
        self.draw_canvas_visualizer()

        def _home():
            self.is_moving_thread = True
            self.robot.home()
            self.is_moving_thread = False

        threading.Thread(target=_home, daemon=True).start()

    def run_async_routine(self, routine_func):
        """Runs automated tests in background thread and updates sliders in real time."""
        if self.is_moving_thread:
            messagebox.showinfo("Busy", "A motion routine is already running.")
            return

        def _worker():
            self.is_moving_thread = True
            self.status_lbl.config(text="● Running Automated Routine...", fg="#38bdf8")

            # Monkey-patch send_angles temporarily to update GUI sliders during routine
            orig_send = self.robot.send_angles

            def sync_send(angles):
                orig_send(angles)
                for idx, rad in enumerate(angles):
                    deg = math.degrees(rad)
                    self.motor_vars[idx].set(deg)
                    self.val_labels[idx].config(text=f"{deg:5.1f}° ({rad:.2f} rad)")
                self.draw_canvas_visualizer()

            self.robot.send_angles = sync_send

            try:
                routine_func()
            finally:
                self.robot.send_angles = orig_send
                self.is_moving_thread = False
                self.home_all()
                self.connect_hardware()

        threading.Thread(target=_worker, daemon=True).start()

    def on_close(self):
        try:
            self.robot.disconnect()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CableRobotGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
