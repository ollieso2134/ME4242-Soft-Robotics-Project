"""
cartesian_controller.py
-----------------------
Cartesian (XYZ) Task-Space Controller & Inverse Kinematics GUI
for ME4242 Soft Robotics EMIO Cable-Driven Continuum Robot.

Physical Rig Specifications:
- Machine Height (Ceiling to Base Plate): 280.0 mm
- Manipulator Arm Arc Length: 185.0 mm (strictly constant length)
- Base Plate Span: 365.0 mm (table diameter)
- Backbone-to-Cable Radial Offset: 5.0 mm (tendon guide holes on spacer disks)
- Flexible Spine: 1.5 - 2.0 mm central magnetic rod with green magnetic tip
- Actuation: 4 antagonistic cables driven by Dynamixel spools (r_spool = 12.0 mm)
    * Motor 1: Cable +X (0 deg)
    * Motor 2: Cable +Y (90 deg)
    * Motor 3: Cable -X (180 deg)
    * Motor 4: Cable -Y (270 deg)

Kinematics Formulation:
- Piecewise Constant Curvature (PCC) Model:
    * Bending angle: theta (radians)
    * Bending plane: phi = atan2(Y, X)
    * Radial tip displacement: r_xy = (L / theta) * (1 - cos(theta))
    * Vertical tip drop: Delta_Z = (L / theta) * sin(theta)
    * Tip height above baseplate: Z = H_machine - Delta_Z
- Inverse Kinematics (IK):
    * Computes (theta, phi) from target (X, Y) using Newton-Raphson.
    * Computes cable length changes: Delta_l_i = -d * theta * cos(phi - (i-1)*pi/2).
    * Maps to Dynamixel motor rotation angles: M_i = max(0, -Delta_l_i / r_spool).
"""

import os
import sys
import math
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# Ensure current directory is in sys.path
_current_dir = os.path.dirname(os.path.abspath(__file__))
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

try:
    from cabledriven import CableDrivenRobotController, list_serial_ports, MAX_ANGLE_DEG
except ImportError:
    # Minimal fallback controller if cabledriven is missing
    MAX_ANGLE_DEG = 180.0
    def list_serial_ports(): return []
    class CableDrivenRobotController:
        def __init__(self, **kwargs):
            self.current_angles = [0.0] * 4
            self.is_connected = False
            self.is_mock = True
        def connect(self, port=None): self.is_connected = True
        def disconnect(self): self.is_connected = False
        def send_angles(self, a, *args, **kwargs): self.current_angles = list(a)
        def move_smooth(self, a, duration=0.8, steps=20): self.current_angles = list(a)
        def home(self): self.current_angles = [0.0] * 4


# =====================================================================
# 1. Physical Model & Kinematics Engine
# =====================================================================

class CartesianKinematics:
    """
    Mathematical Kinematics Engine for 4-Cable Continuum Robot
    with exact user-specified dimensions.
    """

    def __init__(
        self,
        arm_length_mm=185.0,
        machine_height_mm=280.0,
        base_plate_span_mm=365.0,
        tendon_offset_mm=5.0,
        spool_radius_mm=12.0,
        backbone_diameter_mm=1.8,
    ):
        self.L = float(arm_length_mm)              # 185 mm
        self.H = float(machine_height_mm)          # 280 mm
        self.base_span = float(base_plate_span_mm) # 365 mm
        self.d = float(tendon_offset_mm)           # 5.0 mm
        self.r_spool = float(spool_radius_mm)      # 12.0 mm
        self.backbone_diam = float(backbone_diameter_mm)

        # Maximum reachable horizontal radius in free space (at theta ~ 133 deg)
        self.max_reach_r = 0.7246 * self.L         # ~134.0 mm

    def forward_kinematics(self, motor_angles_rad):
        """
        Computes (X, Y, Z) of the green magnetic end effector from 4 motor angles.
        :param motor_angles_rad: [m1, m2, m3, m4] in radians
        :return: dict with tip_xyz, theta, phi, spine_pts
        """
        m1, m2, m3, m4 = [float(a) for a in motor_angles_rad[:4]]

        # Cable shortening amounts: +X bend when m1 pulls, +Y bend when m2 pulls
        dl_x = (m1 - m3) * self.r_spool  # +X pull (m1) vs -X pull (m3)
        dl_y = (m2 - m4) * self.r_spool  # +Y pull (m2) vs -Y pull (m4)

        # Resulting bending components
        # dl_i = -d * theta * cos(phi - phi_i)
        curv_x = dl_x / self.d
        curv_y = dl_y / self.d

        theta = math.hypot(curv_x, curv_y)
        phi = math.atan2(curv_y, curv_x)

        # Compute spine 3D curve (from ceiling Z = H down to tip)
        num_pts = 25
        spine_pts = []

        if theta < 1e-5:
            # Straight hang downward along -Z
            tip_x = 0.0
            tip_y = 0.0
            tip_z = self.H - self.L  # 280 - 185 = 95 mm
            for i in range(num_pts):
                s = float(i) / float(num_pts - 1)
                spine_pts.append((0.0, 0.0, self.H - s * self.L))
        else:
            R = self.L / theta
            for i in range(num_pts):
                s = float(i) / float(num_pts - 1)
                th_s = s * theta
                r_s = R * (1.0 - math.cos(th_s))
                px = r_s * math.cos(phi)
                py = r_s * math.sin(phi)
                pz = self.H - R * math.sin(th_s)
                spine_pts.append((px, py, pz))

            tip_x, tip_y, tip_z = spine_pts[-1]

        return {
            "tip_xyz": (tip_x, tip_y, tip_z),
            "theta_rad": theta,
            "theta_deg": math.degrees(theta),
            "phi_rad": phi,
            "phi_deg": (math.degrees(phi) + 360.0) % 360.0,
            "spine_pts": spine_pts,
        }

    def solve_theta(self, r_xy):
        """Solves (L / theta) * (1 - cos(theta)) = r_xy via Newton-Raphson."""
        if r_xy <= 1e-4:
            return 0.0
        r_clamped = min(r_xy, self.max_reach_r)
        theta = 2.0 * r_clamped / self.L
        for _ in range(6):
            g = (self.L / theta) * (1.0 - math.cos(theta)) - r_clamped
            gp = (self.L / theta) * math.sin(theta) - (self.L / (theta**2)) * (1.0 - math.cos(theta))
            if abs(gp) < 1e-6:
                break
            step = g / gp
            theta -= step
            if abs(step) < 1e-6:
                break
        return max(0.0, theta)

    def inverse_kinematics(self, target_x_mm, target_y_mm):
        """
        Computes 4 Dynamixel motor angles [m1, m2, m3, m4] for desired (X, Y) tip position.
        :param target_x_mm: Commanded lateral X displacement (mm)
        :param target_y_mm: Commanded longitudinal Y displacement (mm)
        :return: dict with motor_angles_rad, motor_angles_deg, achieved_xyz, cable_pulls_mm
        """
        tx = float(target_x_mm)
        ty = float(target_y_mm)

        r_xy = math.hypot(tx, ty)
        phi = math.atan2(ty, tx)

        # Enforce physical reach limit
        clamped = False
        if r_xy > self.max_reach_r:
            tx = self.max_reach_r * math.cos(phi)
            ty = self.max_reach_r * math.sin(phi)
            r_xy = self.max_reach_r
            clamped = True

        theta = self.solve_theta(r_xy)

        # Cable length changes on 4 tendons at distance d = 5.0 mm
        # delta_l_i = -d * theta * cos(phi - phi_i)
        dl1 = -self.d * theta * math.cos(phi)             # Cable 1 (+X, 0 deg)
        dl2 = -self.d * theta * math.sin(phi)             # Cable 2 (+Y, 90 deg)
        dl3 = +self.d * theta * math.cos(phi)             # Cable 3 (-X, 180 deg)
        dl4 = +self.d * theta * math.sin(phi)             # Cable 4 (-Y, 270 deg)

        # Antagonistic motor angles (only pulling cables rotate, slack cables stay at 0)
        m1 = max(0.0, -dl1 / self.r_spool)
        m2 = max(0.0, -dl2 / self.r_spool)
        m3 = max(0.0, -dl3 / self.r_spool)
        m4 = max(0.0, -dl4 / self.r_spool)

        # Maximum travel limit (e.g. 180 deg)
        max_rad = math.radians(MAX_ANGLE_DEG)
        m1 = min(max_rad, m1)
        m2 = min(max_rad, m2)
        m3 = min(max_rad, m3)
        m4 = min(max_rad, m4)

        # Forward kinematics on achieved angles
        fk_result = self.forward_kinematics([m1, m2, m3, m4])

        return {
            "motor_angles_rad": [m1, m2, m3, m4],
            "motor_angles_deg": [math.degrees(a) for a in [m1, m2, m3, m4]],
            "achieved_xyz": fk_result["tip_xyz"],
            "achieved_theta_deg": fk_result["theta_deg"],
            "achieved_phi_deg": fk_result["phi_deg"],
            "cable_pulls_mm": [-dl1 if dl1 < 0 else 0.0,
                               -dl2 if dl2 < 0 else 0.0,
                               -dl3 if dl3 < 0 else 0.0,
                               -dl4 if dl4 < 0 else 0.0],
            "was_clamped": clamped,
            "spine_pts": fk_result["spine_pts"],
        }


# =====================================================================
# 2. Interactive Cartesian GUI
# =====================================================================

class CartesianControlGUI:
    """
    Modern Desktop Controller GUI for Cartesian (XYZ) Continuum Robot Manipulation.
    """

    def __init__(self, root, controller=None):
        self.root = root
        self.root.title("EMIO Cartesian XYZ Task-Space Controller (ME4242)")
        self.root.geometry("1180x780")
        self.root.minsize(1020, 680)

        # Dark theme palette
        self.bg_dark = "#0f172a"
        self.panel_bg = "#1e293b"
        self.accent = "#38bdf8"
        self.accent_green = "#22c55e"
        self.accent_purple = "#c084fc"
        self.text_light = "#f8fafc"
        self.text_dim = "#94a3b8"

        self.root.configure(bg=self.bg_dark)

        # Kinematics engine with user's exact specs
        self.kin = CartesianKinematics(
            arm_length_mm=185.0,
            machine_height_mm=280.0,
            base_plate_span_mm=365.0,
            tendon_offset_mm=5.0,
            spool_radius_mm=12.0,
            backbone_diameter_mm=1.8,
        )

        # Robot controller (physical or mock)
        self.robot = controller or CableDrivenRobotController()
        self.robot.connect()

        # Commanded Cartesian state
        self.cmd_x = tk.DoubleVar(value=0.0)
        self.cmd_y = tk.DoubleVar(value=0.0)
        self.step_size = tk.DoubleVar(value=5.0)  # default 5 mm step

        # Auto-send toggle
        self.auto_send_var = tk.BooleanVar(value=True)
        self.is_interpolating = False

        self.setup_ui()
        self.update_kinematics_display()

    def setup_ui(self):
        # 1. Top Header & Specs Banner
        header = tk.Frame(self.root, bg=self.panel_bg, padx=16, pady=10)
        header.pack(fill=tk.X, padx=12, pady=(12, 6))

        # Title & Subtitle
        title_box = tk.Frame(header, bg=self.panel_bg)
        title_box.pack(side=tk.LEFT)

        tk.Label(
            title_box,
            text="🕹️ Cartesian XYZ Task-Space Controller",
            font=("Segoe UI", 14, "bold"),
            fg=self.accent,
            bg=self.panel_bg,
        ).pack(anchor="w")

        tk.Label(
            title_box,
            text="Direct End-Effector XYZ Positioning | Constant Arc Length L = 185 mm | Machine Height H = 280 mm",
            font=("Segoe UI", 8),
            fg=self.text_dim,
            bg=self.panel_bg,
        ).pack(anchor="w")

        # Hardware Status on Right
        status_box = tk.Frame(header, bg=self.panel_bg)
        status_box.pack(side=tk.RIGHT)

        st_text = "● HARDWARE CONNECTED" if self.robot.is_connected and not getattr(self.robot, "is_mock", True) else "○ SIMULATION / MOCK MODE"
        st_color = self.accent_green if "HARDWARE" in st_text else "#f59e0b"

        self.hw_status_lbl = tk.Label(
            status_box,
            text=st_text,
            font=("Segoe UI", 9, "bold"),
            fg=st_color,
            bg=self.panel_bg,
        )
        self.hw_status_lbl.pack(anchor="e")

        btn_home = tk.Button(
            status_box,
            text="🏠 Home (0,0)",
            command=self.cmd_home,
            bg="#334155",
            fg="white",
            relief=tk.FLAT,
            padx=10,
            pady=2,
            font=("Segoe UI", 8, "bold"),
            cursor="hand2",
        )
        btn_home.pack(side=tk.RIGHT, pady=(4, 0))

        # 2. Main Body: Split Left (Controls & Readout) and Right (Visualizers)
        main_body = tk.Frame(self.root, bg=self.bg_dark)
        main_body.pack(fill=tk.BOTH, expand=True, padx=12, pady=6)

        left_pane = tk.Frame(main_body, bg=self.bg_dark, width=460)
        left_pane.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 6))

        right_pane = tk.Frame(main_body, bg=self.bg_dark)
        right_pane.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(6, 0))

        # Build Left Pane Components
        self.build_spec_card(left_pane)
        self.build_cartesian_sliders(left_pane)
        self.build_teleop_dpad(left_pane)
        self.build_telemetry_table(left_pane)

        # Build Right Pane Visualizers
        self.build_visualizer_panel(right_pane)

    def build_spec_card(self, parent):
        box = tk.LabelFrame(
            parent,
            text=" Physical Rig Parameters ",
            font=("Segoe UI", 9, "bold"),
            fg=self.accent,
            bg=self.panel_bg,
            padx=10,
            pady=6,
        )
        box.pack(fill=tk.X, pady=(0, 8))

        specs = [
            ("Machine Height (Ceiling to Table):", f"{self.kin.H:.0f} mm"),
            ("Backbone Arc Length (L):", f"{self.kin.L:.0f} mm"),
            ("Base Plate Span (Diameter):", f"{self.kin.base_span:.0f} mm"),
            ("Backbone-to-Cable Offset (d):", f"{self.kin.d:.1f} mm"),
            ("Central Magnetic Rod Diam:", f"{self.kin.backbone_diam:.1f} mm"),
            ("Max Reach Radius (r_max):", f"{self.kin.max_reach_r:.1f} mm"),
        ]

        grid_frame = tk.Frame(box, bg=self.panel_bg)
        grid_frame.pack(fill=tk.X)

        for idx, (label, val) in enumerate(specs):
            row = idx // 2
            col = (idx % 2) * 2
            tk.Label(grid_frame, text=label, font=("Segoe UI", 7), fg=self.text_dim, bg=self.panel_bg, anchor="w").grid(row=row, column=col, sticky="w", padx=(2, 4), pady=1)
            tk.Label(grid_frame, text=val, font=("Segoe UI", 7, "bold"), fg=self.text_light, bg=self.panel_bg, anchor="w").grid(row=row, column=col + 1, sticky="w", padx=(0, 8), pady=1)

    def build_cartesian_sliders(self, parent):
        box = tk.LabelFrame(
            parent,
            text=" Cartesian (X, Y) Coordinates ",
            font=("Segoe UI", 9, "bold"),
            fg=self.accent,
            bg=self.panel_bg,
            padx=12,
            pady=8,
        )
        box.pack(fill=tk.X, pady=(0, 8))

        # X Slider
        x_row = tk.Frame(box, bg=self.panel_bg)
        x_row.pack(fill=tk.X, pady=4)
        tk.Label(x_row, text="Target X (mm):", font=("Segoe UI", 9, "bold"), fg="#38bdf8", bg=self.panel_bg, width=14, anchor="w").pack(side=tk.LEFT)
        self.x_slider = tk.Scale(
            x_row,
            from_=-110.0,
            to=110.0,
            resolution=0.5,
            orient=tk.HORIZONTAL,
            variable=self.cmd_x,
            command=lambda v: self.on_slider_change(),
            bg=self.panel_bg,
            fg="white",
            highlightthickness=0,
            troughcolor="#334155",
            activebackground=self.accent,
        )
        self.x_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self.x_val_lbl = tk.Label(x_row, text="0.0 mm", font=("Consolas", 9, "bold"), fg="#38bdf8", bg=self.panel_bg, width=8, anchor="e")
        self.x_val_lbl.pack(side=tk.RIGHT)

        # Y Slider
        y_row = tk.Frame(box, bg=self.panel_bg)
        y_row.pack(fill=tk.X, pady=4)
        tk.Label(y_row, text="Target Y (mm):", font=("Segoe UI", 9, "bold"), fg="#a855f7", bg=self.panel_bg, width=14, anchor="w").pack(side=tk.LEFT)
        self.y_slider = tk.Scale(
            y_row,
            from_=-110.0,
            to=110.0,
            resolution=0.5,
            orient=tk.HORIZONTAL,
            variable=self.cmd_y,
            command=lambda v: self.on_slider_change(),
            bg=self.panel_bg,
            fg="white",
            highlightthickness=0,
            troughcolor="#334155",
            activebackground=self.accent_purple,
        )
        self.y_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self.y_val_lbl = tk.Label(y_row, text="0.0 mm", font=("Consolas", 9, "bold"), fg="#a855f7", bg=self.panel_bg, width=8, anchor="e")
        self.y_val_lbl.pack(side=tk.RIGHT)

        # Z Elevation (Coupled Readout)
        z_row = tk.Frame(box, bg=self.panel_bg)
        z_row.pack(fill=tk.X, pady=(4, 2))
        tk.Label(z_row, text="Resulting Z (mm):", font=("Segoe UI", 9, "bold"), fg="#22c55e", bg=self.panel_bg, width=14, anchor="w").pack(side=tk.LEFT)
        self.z_val_lbl = tk.Label(z_row, text="95.0 mm (Table dist: 95.0 mm)", font=("Consolas", 9, "bold"), fg="#22c55e", bg=self.panel_bg, anchor="w")
        self.z_val_lbl.pack(side=tk.LEFT, padx=8)

    def build_teleop_dpad(self, parent):
        box = tk.LabelFrame(
            parent,
            text=" Incremental Step Jogging ",
            font=("Segoe UI", 9, "bold"),
            fg=self.accent,
            bg=self.panel_bg,
            padx=10,
            pady=6,
        )
        box.pack(fill=tk.X, pady=(0, 8))

        top_ctrl = tk.Frame(box, bg=self.panel_bg)
        top_ctrl.pack(fill=tk.X, pady=(0, 6))

        tk.Label(top_ctrl, text="Step Size:", font=("Segoe UI", 8), fg=self.text_dim, bg=self.panel_bg).pack(side=tk.LEFT, padx=(0, 6))
        for step in [1.0, 5.0, 10.0, 20.0]:
            btn = tk.Radiobutton(
                top_ctrl,
                text=f"{step:g} mm",
                variable=self.step_size,
                value=step,
                bg=self.panel_bg,
                fg=self.text_light,
                selectcolor="#334155",
                activebackground=self.panel_bg,
                font=("Segoe UI", 8),
            )
            btn.pack(side=tk.LEFT, padx=4)

        # D-pad grid
        dpad = tk.Frame(box, bg=self.panel_bg)
        dpad.pack(pady=4)

        b_style = {"width": 7, "height": 1, "bg": "#334155", "fg": "white", "font": ("Segoe UI", 8, "bold"), "relief": tk.FLAT, "cursor": "hand2"}

        tk.Button(dpad, text="▲ +Y", command=lambda: self.nudge(0, 1), **b_style).grid(row=0, column=1, padx=3, pady=3)
        tk.Button(dpad, text="◄ -X", command=lambda: self.nudge(-1, 0), **b_style).grid(row=1, column=0, padx=3, pady=3)
        tk.Button(dpad, text="● (0,0)", command=self.cmd_home, bg="#0284c7", fg="white", font=("Segoe UI", 8, "bold"), width=7, relief=tk.FLAT, cursor="hand2").grid(row=1, column=1, padx=3, pady=3)
        tk.Button(dpad, text="+X ►", command=lambda: self.nudge(1, 0), **b_style).grid(row=1, column=2, padx=3, pady=3)
        tk.Button(dpad, text="▼ -Y", command=lambda: self.nudge(0, -1), **b_style).grid(row=2, column=1, padx=3, pady=3)

    def build_telemetry_table(self, parent):
        box = tk.LabelFrame(
            parent,
            text=" Inverse Kinematics Motor & Cable Output ",
            font=("Segoe UI", 9, "bold"),
            fg=self.accent,
            bg=self.panel_bg,
            padx=10,
            pady=6,
        )
        box.pack(fill=tk.BOTH, expand=True)

        headers = ["Tendon", "Axis", "Shortening", "Motor Angle"]
        h_frame = tk.Frame(box, bg="#334155")
        h_frame.pack(fill=tk.X, pady=(0, 4))
        for col_name, w in zip(headers, [10, 8, 12, 12]):
            tk.Label(h_frame, text=col_name, font=("Segoe UI", 8, "bold"), fg=self.accent, bg="#334155", width=w, anchor="w").pack(side=tk.LEFT, padx=3, pady=2)

        self.telemetry_rows = []
        cables_info = [
            ("Cable 1", "+X (Right)", "#38bdf8"),
            ("Cable 2", "+Y (Forward)", "#a855f7"),
            ("Cable 3", "-X (Left)", "#f97316"),
            ("Cable 4", "-Y (Back)", "#10b981"),
        ]

        for c_name, axis_name, color in cables_info:
            r_frame = tk.Frame(box, bg=self.panel_bg)
            r_frame.pack(fill=tk.X, pady=2)

            l1 = tk.Label(r_frame, text=c_name, font=("Segoe UI", 8, "bold"), fg=color, bg=self.panel_bg, width=10, anchor="w")
            l1.pack(side=tk.LEFT, padx=3)

            l2 = tk.Label(r_frame, text=axis_name, font=("Segoe UI", 8), fg=self.text_dim, bg=self.panel_bg, width=8, anchor="w")
            l2.pack(side=tk.LEFT, padx=3)

            l3 = tk.Label(r_frame, text="0.0 mm", font=("Consolas", 8), fg=self.text_light, bg=self.panel_bg, width=12, anchor="w")
            l3.pack(side=tk.LEFT, padx=3)

            l4 = tk.Label(r_frame, text="0.0° (0.00 rad)", font=("Consolas", 8, "bold"), fg=self.text_light, bg=self.panel_bg, width=14, anchor="w")
            l4.pack(side=tk.LEFT, padx=3)

            self.telemetry_rows.append((l3, l4))

    def build_visualizer_panel(self, parent):
        box = tk.LabelFrame(
            parent,
            text=" Real-Time Workspace Visualizers (Click to Move) ",
            font=("Segoe UI", 9, "bold"),
            fg=self.accent,
            bg=self.panel_bg,
            padx=10,
            pady=8,
        )
        box.pack(fill=tk.BOTH, expand=True)

        nb = ttk.Notebook(box)
        nb.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Overhead 2D Baseplate View (Top-Down)
        tab_top = tk.Frame(nb, bg="#090d16")
        nb.add(tab_top, text=" 🎯 Overhead Table Arena (X-Y) ")

        self.canvas_top = tk.Canvas(tab_top, bg="#090d16", highlightthickness=0)
        self.canvas_top.pack(fill=tk.BOTH, expand=True)
        self.canvas_top.bind("<Button-1>", self.on_canvas_click)
        self.canvas_top.bind("<B1-Motion>", self.on_canvas_click)

        # Tab 2: Elevation Profile View (X-Z Side View)
        tab_side = tk.Frame(nb, bg="#090d16")
        nb.add(tab_side, text=" 📐 Elevation Profile (X-Z) ")

        self.canvas_side = tk.Canvas(tab_side, bg="#090d16", highlightthickness=0)
        self.canvas_side.pack(fill=tk.BOTH, expand=True)

    def nudge(self, dx_sign, dy_sign):
        step = self.step_size.get()
        nx = self.cmd_x.get() + dx_sign * step
        ny = self.cmd_y.get() + dy_sign * step
        self.cmd_x.set(max(-110.0, min(110.0, nx)))
        self.cmd_y.set(max(-110.0, min(110.0, ny)))
        self.on_slider_change()

    def cmd_home(self):
        self.cmd_x.set(0.0)
        self.cmd_y.set(0.0)
        self.on_slider_change()

    def on_slider_change(self):
        self.update_kinematics_display()
        if self.auto_send_var.get():
            self.send_to_robot()

    def on_canvas_click(self, event):
        """Allows user to click directly on the overhead canvas to move tip."""
        w = self.canvas_top.winfo_width()
        h = self.canvas_top.winfo_height()
        if w < 10 or h < 10:
            return

        cx, cy = w / 2.0, h / 2.0
        scale = min(w, h) / (self.kin.base_span * 1.08)

        # Invert from pixel to mm
        x_mm = (event.x - cx) / scale
        y_mm = -(event.y - cy) / scale  # screen +Y is down, world +Y is up

        x_clamped = max(-110.0, min(110.0, x_mm))
        y_clamped = max(-110.0, min(110.0, y_mm))

        self.cmd_x.set(round(x_clamped, 1))
        self.cmd_y.set(round(y_clamped, 1))
        self.on_slider_change()

    def update_kinematics_display(self):
        tx = self.cmd_x.get()
        ty = self.cmd_y.get()

        ik = self.kin.inverse_kinematics(tx, ty)

        # Update labels
        self.x_val_lbl.config(text=f"{tx:+.1f} mm")
        self.y_val_lbl.config(text=f"{ty:+.1f} mm")

        ach_x, ach_y, ach_z = ik["achieved_xyz"]
        self.z_val_lbl.config(text=f"{ach_z:.1f} mm (H = {self.kin.H:.0f} mm, Table = 0 mm)")

        # Update telemetry table
        m_deg = ik["motor_angles_deg"]
        m_rad = ik["motor_angles_rad"]
        pulls = ik["cable_pulls_mm"]

        for i in range(4):
            lbl_pull, lbl_motor = self.telemetry_rows[i]
            lbl_pull.config(text=f"{pulls[i]:.2f} mm")
            lbl_motor.config(text=f"{m_deg[i]:5.1f}° ({m_rad[i]:.2f} rad)")

        # Draw visualizers
        self.draw_top_view(ik)
        self.draw_side_view(ik)

    def send_to_robot(self):
        """Sends motor angles to physical/mock robot asynchronously."""
        if self.is_interpolating:
            return

        ik = self.kin.inverse_kinematics(self.cmd_x.get(), self.cmd_y.get())
        angles = ik["motor_angles_rad"]

        def _worker():
            self.is_interpolating = True
            try:
                self.robot.send_angles(angles)
            except Exception as e:
                print(f"[ERROR] Motor send failed: {e}")
            finally:
                self.is_interpolating = False

        threading.Thread(target=_worker, daemon=True).start()

    def draw_top_view(self, ik):
        """Draws top-down arena view showing 365mm baseplate, reach envelope, and green tip."""
        cv = self.canvas_top
        cv.delete("all")

        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 10 or h < 10:
            return

        cx, cy = w / 2.0, h / 2.0
        scale = min(w, h) / (self.kin.base_span * 1.08)

        # 1. Outer 365 mm Baseplate
        r_base = (self.kin.base_span / 2.0) * scale
        cv.create_oval(cx - r_base, cy - r_base, cx + r_base, cy + r_base, fill="#1e293b", outline="#475569", width=2)
        cv.create_text(cx, cy - r_base + 14, text=f"365 mm BASE PLATE", fill="#64748b", font=("Segoe UI", 8, "bold"))

        # 2. Maximum Reachable Workspace Circle
        r_reach = self.kin.max_reach_r * scale
        cv.create_oval(cx - r_reach, cy - r_reach, cx + r_reach, cy + r_reach, outline="#38bdf8", width=1, dash=(4, 4))
        cv.create_text(cx, cy + r_reach - 12, text=f"Max Reach r = {self.kin.max_reach_r:.0f} mm", fill="#38bdf8", font=("Segoe UI", 7))

        # 3. Coordinate Axes
        cv.create_line(cx - r_base, cy, cx + r_base, cy, fill="#334155", width=1)
        cv.create_line(cx, cy - r_base, cx, cy + r_base, fill="#334155", width=1)
        cv.create_text(cx + r_base - 14, cy - 8, text="+X", fill="#38bdf8", font=("Segoe UI", 8, "bold"))
        cv.create_text(cx + 8, cy - r_base + 14, text="+Y", fill="#a855f7", font=("Segoe UI", 8, "bold"))

        # 4. Continuum Backbone Projection (Spine line)
        pts = ik["spine_pts"]
        screen_pts = []
        for p in pts:
            sx = cx + p[0] * scale
            sy = cy - p[1] * scale
            screen_pts.extend([sx, sy])

        if len(screen_pts) >= 4:
            cv.create_line(*screen_pts, fill="#94a3b8", width=3, smooth=True)

        # Center Base Clamp
        cv.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#e2e8f0", outline="#ffffff", width=2)
        cv.create_text(cx, cy + 14, text="BASE (0,0)", fill="#cbd5e1", font=("Segoe UI", 7))

        # Green Magnetic End Effector Tip
        ach_x, ach_y, _ = ik["achieved_xyz"]
        tip_sx = cx + ach_x * scale
        tip_sy = cy - ach_y * scale

        # Target crosshair
        cv.create_line(tip_sx - 12, tip_sy, tip_sx + 12, tip_sy, fill=self.accent_green, width=1)
        cv.create_line(tip_sx, tip_sy - 12, tip_sx, tip_sy + 12, fill=self.accent_green, width=1)

        # Glowing green magnetic tip marker
        cv.create_oval(tip_sx - 10, tip_sy - 10, tip_sx + 10, tip_sy + 10, fill=self.accent_green, outline="#ffffff", width=2)
        cv.create_text(tip_sx, tip_sy - 16, text=f"TIP ({ach_x:+.1f}, {ach_y:+.1f})", fill=self.accent_green, font=("Segoe UI", 8, "bold"))

    def draw_side_view(self, ik):
        """Draws elevation profile view showing 280mm ceiling, descending 185mm arm, and table floor."""
        cv = self.canvas_side
        cv.delete("all")

        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 10 or h < 10:
            return

        cx = w / 2.0
        margin_y = 40
        scale = (h - 2 * margin_y) / (self.kin.H * 1.1)

        y_ceiling = margin_y
        y_table = margin_y + self.kin.H * scale

        # 1. Ceiling Mount (Z = 280 mm)
        cv.create_line(cx - 140, y_ceiling, cx + 140, y_ceiling, fill="#64748b", width=3)
        cv.create_text(cx, y_ceiling - 12, text="CEILING / TOP MOUNT (Z = 280 mm)", fill="#94a3b8", font=("Segoe UI", 8, "bold"))
        cv.create_oval(cx - 10, y_ceiling - 5, cx + 10, y_ceiling + 5, fill="#e2e8f0", outline="white", width=2)

        # 2. Table Floor (Z = 0 mm)
        cv.create_line(cx - 160, y_table, cx + 160, y_table, fill="#475569", width=3)
        cv.create_text(cx, y_table + 14, text="TABLE BASE PLATE (Z = 0 mm)", fill="#94a3b8", font=("Segoe UI", 8, "bold"))

        # Dimension Arrow for Machine Height
        cv.create_line(cx - 150, y_ceiling, cx - 150, y_table, fill="#38bdf8", width=1, arrow=tk.BOTH)
        cv.create_text(cx - 170, (y_ceiling + y_table) / 2.0, text="H = 280 mm", fill="#38bdf8", font=("Segoe UI", 7, "bold"))

        # 3. 3D Spine Curve Projected to Radial / Lateral View
        pts = ik["spine_pts"]
        screen_pts = []
        for p in pts:
            # Lateral deflection in plane of bending
            r = math.hypot(p[0], p[1]) * (1.0 if p[0] >= 0 else -1.0)
            sx = cx + r * scale
            sy = y_ceiling + (self.kin.H - p[2]) * scale
            screen_pts.extend([sx, sy])

        if len(screen_pts) >= 4:
            cv.create_line(*screen_pts, fill="#cbd5e1", width=4, smooth=True)

        # 4. Green End Effector Tip Marker
        ach_x, ach_y, ach_z = ik["achieved_xyz"]
        tip_r = math.hypot(ach_x, ach_y) * (1.0 if ach_x >= 0 else -1.0)
        tip_sx = cx + tip_r * scale
        tip_sy = y_ceiling + (self.kin.H - ach_z) * scale

        cv.create_oval(tip_sx - 9, tip_sy - 9, tip_sx + 9, tip_sy + 9, fill=self.accent_green, outline="#ffffff", width=2)
        cv.create_text(tip_sx + 24, tip_sy, text=f"Z = {ach_z:.1f} mm\n(L = {self.kin.L:.0f} mm)", fill=self.accent_green, font=("Segoe UI", 8, "bold"))


# =====================================================================
# 3. Standalone Execution Entry Point
# =====================================================================

def main():
    root = tk.Tk()
    app = CartesianControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
