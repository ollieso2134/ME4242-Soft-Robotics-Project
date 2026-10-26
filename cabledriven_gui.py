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
- Intel RealSense D405 Depth Camera live feed & continuum robot marker tracking
"""

import argparse
import math
import os
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

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
            self.camera = None
            self.camera_serial = None
        def connect(self, port=None): self.is_connected = True
        def disconnect(self): self.is_connected = False
        def connect_camera(self, camera_serial=None, track_markers=False): return False, "Mock"
        def disconnect_camera(self): pass
        def send_angles(self, a, *args, **kwargs): self.current_angles = a
        def move_smooth(self, a, duration=1.0): self.current_angles = a
        def home(self): self.current_angles = [0.0] * 4

# Import visual servoing, obstacle planner, and retrieval managers
from visual_servoing import (
    ColorDetector,
    BezierTrajectory,
    VisualServoController,
    PickAndPlaceManager,
    ObstacleEnvironment,
    ObstacleAidedRetriever,
    SimulatedObstacleCamera,
)

# Cross-platform Operating System & Font Detection
IS_WINDOWS = sys.platform.startswith("win") or os.name == "nt"
IS_LINUX = sys.platform.startswith("linux")
IS_MACOS = sys.platform == "darwin"

FONT_MAIN = "Segoe UI" if IS_WINDOWS else ("DejaVu Sans" if IS_LINUX else "Helvetica")
FONT_MONO = "Consolas" if IS_WINDOWS else ("DejaVu Sans Mono" if IS_LINUX else "Courier")


def list_serial_ports():
    """Detect all available serial/COM ports across Windows and Linux."""
    ports = []
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            ports.append(p.device)
    except Exception:
        pass

    # Linux-specific filesystem scan for USB/serial devices
    if not IS_WINDOWS:
        import glob
        for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
            if dev not in ports:
                ports.append(dev)

    if HAS_EMIO_API:
        try:
            from emioapi import EmioAPI
            emio_devs = EmioAPI.listEmioDevices()
            for d in emio_devs:
                if d not in ports:
                    ports.append(d)
        except Exception:
            pass

    if not ports:
        ports = ["COM12", "COM1", "COM3"] if IS_WINDOWS else ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1"]

    unique_ports = list(dict.fromkeys(ports))
    if IS_WINDOWS:
        return sorted(unique_ports, key=lambda x: (x.upper() != "COM12", x))
    else:
        return sorted(unique_ports, key=lambda x: (not x.startswith("/dev/ttyUSB"), x))


def list_camera_serials():
    """Detect all available Intel RealSense cameras via emioapi or pyrealsense2."""
    if HAS_EMIO_API:
        try:
            from emioapi import EmioCamera
            cams = EmioCamera.listCameras()
            if cams:
                return cams
        except Exception:
            pass
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        return [d.get_info(rs.camera_info.serial_number) for d in ctx.devices]
    except Exception:
        return []


class CableRobotGUI:
    def __init__(self, root, initial_port=None, initial_camera_serial=None, auto_connect_camera=False):
        self.root = root
        self.root.title("EMIO Cable-Driven Soft Robot & Vision Controller")
        self.root.geometry("1140x820")
        self.root.minsize(1050, 750)

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

        # Configure Combobox and Notebook style for dark theme
        self.style.configure(
            "TCombobox",
            fieldbackground="#1e1e24",
            background="#3a0ca3",
            foreground="white",
            arrowcolor="#4cc9f0",
        )
        self.style.configure("TNotebook", background=self.bg_dark, borderwidth=0)
        self.style.configure(
            "TNotebook.Tab",
            background="#2b2d42",
            foreground=self.text_color,
            padding=[12, 6],
            font=(FONT_MAIN, 9, "bold"),
        )
        self.style.map(
            "TNotebook.Tab",
            background=[("selected", "#3a0ca3")],
            foreground=[("selected", "#4cc9f0")],
        )

        # Robot Port setup
        available_ports = list_serial_ports()
        preferred_port = "COM12" if IS_WINDOWS else "/dev/ttyUSB0"
        default_port = initial_port or (preferred_port if preferred_port in available_ports else (available_ports[0] if available_ports else preferred_port))
        self.port_var = tk.StringVar(value=default_port)

        # Camera setup
        available_cams = list_camera_serials()
        default_cam = initial_camera_serial or (available_cams[0] if available_cams else "")
        self.camera_var = tk.StringVar(value=default_cam)
        self.camera_streaming = False
        self.track_markers_var = tk.BooleanVar(value=True)
        self.depth_view_var = tk.BooleanVar(value=False)
        self.cam_rotation_var = tk.StringVar(value="90_ccw")
        self.cam_photo = None

        # Hardware Controller
        self.robot = CableDrivenRobotController(min_angle=0.0, max_angle=math.radians(MAX_ANGLE_DEG), num_motors=4)
        self.is_moving_thread = False
        self.motor_vars = [tk.DoubleVar(value=0.0) for _ in range(4)]
        self.max_limit_var = tk.DoubleVar(value=MAX_ANGLE_DEG)  # Max angle limit in degrees

        # Obstacle-Aided Continuum Retrieval & Vision Managers
        self.obstacle_env = ObstacleEnvironment("Config 1: Dual-Peg S-Bend")
        self.color_detector = ColorDetector(min_contour_area=200)
        self.servo_controller = VisualServoController(self.robot, kp=0.0035, ki=0.0003, max_step_rad=0.08)
        self.retriever = ObstacleAidedRetriever(self.robot, self.color_detector, self.servo_controller, self.obstacle_env)
        self.pick_manager = self.retriever  # Backward compatible alias
        self.obstacle_preset_var = tk.StringVar(value="Config 1: Dual-Peg S-Bend")

        self.active_target_color = tk.StringVar(value="")  # "red", "blue", "yellow", or ""
        self.is_servoing_active = False
        self.servoing_thread = None
        self.invert_cam_x = tk.BooleanVar(value=False)
        self.invert_cam_y = tk.BooleanVar(value=False)
        self.latest_detections = {}
        self.latest_tip_pos = None
        self.latest_err_dist = None
        self.target_btn_widgets = {}

        self.setup_ui()
        self.connect_hardware(port=self.port_var.get())

        if auto_connect_camera or default_cam:
            self.connect_camera(serial=self.camera_var.get())

    def setup_ui(self):
        # Top Header & Connection Bar
        top_frame = tk.Frame(self.root, bg=self.panel_bg, padx=15, pady=8)
        top_frame.pack(fill=tk.X, padx=10, pady=(10, 5))

        title_frame = tk.Frame(top_frame, bg=self.panel_bg)
        title_frame.pack(side=tk.LEFT)

        title_lbl = tk.Label(
            title_frame,
            text="EMIO Soft Robot Controller",
            font=(FONT_MAIN, 13, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
        )
        title_lbl.pack(anchor="w")

        subtitle_lbl = tk.Label(
            title_frame,
            text="Continuum Actuation + RealSense D405 Vision",
            font=(FONT_MAIN, 8),
            fg="#94a3b8",
            bg=self.panel_bg,
        )
        subtitle_lbl.pack(anchor="w")

        # Right Header: 2 Connection Rows (Robot & Camera)
        right_header = tk.Frame(top_frame, bg=self.panel_bg)
        right_header.pack(side=tk.RIGHT)

        # Row 1: Robot Connection
        robot_row = tk.Frame(right_header, bg=self.panel_bg)
        robot_row.pack(fill=tk.X, pady=1)

        lbl_robot = tk.Label(
            robot_row,
            text="Robot:",
            font=(FONT_MAIN, 9, "bold"),
            fg=self.text_color,
            bg=self.panel_bg,
            width=7,
            anchor="e",
        )
        lbl_robot.pack(side=tk.LEFT, padx=(5, 3))

        self.port_combo = ttk.Combobox(
            robot_row,
            textvariable=self.port_var,
            values=list_serial_ports(),
            width=9,
            font=(FONT_MAIN, 9),
        )
        self.port_combo.pack(side=tk.LEFT, padx=2)

        btn_refresh_ports = tk.Button(
            robot_row,
            text="⟳",
            command=self.refresh_ports,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            padx=5,
            pady=1,
            font=(FONT_MAIN, 9, "bold"),
            cursor="hand2",
        )
        btn_refresh_ports.pack(side=tk.LEFT, padx=2)

        self.btn_connect = tk.Button(
            robot_row,
            text="Connect",
            command=self.toggle_connection,
            bg="#3a0ca3",
            fg="white",
            relief=tk.FLAT,
            padx=8,
            pady=1,
            font=(FONT_MAIN, 8, "bold"),
            cursor="hand2",
            width=12,
        )
        self.btn_connect.pack(side=tk.LEFT, padx=4)

        self.status_lbl = tk.Label(
            robot_row,
            text="● Disconnected",
            font=(FONT_MAIN, 9, "bold"),
            fg="#ef4444",
            bg=self.panel_bg,
            width=26,
            anchor="w",
        )
        self.status_lbl.pack(side=tk.LEFT, padx=4)

        # Row 2: Camera Connection
        cam_row = tk.Frame(right_header, bg=self.panel_bg)
        cam_row.pack(fill=tk.X, pady=1)

        lbl_cam = tk.Label(
            cam_row,
            text="Camera:",
            font=(FONT_MAIN, 9, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            width=7,
            anchor="e",
        )
        lbl_cam.pack(side=tk.LEFT, padx=(5, 3))

        detected_cams = list_camera_serials()
        self.cam_combo = ttk.Combobox(
            cam_row,
            textvariable=self.camera_var,
            values=detected_cams,
            width=9,
            font=(FONT_MAIN, 9),
        )
        self.cam_combo.pack(side=tk.LEFT, padx=2)

        btn_refresh_cams = tk.Button(
            cam_row,
            text="⟳",
            command=self.refresh_cameras,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            padx=5,
            pady=1,
            font=(FONT_MAIN, 9, "bold"),
            cursor="hand2",
        )
        btn_refresh_cams.pack(side=tk.LEFT, padx=2)

        self.btn_connect_cam = tk.Button(
            cam_row,
            text="Connect Cam",
            command=self.toggle_camera_connection,
            bg="#0284c7",
            fg="white",
            relief=tk.FLAT,
            padx=8,
            pady=1,
            font=(FONT_MAIN, 8, "bold"),
            cursor="hand2",
            width=12,
        )
        self.btn_connect_cam.pack(side=tk.LEFT, padx=4)

        self.cam_status_lbl = tk.Label(
            cam_row,
            text="● Disconnected",
            font=(FONT_MAIN, 9, "bold"),
            fg="#ef4444",
            bg=self.panel_bg,
            width=26,
            anchor="w",
        )
        self.cam_status_lbl.pack(side=tk.LEFT, padx=4)

        # Main Layout: 2 Columns (Left: Sliders & Controls, Right: Notebook with Visualizer & Camera)
        main_pane = tk.Frame(self.root, bg=self.bg_dark)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        left_frame = tk.Frame(main_pane, bg=self.bg_dark)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        right_frame = tk.Frame(main_pane, bg=self.bg_dark, width=420)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))

        # Notebook for Right Frame Tabs
        self.right_notebook = ttk.Notebook(right_frame)
        self.right_notebook.pack(fill=tk.BOTH, expand=True)

        tab_controls = tk.Frame(self.right_notebook, bg=self.panel_bg)
        tab_camera = tk.Frame(self.right_notebook, bg=self.panel_bg)
        tab_servoing = tk.Frame(self.right_notebook, bg=self.panel_bg)

        self.right_notebook.add(tab_controls, text=" 🎯 Tension & D-Pad ")
        self.right_notebook.add(tab_camera, text=" 📷 RealSense Vision ")
        self.right_notebook.add(tab_servoing, text=" 🧲 Obstacle Retrieval ")

        self.create_slider_controls(left_frame)
        self.create_visualization_panel(tab_controls)
        self.create_dpad_panel(tab_controls)
        self.create_camera_panel(tab_camera)
        self.create_servoing_panel(tab_servoing)
        self.create_motion_routines_panel(left_frame)
        self.create_bottom_bar()

    def create_slider_controls(self, parent):
        group = tk.LabelFrame(
            parent,
            text=f" Individual Motor / Cable Tension (0° - {MAX_ANGLE_DEG:g}°) ",
            font=(FONT_MAIN, 11, "bold"),
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
                font=(FONT_MAIN, 9, "bold"),
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
                font=(FONT_MONO, 9),
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
                font=(FONT_MAIN, 8),
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
                font=(FONT_MAIN, 8),
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
                font=(FONT_MAIN, 8),
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
                font=(FONT_MAIN, 8),
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
                font=(FONT_MAIN, 8, "bold"),
            )
            btn_zero.pack(side=tk.LEFT, padx=(3, 0))

    def create_visualization_panel(self, parent):
        group = tk.LabelFrame(
            parent,
            text=" Real-time Cable Tension Visualizer ",
            font=(FONT_MAIN, 11, "bold"),
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
            self.canvas.create_text(ax, ay + (14 if ay >= cy else -14), text=label, fill=color, font=(FONT_MAIN, 7))

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
            font=(FONT_MAIN, 11, "bold"),
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
            font=(FONT_MAIN, 9, "bold"),
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
            font=(FONT_MAIN, 9, "bold"),
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
            font=(FONT_MAIN, 9, "bold"),
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
            font=(FONT_MAIN, 9, "bold"),
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
            font=(FONT_MAIN, 9, "bold"),
        )
        btn_down.grid(row=2, column=1, padx=4, pady=4)

    def create_motion_routines_panel(self, parent):
        group = tk.LabelFrame(
            parent,
            text=" Automated Motion Routines & Tests ",
            font=(FONT_MAIN, 11, "bold"),
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
            font=(FONT_MAIN, 9),
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
            font=(FONT_MAIN, 9),
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
            font=(FONT_MAIN, 9),
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
            font=(FONT_MAIN, 9, "bold"),
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
            font=(FONT_MAIN, 11, "bold"),
            cursor="hand2",
        )
        btn_estop.pack(side=tk.LEFT)

        # Cartesian XYZ Task-Space Control Button
        btn_cartesian = tk.Button(
            bottom_frame,
            text="🕹️ Cartesian XYZ Control",
            command=self.open_cartesian_controller,
            bg="#0284c7",
            fg="white",
            relief=tk.FLAT,
            padx=12,
            pady=6,
            font=(FONT_MAIN, 10, "bold"),
            cursor="hand2",
        )
        btn_cartesian.pack(side=tk.LEFT, padx=10)

        # Safety Max Angle Limit
        limit_frame = tk.Frame(bottom_frame, bg=self.panel_bg)
        limit_frame.pack(side=tk.RIGHT)

        lbl_limit = tk.Label(
            limit_frame,
            text="Max Safe Angle Limit: ",
            fg=self.text_color,
            bg=self.panel_bg,
            font=(FONT_MAIN, 9),
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
            font=(FONT_MAIN, 9),
        )
        limit_spin.pack(side=tk.LEFT, padx=3)

    def open_cartesian_controller(self):
        """Launches the dedicated Cartesian XYZ task-space controller."""
        try:
            import cartesian_controller as cc
            win = tk.Toplevel(self.root)
            cc.CartesianControlGUI(win, controller=self.robot)
        except Exception as e:
            messagebox.showerror("Cartesian Controller", f"Could not open Cartesian Controller: {e}")

    def create_camera_panel(self, parent):
        group = tk.LabelFrame(
            parent,
            text=" Intel RealSense D405 Live Video & Tracking ",
            font=(FONT_MAIN, 11, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            padx=10,
            pady=8,
        )
        group.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Controls bar
        ctrl_frame = tk.Frame(group, bg=self.panel_bg)
        ctrl_frame.pack(fill=tk.X, pady=(0, 6))

        chk_track = tk.Checkbutton(
            ctrl_frame,
            text="Track Markers",
            variable=self.track_markers_var,
            command=self.on_toggle_track_markers,
            bg=self.panel_bg,
            fg=self.text_color,
            selectcolor="#1e1e24",
            activebackground=self.panel_bg,
            activeforeground=self.accent_color,
            font=(FONT_MAIN, 9),
        )
        chk_track.pack(side=tk.LEFT, padx=3)

        chk_depth = tk.Checkbutton(
            ctrl_frame,
            text="Depth View",
            variable=self.depth_view_var,
            bg=self.panel_bg,
            fg=self.text_color,
            selectcolor="#1e1e24",
            activebackground=self.panel_bg,
            activeforeground=self.accent_color,
            font=(FONT_MAIN, 9),
        )
        chk_depth.pack(side=tk.LEFT, padx=3)

        lbl_rot = tk.Label(
            ctrl_frame,
            text="Rot:",
            font=(FONT_MAIN, 9, "bold"),
            fg=self.text_color,
            bg=self.panel_bg,
        )
        lbl_rot.pack(side=tk.LEFT, padx=(5, 1))

        rot_combo = ttk.Combobox(
            ctrl_frame,
            textvariable=self.cam_rotation_var,
            values=["90_ccw", "180", "90_cw", "0"],
            width=7,
            font=(FONT_MAIN, 8),
            state="readonly",
        )
        rot_combo.pack(side=tk.LEFT, padx=1)

        btn_snap = tk.Button(
            ctrl_frame,
            text="📷 Snapshot",
            command=self.capture_snapshot,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            padx=6,
            pady=1,
            font=(FONT_MAIN, 8),
            cursor="hand2",
        )
        btn_snap.pack(side=tk.RIGHT, padx=2)

        btn_popout = tk.Button(
            ctrl_frame,
            text="⛶ Popout Window",
            command=self.popout_camera,
            bg="#3a0ca3",
            fg="white",
            relief=tk.FLAT,
            padx=6,
            pady=1,
            font=(FONT_MAIN, 8, "bold"),
            cursor="hand2",
        )
        btn_popout.pack(side=tk.RIGHT, padx=2)

        # Video canvas
        self.cam_width = 380
        self.cam_height = 285
        self.cam_canvas = tk.Canvas(
            group,
            width=self.cam_width,
            height=self.cam_height,
            bg="#111217",
            highlightthickness=1,
            highlightbackground="#374151",
        )
        self.cam_canvas.pack(pady=4)
        self.cam_canvas_text = self.cam_canvas.create_text(
            self.cam_width // 2,
            self.cam_height // 2,
            text="Camera Disconnected\nSelect Camera above and click 'Connect Cam'",
            fill="#9ca3af",
            justify=tk.CENTER,
            font=(FONT_MAIN, 10),
        )

        # Tracking readout
        readout_frame = tk.Frame(group, bg="#111217", padx=8, pady=6)
        readout_frame.pack(fill=tk.X, pady=(4, 0))

        lbl_track_title = tk.Label(
            readout_frame,
            text="Continuum Tip / Marker Coordinates (X, Y, Z mm):",
            font=(FONT_MAIN, 8, "bold"),
            fg="#94a3b8",
            bg="#111217",
            anchor="w",
        )
        lbl_track_title.pack(fill=tk.X)

        self.lbl_marker_pos = tk.Label(
            readout_frame,
            text="Standby...",
            font=(FONT_MONO, 8),
            fg="#38bdf8",
            bg="#111217",
            anchor="w",
        )
        self.lbl_marker_pos.pack(fill=tk.X)

        # Target Selection & Bézier Visual Servoing Controls
        servo_group = tk.LabelFrame(
            group,
            text=" 🎯 Bézier Visual Servoing (Red/Blue/Yellow) ",
            font=(FONT_MAIN, 9, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            padx=6,
            pady=4,
        )
        servo_group.pack(fill=tk.X, pady=(6, 0))

        # Row 1: Target Color Selection Buttons
        tgt_btn_row = tk.Frame(servo_group, bg=self.panel_bg)
        tgt_btn_row.pack(fill=tk.X, pady=2)

        self.btn_tgt_red = tk.Button(
            tgt_btn_row,
            text="🔴 Red (N)",
            command=lambda: self.select_target_color("red"),
            bg="#b91c1c",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=4,
            pady=3,
            cursor="hand2",
        )
        self.btn_tgt_red.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_tgt_blue = tk.Button(
            tgt_btn_row,
            text="🔵 Blue (U)",
            command=lambda: self.select_target_color("blue"),
            bg="#1d4ed8",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=4,
            pady=3,
            cursor="hand2",
        )
        self.btn_tgt_blue.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_tgt_yellow = tk.Button(
            tgt_btn_row,
            text="🟡 Yellow (S)",
            command=lambda: self.select_target_color("yellow"),
            bg="#b45309",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=4,
            pady=3,
            cursor="hand2",
        )
        self.btn_tgt_yellow.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_tgt_clear = tk.Button(
            tgt_btn_row,
            text="✕ Clear",
            command=lambda: self.select_target_color(None),
            bg="#4b5563",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8),
            padx=4,
            pady=3,
            cursor="hand2",
        )
        self.btn_tgt_clear.pack(side=tk.LEFT, padx=2)

        # Row 2: Action Buttons & Inversion
        action_row = tk.Frame(servo_group, bg=self.panel_bg)
        action_row.pack(fill=tk.X, pady=3)

        self.btn_follow_target = tk.Button(
            action_row,
            text="▶ Follow Target",
            command=self.toggle_follow_target,
            bg="#059669",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=6,
            pady=3,
            cursor="hand2",
        )
        self.btn_follow_target.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_pick_place_quick = tk.Button(
            action_row,
            text="★ Pick & Place",
            command=self.execute_active_pick_and_place,
            bg="#7c3aed",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=6,
            pady=3,
            cursor="hand2",
        )
        self.btn_pick_place_quick.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        chk_inv_x = tk.Checkbutton(
            action_row,
            text="Inv X",
            variable=self.invert_cam_x,
            bg=self.panel_bg,
            fg=self.text_color,
            selectcolor="#1e1e24",
            activebackground=self.panel_bg,
            activeforeground=self.accent_color,
            font=(FONT_MAIN, 7),
        )
        chk_inv_x.pack(side=tk.LEFT, padx=1)

        chk_inv_y = tk.Checkbutton(
            action_row,
            text="Inv Y",
            variable=self.invert_cam_y,
            bg=self.panel_bg,
            fg=self.text_color,
            selectcolor="#1e1e24",
            activebackground=self.panel_bg,
            activeforeground=self.accent_color,
            font=(FONT_MAIN, 7),
        )
        chk_inv_y.pack(side=tk.LEFT, padx=1)

        # Row 3: Status readout
        self.lbl_servo_status = tk.Label(
            servo_group,
            text="● Servoing Idle: Select target color and click 'Follow Target'",
            font=(FONT_MONO, 8),
            fg="#94a3b8",
            bg=self.panel_bg,
            anchor="w",
        )
        self.lbl_servo_status.pack(fill=tk.X, pady=(1, 2))

    def create_servoing_panel(self, parent):
        """Creates the dedicated Obstacle-Aided Multi-Color Retrieval Controller tab."""
        group = tk.LabelFrame(
            parent,
            text=" Obstacle-Aided Multi-Curvature Retrieval Controller ",
            font=(FONT_MAIN, 11, "bold"),
            fg=self.accent_color,
            bg=self.panel_bg,
            padx=10,
            pady=6,
        )
        group.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Header description
        info_lbl = tk.Label(
            group,
            text="Dual-Bend Obstacle Contact Actuation + Visual Servoing + Slot Stripping\n"
                 "Targets: Red ('N') ➔ Blue ('U') ➔ Yellow ('S') | Green Magnetic Tip",
            font=(FONT_MAIN, 8),
            fg="#94a3b8",
            bg=self.panel_bg,
            justify=tk.LEFT,
        )
        info_lbl.pack(anchor="w", pady=(0, 4))

        # Row 1: Obstacle Configuration Preset Selector & Mission Benchmark
        preset_frame = tk.Frame(group, bg=self.panel_bg)
        preset_frame.pack(fill=tk.X, pady=(0, 4))

        lbl_preset = tk.Label(
            preset_frame,
            text="Obstacle Config:",
            font=(FONT_MAIN, 9, "bold"),
            fg=self.text_color,
            bg=self.panel_bg,
        )
        lbl_preset.pack(side=tk.LEFT, padx=(2, 4))

        preset_combo = ttk.Combobox(
            preset_frame,
            textvariable=self.obstacle_preset_var,
            values=[
                "Config 1: Dual-Peg S-Bend",
                "Config 2: Lateral Deflector",
                "Config 3: Corridor",
            ],
            width=22,
            font=(FONT_MAIN, 8),
            state="readonly",
        )
        preset_combo.pack(side=tk.LEFT, padx=2)
        preset_combo.bind("<<ComboboxSelected>>", self.on_change_obstacle_preset)

        self.lbl_benchmark = tk.Label(
            preset_frame,
            text="Benchmark: <1m=3pts",
            font=(FONT_MONO, 8, "bold"),
            fg="#4ade80",
            bg=self.panel_bg,
        )
        self.lbl_benchmark.pack(side=tk.RIGHT, padx=4)

        # Row 2: 3-Target Status Dashboard Cards
        cards_frame = tk.Frame(group, bg=self.panel_bg)
        cards_frame.pack(fill=tk.X, pady=(0, 4))

        # Target N Card
        self.card_n = tk.Label(
            cards_frame,
            text="🔴 Target N (Red)\n[PENDING]",
            font=(FONT_MAIN, 8, "bold"),
            fg="white",
            bg="#7f1d1d",
            relief=tk.GROOVE,
            padx=4,
            pady=3,
            width=14,
        )
        self.card_n.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Target U Card
        self.card_u = tk.Label(
            cards_frame,
            text="🔵 Target U (Blue)\n[PENDING]",
            font=(FONT_MAIN, 8, "bold"),
            fg="white",
            bg="#1e3a8a",
            relief=tk.GROOVE,
            padx=4,
            pady=3,
            width=14,
        )
        self.card_u.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Target S Card
        self.card_s = tk.Label(
            cards_frame,
            text="🟡 Target S (Yellow)\n[PENDING]",
            font=(FONT_MAIN, 8, "bold"),
            fg="white",
            bg="#78350f",
            relief=tk.GROOVE,
            padx=4,
            pady=3,
            width=14,
        )
        self.card_s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Row 3: Main Autonomous Action Bar
        action_frame = tk.LabelFrame(
            group,
            text=" Mission Execution ",
            font=(FONT_MAIN, 9, "bold"),
            fg="#38bdf8",
            bg=self.panel_bg,
            padx=6,
            pady=4,
        )
        action_frame.pack(fill=tk.X, pady=(0, 4))

        # Big Button: Full 3-Target Sequence
        self.btn_full_mission = tk.Button(
            action_frame,
            text="★ RUN FULL 3-TARGET OBSTACLE RETRIEVAL ('N' ➔ 'U' ➔ 'S')",
            command=self.execute_full_obstacle_sequence,
            bg="#7209b7",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 9, "bold"),
            pady=4,
            cursor="hand2",
        )
        self.btn_full_mission.pack(fill=tk.X, pady=(0, 3))

        # Single target retrieval buttons
        singles_row = tk.Frame(action_frame, bg=self.panel_bg)
        singles_row.pack(fill=tk.X)

        btn_pick_n = tk.Button(
            singles_row,
            text="Retrieve 'N' (Red)",
            command=lambda: self.execute_single_obstacle_retrieval("red"),
            bg="#b91c1c",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=3,
            pady=3,
            cursor="hand2",
        )
        btn_pick_n.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        btn_pick_u = tk.Button(
            singles_row,
            text="Retrieve 'U' (Blue)",
            command=lambda: self.execute_single_obstacle_retrieval("blue"),
            bg="#1d4ed8",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=3,
            pady=3,
            cursor="hand2",
        )
        btn_pick_u.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        btn_pick_s = tk.Button(
            singles_row,
            text="Retrieve 'S' (Yellow)",
            command=lambda: self.execute_single_obstacle_retrieval("yellow"),
            bg="#b45309",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 8, "bold"),
            padx=3,
            pady=3,
            cursor="hand2",
        )
        btn_pick_s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        # Row 4: Step-by-Step Obstacle Workflow Verification
        step_frame = tk.LabelFrame(
            group,
            text=" Step-by-Step Obstacle Workflow Testing ",
            font=(FONT_MAIN, 8, "bold"),
            fg="#94a3b8",
            bg=self.panel_bg,
            padx=4,
            pady=2,
        )
        step_frame.pack(fill=tk.X, pady=(0, 4))

        step_row1 = tk.Frame(step_frame, bg=self.panel_bg)
        step_row1.pack(fill=tk.X, pady=1)

        tk.Button(
            step_row1,
            text="1. Enter Obstacles",
            command=self.step_enter_obstacles,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 7),
            cursor="hand2",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            step_row1,
            text="2. Dual-Bend Contact",
            command=self.step_contact_bend,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 7),
            cursor="hand2",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            step_row1,
            text="3. Magnet Pickup",
            command=self.step_magnet_pickup,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 7),
            cursor="hand2",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        step_row2 = tk.Frame(step_frame, bg=self.panel_bg)
        step_row2.pack(fill=tk.X, pady=1)

        tk.Button(
            step_row2,
            text="4. Withdraw Obstacles",
            command=self.step_withdraw_obstacles,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 7),
            cursor="hand2",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            step_row2,
            text="5. Deliver to Slot",
            command=self.step_deliver_slot,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 7),
            cursor="hand2",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            step_row2,
            text="6. Slot Detach Wipe",
            command=self.step_slot_wipe,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 7),
            cursor="hand2",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            step_row2,
            text="7. Return to Obstacles",
            command=self.step_return_obstacles,
            bg="#374151",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 7),
            cursor="hand2",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        # Row 5: Launch 3D Continuum Simulator
        sim3d_btn = tk.Button(
            group,
            text="🌐 Launch Interactive 3D Continuum Space Simulator",
            command=self.launch_3d_simulator,
            bg="#4338ca",
            fg="white",
            relief=tk.FLAT,
            font=(FONT_MAIN, 9, "bold"),
            padx=4,
            pady=4,
            cursor="hand2",
        )
        sim3d_btn.pack(fill=tk.X, pady=(3, 4))

        # Row 6: Sequence Activity Log
        log_frame = tk.LabelFrame(
            group,
            text=" Mission Activity Log & Diagnostics ",
            font=(FONT_MAIN, 8, "bold"),
            fg="#38bdf8",
            bg=self.panel_bg,
            padx=4,
            pady=2,
        )
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.seq_log_text = tk.Text(
            log_frame,
            height=6,
            bg="#111217",
            fg="#4ade80",
            font=(FONT_MONO, 8),
            relief=tk.FLAT,
            wrap=tk.WORD,
        )
        self.seq_log_text.pack(fill=tk.BOTH, expand=True)
        self.log_seq_message("Obstacle-aided continuum retrieval ready. Select config or click Run Full Sequence.")

    # -------------------------------------------------------------
    # Action Handlers
    # -------------------------------------------------------------

    def refresh_ports(self):
        """Re-scan for connected COM/serial ports and update the combobox dropdown."""
        ports = list_serial_ports()
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def refresh_cameras(self):
        """Re-scan for connected RealSense camera devices."""
        cams = list_camera_serials()
        self.cam_combo["values"] = cams
        if cams and self.camera_var.get() not in cams:
            self.camera_var.set(cams[0])

    def toggle_connection(self):
        """Toggle robot connection between connect and disconnect."""
        if self.robot.is_connected and not self.robot.is_mock:
            self.robot.disconnect()
            self.status_lbl.config(text="● Disconnected", fg="#ef4444")
            self.btn_connect.config(text="Connect", bg="#3a0ca3")
        else:
            self.connect_hardware()

    def connect_hardware(self, port=None):
        """Connect to the selected robot port in a background thread and sync GUI sliders."""
        if port is None:
            port = self.port_var.get().strip()
        if not port:
            port = None

        self.btn_connect.config(state=tk.DISABLED)
        self.status_lbl.config(
            text=f"● Connecting ({port or 'auto'})...",
            fg="#38bdf8",
        )

        def _connect():
            try:
                self.robot.connect(port=port)
            except Exception as e:
                print(f"[ERROR] Connection attempt failed: {e}")

            def _update_ui():
                self.btn_connect.config(state=tk.NORMAL)
                if self.robot.is_mock:
                    err = getattr(self.robot, "last_connection_error", "") or ""
                    if "dialout" in err or "PermissionError" in err or "access" in err.lower():
                        self.status_lbl.config(
                            text="● Port Permission Denied",
                            fg="#ef4444",
                        )
                        if IS_LINUX:
                            messagebox.showwarning(
                                "Serial Port Permission Required",
                                f"Could not access {port or 'EMIO port'}: Permission denied.\n\n"
                                "To fix this on Linux, run in a terminal:\n"
                                "   sudo usermod -aG dialout $USER\n\n"
                                "Then log out and log back in (or run 'sudo chmod 666 /dev/ttyUSB*').\n\n"
                                "Running in Simulation / Mock mode for now."
                            )
                    elif "Access is denied" in err:
                        self.status_lbl.config(
                            text="● Port Busy / Occupied",
                            fg="#ef4444",
                        )
                        if IS_WINDOWS:
                            messagebox.showwarning(
                                "Port Busy",
                                f"Port {port or 'COM'} is currently locked by another application.\n"
                                "Ensure any running Python terminal or background script is closed."
                            )
                    else:
                        self.status_lbl.config(
                            text="● Simulation / Mock Mode",
                            fg="#fbbf24",
                        )
                    self.btn_connect.config(text="Connect Hardware", bg="#3a0ca3")
                else:
                    active_port = getattr(self.robot, "port", port or "EMIO")
                    self.status_lbl.config(
                        text=f"● Connected: {active_port}",
                        fg=self.green_color,
                    )
                    self.btn_connect.config(text="Disconnect", bg="#b91c1c")
                    # Synchronize sliders and labels with actual current motor angles
                    for i, rad in enumerate(self.robot.current_angles):
                        deg = math.degrees(rad)
                        clamped_deg = max(0.0, min(self.max_limit_var.get(), deg))
                        self.motor_vars[i].set(clamped_deg)
                        self.val_labels[i].config(text=f"{deg:5.1f}° ({rad:.2f} rad)")
                    self.draw_canvas_visualizer()

            self.root.after(0, _update_ui)

        threading.Thread(target=_connect, daemon=True).start()

    def toggle_camera_connection(self):
        """Toggle camera connection on/off."""
        if self.robot.camera is not None and getattr(self.robot.camera, "is_running", False):
            self.disconnect_camera()
        else:
            self.connect_camera(serial=self.camera_var.get())

    def connect_camera(self, serial=None):
        """Connect to RealSense camera in background thread."""
        if not serial:
            serial = self.camera_var.get().strip() or None

        self.btn_connect_cam.config(state=tk.DISABLED)
        self.cam_status_lbl.config(
            text=f"● Connecting ({serial or 'auto'})...",
            fg="#38bdf8",
        )

        def _worker():
            success, msg = self.robot.connect_camera(
                camera_serial=serial,
                track_markers=self.track_markers_var.get(),
            )

            def _update_ui():
                self.btn_connect_cam.config(state=tk.NORMAL)
                if success and self.robot.camera is not None:
                    active_serial = self.robot.camera_serial or serial or "D405"
                    self.cam_status_lbl.config(
                        text=f"● Connected: {active_serial}",
                        fg=self.green_color,
                    )
                    self.btn_connect_cam.config(text="Disconnect Cam", bg="#b91c1c")
                    # Switch to camera tab in right notebook
                    try:
                        self.right_notebook.select(1)
                    except Exception:
                        pass
                    self.start_camera_stream()
                else:
                    self.cam_status_lbl.config(
                        text=f"● Failed ({msg[:16]})",
                        fg="#ef4444",
                    )
                    self.btn_connect_cam.config(text="Connect Cam", bg="#0284c7")

            self.root.after(0, _update_ui)

        threading.Thread(target=_worker, daemon=True).start()

    def disconnect_camera(self):
        """Disconnect camera and reset canvas."""
        self.camera_streaming = False
        self.robot.disconnect_camera()
        self.cam_status_lbl.config(text="● Disconnected", fg="#ef4444")
        self.btn_connect_cam.config(text="Connect Cam", bg="#0284c7")
        self.cam_canvas.delete("all")
        self.cam_canvas.create_text(
            self.cam_width // 2,
            self.cam_height // 2,
            text="Camera Disconnected\nSelect Camera above and click 'Connect Cam'",
            fill="#9ca3af",
            justify=tk.CENTER,
            font=(FONT_MAIN, 10),
        )
        self.lbl_marker_pos.config(text="Standby...", fg="#9ca3af")

    def start_camera_stream(self):
        """Start periodic GUI frame updates."""
        self.camera_streaming = True
        self.update_camera_frame()

    def update_camera_frame(self):
        """Grabs latest frame from RealSense and renders to GUI canvas with multi-color Bézier HUD."""
        if not self.camera_streaming or self.robot.camera is None:
            return

        try:
            self.robot.camera.update()
            raw_frame = self.robot.camera.frame
            if raw_frame is not None:
                if self.depth_view_var.get() and self.robot.camera.depth_frame is not None:
                    depth = self.robot.camera.depth_frame
                    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    bgr = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                else:
                    bgr = raw_frame

                # Apply camera rotation (default: 90 deg CCW)
                rot = self.cam_rotation_var.get()
                if rot == "90_ccw":
                    bgr = cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif rot == "180":
                    bgr = cv2.rotate(bgr, cv2.ROTATE_180)
                elif rot == "90_cw":
                    bgr = cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)

                # Detect Red (N), Blue (U), Yellow (S), and Green tip
                self.latest_detections = self.color_detector.detect_all_targets(bgr)

                tip_pos = None
                if "orange" in self.latest_detections:
                    tip_pos = self.latest_detections["orange"]["centroid"]
                elif getattr(self.robot.camera, "trackers_pos", []):
                    tip_pos = (bgr.shape[1] // 2, int(bgr.shape[0] * 0.45))
                else:
                    tip_pos = (bgr.shape[1] // 2, int(bgr.shape[0] * 0.4))
                self.latest_tip_pos = tip_pos

                target_col = self.active_target_color.get()
                err_dist = None
                if target_col and target_col in self.latest_detections and tip_pos:
                    tx, ty = self.latest_detections[target_col]["centroid"]
                    err_dist = math.hypot(tx - tip_pos[0], ty - tip_pos[1])
                self.latest_err_dist = err_dist

                # Render rich HUD overlay with Obstacles, Slots, Bézier curve onto bgr
                m_time = (time.time() - self.retriever.mission_start_time) if getattr(self.retriever, "mission_start_time", None) else None
                active_state = self.retriever.current_state if getattr(self.retriever, "is_running", False) else state_str
                annotated = self.color_detector.draw_hud(
                    bgr,
                    self.latest_detections,
                    target_color=self.retriever.active_color or (target_col if target_col else None),
                    tip_pos=tip_pos,
                    error_dist=err_dist,
                    state_text=active_state,
                    obstacle_env=self.obstacle_env,
                    target_statuses=self.retriever.target_statuses,
                    mission_time=m_time,
                )

                rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
                h, w = rgb.shape[:2]
                scale = min(self.cam_width / w, self.cam_height / h)
                target_w = max(1, int(w * scale))
                target_h = max(1, int(h * scale))
                resized = cv2.resize(rgb, (target_w, target_h))

                img = Image.fromarray(resized)
                self.cam_photo = ImageTk.PhotoImage(image=img)
                self.cam_canvas.delete("all")
                self.cam_canvas.create_image(self.cam_width // 2, self.cam_height // 2, image=self.cam_photo)

                # Coordinate readout
                if target_col and target_col in self.latest_detections:
                    tdet = self.latest_detections[target_col]
                    tcx, tcy = tdet["centroid"]
                    pos_3d = self.color_detector.get_3d_coordinates(
                        tcx, tcy, getattr(self.robot.camera, "depth_frame", None)
                    )
                    z_str = f"| Z:{pos_3d[2]:.0f}mm" if pos_3d else ""
                    self.lbl_marker_pos.config(
                        text=f"Target: {tdet['label']} ({tcx}, {tcy}) {z_str} | Err: {err_dist:.1f}px",
                        fg=self.accent_color,
                    )
                elif "orange" in self.latest_detections:
                    gx, gy = self.latest_detections["orange"]["centroid"]
                    self.lbl_marker_pos.config(text=f"Arm Tip: ({gx}, {gy}) | Standby for Target", fg=self.green_color)
                else:
                    trackers = getattr(self.robot.camera, "trackers_pos", [])
                    if trackers:
                        coords = " | ".join(
                            f"M{i+1}: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})" for i, p in enumerate(trackers[:3])
                        )
                        self.lbl_marker_pos.config(text=coords, fg=self.green_color)
                    else:
                        self.lbl_marker_pos.config(text="Standby: Searching for Red/Blue/Yellow targets...", fg="#94a3b8")
        except Exception:
            pass

        if self.camera_streaming:
            self.root.after(33, self.update_camera_frame)  # ~30 FPS

    def select_target_color(self, color_name):
        """Sets the active target color ('red', 'blue', 'yellow', or None)."""
        self.active_target_color.set(color_name or "")
        
        # Reset visual styles of buttons
        colors = {
            "red": (self.btn_tgt_red, "#b91c1c", "#ef4444"),
            "blue": (self.btn_tgt_blue, "#1d4ed8", "#3b82f6"),
            "yellow": (self.btn_tgt_yellow, "#b45309", "#f59e0b"),
        }
        for c, (btn, norm_col, active_col) in colors.items():
            if c == color_name:
                btn.config(bg=active_col, relief=tk.SUNKEN)
            else:
                btn.config(bg=norm_col, relief=tk.FLAT)

        if color_name:
            label = self.color_detector.COLOR_RANGES[color_name]["label"]
            self.lbl_servo_status.config(
                text=f"● Locked onto {label}. Click 'Follow Target' to track.",
                fg=self.green_color,
            )
            self.log_seq_message(f"Selected target: {label}")
        else:
            self.lbl_servo_status.config(
                text="● Standby: Select a target color below.",
                fg="#94a3b8",
            )
            self.log_seq_message("Target cleared.")

    def toggle_follow_target(self):
        """Toggles real-time closed-loop visual servoing to follow the targeted color."""
        if self.is_servoing_active:
            self.stop_servoing()
        else:
            target_col = self.active_target_color.get()
            if not target_col:
                messagebox.showinfo("Target Color", "Please select a target color (Red, Blue, or Yellow) first.")
                return
            if not self.camera_streaming or self.robot.camera is None:
                messagebox.showinfo("Camera Required", "Please connect the camera first to use visual servoing.")
                return

            self.is_servoing_active = True
            self.btn_follow_target.config(text="⏹ Stop Following", bg="#b91c1c")
            self.lbl_servo_status.config(text=f"● Active PBVS: Following {target_col.upper()}...", fg="#38bdf8")
            self.log_seq_message(f"Started Visual Servoing tracking for {target_col.upper()}.")

            # Reset integral error
            self.servo_controller.reset_integral()
            self.servo_controller.kp = self.kp_var.get()
            self.servo_controller.max_step_rad = self.max_step_var.get()

            # Start background thread
            threading.Thread(target=self._servoing_worker, daemon=True).start()

    def stop_servoing(self):
        """Stops visual servoing loop."""
        self.is_servoing_active = False
        self.btn_follow_target.config(text="▶ Follow Target", bg="#059669")
        self.lbl_servo_status.config(text="● Servoing Stopped.", fg="#94a3b8")
        self.log_seq_message("Visual Servoing stopped.")

    def _servoing_worker(self):
        """High-frequency background PBVS control loop."""
        while self.is_servoing_active:
            target_col = self.active_target_color.get()
            if not target_col:
                break

            self.servo_controller.sign_x = -1.0 if self.invert_cam_x.get() else 1.0
            self.servo_controller.sign_y = -1.0 if self.invert_cam_y.get() else 1.0

            det = self.latest_detections.get(target_col)
            tip_pos = self.latest_tip_pos

            if det and tip_pos:
                target_pt = det["centroid"]
                new_angles, dist, converged = self.servo_controller.compute_step(tip_pos, target_pt)
                self.robot.send_angles(new_angles)

                # Sync sliders on GUI
                def _sync(angles=new_angles, d=dist, conv=converged):
                    for i, rad in enumerate(angles):
                        deg = math.degrees(rad)
                        self.motor_vars[i].set(max(0.0, min(self.max_limit_var.get(), deg)))
                        self.val_labels[i].config(text=f"{deg:5.1f}° ({rad:.2f} rad)")
                    self.draw_canvas_visualizer()
                    if conv or d < 8.0:
                        self.lbl_servo_status.config(text=f"● AT TARGET (|e|={d:.1f}px)", fg=self.green_color)
                    else:
                        self.lbl_servo_status.config(text=f"● SERVOING (|e|={d:.1f}px)", fg="#38bdf8")

                self.root.after(0, _sync)

            time.sleep(0.035)  # ~28 Hz control loop

    def execute_active_pick_and_place(self):
        """Executes pick and place for currently selected target color."""
        target_col = self.active_target_color.get()
        if not target_col:
            messagebox.showinfo("Target Color", "Please select a target color (Red, Blue, or Yellow) first.")
            return
        self.execute_single_obstacle_retrieval(target_col)

    def on_change_obstacle_preset(self, event=None):
        """Switches active obstacle geometry preset."""
        preset = self.obstacle_preset_var.get()
        self.obstacle_env.configure_preset(preset)
        self.log_seq_message(f"Switched obstacle preset to: {preset}")

    def update_target_cards(self):
        """Updates GUI target status dashboard cards."""
        statuses = getattr(self.retriever, "target_statuses", {})

        def _style(card, color_name, letter_name, bg_norm, bg_act, bg_done):
            st = statuses.get(color_name, "PENDING")
            if st == "SLOTTED":
                card.config(text=f"✓ Target {letter_name} ({color_name.capitalize()})\n[SLOTTED]", bg=bg_done)
            elif st == "ACTIVE":
                card.config(text=f"▶ Target {letter_name} ({color_name.capitalize()})\n[RETRIEVING...]", bg=bg_act)
            else:
                card.config(text=f"○ Target {letter_name} ({color_name.capitalize()})\n[PENDING]", bg=bg_norm)

        if hasattr(self, "card_n"):
            _style(self.card_n, "red", "N", "#7f1d1d", "#dc2626", "#15803d")
            _style(self.card_u, "blue", "U", "#1e3a8a", "#2563eb", "#15803d")
            _style(self.card_s, "yellow", "S", "#78350f", "#d97706", "#15803d")

    def execute_single_obstacle_retrieval(self, color_name):
        """Runs autonomous obstacle retrieval for a single target letter."""
        if self.is_moving_thread or self.retriever.is_running:
            messagebox.showinfo("Busy", "A routine is already running.")
            return

        try:
            self.right_notebook.select(2)
        except Exception:
            pass

        def _runner():
            self.is_moving_thread = True
            letter = self.obstacle_env.color_to_letter.get(color_name, "?")
            self.log_seq_message(f"=== Starting Obstacle-Aided Retrieval for '{letter}' ({color_name.upper()}) ===")

            orig_send = self.robot.send_angles
            def sync_send(angles, *args, **kwargs):
                orig_send(angles, *args, **kwargs)
                for idx, rad in enumerate(angles):
                    deg = math.degrees(rad)
                    self.motor_vars[idx].set(max(0.0, min(self.max_limit_var.get(), deg)))
                    self.val_labels[idx].config(text=f"{deg:5.1f}° ({rad:.2f} rad)")
                self.draw_canvas_visualizer()
            self.robot.send_angles = sync_send

            try:
                self.retriever.retrieve_single_target(
                    color_name,
                    progress_callback=lambda st, msg: self.root.after(0, lambda: (self.log_seq_message(f"[{st}] {msg}"), self.update_target_cards())),
                )
            finally:
                self.robot.send_angles = orig_send
                self.is_moving_thread = False
                self.root.after(0, self.update_target_cards)

        threading.Thread(target=_runner, daemon=True).start()

    def execute_full_obstacle_sequence(self):
        """Runs the complete 3-target sequence: Red ('N') ➔ Blue ('U') ➔ Yellow ('S')."""
        if self.is_moving_thread or self.retriever.is_running:
            messagebox.showinfo("Busy", "A routine is already running.")
            return

        try:
            self.right_notebook.select(2)
        except Exception:
            pass

        def _full_runner():
            self.is_moving_thread = True
            self.btn_full_mission.config(state=tk.DISABLED, bg="#4b5563")

            orig_send = self.robot.send_angles
            def sync_send(angles, *args, **kwargs):
                orig_send(angles, *args, **kwargs)
                for idx, rad in enumerate(angles):
                    deg = math.degrees(rad)
                    self.motor_vars[idx].set(max(0.0, min(self.max_limit_var.get(), deg)))
                    self.val_labels[idx].config(text=f"{deg:5.1f}° ({rad:.2f} rad)")
                self.draw_canvas_visualizer()
            self.robot.send_angles = sync_send

            try:
                self.retriever.run_full_3target_sequence(
                    order=["red", "blue", "yellow"],
                    progress_callback=lambda st, msg: self.root.after(0, lambda: (self.log_seq_message(f"[{st}] {msg}"), self.update_target_cards())),
                )
            finally:
                self.robot.send_angles = orig_send
                self.is_moving_thread = False
                self.root.after(0, lambda: (self.btn_full_mission.config(state=tk.NORMAL, bg="#7209b7"), self.update_target_cards()))

        threading.Thread(target=_full_runner, daemon=True).start()

    # -------------------------------------------------------------
    # Step-by-Step Testing & Verification Handlers
    # -------------------------------------------------------------

    def run_step_action(self, name, action_func):
        if self.is_moving_thread or self.retriever.is_running:
            messagebox.showinfo("Busy", "A routine is already running.")
            return

        def _step_worker():
            self.is_moving_thread = True
            self.log_seq_message(f"Starting test step: {name}")
            try:
                action_func()
                self.log_seq_message(f"[OK] Completed test step: {name}")
            except Exception as e:
                self.log_seq_message(f"[ERROR] Step {name} failed: {e}")
            finally:
                self.is_moving_thread = False

        threading.Thread(target=_step_worker, daemon=True).start()

    def step_enter_obstacles(self):
        """Step 1: Move from home to obstacle arena entrance."""
        self.run_step_action("1. Enter Obstacles", lambda: self.robot.move_smooth(self.obstacle_env.entry_pose, duration=1.2))

    def step_contact_bend(self):
        """Step 2: Actuate cables to establish obstacle contact and form dual bend."""
        target_col = self.active_target_color.get() or "red"
        pose = self.obstacle_env.contact_poses.get(target_col, [0.55, 0.15, 0.40, 0.0])
        self.run_step_action(f"2. Dual-Bend Contact ({target_col.upper()})", lambda: self.robot.move_smooth(pose, duration=1.6))

    def step_magnet_pickup(self):
        """Step 3: Lower tip into magnetic contact with letter."""
        curr = list(self.robot.current_angles)
        press = [a * 1.05 for a in curr]
        lift = [a * 0.90 for a in curr]
        def _action():
            self.robot.move_smooth(press, duration=0.6)
            time.sleep(0.5)
            self.robot.move_smooth(lift, duration=0.8)
        self.run_step_action("3. Magnet Pickup", _action)

    def step_withdraw_obstacles(self):
        """Step 4: Unwind bends and safely come out of obstacle field."""
        target_col = self.active_target_color.get() or "red"
        wps = self.obstacle_env.withdrawal_paths.get(target_col, [self.obstacle_env.entry_pose, [0.0]*4])
        def _action():
            for wp in wps:
                self.robot.move_smooth(wp, duration=0.9)
                time.sleep(0.15)
        self.run_step_action("4. Withdraw Obstacles", _action)

    def step_deliver_slot(self):
        """Step 5: Transfer letter to designated drop slot."""
        target_col = self.active_target_color.get() or "red"
        pose = self.obstacle_env.slot_delivery_poses.get(target_col, [0.0, 0.0, 0.2, 0.4])
        self.run_step_action(f"5. Deliver to Slot ({target_col.upper()})", lambda: self.robot.move_smooth(pose, duration=1.4))

    def step_slot_wipe(self):
        """Step 6: Execute shear wipe against slot lip to strip magnet."""
        target_col = self.active_target_color.get() or "red"
        delivery_pose = self.obstacle_env.slot_delivery_poses.get(target_col, [0.0, 0.0, 0.2, 0.4])
        delta = self.obstacle_env.slot_wipe_deltas.get(target_col, [0.10, 0.0, -0.10, 0.15])
        wipe_pose = [delivery_pose[i] + delta[i] for i in range(4)]
        clear_pose = [a * 0.5 for a in wipe_pose]
        def _action():
            self.robot.move_smooth(wipe_pose, duration=0.8)
            time.sleep(0.3)
            self.robot.move_smooth(clear_pose, duration=0.7)
        self.run_step_action("6. Slot Detach Wipe", _action)

    def step_return_obstacles(self):
        """Step 7: Return from slot to obstacle entrance staging pose."""
        self.run_step_action("7. Return to Obstacles", lambda: self.robot.move_smooth(self.obstacle_env.entry_pose, duration=1.2))

    # Aliases for backward compatibility
    def execute_specific_pick_and_place(self, color_name):
        self.execute_single_obstacle_retrieval(color_name)

    def execute_full_nus_sequence(self):
        self.execute_full_obstacle_sequence()

    def stop_all_autonomous_actions(self):
        """Emergency stop for all autonomous vision and servoing actions."""
        self.stop_servoing()
        self.retriever.stop()

    def log_seq_message(self, msg):
        """Appends timestamped message to the activity log."""
        if hasattr(self, "seq_log_text"):
            t_str = time.strftime("%H:%M:%S")
            self.seq_log_text.insert(tk.END, f"[{t_str}] {msg}\n")
            self.seq_log_text.see(tk.END)

    def launch_3d_simulator(self):
        """Launches the interactive 3D continuum simulator in an external process."""
        import subprocess
        base_dir = os.path.dirname(os.path.abspath(__file__))
        sim_script = os.path.join(base_dir, "simulation", "simulate_obstacle_retrieval.py")
        if not os.path.exists(sim_script):
            sim_script = os.path.join(base_dir, "simulate_obstacle_retrieval.py")
        subprocess.Popen([sys.executable, sim_script])
        self.log_seq_message("Launched Interactive 3D Continuum Simulator in external window.")


    def on_toggle_track_markers(self):
        if self.robot.camera is not None:
            self.robot.camera.track_markers = self.track_markers_var.get()

    def capture_snapshot(self):
        if self.robot.camera is None or not getattr(self.robot.camera, "is_running", False) or self.robot.camera.frame is None:
            messagebox.showinfo("Camera", "Please connect the camera first to capture snapshots.")
            return
        frame = self.robot.camera.frame.copy()
        rot = self.cam_rotation_var.get()
        if rot == "90_ccw":
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif rot == "180":
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rot == "90_cw":
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        filename = f"snapshot_{int(time.time())}.png"
        cv2.imwrite(filename, frame)
        messagebox.showinfo("Snapshot Saved", f"Saved rotated camera snapshot to:\n{filename}")

    def popout_camera(self):
        """Launches standalone high-resolution camera viewer."""
        try:
            py_exe = sys.executable
            script = os.path.abspath(os.path.join(os.path.dirname(__file__), "camera_viewer.py"))
            cmd = [py_exe, script]
            if self.track_markers_var.get():
                cmd.append("--track")
            if self.depth_view_var.get():
                cmd.append("--depth")
            if self.camera_var.get():
                cmd.extend(["--serial", self.camera_var.get()])
            cmd.extend(["--rotate", self.cam_rotation_var.get()])
            if self.active_target_color.get():
                cmd.extend(["--target", self.active_target_color.get()])

            if self.robot.camera is not None and getattr(self.robot.camera, "is_running", False):
                self.disconnect_camera()

            subprocess.Popen(cmd)
        except Exception as e:
            messagebox.showerror("Error", f"Could not launch camera viewer: {e}")

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
        """Immediately relaxes all motors to zero and stops any active servoing/routines."""
        self.stop_all_autonomous_actions()
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

            def sync_send(angles, *args, **kwargs):
                orig_send(angles, *args, **kwargs)
                for idx, rad in enumerate(angles):
                    deg = math.degrees(rad)
                    self.motor_vars[idx].set(max(0.0, min(self.max_limit_var.get(), deg)))
                    self.val_labels[idx].config(text=f"{deg:5.1f}° ({rad:.2f} rad)")
                self.draw_canvas_visualizer()

            self.robot.send_angles = sync_send

            try:
                routine_func()
            finally:
                self.robot.send_angles = orig_send
                self.is_moving_thread = False
                self.home_all()
                if not self.robot.is_mock and self.robot.is_connected:
                    active_port = getattr(self.robot, "port", self.port_var.get())
                    self.status_lbl.config(text=f"● Connected: {active_port}", fg=self.green_color)
                else:
                    self.status_lbl.config(text="● Simulation / Mock Mode", fg="#fbbf24")

        threading.Thread(target=_worker, daemon=True).start()

    def on_close(self):
        self.camera_streaming = False
        self.stop_all_autonomous_actions()
        try:
            self.robot.disconnect()
        except Exception:
            pass
        self.root.destroy()


def main(port=None, camera_serial=None):
    parser = argparse.ArgumentParser(description="EMIO Cable-Driven Robot & Vision GUI (Windows & Linux)")
    parser.add_argument("--port", type=str, default=None, help="Robot serial port (e.g. COM12 on Windows or /dev/ttyUSB0 on Linux)")
    parser.add_argument("--serial", "--camera-serial", type=str, default=None, help="RealSense camera serial number")
    parser.add_argument("--camera", "-c", action="store_true", help="Auto-connect RealSense camera on startup")
    args, _ = parser.parse_known_args()

    port = port or args.port
    camera_serial = camera_serial or args.serial

    root = tk.Tk()
    app = CableRobotGUI(root, initial_port=port, initial_camera_serial=camera_serial, auto_connect_camera=args.camera)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
