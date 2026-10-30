"""
motor_panel.py
--------------
Small Tkinter window with one slider per motor, opened next to the SOFA window by
cable_rod_scene.py. It runs in its own thread; the scene reads the slider values
every simulation step, and the panel shows the tip position sent back by the scene.
"""

import math
import threading

_panel = None  # one panel per SOFA session, reused when the scene is reloaded


class MotorPanel(threading.Thread):

    def __init__(self, max_angle_deg):
        super().__init__(daemon=True, name="MotorPanel")
        self.max_angle_deg = max_angle_deg
        self.targets_deg = [0.0] * 4
        self.changed = False  # set when a slider moved; cleared by the scene
        self.status = ""
        self.ready = threading.Event()
        self.closed = False
        self.recording = False       # set by the Record button; the scene records while it is on
        self.save_requested = False  # set by Stop; the scene saves the recording and clears it
        self.message = ""

    # Called from the scene thread -------------------------------------------
    def take_targets(self):
        """Return the slider targets in rad if a slider moved since the last call, else None."""
        if not self.changed:
            return None
        self.changed = False
        return [math.radians(a) for a in self.targets_deg]

    def set_status(self, text):
        self.status = text

    def set_message(self, text):
        self.message = text

    # Tk thread ---------------------------------------------------------------
    def run(self):
        import tkinter as tk

        root = tk.Tk()
        root.title("Cable rod: motors")
        root.attributes("-topmost", True)
        root.protocol("WM_DELETE_WINDOW", lambda: self._close(root))

        tk.Label(root, text="Press Play in SOFA, then drag a slider.",
                 font=("Segoe UI", 10)).pack(padx=12, pady=(10, 4))

        names = ["M1 (+X)", "M2 (+Y)", "M3 (-X)", "M4 (-Y)"]
        colors = ["#38bdf8", "#a855f7", "#f97316", "#10b981"]
        self._scales = []
        for i in range(4):
            row = tk.Frame(root)
            row.pack(fill="x", padx=12, pady=2)
            tk.Label(row, text=names[i], width=8, anchor="w", fg=colors[i],
                     font=("Segoe UI", 10, "bold")).pack(side="left")
            scale = tk.Scale(row, from_=0, to=self.max_angle_deg, resolution=1, orient="horizontal",
                             length=260, label="", command=lambda v, i=i: self._on_slide(i, v))
            scale.pack(side="left")
            self._scales.append(scale)
        tk.Label(root, text="motor angle (deg)", font=("Segoe UI", 8)).pack()

        buttons = tk.Frame(root)
        buttons.pack(pady=6)
        tk.Button(buttons, text="Home (all 0)", command=self._home).pack(side="left", padx=4)
        self._rec_button = tk.Button(buttons, text="● Record", fg="#dc2626", width=14,
                                     command=self._toggle_record)
        self._rec_button.pack(side="left", padx=4)

        status = tk.Label(root, text="", justify="left", font=("Consolas", 9))
        status.pack(padx=12, pady=(0, 4), anchor="w")
        message = tk.Label(root, text="", justify="left", wraplength=360, font=("Segoe UI", 9))
        message.pack(padx=12, pady=(0, 10), anchor="w")

        def refresh():
            status.config(text=self.status)
            message.config(text=self.message)
            root.after(200, refresh)

        refresh()
        self.ready.set()
        root.mainloop()

    def _on_slide(self, i, value):
        self.targets_deg[i] = float(value)
        self.changed = True

    def _home(self):
        for s in self._scales:
            s.set(0)
        self.targets_deg = [0.0] * 4
        self.changed = True

    def _toggle_record(self):
        if not self.recording:
            self.recording = True
            self.message = "Recording... move the rod, then press Stop."
            self._rec_button.config(text="■ Stop and save")
        else:
            self.recording = False
            self.save_requested = True
            self._rec_button.config(text="● Record")

    def _close(self, root):
        self.closed = True
        root.destroy()


def open_panel(max_angle_deg):
    """Open the panel (or reuse the one already open). Returns None if Tkinter is missing."""
    global _panel
    if _panel is not None and _panel.is_alive() and not _panel.closed:
        return _panel
    try:
        import tkinter  # noqa: F401
    except ImportError:
        return None
    _panel = MotorPanel(max_angle_deg)
    _panel.start()
    _panel.ready.wait(timeout=5)
    return _panel
