"""
cable_rod_scene.py
------------------
SOFA simulation of the modified EMIO rig: 4 EMIO motors, each pulling one cable,
routed through spacer disks along a central stiff-but-bendable rod that hangs
down from the top mount. Pulling a cable shortens one side of the rod, so it bends.

Model:
- Rod: BeamAdapter beam (circular cross-section), clamped at the top mount.
- Spacer disks: rigidly attached to the rod, each with 4 cable holes at
  TENDON_OFFSET from the rod axis (+X, +Y, -X, -Y as in cartesian_controller.py).
- Cables: SoftRobots CableConstraint in displacement mode. Each cable starts at
  its motor above the mount and passes through the matching hole of every disk.
  Cable pull (mm) = SPOOL_RADIUS * motor angle (rad).

Frames: SOFA uses Y up, with the platform (table) at y = 0. Cable/motor Mi runs to
Emio motor i-1 (the emioapi index), placed as in Emio Labs: M1 at SOFA +z, M2 at +x,
M3 at -z, M4 at -x. Rig coordinates (as in cartesian_controller.py: X towards M1,
Y towards M2, Z = height above the platform) are X = SOFA z, Y = SOFA x, Z = SOFA y.
Units: mm, kg, s.

Controls: press Play, then use the sliders in the "Cable rod: motors" window that
opens next to SOFA (motor_panel.py), or the keyboard (see CableRodController).

Run it from the Emio Labs app (runsofa-button), or from a terminal:
    runSofa -l SofaPython3,SofaImGui -g imgui sofa_sim/cable_rod_scene.py
Optional argument (runSofa forwards one token after --argv):
    --argv --demo=bend     pull each cable in turn
    --argv --demo=circle   circular sweep
    --argv --demo=path     play the keyframes in trajectory.py
    --argv --demo=weave    hardcoded animation: the rod bends into an S through the bars (weave.py)
    --argv --demo=play     replay the newest recording (recording.py); play:<name> for another
"""

import math
import os
import sys

import Sofa
import Sofa.Core

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig_visuals import add_rig_visuals  # noqa: E402
from motor_panel import open_panel  # noqa: E402
from weave import WeaveController  # noqa: E402
import recording  # noqa: E402
from obstacles import (add_collision_header, add_rod_collision, add_stl_obstacle,  # noqa: E402
                       stl_bounds, translation_for_point)

# ---------------------------------------------------------------------------
# Rig parameters (same values as cartesian_controller.py, edit to match the build)
# ---------------------------------------------------------------------------
MACHINE_HEIGHT = 280.0   # mm, top mount to base plate
ROD_LENGTH = 185.0       # mm, free length of the rod below the mount
TENDON_OFFSET = 5.0      # mm, radial distance from rod axis to the cable holes
SPOOL_RADIUS = 12.0      # mm, cable spool radius on each motor
ROD_RADIUS = 0.9         # mm (1.8 mm diameter rod)
NUM_DISKS = 9            # spacer disks, evenly spaced, the last one at the tip
DISK_RADIUS = 10.0       # mm, only used for rendering

# Rod material. Not measured yet: the defaults are spring steel. Tune these so the
# simulated bend under a given motor angle matches the real rod.
# Young's modulus in kg/(mm.s^2) = Pa * 1e-3.
ROD_YOUNG_MODULUS = 2.0e8   # 200 GPa
ROD_POISSON_RATIO = 0.3
ROD_MASS_DENSITY = 7.85e-6  # kg/mm^3

# When one cable is pulled, the cable on the opposite side must let out the same
# length, or the two cables squeeze the rod (and a thin rod buckles sideways).
# True: the opposite cable is free to let out what the pulled one takes in. This is
#   what cartesian_controller.py assumes (bend ~ m1 - m3), and on the robot it needs
#   slack in the cables or the opposite motor turning back.
# False: a motor at 0 holds its cable at its starting length (taut cables, motors in
#   position mode), which is what the hardware does if the cables start taut.
OPPOSITE_CABLE_PAYS_OUT = True

MAX_MOTOR_ANGLE = math.pi      # rad, slider range (same as MAX_ANGLE_DEG = 180 in cabledriven.py)
# Highest cable tension a motor can produce before it stalls. The XM430-W210 stalls at
# about 3 N.m, i.e. ~250 N on a 12 mm spool; a lower cap keeps contacts stable when the
# rod is jammed against an obstacle. Force unit in SOFA here is mN (kg.mm/s^2).
MAX_CABLE_FORCE = 30.0         # N
MAX_MOTOR_SPEED = 2.0          # rad/s, the motors move towards the slider targets at this speed
BEAM_NODES_PER_DISK = 2        # beam resolution between disks

# ---------------------------------------------------------------------------
# Obstacles
# ---------------------------------------------------------------------------
# Backboard assembly from SolidWorks ("Backboard Assembly.STEP", converted to STL): the
# pegboard, its main leg and 2 side legs, and the 3 obstacle bars mounted on the board.
# SolidWorks axes match SOFA here: Y up, Front plane = X-Y, Right plane = Y-Z.
# Placement:
# - the 12.3 mm hole in the main leg is concentric (in plan view) with the 25 mm hole in
#   the Emio platform (off-centre, measured from Emio's platform.stl);
# - the board is turned about that hole to face the rod;
# - the top surface of the board is level with the top platform the rod hangs from.
BACKBOARD_STL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes", "backboard.stl")
BACKBOARD_HOLE_CAD = [1448.72, 506.07, 791.40]  # centre of the hole's bottom face, assembly coordinates
PLATFORM_HOLE = [79.85, 0.0, 79.74]            # centre of the platform hole, on the platform surface
BACKBOARD_TOP_Y = MACHINE_HEIGHT               # height of the board's top surface
# Rotation about the vertical axis. In the CAD the board is 99.5 mm behind the hole along -Z;
# this angle turns that direction towards the rod axis, so the board faces the rod.
BACKBOARD_ROTATION_Y = math.degrees(math.atan2(PLATFORM_HOLE[0], PLATFORM_HOLE[2]))
BACKBOARD_COLOR = [0.95, 0.85, 0.6, 1]


def backboard_translation():
    """Loader translation for the placement above (the board top is the highest point of the STL)."""
    _, hi = stl_bounds(BACKBOARD_STL)
    hole_y = BACKBOARD_TOP_Y - (hi[1] - BACKBOARD_HOLE_CAD[1])
    target = [PLATFORM_HOLE[0], hole_y, PLATFORM_HOLE[2]]
    return translation_for_point(BACKBOARD_HOLE_CAD, target, BACKBOARD_ROTATION_Y)


# Real Emio (extended configuration) around the rod. The platform (table) is at y = 0;
# the motor shafts sit 303.1 mm above platform level 0 (Emio meshes), and each
# platform level raises the platform by 35 mm, i.e. brings the motors 35 mm closer.
EMIO_PLATFORM_LEVEL = 0                                  # 0, 1 or 2, as on the real box
EMIO_MOTOR_Y = 303.1 - 35.0 * EMIO_PLATFORM_LEVEL       # mm, motor shafts above the platform

# Motor i pulls the cable at this angle around the rod, in the rig X-Y plane.
CABLE_ANGLES = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]  # M1 +X, M2 +Y, M3 -X, M4 -Y
CABLE_COLORS = [[0.22, 0.74, 0.97, 1], [0.66, 0.33, 0.97, 1], [0.98, 0.45, 0.09, 1], [0.06, 0.73, 0.51, 1]]


def rig_to_sofa(x, y, z):
    return [y, z, x]


def sofa_to_rig(p):
    return [p[2], p[0], p[1]]


def hole_offset(i):
    """Horizontal offset of cable i's hole from the rod axis, in SOFA coordinates."""
    a = CABLE_ANGLES[i]
    return rig_to_sofa(TENDON_OFFSET * math.cos(a), TENDON_OFFSET * math.sin(a), 0.0)


def pcc_tip(motor_angles):
    """Constant-curvature prediction of the tip (rig frame), same as cartesian_controller.py."""
    m1, m2, m3, m4 = motor_angles
    kx = (m1 - m3) * SPOOL_RADIUS / TENDON_OFFSET
    ky = (m2 - m4) * SPOOL_RADIUS / TENDON_OFFSET
    theta = math.hypot(kx, ky)
    if theta < 1e-6:
        return [0.0, 0.0, MACHINE_HEIGHT - ROD_LENGTH]
    phi = math.atan2(ky, kx)
    R = ROD_LENGTH / theta
    r = R * (1 - math.cos(theta))
    return [r * math.cos(phi), r * math.sin(phi), MACHINE_HEIGHT - R * math.sin(theta)]


class CableRodController(Sofa.Core.Controller):
    """
    Holds the 4 target motor angles (rad), moves the simulated motors towards them
    at MAX_MOTOR_SPEED, and converts the motor angles to cable pulls every step.
    Keyboard (click the 3D view first): 1-4 pull M1-M4, Ctrl+1-4 release, 0 home.
    """

    def __init__(self, cables, tip, demo=None, panel=None, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "CableRodController"
        self.cables = cables
        self.tip = tip
        self.demo = demo
        self.panel = panel
        self.rec = None        # samples being recorded: [t, m1, m2, m3, m4] (s, deg)
        self.rec_t0 = 0.0
        self.playback = None   # samples being played back ("play" demo)
        if demo and demo.startswith("play"):
            name = demo.split(":", 1)[1] if ":" in demo else None
            path, self.playback = recording.load(name)
            print(f"[play] {os.path.basename(path)}: {len(self.playback)} samples, "
                  f"{self.playback[-1][0]:.1f} s")
        self.time = 0.0
        self.step = 0
        self.applied = [0.0] * 4  # current (rate-limited) motor angles
        self.angles = []          # targets, edited from the GUI or the keyboard
        for i in range(4):
            self.angles.append(self.addData(name=f"M{i + 1}", type="double", value=0.0,
                                            help=f"Motor {i + 1} angle (rad)", group="Motors",
                                            default=0.0))

    def onAnimateBeginEvent(self, event):
        dt = self.getContext().getRoot().dt.value
        self.time += dt
        if self.demo:
            for data, a in zip(self.angles, self._demo_angles(self.time)):
                data.value = a
        elif self.panel is not None:
            targets = self.panel.take_targets()
            if targets is not None:
                for data, a in zip(self.angles, targets):
                    data.value = a
        max_step = MAX_MOTOR_SPEED * dt
        for i, data in enumerate(self.angles):
            data.value = min(max(data.value, 0.0), MAX_MOTOR_ANGLE)
            delta = min(max(data.value - self.applied[i], -max_step), max_step)
            self.applied[i] += delta
        angles = self.applied
        for i, cable in enumerate(self.cables):
            pull = angles[i]
            if OPPOSITE_CABLE_PAYS_OUT:
                pull -= angles[(i + 2) % 4]
            # Positive shortens the cable. minForce=0 means a cable can only pull,
            # so a negative value just gives it slack.
            cable.value = [SPOOL_RADIUS * pull]

    def onAnimateEndEvent(self, event):
        self.step += 1
        self._record()
        if self.panel is not None and self.step % 5 == 0:
            tip = sofa_to_rig(self.tip.position.value[-1][0:3])
            self.panel.set_status(
                "motors (deg): " + "  ".join(f"{math.degrees(a):4.0f}" for a in self.applied) +
                f"\ntip X {tip[0]:6.1f}  Y {tip[1]:6.1f}  Z {tip[2]:6.1f} mm" +
                f"\nsim time {self.time:6.2f} s")
        if self.step % 50 == 0:
            angles = self.applied
            sim = sofa_to_rig(self.tip.position.value[-1][0:3])
            pcc = pcc_tip(angles)
            forces = [c.force.value for c in self.cables]
            print("t={:5.2f}s  motors(deg)=[{}]  tip sim=({:6.1f},{:6.1f},{:6.1f})  "
                  "PCC=({:6.1f},{:6.1f},{:6.1f}) mm  cable force(N)=[{}]".format(
                      self.time, ", ".join(f"{math.degrees(a):5.1f}" for a in angles),
                      *sim, *pcc, ", ".join(f"{f * 1e-3:6.1f}" for f in forces)))

    def onKeypressedEvent(self, event):
        key = event["key"]
        if key == "0":
            for d in self.angles:
                d.value = 0.0
            print("[keys] all motor targets -> 0")
            return
        step = math.radians(2.0)
        pull = {"1": 0, "2": 1, "3": 2, "4": 3}
        release = {"!": 0, "@": 1, "#": 2, "$": 3}
        if key in pull:
            i = pull[key]
            self.angles[i].value = min(self.angles[i].value + step, MAX_MOTOR_ANGLE)
        elif key in release:
            i = release[key]
            self.angles[i].value = max(self.angles[i].value - step, 0.0)
        else:
            return
        print(f"[keys] M{i + 1} target -> {math.degrees(self.angles[i].value):.0f} deg")

    def _record(self):
        """Record the reached motor angles while the panel's Record button is on."""
        p = self.panel
        if p is None:
            return
        if p.recording:
            if self.rec is None:
                self.rec, self.rec_t0 = [], self.time
            t = self.time - self.rec_t0
            if not self.rec or t - self.rec[-1][0] >= recording.SAMPLE_PERIOD - 1e-9:
                self.rec.append([t] + [math.degrees(a) for a in self.applied])
                p.set_message(f"Recording... {t:5.1f} s, {len(self.rec)} samples. Press Stop to save.")
        elif p.save_requested:
            p.save_requested = False
            if self.rec:
                path = recording.save(self.rec)
                name = os.path.splitext(os.path.basename(path))[0]
                p.set_message(f"Saved {len(self.rec)} samples ({self.rec[-1][0]:.1f} s) to recordings/{name}.json\n"
                              f"Replay: run_sim.bat play   (or play:{name})")
                print(f"[record] saved {path}")
            self.rec = None

    def _demo_angles(self, t):
        if self.playback is not None:
            if t > self.playback[-1][0] and not getattr(self, "_play_done", False):
                self._play_done = True
                print(f"[play] finished at t={t:.2f}s, holding the last pose")
            return [math.radians(a) for a in recording.angles_at(self.playback, t)]
        if self.demo == "path":
            import importlib
            import trajectory
            importlib.reload(trajectory)  # pick up edits when the scene is reloaded
            start = 0.0
            for duration, angles in trajectory.KEYFRAMES:
                if t < start + duration:
                    return [math.radians(a) for a in angles]
                start += duration
            return [math.radians(a) for a in trajectory.KEYFRAMES[-1][1]]
        amp = math.radians(25.0)
        if self.demo == "circle":
            theta = 2 * math.pi * t / 6.0
            return [max(0.0, math.cos(theta - p)) * amp for p in CABLE_ANGLES]
        # "bend": pull each cable in turn, 4 s per cable
        k = int(t // 4.0) % 4
        s = math.sin(math.pi * (t % 4.0) / 4.0)
        return [amp * s if i == k else 0.0 for i in range(4)]


def add_plugins(root):
    root.addObject("RequiredPlugin", pluginName=[
        "BeamAdapter", "SoftRobots",
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.Mapping.NonLinear",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Visual",
        "Sofa.GL.Component.Rendering3D",
    ])


def parse_demo():
    for arg in sys.argv:
        if arg.startswith("--demo="):
            demo = arg.split("=", 1)[1]
            if demo == "play" or demo.startswith("play:"):
                return demo
            return demo if demo in ("bend", "circle", "path", "weave") else "bend"
    return None


def createScene(root):
    add_plugins(root)
    demo = parse_demo()
    # "weave" is a hardcoded animation: the rod is only a set of frames moved by WeaveController
    kinematic = demo == "weave"
    # 5 ms steps: with 10 ms the rod could pass through the 5 mm backboard when it
    # springs back quickly from a hard curl.
    root.dt = 0.005
    root.gravity = [0.0, -9810.0, 0.0]
    root.addObject("VisualStyle", displayFlags=["showVisualModels"])
    # Camera on the rod's side of the backboard (opposite the platform hole), a little to the
    # side, so the rod and the bars are in front of the board.
    away = math.atan2(-PLATFORM_HOLE[0], -PLATFORM_HOLE[2]) + math.radians(25)
    root.addObject("InteractiveCamera", name="camera",
                   position=[520.0 * math.sin(away), 300.0, 520.0 * math.cos(away)],
                   lookAt=[0.0, 185.0, 0.0], fieldOfView=45, zNear=1, zFar=5000,
                   computeZClip=False)
    if kinematic:
        root.addObject("DefaultAnimationLoop")
    else:
        root.addObject("FreeMotionAnimationLoop")
        root.addObject("BlockGaussSeidelConstraintSolver", name="ConstraintSolver",
                       tolerance=1e-8, maxIterations=500)

    # --- Rod: beam nodes from the mount (top) down to the tip -----------------
    num_nodes = NUM_DISKS * BEAM_NODES_PER_DISK + 1
    seg = ROD_LENGTH / (num_nodes - 1)
    # Beam axis is the frame's local x; rotate it to point down (-y).
    q_down = [0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)]
    frames = [[0.0, MACHINE_HEIGHT - i * seg, 0.0] + q_down for i in range(num_nodes)]

    rod = root.addChild("Rod")
    disk_nodes = [(k + 1) * BEAM_NODES_PER_DISK for k in range(NUM_DISKS)]
    if kinematic:
        rod.addObject("MechanicalObject", name="dofs", template="Rigid3", position=frames)
    else:
        _add_rod_mechanics(rod, frames, num_nodes)
    cables = [] if kinematic else _add_cables(rod, frames, disk_nodes)

    # --- Tip marker (used for the printed tip position) ---------------------------
    tip = rod.addChild("Tip")
    tip.addObject("MechanicalObject", name="tip", template="Vec3", position=[frames[-1][0:3]])
    tip.addObject("RigidMapping", index=num_nodes - 1, globalToLocalCoords=True)

    # --- Obstacles and contact ------------------------------------------------------
    if not kinematic:
        add_collision_header(root)
        add_rod_collision(rod, frames, disk_nodes, ROD_RADIUS, DISK_RADIUS)
    if os.path.isfile(BACKBOARD_STL):
        translation = backboard_translation()
        add_stl_obstacle(root, BACKBOARD_STL, translation, [0.0, BACKBOARD_ROTATION_Y, 0.0],
                         color=BACKBOARD_COLOR, name="Backboard")
        print(f"[obstacles] loaded {os.path.normpath(BACKBOARD_STL)}, turned {BACKBOARD_ROTATION_Y:.1f} deg, "
              f"board top at {BACKBOARD_TOP_Y:.1f} mm")
    else:
        print(f"[obstacles] {os.path.normpath(BACKBOARD_STL)} not found, no obstacles")

    # --- Rough 3D model of the rig -------------------------------------------------
    add_rig_visuals(root, rod, frames, disk_nodes,
                    hole_offsets=[hole_offset(i) for i in range(4)], cable_colors=CABLE_COLORS,
                    top_y=MACHINE_HEIGHT, rod_radius=ROD_RADIUS, disk_radius=DISK_RADIUS,
                    motor_y=EMIO_MOTOR_Y, platform_level=EMIO_PLATFORM_LEVEL)

    # --- Controller and GUI -------------------------------------------------------
    if kinematic:
        root.addObject(WeaveController(rod.dofs, MACHINE_HEIGHT, seg, num_nodes, BACKBOARD_ROTATION_Y))
        return root
    # Slider window (not opened for the scripted demos)
    panel = None if demo else open_panel(math.degrees(MAX_MOTOR_ANGLE))
    controller = CableRodController(cables, tip.tip, demo=demo, panel=panel)
    root.addObject(controller)
    _add_imgui(controller, tip.tip, cables)

    return root


def _add_rod_mechanics(rod, frames, num_nodes):
    """Beam model of the rod, clamped at the top mount."""
    rod.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    rod.addObject("SparseLDLSolver", name="solver", template="CompressedRowSparseMatrixd")
    rod.addObject("GenericConstraintCorrection", linearSolver=rod.solver.linkpath)
    rod.addObject("MeshTopology", position=[f[0:3] for f in frames],
                  edges=[[i, i + 1] for i in range(num_nodes - 1)])
    rod.addObject("MechanicalObject", name="dofs", template="Rigid3", position=frames)
    rod.addObject("BeamInterpolation", crossSectionShape="circular", radius=ROD_RADIUS,
                  defaultYoungModulus=ROD_YOUNG_MODULUS, defaultPoissonRatio=ROD_POISSON_RATIO)
    rod.addObject("AdaptiveBeamForceFieldAndMass", computeMass=True, massDensity=ROD_MASS_DENSITY)
    rod.addObject("FixedProjectiveConstraint", indices=[0])  # clamped at the top mount


def _add_cables(rod, frames, disk_nodes):
    """Cable holes on the disks (mapped rigidly to the rod) and the 4 cable constraints."""
    holes = rod.addChild("CableHoles")
    points, parents = [], []
    for i in range(4):
        off = hole_offset(i)
        for n in disk_nodes:
            points.append([frames[n][0] + off[0], frames[n][1] + off[1], frames[n][2] + off[2]])
            parents.append(n)
    holes.addObject("MechanicalObject", name="holes", template="Vec3", position=points)
    holes.addObject("RigidMapping", rigidIndexPerPoint=parents, globalToLocalCoords=True)

    cables = []
    for i in range(4):
        off = hole_offset(i)
        motor = [off[0], MACHINE_HEIGHT + 20.0, off[2]]  # cable leaves towards its motor above the mount
        indices = list(range(i * NUM_DISKS, (i + 1) * NUM_DISKS))
        cables.append(holes.addObject("CableConstraint", name=f"Cable{i + 1}",
                                      indices=indices, pullPoint=motor,
                                      valueType="displacement", value=0.0,
                                      minForce=0,  # a cable can pull, not push
                                      maxForce=MAX_CABLE_FORCE * 1e3,  # motor stalls beyond this
                                      hasPullPoint=True,
                                      drawPullPoint=False, drawPoints=False))
    return cables


def _add_imgui(controller, tip_state, cables):
    """
    Live values in the Emio Labs (SofaImGui) 'Simulation State' and 'Plotting' windows.
    The motor sliders are in motor_panel.py instead: sliders added to SofaImGui's
    Move / My Robot windows did not move anything in this scene.
    """
    try:
        import Sofa.ImGui as MyGui
    except ImportError:
        return
    try:
        MyGui.SimulationState.addData("Tip", "Position", tip_state.position)
        for i, c in enumerate(cables):
            MyGui.SimulationState.addData("Cable force (1e-3 N)", f"M{i + 1}", c.force)
        for i, d in enumerate(controller.angles):
            MyGui.PlottingWindow.addData(f" angle M{i + 1} (rad) ", d)
    except Exception as e:
        Sofa.msg_warning("CableRodScene", f"Could not set up the ImGui windows: {e}")
