# ME4242 Soft Robotics - Project 2
## Obstacle-Aided Continuum Robot Multi-Color Magnetic Retrieval

Autonomous visual servoing and control system for an EMIO 4-cable-driven continuum robot. 

### Physical Setup Characteristics:
- **Overhead Center Mount**: Manipulator is suspended vertically downward from the top center clamp `(X = 0, Y = 0, Z = 380 mm)` in the middle of the setup.
- **Horizontal Obstacle Pegs Mounted on Vertical Struts**: Clamped to the vertical frame columns at different heights (`Z = 235 mm` and `Z = 125 mm`), extending horizontally inwards past the center line.
- **Pre-Bent Dual S-Curve at Rest**: The horizontal pegs physically intercept and pre-bend the descending arm into an S-curve even at rest (`0°` motor actuation).
- **Inextensible Continuum Spine**: Strict constant arc length $L = 370.0$ mm ($\sum \Delta s = L$) with 9 blue spacer disks and green magnetic end-effector.
- **Table Arena Below (Z = 0)**: Targets (Red 'N', Blue 'U', Yellow 'S') and a 3-compartment side-by-side sorting tray `[ Slot N | Slot U | Slot S ]`.

---

## Workspace Directory Structure

```text
ME4242 Soft Robotics/Project/
├── cabledriven.py              # EMIO 4-motor Dynamixel hardware controller & safe homing
├── cabledriven_gui.py          # Main graphical user interface with live telemetry & 3D launcher
├── cartesian_controller.py     # 🕹️ Cartesian (XYZ) Task-Space Controller & Inverse Kinematics GUI
├── visual_servoing.py          # PBVS controller, ColorDetector, ObstacleEnvironment, State Machine
├── camera_viewer.py            # RealSense depth camera viewer & calibration tool
├── requirements.txt            # Python dependencies
│
├── simulation/                 # 🚀 Simulation Package
│   ├── simulate_obstacle_retrieval.py  # Dual-View 3D + 2D RealSense CV Camera & Dashboard
│   ├── simulate_3d_space.py            # Standalone fullscreen 3D interactive orbit simulator
│   ├── continuum_3d_engine.py          # 3D forward kinematics, constant length physics, gantry rig
│   ├── test_obstacle_retrieval.py      # Automated benchmark test suite (53.0s, 100% success, 3/3 marks)
│   └── __init__.py                     # Simulation package module marker
│
├── assets/                     # 🖼️ Images & Media
│   ├── image5.jpeg                     # RealSense overhead camera photo
│   ├── image9.jpeg                     # EMIO Compliance Robotics gantry test rig
│   ├── image10.png                     # Top motor mount and suspended continuum arm
│   ├── image11.jpeg                    # Blue spacer disks and green magnet tip
│   ├── simulation_3d_dual_screenshot.png # Dual-view visual simulation snapshot
│   └── test_3d_render.png              # 3D model render snapshot
│
├── docs/                       # 📚 Documentation & Unused Templates
│   ├── ME4242 7a. Briefing Project Assignment.pptx # Project briefing slide deck
│   ├── ME4242 7b. installl_packages.pptx           # Package installation guide
│   ├── diagnostics.md                  # Hardware & serial port diagnostics
│   ├── lab_me4242.md                   # Legacy lab assignment guide
│   ├── lab_me4242.py                   # Legacy lab execution script
│   ├── lab.json                        # Legacy lab metadata configuration
│   ├── Circle.py                       # Legacy circle trajectory script
│   ├── setLabName.sh                   # Legacy lab renaming shell script
│   ├── sections/                       # Legacy lab markdown sections
│   └── solutions/                      # Legacy lab solution notes
│
└── modules/site-packages/      # 📦 Local Python dependencies (emioapi, pyrealsense2, etc.)
```

---

## Quick Start

### 1. Launch Main Controller GUI
```powershell
python cabledriven_gui.py
```
- Click **`🕹️ Cartesian XYZ Control`** on the bottom bar to open the task-space controller.
- Switch to the **"🧲 Obstacle Retrieval"** tab to run the autonomous sequence or step-by-step testers.

### 2. Launch Dedicated Cartesian XYZ Controller
```powershell
python cartesian_controller.py
```
- Direct $(X, Y, Z)$ positioning with exact machine dimensions (280mm height, 185mm arm, 365mm baseplate, 5mm tendon offset).
- **Interactive Click-to-Move**: Click anywhere on the overhead table arena to servo the green tip directly there.
- **Incremental Step Jogging**: Nudge buttons ($\pm 1\text{ mm}$, $\pm 5\text{ mm}$, $\pm 10\text{ mm}$, $\pm 20\text{ mm}$) for fine tuning.
- **Live Visualizers**: Real-time overhead table view (X-Y) and elevation profile (X-Z).

### 3. Launch Interactive 3D Dual-View Simulator
```powershell
python simulation/simulate_obstacle_retrieval.py
```
- **3D Mouse Orbit**: Left-click + drag to rotate around the overhead gantry rig and arena table.
- **Zoom**: Mouse wheel or right-click drag.
- **Key Controls**:
  - `[SPACE]`: Run full autonomous 3-target sequence (Red $\to$ Blue $\to$ Yellow).
  - `[V]` / `[TAB]`: Toggle between Dual View, 3D Only, and 2D Camera View.
  - `[1]`, `[2]`, `[3]`: Retrieve individual target letter.
  - `[C]`: Cycle obstacle configuration preset (Dual-Peg S-Bend $\leftrightarrow$ Lateral Deflector $\leftrightarrow$ Corridor).
  - `[R]`: Reset 3D camera to default isometric perspective.
  - `[H]`: Home arm and reset arena letters.
  - `[Q]`: Quit simulator.

### 4. SOFA Physics Simulation of the Cable-Driven Rod
```powershell
sofa_sim\run_sim.bat           # sliders in the "Cable rod: motors" window (press Play first)
sofa_sim\run_sim.bat bend      # pull each cable in turn
sofa_sim\run_sim.bat circle    # circular sweep
```
- Uses the SOFA bundled with Emio Labs (Windows). The rod is a BeamAdapter beam clamped at the top mount; 4 SoftRobots cables run through the spacer disks, cable pull = 12 mm spool radius x motor angle.
- The console prints the simulated tip position next to the PCC prediction from `cartesian_controller.py`, plus each cable's tension.
- Rig dimensions and the rod material are constants at the top of `sofa_sim/cable_rod_scene.py`. The rod material is a spring-steel guess and should be tuned against the real rod.

### 5. Run Automated Benchmark Test
```powershell
python simulation/test_obstacle_retrieval.py
```
- Automatically executes full 3-target retrieval and verifies benchmark speed (< 1 min for full 3/3 marks).
