"""
rig_visuals.py
--------------
Rough 3D model of the cable-driven rig for cable_rod_scene.py, built from simple
generated meshes (no CAD files needed):
- static: base plate, 4 frame posts, top mount plate, 4 motors with spools
- moving (attached to the rod): rod tube, spacer disks, the 4 cables, magnet tip

Sizes are approximate. All positions are SOFA world coordinates (Y up) of the rig
at rest, with the rod hanging straight down from (0, top_y, 0).
"""

import math


# ---------------------------------------------------------------------------
# Mesh helpers. Each returns (points, triangles).
# ---------------------------------------------------------------------------

def _ring(cx, y, cz, r, n):
    return [[cx + r * math.cos(2 * math.pi * j / n), y, cz + r * math.sin(2 * math.pi * j / n)]
            for j in range(n)]


def _tube(rings, closed_ends=False):
    """Triangulated tube through a list of rings (each ring has the same point count)."""
    n = len(rings[0])
    pts = [p for ring in rings for p in ring]
    tris = []
    for k in range(len(rings) - 1):
        for j in range(n):
            a, b = k * n + j, k * n + (j + 1) % n
            c, d = a + n, b + n
            tris += [[a, b, c], [b, d, c]]
    if closed_ends:
        for ring_idx, flip in ((0, True), (len(rings) - 1, False)):
            ring = rings[ring_idx]
            centre = [sum(p[i] for p in ring) / n for i in range(3)]
            ci = len(pts)
            pts.append(centre)
            base = ring_idx * n
            for j in range(n):
                a, b = base + j, base + (j + 1) % n
                tris.append([ci, b, a] if flip else [ci, a, b])
    return pts, tris


def _cylinder_y(cx, cz, y0, y1, r, n=24):
    """Vertical solid cylinder."""
    return _tube([_ring(cx, y0, cz, r, n), _ring(cx, y1, cz, r, n)], closed_ends=True)


def _cylinder_axis(centre, axis, length, r, n=20):
    """Solid cylinder along an arbitrary horizontal axis (unit vector in x-z)."""
    ax, az = axis
    ux, uz = -az, ax  # horizontal vector perpendicular to the axis
    rings = []
    for s in (-0.5, 0.5):
        c = [centre[0] + s * length * ax, centre[1], centre[2] + s * length * az]
        rings.append([[c[0] + r * math.cos(t) * ux, c[1] + r * math.sin(t), c[2] + r * math.cos(t) * uz]
                      for t in (2 * math.pi * j / n for j in range(n))])
    return _tube(rings, closed_ends=True)


def _box(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    pts = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
           [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]
    quads = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 4, 7, 3]]
    tris = [t for a, b, c, d in quads for t in ([a, b, c], [a, c, d])]
    return pts, tris


def _sphere(c, r, nlat=10, nlon=16):
    rings = []
    for i in range(1, nlat):
        phi = math.pi * i / nlat
        rings.append(_ring(c[0], c[1] + r * math.cos(phi), c[2], r * math.sin(phi), nlon))
    pts, tris = _tube(rings)
    top, bottom = len(pts), len(pts) + 1
    pts += [[c[0], c[1] + r, c[2]], [c[0], c[1] - r, c[2]]]
    last = (len(rings) - 1) * nlon
    for j in range(nlon):
        tris.append([top, (j + 1) % nlon, j])
        tris.append([bottom, last + j, last + (j + 1) % nlon])
    return pts, tris


class _Mesh:
    """Accumulates several meshes into one, with a parent rod node per point."""

    def __init__(self):
        self.pts, self.tris, self.parents = [], [], []

    def add(self, mesh, parent=0):
        pts, tris = mesh
        off = len(self.pts)
        self.pts += pts
        self.tris += [[a + off, b + off, c + off] for a, b, c in tris]
        self.parents += [parent] * len(pts)


def _static_model(node, name, mesh, color):
    child = node.addChild(name)
    child.addObject("OglModel", position=mesh.pts, triangles=mesh.tris, color=color)


def _moving_model(rod, name, mesh, color):
    child = rod.addChild(name)
    child.addObject("OglModel", position=mesh.pts, triangles=mesh.tris, color=color)
    child.addObject("RigidMapping", rigidIndexPerPoint=mesh.parents, globalToLocalCoords=True)


# ---------------------------------------------------------------------------
# The rig
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Real Emio hardware (meshes shipped with Emio Labs)
# ---------------------------------------------------------------------------
EMIO_MOTOR_RADIUS = 100.0  # mm from the centre to each motor shaft, as in emio/parts/robot.py
SPOOL_RADIUS_VISUAL = 12.0


def find_emio_meshes():
    """The data/meshes folder of the newest Emio Labs install, or None."""
    import glob
    import os
    found = sorted(glob.glob(os.path.join(os.path.expanduser("~"), "emio-labs", "*", "assets", "data", "meshes")))
    found = [d for d in found if os.path.isfile(os.path.join(d, "motor.stl"))]
    return found[-1] if found else None


def emio_motor_frame(i, motor_y):
    """Shaft position and Y rotation (deg) of Emio motor i, matching emio/parts/robot.py."""
    a = 2 * math.pi * i / 4
    translation = [EMIO_MOTOR_RADIUS * math.sin(a), motor_y, EMIO_MOTOR_RADIUS * math.cos(a)]
    rotation_y = math.degrees(a) + 180.0 * (i % 2 + 1)
    return translation, rotation_y


def _add_stl(node, name, filename, color, translation=(0, 0, 0), rotation=(0, 0, 0)):
    child = node.addChild(name)
    child.addObject("MeshSTLLoader", name="loader", filename=filename,
                    translation=list(translation), rotation=list(rotation))
    child.addObject("OglModel", src=child.loader.linkpath, color=list(color))
    return child


def _add_emio_hardware(static, meshes, motor_y, platform_level):
    import os

    def mesh(f):
        return os.path.join(meshes, f)

    # Box of the extended configuration, platform (the table), camera in the top corner
    _add_stl(static, "EmioBox", mesh("base-extended.stl"), [1, 1, 1, 0.12], translation=[0, motor_y, 0])
    _add_stl(static, "EmioPlatform", mesh(os.path.join("supports", "platform.stl")), [0.9, 0.9, 0.9, 0.5],
             translation=[0, motor_y + 35.0 * platform_level, 0])
    _add_stl(static, "EmioCamera", mesh("camera.stl"), [0.4, 0.4, 0.4, 1],
             translation=[-103.94, motor_y + 5, -103.94], rotation=[45, 45, 0])

    # Motors: same placement as the Motor prefab in emio/parts/motor.py and robot.py
    spools = _Mesh()
    for i in range(4):
        translation, rotation_y = emio_motor_frame(i, motor_y)
        motor = static.addChild(f"EmioMotor{i}")
        motor.addObject("MechanicalObject", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]],
                        translation=translation, rotation=[0, rotation_y, 0])
        visual = motor.addChild("Visual")
        visual.addObject("MeshSTLLoader", name="loader", filename=mesh("motor.stl"),
                         rotation=[[-90, 90][i % 2], 180, 0])
        visual.addObject("OglModel", src=visual.loader.linkpath, color=[0.15, 0.15, 0.15, 1])
        visual.addObject("RigidMapping", index=0)
        # Cable spool on the shaft (the shaft is the motor's local x axis)
        t = math.radians(rotation_y)
        spools.add(_cylinder_axis(translation, (math.cos(t), -math.sin(t)), 8, SPOOL_RADIUS_VISUAL))
    _static_model(static, "Spools", spools, [0.9, 0.75, 0.2, 1])


def _add_rough_hardware(static, top_y, base_radius=182.5):
    """Simple stand-in frame, used when the Emio Labs meshes are not found."""
    plate_t = 6.0
    mount_y = top_y + 4.0
    m = _Mesh()
    m.add(_cylinder_y(0, 0, -plate_t, 0, base_radius, 64))
    _static_model(static, "BasePlate", m, [0.85, 0.85, 0.82, 1])
    m = _Mesh()
    post = 130.0
    for sx in (-1, 1):
        for sz in (-1, 1):
            m.add(_box([sx * post - 8, 0, sz * post - 8], [sx * post + 8, mount_y + plate_t, sz * post + 8]))
    m.add(_box([-post, mount_y + plate_t, -8], [post, mount_y + plate_t + 12, 8]))
    m.add(_box([-8, mount_y + plate_t, -post], [8, mount_y + plate_t + 12, post]))
    _static_model(static, "Frame", m, [0.55, 0.57, 0.6, 1])


def add_rig_visuals(root, rod, frames, disk_nodes, hole_offsets, cable_colors,
                    top_y, rod_radius, disk_radius, motor_y, platform_level=0):
    """
    :param root: scene root (static parts go under a "RigVisual" child)
    :param rod: rod node holding the Rigid3 beam dofs (moving parts are mapped to it)
    :param frames: rest Rigid3 frames of the rod, from the mount (index 0) to the tip
    :param disk_nodes: rod node index of each spacer disk
    :param hole_offsets: horizontal [x, 0, z] offset of each cable hole from the rod axis;
                         cable i runs to Emio motor i
    :param motor_y: height of the Emio motor shafts
    """
    static = root.addChild("RigVisual")
    meshes = find_emio_meshes()
    if meshes:
        _add_emio_hardware(static, meshes, motor_y, platform_level)
    else:
        print("[visuals] Emio Labs meshes not found, drawing a simple frame instead")
        _add_rough_hardware(static, top_y)

    # Rod tube: one ring per rod node, attached to that node
    m = _Mesh()
    n = 10
    rings = [_ring(f[0], f[1], f[2], max(rod_radius, 1.0), n) for f in frames]
    pts, tris = _tube(rings)
    m.pts, m.tris = pts, tris
    m.parents = [i for i in range(len(frames)) for _ in range(n)]
    _moving_model(rod, "RodVisual", m, [0.75, 0.75, 0.78, 1])

    # Spacer disks
    m = _Mesh()
    for k in disk_nodes:
        y = frames[k][1]
        m.add(_cylinder_y(0, 0, y - 1.5, y + 1.5, disk_radius, 28), parent=k)
    _moving_model(rod, "DiskVisual", m, [0.15, 0.35, 0.95, 1])

    # Magnet tip
    m = _Mesh()
    tip = frames[-1]
    m.add(_sphere([tip[0], tip[1] - 4.0, tip[2]], 5.0), parent=len(frames) - 1)
    _moving_model(rod, "MagnetTip", m, [0.1, 0.85, 0.25, 1])

    # Cables: straight segments between holes. The part above the first disk runs up
    # through the mount to the spool; it is attached to the clamped rod node 0, so it stays put.
    # Above the rod the cable rises to the top of its motor's spool, then runs out to it.
    cr, nc = 0.6, 6
    run_y = motor_y + SPOOL_RADIUS_VISUAL
    for i, off in enumerate(hole_offsets):
        r = math.hypot(off[0], off[2])
        dx, dz = off[0] / r, off[2] / r
        spool_x, spool_z = EMIO_MOTOR_RADIUS * dx, EMIO_MOTOR_RADIUS * dz
        m = _Mesh()
        m.add(_tube([_ring(off[0], run_y, off[2], cr, nc), _ring(off[0], top_y, off[2], cr, nc)]))
        m.add(_box([min(off[0], spool_x) - cr, run_y - cr, min(off[2], spool_z) - cr],
                   [max(off[0], spool_x) + cr, run_y + cr, max(off[2], spool_z) + cr]))
        chain = [0] + list(disk_nodes)
        for a, b in zip(chain, chain[1:]):
            ya, yb = frames[a][1], frames[b][1]
            seg_pts, seg_tris = _tube([_ring(off[0], ya, off[2], cr, nc), _ring(off[0], yb, off[2], cr, nc)])
            base = len(m.pts)
            m.pts += seg_pts
            m.tris += [[p + base for p in t] for t in seg_tris]
            m.parents += [a] * nc + [b] * nc
        _moving_model(rod, f"Cable{i + 1}Visual", m, cable_colors[i])
