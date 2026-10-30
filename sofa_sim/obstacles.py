"""
obstacles.py
------------
Contact between the rod and fixed obstacles, for cable_rod_scene.py.

- Rod side: small collision spheres along the rod axis and around each spacer disk
  rim, rigidly attached to the rod, so the obstacles push on the rod and disks.
- Obstacle side: either an STL exported from CAD, or placeholder horizontal pegs.
  Pegs slide in from the side at the start, so they never begin inside the rod.

Contacts are friction contacts solved together with the cable constraints.
"""

import math

import Sofa
import Sofa.Core

from rig_visuals import _cylinder_axis


def add_collision_header(root, friction=0.3, alarm_distance=2.0, contact_distance=0.2):
    # Tight distances: with larger ones the rod, wedged between the board and the bars,
    # made the contact solver diverge.
    root.addObject("RequiredPlugin", name="CollisionPlugins", pluginName=[
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.Mapping.Linear",
    ])
    root.addObject("CollisionPipeline")
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject("LocalMinDistance", alarmDistance=alarm_distance, contactDistance=contact_distance,
                   angleCone=0.0)
    root.addObject("RuleBasedContactManager", responseParams=f"mu={friction}",
                   response="FrictionContactConstraint")


def add_rod_collision(rod, frames, disk_nodes, rod_radius, disk_radius, spacing=3.0, rim_points=12):
    """Collision spheres along the rod and around the spacer disk rims."""
    pts, parents, radii = [], [], []
    top_y, tip_y = frames[0][1], frames[-1][1]
    seg = (top_y - tip_y) / (len(frames) - 1)
    sphere_r = max(rod_radius, 1.0)

    # Along the rod, skipping the clamped top: each sphere follows its nearest rod node
    n = int((top_y - tip_y) / spacing)
    for k in range(1, n + 1):
        y = top_y - k * spacing
        node = min(len(frames) - 1, int(round((top_y - y) / seg)))
        pts.append([0.0, y, 0.0])
        parents.append(node)
        radii.append(sphere_r)

    # Disk rims
    rim_r = 1.5
    for node in disk_nodes:
        y = frames[node][1]
        for j in range(rim_points):
            a = 2 * math.pi * j / rim_points
            pts.append([(disk_radius - rim_r) * math.cos(a), y, (disk_radius - rim_r) * math.sin(a)])
            parents.append(node)
            radii.append(rim_r)

    col = rod.addChild("Collision")
    col.addObject("MechanicalObject", name="spheres", template="Vec3", position=pts)
    col.addObject("SphereCollisionModel", listRadius=radii, group=1)
    col.addObject("RigidMapping", rigidIndexPerPoint=parents, globalToLocalCoords=True)
    return col


def _add_static_mesh(root, name, pts, tris, color, moving=False):
    node = root.addChild(name)
    node.addObject("MeshTopology", position=pts, triangles=tris)
    node.addObject("MechanicalObject", name="dofs", template="Vec3", position=pts)
    for model in ("TriangleCollisionModel", "LineCollisionModel", "PointCollisionModel"):
        node.addObject(model, moving=moving, simulated=False, group=2)
    visual = node.addChild("Visual")
    visual.addObject("OglModel", position=pts, triangles=tris, color=color)
    visual.addObject("IdentityMapping")
    return node


def stl_bounds(filename):
    """Bounding box ([xmin, ymin, zmin], [xmax, ymax, zmax]) of a binary STL file."""
    import struct
    with open(filename, "rb") as f:
        data = f.read()
    count = struct.unpack("<I", data[80:84])[0]
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for i in range(count):
        v = struct.unpack("<9f", data[84 + i * 50 + 12:84 + i * 50 + 48])
        for k in range(9):
            lo[k % 3] = min(lo[k % 3], v[k])
            hi[k % 3] = max(hi[k % 3], v[k])
    return lo, hi


def add_stl_obstacle(root, filename, translation=(0, 0, 0), rotation=(0, 0, 0), scale=1.0,
                     color=(0.85, 0.3, 0.3, 1), name="ObstacleSTL"):
    """
    Fixed obstacle from an STL file (SOFA frame, mm). Returns the node.
    The loader scales, then rotates (Euler angles in degrees), then translates.
    """
    node = root.addChild(name)
    node.addObject("MeshSTLLoader", name="loader", filename=filename,
                   translation=list(translation), rotation=list(rotation), scale=scale)
    node.addObject("MeshTopology", src="@loader")
    node.addObject("MechanicalObject", name="dofs", template="Vec3")
    for model in ("TriangleCollisionModel", "LineCollisionModel", "PointCollisionModel"):
        node.addObject(model, moving=False, simulated=False, group=2)
    visual = node.addChild("Visual")
    visual.addObject("OglModel", src="@../loader", color=list(color))
    visual.addObject("IdentityMapping")
    return node


class PegInserter(Sofa.Core.Controller):
    """Slides each peg from its start offset to its final position over `duration` seconds."""

    def __init__(self, pegs, duration=2.0, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "PegInserter"
        self.pegs = pegs  # list of (mechanical object, rest points, direction, start offset)
        self.duration = duration
        self.time = 0.0

    def onAnimateBeginEvent(self, event):
        if self.time > self.duration:
            return
        self.time += self.getContext().getRoot().dt.value
        s = min(self.time / self.duration, 1.0)
        s = (1 - math.cos(math.pi * s)) / 2  # smooth start and stop
        for mo, rest, (ux, uz), start in self.pegs:
            off = (1 - s) * start
            mo.position.value = [[p[0] + off * ux, p[1], p[2] + off * uz] for p in rest]


def add_pegs(root, pegs, rod_radius, post_distance=130.0):
    """
    Placeholder pegs: horizontal cylinders fixed to the frame, pointing at the rod.
    Each peg is a dict with:
      height (mm above the base plate), side (deg, rig X-Y plane: 0 = +X, 90 = +Y),
      past_centre (mm the tip reaches past the rod axis), radius (mm).
    """
    entries = []
    for i, peg in enumerate(pegs):
        a = math.radians(peg["side"])
        ux, uz = math.cos(a), -math.sin(a)  # direction from the rod towards the peg's post (SOFA x-z)
        r_tip, r_end = -peg["past_centre"], post_distance
        mid = (r_tip + r_end) / 2
        centre = [mid * ux, peg["height"], mid * uz]
        pts, tris = _cylinder_axis(centre, (ux, uz), r_end - r_tip, peg["radius"], 20)
        node = _add_static_mesh(root, f"Peg{i + 1}", pts, tris, [0.85, 0.3, 0.3, 1], moving=True)
        # Start with the tip clear of the rod, then slide in
        start = peg["past_centre"] + peg["radius"] + rod_radius + 3.0
        entries.append((node.dofs, pts, (ux, uz), start))
        node.dofs.position.value = [[p[0] + start * ux, p[1], p[2] + start * uz] for p in pts]
    root.addObject(PegInserter(entries))


def translation_for_point(point, target, rotation_y_deg, scale=1.0):
    """
    Translation that puts `point` (in the STL's own coordinates) at `target` after the
    loader has scaled it and rotated it by rotation_y_deg about the vertical axis.
    """
    t = math.radians(rotation_y_deg)
    x, y, z = (c * scale for c in point)
    rx = x * math.cos(t) + z * math.sin(t)
    rz = -x * math.sin(t) + z * math.cos(t)
    return [target[0] - rx, target[1] - y, target[2] - rz]
