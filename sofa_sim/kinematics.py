"""
kinematics.py
-------------
Bending model of the cable-driven rod (pure Python, no SOFA), for planning and control.

The rod cannot translate: it is clamped at the top and can only bend. In free space it
bends with (piecewise) constant curvature, the standard model for tendon-driven continuum
robots, so its whole shape is one circular arc described by two numbers:
    theta  total bending angle (rad), 0 = hanging straight
    phi    bending direction (rad) in the horizontal plane, 0 = towards M1
With 4 motors but only 2 numbers, the tip can only reach a 2D surface (a dome) under the
clamp, never an arbitrary 3D point, and it moves along curves, not straight lines.

Tendon equation: cable i, at angle psi_i around the rod and offset d from its axis, is
shortened by  dl_i = d * theta * cos(phi - psi_i).  A motor angle a_i (rad) pulls in
r_spool * a_i of cable; the opposite cable lets out the same amount (see
OPPOSITE_CABLE_PAYS_OUT in cable_rod_scene.py), so
    theta * cos(phi) = r_spool * (a1 - a3) / d
    theta * sin(phi) = r_spool * (a2 - a4) / d

Frame (rig, same as cartesian_controller.py): X towards M1, Y towards M2, Z = height above
the platform. Units: mm, rad. Contact with obstacles is not part of this model (it is in
the SOFA simulation); collides() tells you when a predicted shape would hit something.
"""

import math

# Rig values, kept equal to cable_rod_scene.py
ROD_LENGTH = 185.0
CLAMP_HEIGHT = 280.0
TENDON_OFFSET = 5.0
SPOOL_RADIUS = 12.0
DISK_RADIUS = 10.0
NUM_DISKS = 9
CABLE_DIRECTIONS = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]  # M1..M4


# ---------------------------------------------------------------------------
# Motors <-> bending
# ---------------------------------------------------------------------------

def motors_to_bending(angles):
    """Motor angles [a1..a4] (rad) -> (theta, phi)."""
    a1, a2, a3, a4 = angles
    kx = SPOOL_RADIUS * (a1 - a3) / TENDON_OFFSET
    ky = SPOOL_RADIUS * (a2 - a4) / TENDON_OFFSET
    return math.hypot(kx, ky), math.atan2(ky, kx)


def bending_to_motors(theta, phi):
    """(theta, phi) -> motor angles [a1..a4] (rad). Each axis pulls one motor, the opposite stays at 0."""
    kx, ky = theta * math.cos(phi), theta * math.sin(phi)
    s = TENDON_OFFSET / SPOOL_RADIUS
    return [max(kx, 0) * s, max(ky, 0) * s, max(-kx, 0) * s, max(-ky, 0) * s]


def cable_length_changes(theta, phi):
    """Shortening of each cable (mm, > 0 = pulled in) needed for this bend."""
    return [TENDON_OFFSET * theta * math.cos(phi - psi) for psi in CABLE_DIRECTIONS]


# ---------------------------------------------------------------------------
# Shape
# ---------------------------------------------------------------------------

def point_at(theta, phi, s):
    """Position (X, Y, Z) of the rod axis at arc length s (0 = clamp, ROD_LENGTH = tip)."""
    if abs(theta) < 1e-9:
        return (0.0, 0.0, CLAMP_HEIGHT - s)
    k = theta / ROD_LENGTH                    # curvature (1/mm)
    radial = (1 - math.cos(k * s)) / k        # sideways distance from the clamp axis
    drop = math.sin(k * s) / k                # how far below the clamp
    return (radial * math.cos(phi), radial * math.sin(phi), CLAMP_HEIGHT - drop)


def tangent_at(theta, phi, s):
    """Unit direction of the rod at arc length s (points from the clamp towards the tip)."""
    a = theta * s / ROD_LENGTH  # angle turned so far
    return (math.sin(a) * math.cos(phi), math.sin(a) * math.sin(phi), -math.cos(a))


def backbone(theta, phi, n=37):
    """n points along the rod axis, clamp to tip."""
    return [point_at(theta, phi, ROD_LENGTH * i / (n - 1)) for i in range(n)]


def tip(theta, phi):
    return point_at(theta, phi, ROD_LENGTH)


def disk_centres(theta, phi):
    return [point_at(theta, phi, ROD_LENGTH * (k + 1) / NUM_DISKS) for k in range(NUM_DISKS)]


def forward(angles):
    """Motor angles (rad) -> dict with theta, phi, tip, backbone."""
    theta, phi = motors_to_bending(angles)
    return {"theta": theta, "phi": phi, "tip": tip(theta, phi), "backbone": backbone(theta, phi)}


# ---------------------------------------------------------------------------
# Inverse kinematics and workspace
# ---------------------------------------------------------------------------

def inverse(target, theta_max=2 * math.pi):
    """
    Closest reachable bend to a target tip position (X, Y, Z).
    The direction comes straight from the target; the bend is found by a 1D search, since
    the tip can only lie on the dome of reachable points.
    Returns (theta, phi, motor angles, distance from the target to the reached tip in mm).
    """
    x, y, z = target
    phi = math.atan2(y, x) if math.hypot(x, y) > 1e-9 else 0.0
    best = None
    steps = 720
    for i in range(steps + 1):
        th = theta_max * i / steps
        err = math.dist(tip(th, phi), target)
        if best is None or err < best[1]:
            best = (th, err)
    # refine around the best grid value
    lo, hi = max(0.0, best[0] - theta_max / steps), min(theta_max, best[0] + theta_max / steps)
    for _ in range(40):
        m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
        if math.dist(tip(m1, phi), target) < math.dist(tip(m2, phi), target):
            hi = m2
        else:
            lo = m1
    th = (lo + hi) / 2
    return th, phi, bending_to_motors(th, phi), math.dist(tip(th, phi), target)


def tip_path(theta_start, theta_end, phi, n=20):
    """Tip positions while bending from theta_start to theta_end in direction phi: a curve, not a line."""
    return [tip(theta_start + (theta_end - theta_start) * i / (n - 1), phi) for i in range(n)]


def workspace(theta_max=math.pi, n_theta=30, n_phi=36):
    """Sample of reachable tip positions."""
    return [tip(theta_max * i / (n_theta - 1), 2 * math.pi * j / n_phi)
            for i in range(n_theta) for j in range(n_phi)]


# ---------------------------------------------------------------------------
# Obstacles at any angle: oriented boxes
# ---------------------------------------------------------------------------

class Box:
    """
    A box obstacle at any orientation: centre (X, Y, Z), half sizes along its own axes, and
    rotation as yaw (about Z), pitch (about the box's Y), roll (about its X), in degrees.
    A bar tilted 20 deg downwards is e.g. Box(c, (60, 5, 5), pitch=20).
    """

    def __init__(self, centre, half_sizes, yaw=0.0, pitch=0.0, roll=0.0, name=""):
        self.c = centre
        self.h = half_sizes
        self.name = name
        cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
        cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        # rotation matrix R = Rz(yaw) Ry(pitch) Rx(roll); columns are the box axes in the rig frame
        self.R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]]

    def distance(self, p):
        """Distance (mm) from point p to the box surface; 0 inside."""
        d = [p[i] - self.c[i] for i in range(3)]
        local = [sum(self.R[r][a] * d[r] for r in range(3)) for a in range(3)]  # R^T d
        out = [max(abs(local[a]) - self.h[a], 0.0) for a in range(3)]
        return math.sqrt(sum(o * o for o in out))


def clearance(theta, phi, obstacles, rod_radius=1.0):
    """Smallest gap (mm) between the predicted rod (axis and disk rims) and any obstacle."""
    gap = math.inf
    for p in backbone(theta, phi):
        for ob in obstacles:
            gap = min(gap, ob.distance(p) - rod_radius)
    for k in range(NUM_DISKS):
        s = ROD_LENGTH * (k + 1) / NUM_DISKS
        c, t = point_at(theta, phi, s), tangent_at(theta, phi, s)
        # two unit vectors across the disk (perpendicular to the rod)
        u = (-math.sin(phi), math.cos(phi), 0.0)
        v = (t[1] * u[2] - t[2] * u[1], t[2] * u[0] - t[0] * u[2], t[0] * u[1] - t[1] * u[0])
        for j in range(12):
            a = 2 * math.pi * j / 12
            rim = [c[i] + DISK_RADIUS * (math.cos(a) * u[i] + math.sin(a) * v[i]) for i in range(3)]
            for ob in obstacles:
                gap = min(gap, ob.distance(rim))
    return gap


def collides(theta, phi, obstacles, margin=0.0):
    return clearance(theta, phi, obstacles) < margin


if __name__ == "__main__":
    # Small demo: what one motor does, and an inverse-kinematics round trip.
    for deg in (0, 15, 30, 45, 60, 90):
        th, ph = motors_to_bending([math.radians(deg), 0, 0, 0])
        x, y, z = tip(th, ph)
        print(f"M1 {deg:3d} deg -> bend {math.degrees(th):6.1f} deg, tip X {x:6.1f}  Y {y:5.1f}  Z {z:6.1f} mm")
    target = (60.0, 40.0, 120.0)
    th, ph, motors, err = inverse(target)
    print(f"target {target}: bend {math.degrees(th):.1f} deg towards {math.degrees(ph):.1f} deg, "
          f"motors {[round(math.degrees(a), 1) for a in motors]} deg, closest reachable tip is {err:.1f} mm away")
