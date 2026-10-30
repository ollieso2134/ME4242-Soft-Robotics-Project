"""
weave.py
--------
Hardcoded (kinematic) animation for cable_rod_scene.py: the rod bends, from the top
down, into an S that snakes along the ends of the three bars on the backboard. Played with:
    sofa_sim\run_sim.bat weave

This is not physics: the rod shape is set directly every step (no cables, no contact).
The rod keeps its real length. The shape is given as a list of BENDS and was checked
(see check_clearance) to keep the rod and its 10 mm-radius disks at least 3 mm from
every bar at every moment of the motion, not only at the end.

Board coordinates used here (looking at the board from the rod side):
    l  sideways along the board; the rod hangs at l = 0, the bars are at l = -35..-24
    y  height above the platform (the rod hangs from y = 280)
Bars (heights): top 216.5-226.5, middle 186.5-196.5, low 156.5-166.5 mm; they stick out
50, 35 and 20 mm from the board, and the rod runs about 10 mm from it, so the rod passes
beside them. The rod stays on its own side of the bars (l > -24).
"""

import math

import Sofa
import Sofa.Core

# The final shape, from the clamp down: (length of rod in mm, how much it turns in degrees).
# Positive turns bend away from the bars, negative turns bend towards them. Each bend has a
# constant curvature; the lengths add up to the rod length (185 mm).
BENDS = [
    (11.2, 0.0),       # straight below the clamp
    (41.7, 48.6),      # 1st bend: away from the bars
    (60.6, -140.2),    # 2nd bend: back towards them, hugging the middle bar
    (33.0, 138.1),     # 3rd bend: away again
    (38.5, 110.9),     # tip curls up beside the low bar's end
]
W_FINAL = 12.0        # mm out from the board's back face (the bars stick out up to 60-140 mm)

T_START = 1.0         # s, hold straight before moving
T_SWEEP = 4.0         # s, the bending travels from the top of the rod to the tip in this time
T_RAMP = 2.0          # s, time for each part of the rod to reach its final bend

BARS = [(216.5, 226.5), (186.5, 196.5), (156.5, 166.5)]  # heights of the top, middle, low bars


def final_headings(num_segments, seg):
    """Heading of each rod segment (rad, 0 = straight down, > 0 away from the bars) in the final shape."""
    headings = []
    for i in range(num_segments):
        s = (i + 0.5) * seg  # middle of the segment, measured along the rod from the clamp
        acc, h = 0.0, 0.0
        for length, turn in BENDS:
            if s <= acc + length:
                h += math.radians(turn) * (s - acc) / length
                break
            h += math.radians(turn)
            acc += length
        headings.append(h)
    return headings


def planar_shape(final, t, seg, top_y=280.0):
    """(l, y) of every rod node at time t of the animation, and the segment headings."""
    headings = [phi * _smooth((t - (T_START + T_SWEEP * i / max(1, len(final) - 1))) / T_RAMP)
                for i, phi in enumerate(final)]
    pts = [(0.0, top_y)]
    for h in headings:
        pts.append((pts[-1][0] + seg * math.sin(h), pts[-1][1] - seg * math.cos(h)))
    return pts, headings


def check_clearance(num_nodes, seg, disk_radius=10.0, rod_radius=1.0, disk_every=2):
    """Smallest gap (mm) between the rod or a disk and a bar, over the whole animation."""
    def gap(l, y, bar):
        return math.hypot(l - min(max(l, -35.0), -24.0), y - min(max(y, bar[0]), bar[1]))
    final = final_headings(num_nodes - 1, seg)
    worst = math.inf
    for k in range(int((T_START + T_SWEEP + T_RAMP) / 0.1) + 2):
        pts, hs = planar_shape(final, k * 0.1, seg)
        for a, b in zip(pts, pts[1:]):
            for j in range(9):
                p = (a[0] + j / 8 * (b[0] - a[0]), a[1] + j / 8 * (b[1] - a[1]))
                worst = min(worst, min(gap(*p, bar) for bar in BARS) - rod_radius)
        for n in range(disk_every, num_nodes, disk_every):  # a disk is a thin plate across the rod
            h = hs[min(n, len(hs) - 1)]
            for j in range(-5, 6):
                u = disk_radius * j / 5
                worst = min(worst, min(gap(pts[n][0] + u * math.cos(h), pts[n][1] + u * math.sin(h), bar) for bar in BARS))
    return worst


def _smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


class WeaveController(Sofa.Core.Controller):
    """Sets the rod frames every step, blending from straight to the final S."""

    def __init__(self, rod_dofs, top_y, seg, num_nodes, board_rotation_y_deg, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "WeaveController"
        self.dofs = rod_dofs
        self.top_y = top_y
        self.seg = seg
        self.n = num_nodes
        t = math.radians(board_rotation_y_deg)
        self.L = (math.cos(t), 0.0, -math.sin(t))   # board sideways direction (+l) in SOFA
        self.W = (-math.sin(t), 0.0, -math.cos(t))  # out from the board (+w) in SOFA
        self.axis = (math.sin(t), 0.0, math.cos(t))  # bend axis: rotating "down" about it turns towards +l
        self.final = final_headings(num_nodes - 1, seg)
        gap = check_clearance(num_nodes, seg)
        if gap < 0:
            print(f"[weave] WARNING: with these BENDS the rod or a disk goes {-gap:.1f} mm into a bar during the motion")
        else:
            print(f"[weave] the rod and disks stay at least {gap:.1f} mm from the bars during the whole motion")
        self.time = 0.0
        self.step = 0
        self.q_down = [0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)]

    def frames_at(self, t):
        headings = []
        for i, phi in enumerate(self.final):
            start = T_START + T_SWEEP * i / max(1, len(self.final) - 1)
            headings.append(phi * _smooth((t - start) / T_RAMP))
        frames, l, y = [], 0.0, self.top_y
        for i in range(self.n):
            s = i * self.seg
            w = 8.5 + (W_FINAL - 8.5) * _smooth(s / 40.0)
            pos = [l * self.L[k] + (w - 8.5) * self.W[k] for k in range(3)]
            pos[1] += y
            # node orientation: average heading of the segments around it
            if i == 0:
                phi = 0.0
            elif i == self.n - 1:
                phi = headings[-1]
            else:
                phi = 0.5 * (headings[i - 1] + headings[i])
            h = 0.5 * phi
            q_bend = [self.axis[0] * math.sin(h), self.axis[1] * math.sin(h), self.axis[2] * math.sin(h), math.cos(h)]
            frames.append(pos + _qmul(q_bend, self.q_down))
            if i < self.n - 1:
                l += self.seg * math.sin(headings[i])
                y -= self.seg * math.cos(headings[i])
        return frames

    def onAnimateBeginEvent(self, event):
        self.time += self.getContext().getRoot().dt.value
        self.dofs.position.value = self.frames_at(self.time)
        self.step += 1
        if self.step % 100 == 0:
            end = T_START + T_SWEEP + T_RAMP
            print(f"[weave] t={self.time:5.2f}s  {'done' if self.time > end else f'bending ({100 * min(1, self.time / end):.0f}%)'}")
