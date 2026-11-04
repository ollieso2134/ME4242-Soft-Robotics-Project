r"""
optimize_trajectory.py
----------------------
Shorten recorded trajectories by cutting the waiting: every pause (all motors still) is
cut to HOLD_S, long enough for the motors to reach the target and the rod to settle before
the next move. The moves themselves keep their recorded speed (at most the slider speed),
so the motors can follow them. A pause at the start of a file is dropped.

    emio.bat optimize_trajectory.py "trajectories\new config1\sliders_..._collecting.json" --fast-reset 2
    -> ..._collecting_opt.json

    --hold 0.8       pause kept after each move (default 0.5 s)
    --fast-reset 2   play the moves back to zero (the start pose) 2x faster

Several files with -o join them in that order into one trajectory. If a file does not
start where the previous one ended, a move at the slider speed bridges the gap:

    emio.bat optimize_trajectory.py a.json b.json c.json -o "trajectories\combined.json"
"""

import argparse
import json
import os
import time

from cabledriven import trajectory_changes

HOLD_S = 0.5            # pause kept after each move: the motors reach the target, the rod settles
SPEED_DEG_PER_S = 60.0  # slider speed, used for bridging moves
STILL_DEG = 0.05


def _same(a, b):
    return max(abs(x - y) for x, y in zip(a, b)) < STILL_DEG


def _runs(changes):
    """The steps between points, (dt, delta at the end), grouped into pauses and moves."""
    runs = []
    for (t0, a0), (t1, a1) in zip(changes, changes[1:]):
        kind = "hold" if _same(a0, a1) else "move"
        if not runs or runs[-1][0] != kind:
            runs.append((kind, []))
        runs[-1][1].append((t1 - t0, a1))
    return runs


def optimize(changes, hold=HOLD_S, fast_reset=1.0):
    """[(t, delta)] with every pause cut to `hold` s (a leading pause dropped) and, with
    fast_reset > 1, the moves that end back at zero played that much faster."""
    t, pose = 0.0, changes[0][1]
    out = [(0.0, pose)]
    for i, (kind, steps) in enumerate(_runs(changes)):
        total = sum(dt for dt, _ in steps)
        if kind == "hold":
            if i == 0 or total <= 0:
                continue
            scale = min(1.0, hold / total)
        else:
            to_zero = fast_reset > 1 and _same(steps[-1][1], [0.0] * 4) and not _same(pose, [0.0] * 4)
            scale = 1.0 / fast_reset if to_zero else 1.0
        for dt, d in steps:
            t += dt * scale
            out.append((round(t, 3), d))
        pose = steps[-1][1]
    return out


def join(parts, hold=HOLD_S):
    """Play the parts one after the other, bridging any gap between them at the slider speed."""
    out = list(parts[0])
    for part in parts[1:]:
        t, end = out[-1]
        start = part[0][1]
        if not _same(end, start):
            t += max(abs(a - b) for a, b in zip(start, end)) / SPEED_DEG_PER_S
            out.append((round(t, 3), start))
            t += hold
            out.append((round(t, 3), start))
        out.extend((round(t + pt, 3), d) for pt, d in part[1:])
    return out


def _load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def main():
    p = argparse.ArgumentParser(description="Cut the pauses out of recorded trajectories.")
    p.add_argument("files", nargs="+")
    p.add_argument("-o", "--out", default=None,
                   help="output file; with several input files they are joined into it in order")
    p.add_argument("--hold", type=float, default=HOLD_S, help=f"pause kept after each move (default {HOLD_S} s)")
    p.add_argument("--fast-reset", type=float, default=1.0, help="speed-up of the moves back to zero (default 1)")
    args = p.parse_args()
    if len(args.files) > 1 and not args.out:
        p.error("joining several files needs -o OUT")

    datas = [_load(f) for f in args.files]
    parts = [optimize(trajectory_changes(d), args.hold, args.fast_reset) for d in datas]
    points = join(parts, args.hold)
    out = args.out or os.path.splitext(args.files[0])[0] + "_opt.json"
    base = datas[0].get("start_angles_deg") if isinstance(datas[0], dict) else None
    base = base or [0.0] * 4
    traj = [{"t": t, "angles": [round(b + x, 2) for b, x in zip(base, d)], "delta": [round(x, 2) for x in d]}
            for t, d in points]
    result = {"name": os.path.basename(out), "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
              "optimized_from": [os.path.basename(f) for f in args.files],
              "hold_s": args.hold, "fast_reset": args.fast_reset,
              "duration_s": traj[-1]["t"], "num_points": len(traj),
              "start_angles_deg": base, "trajectory": traj}
    with open(out, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    before = sum(trajectory_changes(d)[-1][0] for d in datas)
    print(f"[saved] {out}")
    print(f"  {before:.1f} s -> {traj[-1]['t']:.1f} s, ends at {traj[-1]['delta']}")


if __name__ == "__main__":
    main()
