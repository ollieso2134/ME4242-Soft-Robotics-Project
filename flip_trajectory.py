r"""
flip_trajectory.py
------------------
Turn trajectories for an obstacle setup rotated 180 degrees about the rod: each cable does
what the opposite one did, so M1 <-> M3 and M2 <-> M4 swap their changes (+ = pull).

    emio.bat flip_trajectory.py "trajectories\new config1\new config 1 optimized.json"
    -> ..._flipped.json

    emio.bat flip_trajectory.py "trajectories\new config1\*_opt.json" -d "trajectories\new config1 flipped"
"""

import argparse
import glob
import json
import os
import time

from cabledriven import trajectory_changes

OPPOSITE = (2, 3, 0, 1)  # new motor i plays old motor OPPOSITE[i]: M1<-M3, M2<-M4, M3<-M1, M4<-M2


def flip(data):
    base = data.get("start_angles_deg") or [0.0] * 4
    return [{"t": t, "angles": [round(b + d[j], 2) for b, j in zip(base, OPPOSITE)],
             "delta": [round(d[j], 2) for j in OPPOSITE]}
            for t, d in trajectory_changes(data)]


def main():
    p = argparse.ArgumentParser(description="Flip trajectories for an obstacle setup rotated 180 degrees.")
    p.add_argument("files", nargs="+", help="trajectory files (wildcards allowed)")
    p.add_argument("-d", "--out-dir", default=None, help="folder for the flipped files (default: next to each file)")
    args = p.parse_args()

    files = [f for pattern in args.files for f in (sorted(glob.glob(pattern)) or [pattern])]
    for path in files:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        points = flip(data)
        name = os.path.splitext(os.path.basename(path))[0] + "_flipped.json"
        out = os.path.join(args.out_dir or os.path.dirname(path), name)
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        result = {"name": name, "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                  "flipped_from": os.path.basename(path), "flip": "180 deg: M1<->M3, M2<->M4",
                  "duration_s": points[-1]["t"], "num_points": len(points),
                  "start_angles_deg": data.get("start_angles_deg") or [0.0] * 4, "trajectory": points}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        print(f"[saved] {out}  ({points[-1]['t']:.1f} s, ends at {points[-1]['delta']})")


if __name__ == "__main__":
    main()
