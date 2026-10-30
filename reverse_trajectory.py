r"""
reverse_trajectory.py
---------------------
Make a trajectory that plays a recording backwards: from its end pose, along the same
path (pauses included), back to its start pose.

    emio.bat reverse_trajectory.py trajectories\sliders_20261029_134036.json
    -> trajectories\sliders_20261029_134036_reverse.json

The reversed file starts at the recording's end pose ("start_angles_deg" = end pose), so
the GUIs (cabledriven_sliders.py, cabledriven_dpad_v3.py) play it from where the robot is
now. Use it with the robot at the end pose of the original: if the GUI was restarted or the
zero was set elsewhere, press "Set zero here" there first, then Load + Replay.
"""

import argparse
import json
import os
import time

from cabledriven import trajectory_changes


def reverse(data):
    traj = data["trajectory"] if isinstance(data, dict) else data
    total = traj[-1]["t"]
    changes = trajectory_changes(data)  # + = pull, old files' M2 / M4 direction corrected
    end = changes[-1][1]
    # "delta" is now the change from the end pose of the recording (where the reverse starts)
    return [{"t": round(total - p["t"], 3), "angles": list(p["angles"]),
             "delta": [round(c - e, 2) for c, e in zip(ch, end)]}
            for p, (_, ch) in zip(reversed(traj), reversed(changes))]


def main():
    p = argparse.ArgumentParser(description="Play a recorded trajectory backwards.")
    p.add_argument("file")
    p.add_argument("-o", "--out", default=None, help="output file (default: <file>_reverse.json)")
    args = p.parse_args()

    with open(args.file, encoding="utf-8") as f:
        data = json.load(f)
    points = reverse(data)
    out = args.out or os.path.splitext(args.file)[0] + "_reverse.json"
    result = {"name": os.path.basename(out), "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
              "reversed_from": os.path.basename(args.file),
              "duration_s": points[-1]["t"], "num_points": len(points),
              "start_angles_deg": points[0]["angles"], "trajectory": points}
    with open(out, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(f"[saved] {out}")
    print(f"  starts at {points[0]['angles']}  (end of the recording)")
    print(f"  ends at   {points[-1]['angles']}  (start of the recording), {points[-1]['t']:.1f} s")


if __name__ == "__main__":
    main()
