"""
recording.py
------------
Record the motor angles while you move the rod by hand (sliders or keys), save them,
and play them back.

- Record: press "Record" in the "Cable rod: motors" window, move the rod, press "Stop".
  The motion is saved to sofa_sim/recordings/rec_<date>_<time>.json.
- Play back the newest recording:   sofa_sim\\run_sim.bat play
  or a specific one:                 sofa_sim\\run_sim.bat play:rec_20261026_143000

What is recorded is the motor angle the simulated motors actually reached, sampled every
SAMPLE_PERIOD of simulation time, so playback repeats the same motion whatever the input was.
Playback starts from the straight rod; start recording from the straight rod (all 0) to
get the same result.
"""

import glob
import json
import os
import time

RECORD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
SAMPLE_PERIOD = 0.05  # s of simulation time between samples


def save(samples):
    """Save [[t, m1, m2, m3, m4], ...] (s, deg). Returns the file path."""
    os.makedirs(RECORD_DIR, exist_ok=True)
    path = os.path.join(RECORD_DIR, time.strftime("rec_%Y%m%d_%H%M%S") + ".json")
    rows = ",\n  ".join(json.dumps([round(v, 3) for v in s]) for s in samples)
    with open(path, "w", encoding="utf-8") as f:
        f.write('{"units": {"t": "s", "angles": "deg"}, "motors": ["M1", "M2", "M3", "M4"],\n'
                f' "sample_period": {SAMPLE_PERIOD},\n'
                f' "samples": [\n  {rows}\n ]}}\n')
    return path


def load(name=None):
    """Load a recording by name (with or without .json), or the newest one. Returns (path, samples)."""
    if name:
        path = name if os.path.isabs(name) else os.path.join(RECORD_DIR, name)
        if not path.endswith(".json"):
            path += ".json"
    else:
        files = sorted(glob.glob(os.path.join(RECORD_DIR, "rec_*.json")))
        if not files:
            raise FileNotFoundError(f"no recordings in {RECORD_DIR}")
        path = files[-1]
    with open(path, encoding="utf-8") as f:
        return path, json.load(f)["samples"]


def angles_at(samples, t):
    """Motor angles (deg) at time t, interpolated linearly between samples."""
    if t <= samples[0][0]:
        return samples[0][1:]
    for a, b in zip(samples, samples[1:]):
        if t <= b[0]:
            f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 1.0
            return [x + f * (y - x) for x, y in zip(a[1:], b[1:])]
    return samples[-1][1:]
