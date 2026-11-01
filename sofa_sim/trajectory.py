"""
trajectory.py
-------------
Scripted motor sequence for cable_rod_scene.py, played with:
    sofa_sim\\run_sim.bat path

Each keyframe is (seconds, [M1, M2, M3, M4] in degrees): the motors move towards the
angles (at MAX_MOTOR_SPEED) and the keyframe lasts that many seconds; the last one is held.

Directions relative to the backboard (from the simulation):
    M1 + M4  sideways along the board, towards the bars
    M2 + M3  sideways along the board, away from the bars
    M3 + M4  out from the board
    M1 + M2  into the board

The sequence below curls the rod towards the bars so it passes under the lowest bar
and its tip rises on the far side, then eases off so the rod rests against that bar.
"""

KEYFRAMES = [
    (2.0, [0, 0, 0, 0]),     # start straight
    (3.0, [80, 0, 0, 60]),   # curl towards the bars: under the lowest bar, tip up the far side
    (3.0, [40, 0, 0, 40]),   # ease off: the rod settles against the lowest bar
]
