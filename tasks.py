"""Task table for the dual-Franka tabletop scene.

Data only, imported by both the Isaac Sim process (to build the scene and verify success)
and the Dream bridge process (to offer the instructions in the dashboard).

World layout (Z up, table top at z = 0, arms look along +x):
    left arm base  y = +0.45 | red cube  (0.50, +0.30) | green bin (0.72, +0.30)
    right arm base y = -0.45 | blue cube (0.50, -0.30) | yellow bin (0.72, -0.30)
"""

TABLE_CENTER = (0.50, 0.0, -0.02)
TABLE_SCALE = (0.90, 1.50, 0.04)
CUBE_HALF = 0.025

OBJECTS = {
    "red": {"color": (1.0, 0.05, 0.05), "start": (0.50, 0.30, CUBE_HALF), "side": "left"},
    "blue": {"color": (0.05, 0.15, 1.0), "start": (0.50, -0.30, CUBE_HALF), "side": "right"},
}

BINS = {
    "green": {"color": (0.05, 0.75, 0.15), "center": (0.72, 0.30)},
    "yellow": {"color": (0.95, 0.80, 0.05), "center": (0.72, -0.30)},
}

# A cube counts as "in" a bin when its centre is inside this radius of the bin centre and low.
BIN_TOLERANCE_XY = 0.09
BIN_MAX_Z = 0.06

TASKS = {
    "bimanual_bins": {
        "title": "Bimanual → two bins",
        "prompt": ("left arm puts the red cube into the green bin, "
                   "right arm puts the blue cube into the yellow bin"),
        "goals": {"red": "green", "blue": "yellow"},
        "needs_both_arms": True,
    },
    "both_to_green": {
        "title": "Both hands → one bin",
        "prompt": "both arms lift the red cube together and put it into the green bin",
        "goals": {"red": "green"},
        "needs_both_arms": True,
    },
    "handover": {
        "title": "Handover L → R",
        "prompt": ("the left arm picks up the red cube and hands it to the right arm, "
                   "which puts it into the yellow bin"),
        "goals": {"red": "yellow"},
        "needs_both_arms": True,
    },
}

DEFAULT_TASK = "bimanual_bins"


def get(task_id):
    return TASKS.get(task_id, TASKS[DEFAULT_TASK])
