"""The Dream-side configuration of this body (dual Franka) and the policy() dict built from it.

Why this exists: dreamlabs/dream-1-beta is trained on YAM arms and answers end-effector poses. To run it on another
arm the client needs `ik` (forward kinematics out, the chunk solve back) *and* the body's embodiment config, because
that is where the client reads which camera roles the body sends in the model's slots. Without it policy() refuses:

    DreamConfigError: config key 'ik' with a model trained on YAM arms needs the body's embodiment config ...

The config is the documented shape (shared.datasets.embodiments): named parts of the state/action vector with their
[start, end) spans, and the camera roles. Our arms have 7 joints and one gripper each, right side first as the real
datasets are laid out. The widths of `state.<side>_arm` are overridden by the client to the YAM's 6 joint slots
(client_ik.ClientIK.sent_checks), so the spans here only have to be self-consistent.
"""
import os

ARM_JOINTS = 7
SIDES = ("right", "left")

EMBODIMENT = {
    "state": {
        "right_arm": [0, ARM_JOINTS],
        "right_gripper": [ARM_JOINTS, ARM_JOINTS + 1],
        "left_arm": [ARM_JOINTS + 1, 2 * ARM_JOINTS + 1],
        "left_gripper": [2 * ARM_JOINTS + 1, 2 * ARM_JOINTS + 2],
    },
    "action": {
        "right_arm": [0, ARM_JOINTS],
        "right_gripper": [ARM_JOINTS, ARM_JOINTS + 1],
        "left_arm": [ARM_JOINTS + 1, 2 * ARM_JOINTS + 1],
        "left_gripper": [2 * ARM_JOINTS + 1, 2 * ARM_JOINTS + 2],
    },
    "video": {
        "overhead": "observation.images.overhead",
        "wrist_l": "observation.images.wrist_l",
        "wrist_r": "observation.images.wrist_r",
    },
    "head_camera": "overhead",
}

DEFAULT_ARM_FILE = os.path.expanduser("~/.dreamlabs/arms/lula-franka-gen.json")
DEFAULT_CHECKPOINT = "dreamlabs/dream-1-beta"


def embodiment_for(joints):
    """The same body layout with `joints` per arm (6 for Dream's own YAM, 7 for a Franka)."""
    n = int(joints)
    return {
        "state": {
            "right_arm": [0, n],
            "right_gripper": [n, n + 1],
            "left_arm": [n + 1, 2 * n + 1],
            "left_gripper": [2 * n + 1, 2 * n + 2],
        },
        "action": {
            "right_arm": [0, n],
            "right_gripper": [n, n + 1],
            "left_arm": [n + 1, 2 * n + 1],
            "left_gripper": [2 * n + 1, 2 * n + 2],
        },
        "video": EMBODIMENT["video"],
        "head_camera": "overhead",
    }


def policy_config(checkpoint=DEFAULT_CHECKPOINT, arm_file=DEFAULT_ARM_FILE, action_hz=30.0,
                  timeout_ms=60000, arm=None, joints=None):
    """The dict DreamClient.policy() takes.

    `arm` names a shipped Dream arm ("yam"): then the client uses its own kinematics for that arm,
    so the sim must build the same arm. Otherwise `arm_file` is our calibrated arm JSON.
    """
    arm_spec = arm if arm else os.path.expanduser(arm_file)
    n = joints if joints else (6 if arm == "yam" else 7)
    return {
        "checkpoint": checkpoint,
        "embodiment": EMBODIMENT if n == ARM_JOINTS else embodiment_for(n),
        "ik": {"arm": arm_spec, "head": "eef"},
        "action_hz": action_hz,
        "execution_horizon": 30,
        "timeout_ms": timeout_ms,
        "_record": False,      # simulated episodes are not the account's data
    }
