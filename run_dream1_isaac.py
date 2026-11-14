"""Isaac Sim side: dual-Franka scene + 3 cameras, driven by Dream-1 through dream_bridge.py.

Needs NO Dream packages (they live in ~/dream-env, behind the bridge).
Run:  ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh run_dream1_isaac.py --help
"""
import argparse, base64, io, os, sys, time
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--task", default="Pick up the red cube and put it in the green container")
p.add_argument("--episodes", type=int, default=3)
p.add_argument("--max-calls-per-ep", type=int, default=20, help="1 call = 48 rows = 1.6 s of robot time ~ $0.0013")
p.add_argument("--exec-rows", type=int, default=48, help="rows executed per call (48 = cheapest; fewer = more reactive but pricier)")
p.add_argument("--port", type=int, default=6006)
p.add_argument("--render-scale", type=int, default=2, help="render cameras at N x 320x176 then downscale (avoids DLSS artefacts)")
p.add_argument("--headless", action="store_true")
p.add_argument("--dump-frames", action="store_true", help="save camera views + diag.txt and exit (free, no bridge)")
args = p.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})

import omni.usd
from pxr import UsdLux
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from isaacsim.robot.manipulators.examples.franka import Franka

PHYS_HZ = 60.0
ROW_HZ = 30.0                                   # Dream-1: 48 rows = 1.6 s
STEPS_PER_ROW = int(PHYS_HZ / ROW_HZ)
OBS_EVERY = 6                                   # camera tick every 6 physics steps (10 Hz sim time)
FRAME_W, FRAME_H = 320, 176                     # size Dream's own sim example uses
S = args.render_scale
GRIP_OPEN_IS_HIGH = True                        # ASSUMPTION: Dream gripper 1.0 = open
FINGER_MAX = 0.04
READY = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])   # Franka ready pose
BASES = {"left": np.array([0.0, 0.45, 0.0]), "right": np.array([0.0, -0.45, 0.0])}  # both face +x; left = +y


def mat_to_quat_wxyz(M):
    t = np.trace(M)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2; w = 0.25 * s
        x = (M[2, 1] - M[1, 2]) / s; y = (M[0, 2] - M[2, 0]) / s; z = (M[1, 0] - M[0, 1]) / s
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        s = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2
        w = (M[2, 1] - M[1, 2]) / s; x = 0.25 * s; y = (M[0, 1] + M[1, 0]) / s; z = (M[0, 2] + M[2, 0]) / s
    elif M[1, 1] > M[2, 2]:
        s = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2
        w = (M[0, 2] - M[2, 0]) / s; x = (M[0, 1] + M[1, 0]) / s; y = 0.25 * s; z = (M[1, 2] + M[2, 1]) / s
    else:
        s = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2
        w = (M[1, 0] - M[0, 1]) / s; x = (M[0, 2] + M[2, 0]) / s; y = (M[1, 2] + M[2, 1]) / s; z = 0.25 * s
    q = np.array([w, x, y, z]); return q / np.linalg.norm(q)


def look_at(pos, target):
    """pose for camera_axes='world' (x forward, y left, z up): camera at pos looking at target."""
    pos = np.asarray(pos, float); f = np.asarray(target, float) - pos; f /= np.linalg.norm(f)
    left = np.cross([0.0, 0.0, 1.0], f); left /= np.linalg.norm(left)
    up = np.cross(f, left)
    return pos, mat_to_quat_wxyz(np.column_stack([f, left, up]))


# ---- scene -------------------------------------------------------------------------------
world = World(stage_units_in_meters=1.0, physics_dt=1 / PHYS_HZ, rendering_dt=1 / PHYS_HZ)
world.scene.add_default_ground_plane(z_position=-0.8)
FixedCuboid("/World/table", name="table", position=np.array([0.5, 0.0, -0.02]),
            scale=np.array([0.9, 1.5, 0.04]), color=np.array([0.55, 0.45, 0.35]))
# two cubes, one per arm: left arm takes red -> green, right arm takes blue -> yellow
CUBES = {
    "red": {"start": np.array([0.45, 0.25, 0.0275]), "color": np.array([1.0, 0.0, 0.0]), "side": "left"},
    "blue": {"start": np.array([0.45, -0.25, 0.0275]), "color": np.array([0.1, 0.3, 1.0]), "side": "right"},
}
BINS = {
    "green": {"center": np.array([0.72, 0.25]), "color": np.array([0.0, 0.8, 0.1])},
    "yellow": {"center": np.array([0.72, -0.25]), "color": np.array([0.95, 0.8, 0.1])},
}
GOAL = {"red": "green", "blue": "yellow"}
BIN_HALF, BIN_TOL, BIN_MAX_Z = 0.10, 0.09, 0.06
cube = {}
for _n, _c in CUBES.items():
    cube[_n] = world.scene.add(DynamicCuboid(
        f"/World/cube_{_n}", name=f"cube_{_n}", position=_c["start"],
        scale=np.array([0.05, 0.05, 0.05]), color=_c["color"], mass=0.05))
for _n, _b in BINS.items():
    FixedCuboid(f"/World/bin_{_n}", name=f"bin_{_n}",
                position=np.array([_b["center"][0], _b["center"][1], 0.005]),
                scale=np.array([2 * BIN_HALF, 2 * BIN_HALF, 0.01]), color=_b["color"])
UsdLux.DomeLight.Define(omni.usd.get_context().get_stage(), "/World/Dome").CreateIntensityAttr(600)

robots = {s: world.scene.add(Franka(prim_path=f"/World/Franka_{s}", name=f"franka_{s}", position=b)) for s, b in BASES.items()}
res = (FRAME_W * S, FRAME_H * S)
cams = {"overhead": Camera("/World/head_cam", resolution=res),
        "wrist_l": Camera("/World/wrist_cam_l", resolution=res),
        "wrist_r": Camera("/World/wrist_cam_r", resolution=res)}
dbg = Camera("/World/debug_cam", resolution=(640, 360)) if args.dump_frames else None
world.reset()
cams["overhead"].initialize()
cams["overhead"].set_world_pose(*look_at([0.10, 0.0, 1.30], [0.55, 0.0, 0.0]), camera_axes="world")
for k in ("wrist_l", "wrist_r"):
    cams[k].initialize()
for c in cams.values():
    c.set_focal_length(1.4)
    c.set_clipping_range(0.02, 20.0)      # near-clip 2 cm: default hides everything closer than ~1 m
if dbg is not None:
    dbg.initialize(); dbg.set_focal_length(1.4); dbg.set_clipping_range(0.02, 50.0)
    dbg.set_world_pose(*look_at([1.7, -1.5, 1.3], [0.3, 0.0, 0.2]), camera_axes="world")

def quat_to_mat(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


WRIST_SIDE = {"wrist_l": "left", "wrist_r": "right"}
# a wrist camera has to show the fingers AND what is in front of them, or the model is blind
WRIST_BACK = 0.07        # behind the tool point, so the hand body is not straddling the lens
WRIST_UP = 0.045          # lifted above the hand, looking forward and slightly down
WRIST_AHEAD = 0.10       # aim a little past the fingertips, at the workspace


def update_wrist_cams():
    for k, side in WRIST_SIDE.items():
        tcp, q = robots[side].end_effector.get_world_pose()
        Rm = quat_to_mat(np.asarray(q, float))
        approach, xax = Rm[:, 2], Rm[:, 0]
        left = np.cross(xax, approach)
        n = np.linalg.norm(left)
        left = np.array([0.0, 1.0, 0.0]) if n < 1e-6 else left / n
        up = np.cross(approach, left)
        pos = np.asarray(tcp, float) - WRIST_BACK * approach + WRIST_UP * up
        aim = np.asarray(tcp, float) + WRIST_AHEAD * approach - 0.03 * up
        cams[k].set_world_pose(*look_at(pos, aim), camera_axes="world")


def world_step_render():
    update_wrist_cams()
    world.step(render=True)


FINGER_IDX = {s: [r.get_dof_index("panda_finger_joint1"), r.get_dof_index("panda_finger_joint2")] for s, r in robots.items()}


def rgb(cam, final=True):
    f = cam.get_rgba()
    if f is None or f.size == 0:
        return np.zeros((FRAME_H, FRAME_W, 3), np.uint8)
    f = f[:, :, :3]
    f = (np.clip(f, 0, 1) * 255).astype(np.uint8) if f.dtype != np.uint8 else np.ascontiguousarray(f)
    if final and S > 1:
        f = np.asarray(Image.fromarray(f).resize((FRAME_W, FRAME_H), Image.LANCZOS))
    return f


def jpeg_b64(arr):
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def arm_state(side):
    q = np.asarray(robots[side].get_joint_positions())
    open_frac = float(np.clip(np.mean(q[FINGER_IDX[side]]) / FINGER_MAX, 0, 1))
    return q[:7], (open_frac if GRIP_OPEN_IS_HIGH else 1.0 - open_frac)


def message_state():
    st = {}
    for side in BASES:
        arm, grip = arm_state(side)
        st[f"state.{side}_arm"] = arm.tolist()
        # Dream-1 model expects 'state.right_grip' (1=open), not 'state.right_gripper'
        st[f"state.{side}_grip"] = [grip]   # same value, different key name
    return st


PANDALIM = np.array([[-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698],
                     [-2.8973, 2.8973], [-0.0175, 3.7525], [-2.8973, 2.8973]])
try:  # prefer the limits the loaded articulation actually reports
    LIMS = {s: np.where(np.isfinite(np.stack([np.asarray(r.dof_properties["lower"])[:7],
                                             np.asarray(r.dof_properties["upper"])[:7]], axis=1)),
                        np.stack([np.asarray(r.dof_properties["lower"])[:7],
                                  np.asarray(r.dof_properties["upper"])[:7]], axis=1), PANDALIM)
            for s, r in robots.items()}
except Exception:  # noqa: BLE001
    LIMS = {s: PANDALIM for s in BASES}
MAX_QDOT = 2.4                       # rad/s, under the Panda's 2.62 limit


def command_row(side, row):
    """Apply one row [7 joints, grip 0..1] with joint-limit and speed clamping, NaN rejected."""
    row = np.asarray(row, float).ravel()
    if row.size < 8 or not np.all(np.isfinite(row)):
        return
    q = np.asarray(robots[side].get_joint_positions(), float)
    want = np.clip(row[:7], LIMS[side][:, 0], LIMS[side][:, 1])
    limit = MAX_QDOT / PHYS_HZ
    want = q[:7] + np.clip(want - q[:7], -limit, limit)
    g = float(np.clip(row[7], 0, 1))
    open_frac = g if GRIP_OPEN_IS_HIGH else 1.0 - g
    w = open_frac * FINGER_MAX
    robots[side].apply_action(ArticulationAction(joint_positions=np.concatenate([want, [w, w]])))


def set_ready(side):
    robots[side].set_joint_positions(np.concatenate([READY, [FINGER_MAX, FINGER_MAX]]))


DOWN_Q = np.array([0.0, 0.0, 1.0, 0.0])          # panda_hand pointing straight down (w,x,y,z)


def phys_step(n=1):
    """Physics only (no render, no camera update): fast settle used by the free prepose."""
    for _ in range(n):
        world.step(render=False)


def prepose(hover=0.20, steps=4000, tol=0.05):
    """Put both hands above their own cube before inference, so the first frames show the task.

    Free: RMPFlow, no Dream calls.
    """
    try:
        from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import (
            RMPFlowController,
        )
        mpc = {s: RMPFlowController(name=f"rmp_{s}", robot_articulation=robots[s],
                                    physics_dt=1 / PHYS_HZ) for s in BASES}
    except Exception as e:  # noqa: BLE001
        print("[isaac] RMPFlow unavailable, skipping prepose:", repr(e), flush=True)
        return
    active = {c["side"]: (np.array([c["start"][0], c["start"][1], c["start"][2] + hover]), DOWN_Q)
              for c in CUBES.values()}
    reached = {s: False for s in active}
    for _ in range(steps):
        for s, (pos, quat) in list(active.items()):
            act = mpc[s].forward(target_end_effector_position=np.asarray(pos, float),
                                 target_end_effector_orientation=np.asarray(quat, float))
            jp = getattr(act, "joint_positions", None) if act is not None else None
            if jp is not None and np.all(np.isfinite(np.asarray(jp, float))):
                cur = np.asarray(robots[s].get_joint_positions(), float)
                want = np.clip(np.asarray(jp, float)[:7], LIMS[s][:, 0], LIMS[s][:, 1])
                limit = 2.0 / PHYS_HZ
                newj = cur[:7] + np.clip(want - cur[:7], -limit, limit)
                robots[s].apply_action(ArticulationAction(
                    joint_positions=np.concatenate([newj, [FINGER_MAX, FINGER_MAX]])))
        phys_step(1)
        for s in list(active):
            tcp, _ = robots[s].end_effector.get_world_pose()
            if np.linalg.norm(np.asarray(tcp, float) - active[s][0]) < tol:
                reached[s] = True
                del active[s]
        if not active:
            break
    print("[isaac] prepose reached:", reached, "| tcp",
          {s: np.round(np.asarray(robots[s].end_effector.get_world_pose()[0], float), 3).tolist()
           for s in BASES}, flush=True)


def cube_xy(name):
    return np.asarray(cube[name].get_world_pose()[0], float)


def bin_dist(name):
    """xy distance from a cube to its goal bin, and whether it counts as placed."""
    p = cube_xy(name)
    c = BINS[GOAL[name]]["center"]
    d = float(np.hypot(p[0] - c[0], p[1] - c[1]))
    return d, bool(d < BIN_TOL and p[2] < BIN_MAX_Z)


def success():
    return all(bin_dist(n)[1] for n in CUBES)


# ---- bridge ------------------------------------------------------------------------------
conn = None


def call(msg):
    conn.send(msg)
    r = conn.recv()
    if "error" in r:
        # os._exit: Isaac's atexit handlers segfault when an exception unwinds, and this is fatal anyway
        print("[isaac] bridge error:", r["error"], flush=True)
        sys.stdout.flush()
        os._exit(4)
    return r


sim_t, tick = 0.0, 0


def send_obs():
    frames = {k: jpeg_b64(rgb(c)) for k, c in cams.items()}
    call({"cmd": "obs", "frames": frames, "state": message_state(), "prompt": args.task, "t": sim_t})


def step(n=1):
    """advance physics; stream camera ticks to the bridge at 10 Hz sim time."""
    global sim_t, tick
    for _ in range(n):
        world_step_render()
        sim_t += 1 / PHYS_HZ
        tick += 1
        if conn is not None and tick % OBS_EVERY == 0:
            send_obs()


def reset_episode(rng):
    global sim_t, tick
    world.reset()
    for s in BASES:
        set_ready(s)
    for _n, _c in CUBES.items():
        cube[_n].set_world_pose(_c["start"] + np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), 0.0]),
                                np.array([1.0, 0.0, 0.0, 0.0]))
    for _ in range(40):
        world_step_render()
    sim_t, tick = 0.0, 0
    if conn is not None:
        call({"cmd": "reset"})


rng = np.random.default_rng(0)

if args.dump_frames:
    reset_episode(rng)
    for _ in range(20):
        world_step_render()
    lines = []
    for k, c in cams.items():
        Image.fromarray(rgb(c)).save(f"frame_{k}.png")
        pos, quat = c.get_world_pose()
        lines.append(f"{k}: cam world pos {np.round(pos, 3)} quat(wxyz) {np.round(quat, 3)} clip {np.round(np.asarray(c.get_clipping_range()), 3)} focal {np.round(c.get_focal_length(), 3)}")
    Image.fromarray(rgb(dbg, final=False)).save("frame_debug.png")
    for sd in BASES:
        hp, hq = robots[sd].end_effector.get_world_pose()
        lines.append(f"{sd}: hand world pos {np.round(hp, 3)} quat(wxyz) {np.round(hq, 3)}")
        lines.append(f"{sd}: joints {np.round(np.asarray(robots[sd].get_joint_positions()), 2)}")
    lines.append(f"cubes {[(n, np.round(cube_xy(n), 3).tolist()) for n in CUBES]}")
    open("diag.txt", "w").write("\n".join(lines) + "\n")
    print("\n".join(lines), flush=True)
    print("saved frame_overhead/wrist_l/wrist_r/debug .png and diag.txt", flush=True)
    sys.stdout.flush(); app.close(); sys.exit(0)

from multiprocessing.connection import Client
try:
    conn = Client(("127.0.0.1", args.port), authkey=b"dream-local-bridge")
except OSError as e:
    print(f"[isaac] cannot reach the bridge on 127.0.0.1:{args.port}: {e}\n"
          "        start it first:  python dream_bridge.py --arm-file ~/.dreamlabs/arms/lula-franka-gen.json --cap-usd 5",
          flush=True)
    app.close(); os._exit(2)          # os._exit: Isaac's atexit handlers segfault on an uncaught error

wins = 0
for ep in range(args.episodes):
    reset_episode(rng)
    step(60)                                   # 1 s settle while the window starts filling
    prepose()                                  # free RMPFlow: both hands above their own cube
    step(30)                    # note: sim_t keeps running on purpose, the client rejects timestamps that go back
    for _k, _c in cams.items():
        Image.fromarray(rgb(_c)).save(f"diagnostics/model_sees_{_k}_ep{ep}.png")
    ok, last_cmd = False, {}
    for ci in range(args.max_calls_per_ep):
        send_obs()
        r = call({"cmd": "act", "commanded": last_cmd, "note": f"ep{ep} call{ci}"})
        R, L = np.asarray(r["q_r_chunk"], float), np.asarray(r["q_l_chunk"], float)
        for i in range(min(args.exec_rows, len(R), len(L))):
            command_row("right", R[i]); command_row("left", L[i])
            # get_action(commanded=...) takes the arm joints, not the gripper column
            last_cmd = {"q_r_chunk": R[i][:7].tolist(), "q_l_chunk": L[i][:7].tolist()}
            step(STEPS_PER_ROW)
            if success():
                ok = True; break
        d = {n: bin_dist(n) for n in CUBES}
        tcp = {s: np.round(np.asarray(robots[s].end_effector.get_world_pose()[0], float), 2).tolist()
               for s in BASES}
        grip = {s: round(float(np.mean(np.asarray(robots[s].get_joint_positions(), float)[FINGER_IDX[s]]) / FINGER_MAX), 2)
                for s in BASES}
        gnorm = float(np.mean(np.abs(R[-1][:7] - np.asarray(robots['right'].get_joint_positions(), float)[:7]))) \
            if len(R) else float('nan')
        print(f"[ep {ep} call {ci}] "
              + " ".join(f"{n}->{GOAL[n]} {d[n][0]:.3f}{'*' if d[n][1] else ''}" for n in CUBES)
              + f" | tcp L{tcp['left']} R{tcp['right']} | grip L{grip['left']} R{grip['right']}"
              + f" | resid {gnorm:.3f} | {r['budget']}", flush=True)
        if ok:
            break
    wins += ok
    for _k, _c in cams.items():
        Image.fromarray(rgb(_c)).save(f"diagnostics/final_{_k}_ep{ep}.png")
    print(f"[ep {ep}] {'SUCCESS' if ok else 'not solved'} "
          + " ".join(f"{n} d={bin_dist(n)[0]:.3f}" for n in CUBES), flush=True)

print(f"done: {wins}/{args.episodes} successes", flush=True)
try:
    conn.send({"cmd": "quit"})
except Exception:  # noqa: BLE001
    pass
app.close()
