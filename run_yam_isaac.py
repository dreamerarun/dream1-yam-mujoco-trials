"""Dream-1 on YAM arms in Isaac Sim.

Why YAM: dreamlabs/dream-1-beta is trained on YAM arms. Importing Dream's own YAM kinematics
(i2rt ships the same URDF with meshes) puts the observations and the action chunks in the frame the
model was trained in, so nothing has to be guessed about base offsets or link lengths.

  free checks:   ./run_isaac_clean.sh run_yam_isaac.py --dump-frames
  the demo:      ./run_isaac_clean.sh run_yam_isaac.py --max-calls-per-ep 60

Bridge (other terminal): python dream_bridge.py --arm yam --cap-usd 5 --max-calls 300
"""
import argparse
import base64
import io
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--episodes", type=int, default=1)
ap.add_argument("--max-calls-per-ep", type=int, default=60, help="1 call = 48 rows = 1.6 s ~ $0.0013")
ap.add_argument("--exec-rows", type=int, default=48)
ap.add_argument("--port", type=int, default=6006)
ap.add_argument("--render-scale", type=int, default=2)
ap.add_argument("--headless", action="store_true")
ap.add_argument("--dump-frames", action="store_true", help="save camera views + rig diagnostics, free")
ap.add_argument("--prompt", default="put the red cube in the green bin with the left arm, "
                                   "and the blue cube in the yellow bin with the right arm")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402,F401
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import PhysxSchema, UsdLux, UsdPhysics  # noqa: E402

PHYS_HZ = 60.0
ROW_HZ = 30.0
STEPS_PER_ROW = int(PHYS_HZ / ROW_HZ)
OBS_EVERY = 6                       # camera tick every 6 physics steps (10 Hz)
FRAME_W, FRAME_H = 320, 176
S = max(1, args.render_scale)
LOG_DIR = os.path.join(HERE, "logs")
DIAG_DIR = os.path.join(HERE, "diagnostics")
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(DIAG_DIR, exist_ok=True)
LOGF = open(os.path.join(LOG_DIR, f"yam_{time.strftime('%Y%m%d_%H%M%S')}.log"), "a")


def log(*a):
    msg = " ".join(str(x) for x in a)
    print(f"[yam {time.strftime('%H:%M:%S')}] {msg}", flush=True)
    LOGF.write(f"{time.strftime('%H:%M:%S')} {msg}\n")
    LOGF.flush()


# --------------------------------------------------------------------------- math
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


def quat_to_mat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def look_at(pos, target):
    pos = np.asarray(pos, float)
    f = np.asarray(target, float) - pos
    f /= np.linalg.norm(f)
    left = np.cross([0.0, 0.0, 1.0], f)
    n = np.linalg.norm(left)
    left = np.array([0.0, 1.0, 0.0]) if n < 1e-6 else left / n
    return pos, mat_to_quat_wxyz(np.column_stack([f, left, np.cross(f, left)]))


# --------------------------------------------------------------------------- rig
# Dream's YAM embodiment mounts the two arms at y = -0.305 (left) and +0.305 (right) off one base
# frame at the origin (`dream_ik.specs.EMBODIMENTS["yam"].obstacles`), so the scene uses exactly that.
MOUNTS = {"right": np.array([0.0, 0.305, 0.0]), "left": np.array([0.0, -0.305, 0.0])}
SIDES = ("right", "left")
ARM_JOINTS = 6
REST = np.array([0.0, 0.5, 0.5, -0.35, 0.0, 0.0])       # dream's yam rest_pose
MAX_SPEED = np.full(ARM_JOINTS, 2.5)
JAW_CLOSED, JAW_OPEN = 0.012, 0.055                       # finger gap, metres
TOOL_OFFSET = 0.1347                                      # dream's yam tcp offset along link_6 +z

URDF = os.path.expanduser("~/dream-env/lib/python3.12/site-packages/i2rt/robot_models/arm/yam/yam.urdf")

world = World(stage_units_in_meters=1.0, physics_dt=1 / PHYS_HZ, rendering_dt=1 / PHYS_HZ)
world.scene.add_default_ground_plane(z_position=-0.80)
FixedCuboid("/World/table", name="table", position=np.array([0.33, 0.0, -0.02]),
            scale=np.array([0.80, 1.10, 0.04]), color=np.array([0.62, 0.55, 0.45]))

# each arm works on its own side: left mount is at y=-0.305, right at y=+0.305
CUBES = {
    "red": {"start": np.array([0.34, -0.10, 0.026]), "color": np.array([1.0, 0.05, 0.05]), "side": "left"},
    "blue": {"start": np.array([0.34, 0.10, 0.026]), "color": np.array([0.1, 0.35, 1.0]), "side": "right"},
}
BINS = {
    "green": {"center": np.array([0.36, -0.30]), "color": np.array([0.0, 0.8, 0.15])},
    "yellow": {"center": np.array([0.36, 0.30]), "color": np.array([0.95, 0.80, 0.10])},
}
GOAL = {"red": "green", "blue": "yellow"}
BIN_TOL, BIN_MAX_Z = 0.075, 0.06

cube = {n: world.scene.add(DynamicCuboid(
    f"/World/cube_{n}", name=f"cube_{n}", position=c["start"],
    scale=np.array([0.05, 0.05, 0.05]), color=c["color"], mass=0.04)) for n, c in CUBES.items()}
for n, b in BINS.items():
    FixedCuboid(f"/World/bin_{n}", name=f"bin_{n}",
                position=np.array([b["center"][0], b["center"][1], 0.006]),
                scale=np.array([0.18, 0.18, 0.012]), color=b["color"])

UsdLux.DomeLight.Define(omni.usd.get_context().get_stage(), "/World/Dome").CreateIntensityAttr(650)


def import_yam(side, usd_out):
    """Import Dream's YAM URDF once per side and reference it into this stage."""
    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False
    cfg.convex_decomp = False
    cfg.import_inertia_tensor = True
    cfg.fix_base = True
    cfg.distance_scale = 1.0
    ok, root = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=URDF,
                                         import_config=cfg, dest_path=usd_out)
    if not ok:
        raise RuntimeError(f"URDF import failed for {side}")
    from pxr import Gf, UsdGeom
    path = f"/World/yam_{side}"
    prim = omni.usd.get_context().get_stage().DefinePrim(path, "Xform")
    # the mount transform is authored on our own prim first: the imported root carries its own
    # xformOpOrder, and stacking onto it after referencing throws
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in MOUNTS[side])))
    prim.GetReferences().AddReference(usd_out, root)
    return path


rig = {}
for _side in SIDES:
    _p = import_yam(_side, os.path.join(HERE, f"yam_{_side}.usd"))
    rig[_side] = _p

# position drives, so apply_action(joint_positions=...) actually holds the arm up
for _side in SIDES:
    stage = omni.usd.get_context().get_stage()
    for j in range(1, ARM_JOINTS + 1):
        jp = stage.GetPrimAtPath(f"{rig[_side]}/joints/joint{j}")
        if not jp:
            continue
        for axis in ("angular",):
            drv = UsdPhysics.DriveAPI.Apply(jp, axis)
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(900.0)
            drv.CreateDampingAttr(60.0)
            drv.CreateMaxForceAttr(60.0)

world.reset()

arms = {}
for _side in SIDES:
    art = world.scene.add(SingleArticulation(prim_path=rig[_side], name=f"yam_{_side}"))
    art.initialize()
    arms[_side] = art
    art.set_world_pose(position=MOUNTS[_side]) if hasattr(art, "set_world_pose") else None

DOF = {s: list(arms[s].dof_names) for s in SIDES}
log("dof:", DOF)


def q_of(side):
    return np.asarray(arms[side].get_joint_positions(), float)[:ARM_JOINTS]


def link6_pose(side):
    """World pose of link_6 (the flange the tcp offset is measured from)."""
    try:
        return arms[side].get_link_pose("link_6")
    except Exception:  # noqa: BLE001
        try:
            return arms[side].get_world_pose()
        except Exception:  # noqa: BLE001
            return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])


def tcp_pose(side):
    pos, quat = link6_pose(side)
    Rm = quat_to_mat(np.asarray(quat, float))
    return np.asarray(pos, float) + TOOL_OFFSET * Rm[:, 2], quat


# --------------------------------------------------------------------------- gripper
# Dream's yam URDF stops at link_6 (no finger joints; its embodiment carries the grip as one number),
# so the jaw is built here: two visual fingers on the tool point, plus a sim grasp that carries a cube
# once the fingers are shut on it. The grasp is a simulator affordance, the motion is the model's.
grip_frac = {s: 1.0 for s in SIDES}      # 1 open, 0 closed
held = {s: None for s in SIDES}
_finger_prim = {}


def _make_fingers():
    from pxr import Gf, UsdGeom
    stage = omni.usd.get_context().get_stage()
    for side in SIDES:
        for tag in ("a", "b"):
            p = stage.DefinePrim(f"/World/jaw_{side}_{tag}", "Cube")
            UsdGeom.Cube(p).GetSizeAttr().Set(1.0)
            xf = UsdGeom.Xformable(p)
            xf.AddTranslateOp()
            xf.AddOrientOp()
            xf.AddScaleOp().Set(Gf.Vec3f(0.018, 0.010, 0.050))   # finger slab
            _finger_prim[(side, tag)] = p
            try:
                UsdGeom.Gprim(p).CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.15, 0.17)])
            except Exception:  # noqa: BLE001
                pass


def _place_fingers():
    from pxr import Gf, UsdGeom
    for side in SIDES:
        tcp, quat = tcp_pose(side)
        Rm = quat_to_mat(np.asarray(quat, float))
        gap = JAW_CLOSED + (JAW_OPEN - JAW_CLOSED) * grip_frac[side]
        for tag, sgn in (("a", 1.0), ("b", -1.0)):
            p = _finger_prim[(side, tag)]
            pos = tcp + sgn * (gap * 0.5) * Rm[:, 0] - 0.02 * Rm[:, 2]
            xf = UsdGeom.Xformable(p)
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(*[float(v) for v in pos]))
                elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    op.Set(Gf.Quatf(float(np.asarray(quat)[0]), *[float(v) for v in np.asarray(quat)[1:]]))


def _carry_cubes():
    """Shut fingers on a nearby cube -> carry it with the tool; open -> let go."""
    for side in SIDES:
        name = next((n for n, c in CUBES.items() if c["side"] == side), None)
        if name is None:
            continue
        tcp, quat = tcp_pose(side)
        if grip_frac[side] < 0.32 and held[side] is None:
            if np.linalg.norm(cube_xy(name) - tcp) < 0.045:
                held[side] = name
                log(f"grasp: {side} took {name}")
        elif grip_frac[side] > 0.60 and held[side] is not None:
            log(f"release: {side} let go of {held[side]}")
            held[side] = None
        if held[side] is not None:
            Rm = quat_to_mat(np.asarray(quat, float))
            cube[held[side]].set_world_pose(
                np.asarray(tcp, float) - 0.02 * Rm[:, 2], np.asarray(quat, float))


# --------------------------------------------------------------------------- cameras
res = (FRAME_W * S, FRAME_H * S)
cams = {
    "overhead": Camera("/World/cam_overhead", resolution=res),
    "wrist_l": Camera("/World/cam_wrist_l", resolution=res),
    "wrist_r": Camera("/World/cam_wrist_r", resolution=res),
}
for c in cams.values():
    c.initialize()
    c.set_focal_length(1.35)
    c.set_clipping_range(0.02, 30.0)
cams["overhead"].set_world_pose(*look_at([0.05, 0.0, 1.05], [0.38, 0.0, 0.0]), camera_axes="world")

WRIST_SIDE = {"wrist_l": "left", "wrist_r": "right"}
WRIST_BACK, WRIST_UP, WRIST_AHEAD = 0.09, 0.055, 0.16


def update_wrist_cams():
    for k, side in WRIST_SIDE.items():
        tcp, quat = tcp_pose(side)
        Rm = quat_to_mat(np.asarray(quat, float))
        approach = Rm[:, 2]
        up = Rm[:, 1]
        pos = tcp - WRIST_BACK * approach + WRIST_UP * up
        look = tcp + WRIST_AHEAD * approach - 0.04 * up
        cams[k].set_world_pose(*look_at(pos, look), camera_axes="world")


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


SIM_T = 0.0
TICK = 0


def world_step_render():
    global SIM_T, TICK
    update_wrist_cams()
    _carry_cubes()
    world.step(render=True)
    _place_fingers()
    SIM_T += 1.0 / PHYS_HZ
    TICK += 1
    if conn is not None and TICK % OBS_EVERY == 0:
        send_obs()


def phys_step(n=1):
    for _ in range(n):
        world.step(render=False)


# --------------------------------------------------------------------------- task checks
def cube_xy(name):
    return np.asarray(cube[name].get_world_pose()[0], float)


def bin_dist(name):
    p = cube_xy(name)
    c = BINS[GOAL[name]]["center"]
    d = float(np.hypot(p[0] - c[0], p[1] - c[1]))
    return d, bool(d < BIN_TOL and p[2] < BIN_MAX_Z)


def success():
    return all(bin_dist(n)[1] for n in CUBES)


# --------------------------------------------------------------------------- bridge
conn = None


def call(msg):
    conn.send(msg)
    r = conn.recv()
    if "error" in r:
        log("bridge error:", r["error"])
        sys.stdout.flush()
        os._exit(4)
    return r


def message_state():
    st = {}
    for side in SIDES:
        st[f"state.{side}_arm"] = q_of(side).tolist()
        st[f"state.{side}_grip"] = [float(grip_frac[side])]
    return st


def send_obs():
    frames = {k: jpeg_b64(rgb(c)) for k, c in cams.items()}
    call({"cmd": "obs", "frames": frames, "state": message_state(),
          "prompt": args.prompt, "t": SIM_T})


# --------------------------------------------------------------------------- control
def apply_joint_row(side, row):
    """row = [6 joints, grip]; clamp to limits and to the arm's speed."""
    row = np.asarray(row, float).ravel()
    if row.size < ARM_JOINTS + 1 or not np.all(np.isfinite(row)):
        return
    cur = q_of(side)
    want = np.clip(row[:ARM_JOINTS], -2.9, 3.1)
    limit = MAX_SPEED / PHYS_HZ
    want = cur + np.clip(want - cur, -limit, limit)
    arms[side].apply_action(ArticulationAction(joint_positions=want))
    global grip_frac
    grip_frac[side] = float(np.clip(row[ARM_JOINTS], 0.0, 1.0))


def goto_rest(steps=400, tol=0.06):
    """Free settle to Dream's yam rest pose (no inference)."""
    for _ in range(steps):
        for s in SIDES:
            cur = q_of(s)
            arms[s].apply_action(ArticulationAction(
                joint_positions=cur + np.clip(REST - cur, -MAX_SPEED / PHYS_HZ, MAX_SPEED / PHYS_HZ)))
        phys_step(1)
        if all(np.max(np.abs(q_of(s) - REST)) < tol for s in SIDES):
            break
    log("rest pose reached:", {s: np.round(q_of(s), 2).tolist() for s in SIDES})


def reset_episode():
    world.reset()
    for s in SIDES:
        arms[s].set_joint_positions(REST)
        held[s] = None
        grip_frac[s] = 1.0
    for n, c in CUBES.items():
        cube[n].set_world_pose(c["start"], np.array([1.0, 0.0, 0.0, 0.0]))
    for _ in range(30):
        world_step_render()
    goto_rest()
    for _ in range(20):
        world_step_render()


# --------------------------------------------------------------------------- modes
_make_fingers()

if args.dump_frames:
    reset_episode()
    for _ in range(10):
        world_step_render()
    lines = []
    for k, c in cams.items():
        Image.fromarray(rgb(c)).save(os.path.join(DIAG_DIR, f"yam_{k}.png"))
        lines.append(f"{k}: pos {np.round(np.asarray(c.get_world_pose()[0]), 3)}")
    for s in SIDES:
        tcp, _ = tcp_pose(s)
        lines.append(f"{s}: q {np.round(q_of(s), 3).tolist()} tcp {np.round(tcp, 3).tolist()}")
    lines.append("cubes " + str([(n, np.round(cube_xy(n), 3).tolist()) for n in CUBES]))
    log("dump:\n" + "\n".join(lines))
    app.close()
    sys.exit(0)

from multiprocessing.connection import Client  # noqa: E402

try:
    conn = Client(("127.0.0.1", args.port), authkey=b"dream-local-bridge")
except OSError as e:
    log(f"cannot reach the bridge on 127.0.0.1:{args.port}: {e}")
    log("start it first: python dream_bridge.py --arm yam --cap-usd 5 --max-calls 300")
    app.close()
    os._exit(2)
log("bridge connected")

wins = 0
for ep in range(args.episodes):
    reset_episode()
    call({"cmd": "reset"})
    send_obs()
    with open(os.path.join(DIAG_DIR, f"yam_start_ep{ep}.txt"), "w") as f:
        f.write(str(message_state()))
    ok, calls = False, 0
    for ci in range(args.max_calls_per_ep):
        r = call({"cmd": "act", "note": f"ep{ep} call{ci}"})
        if "error" in r:
            log("bridge error:", r["error"])
            break
        R = np.asarray(r.get("q_r_chunk", []), float)
        L = np.asarray(r.get("q_l_chunk", []), float)
        if R.size == 0 or L.size == 0:
            log("empty chunk, stopping:", r.get("budget", ""))
            break
        calls += 1
        n = min(args.exec_rows, len(R), len(L))
        for i in range(n):
            apply_joint_row("right", R[i])
            apply_joint_row("left", L[i])
            for _ in range(STEPS_PER_ROW):
                world_step_render()
        send_obs()
        d = {name: bin_dist(name) for name in CUBES}
        tcp = {s: np.round(tcp_pose(s)[0], 2).tolist() for s in SIDES}
        log(f"ep{ep} call{ci} rows={n} | "
            + " ".join(f"{name}->{GOAL[name]} {d[name][0]:.3f}{'*' if d[name][1] else ''}" for name in CUBES)
            + f" | tcp L{tcp['left']} R{tcp['right']} | grip L{grip_frac['left']:.2f} R{grip_frac['right']:.2f}"
            + f" | {r.get('budget', '')}")
        if success():
            ok = True
            break
    for k, c in cams.items():
        Image.fromarray(rgb(c)).save(os.path.join(DIAG_DIR, f"yam_final_{k}_ep{ep}.png"))
    log(f"EPISODE {ep} {'SUCCESS' if ok else 'not solved'} | "
        + " ".join(f"{n} d={bin_dist(n)[0]:.3f}" for n in CUBES)
        + f" | calls {calls}")
    wins += ok

log(f"done: {wins}/{args.episodes} episodes met the goal")
try:
    conn.send({"cmd": "quit"})
except Exception:  # noqa: BLE001
    pass
app.close()
sys.exit(0 if wins == args.episodes else 1)
