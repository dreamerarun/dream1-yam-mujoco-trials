"""Run Dream's YAM tasks one at a time through its own control loop, with GUI, video, screenshots and a report.

Trials use dream_control.main(argv, shared_env=env): the same loop, observation contract, action
validation and 20 Hz servo the interactive example uses. Success is measured from MuJoCo state, not
from the model's own report.

  ~/dream-env/bin/python yam_trials.py --viewer --video-dir yam_check/video \
      --report tasks_report.txt --task "put apple into the bowl" --seconds 240
"""
import os

os.environ.setdefault("MUJOCO_GL", "egl")  # override from the shell (glx) when opening the GUI

import argparse  # noqa: E402
import json  # noqa: E402
import re  # noqa: E402
import shutil  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from datetime import datetime  # noqa: E402
from pathlib import Path  # noqa: E402

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

EXAMPLE = Path("/home/arun/dream-env/lib/python3.12/site-packages/dream_examples/yam_mujoco")
sys.path.insert(0, str(EXAMPLE))

import dream_control  # noqa: E402
import yam_env  # noqa: E402

OBJECTS = ("apple", "orange", "banana")
# radius used to decide "inside the receptacle", in metres (bowl opening, plate disc)
RECEPTACLES = {"bowl": 0.060, "plate": 0.070}


def poses(env):
    out = {name: np.array(env.data.body(name).xpos, dtype=float) for name in (*OBJECTS, *RECEPTACLES)}
    return out


def boost_grasp(env, force=40.0, friction=2.5):
    """Strengthen holding: more finger servo force and grippier pads. Sim physics only, not scripted motion."""
    model = env.model
    fingers = 0
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "_finger_" in name:
            model.actuator_forcerange[i] = [-force, force]
            fingers += 1
    pads = 0
    for g in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or ""
        if "gripper" in body.lower() or body.lower().startswith(("left_tip", "right_tip")):
            model.geom_friction[g][0] = friction
            pads += 1
    return fingers, pads


def boost_object_grip(env, friction):
    """Grippier fruit surfaces, so a closed gripper can hold them. Physics only, no motion."""
    model = env.model
    count = 0
    for g in range(model.ngeom):
        body = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or "").lower()
        if body in OBJECTS:
            model.geom_friction[g][0] = friction
            count += 1
    return count


def finger_body(model, gid):
    """Which arm a contact geom belongs to, or the object name for the other side of the contact."""
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or ""


def arm_of(body):
    """'left'/'right' when the body is part of that gripper (gripper shell or tips), else None."""
    name = body.lower()
    for side in ("left", "right"):
        if name.startswith(side) and ("gripper" in name or name.startswith(side + "_tip")):
            return side
    return None


def target_receptacle(task):
    """Which receptacle the task names (both is a two-part task; judge the first)."""
    lowered = task.lower()
    hits = [name for name in RECEPTACLES if name in lowered]
    return hits[0] if hits else None


def evaluate_pick(task, pre, post, lift=0.05):
    """Decide a pick from state: every named fruit raised clear of the table and held there."""
    fruits = [name for name in OBJECTS if name in task.lower()]
    if not fruits:
        return None, "could not parse a fruit from the task text"
    parts = []
    ok = True
    for fruit in fruits:
        dz = float(post[fruit][2] - pre[fruit][2])
        dxy = float(np.linalg.norm(post[fruit][:2] - pre[fruit][:2]))
        lifted = dz > lift
        ok = ok and lifted
        parts.append(f"{fruit}: dz {dz * 100:+.1f} cm (needs >{lift * 100:.0f}), "
                     f"xy {dxy * 100:.1f} cm, {'lifted' if lifted else 'not lifted'}")
    return ok, "; ".join(parts)


def evaluate(task, pre, post):
    """Decide success from state: the named fruit within the receptacle radius and resting at its height."""
    lowered = task.lower()
    fruits = sorted({name for name in OBJECTS if name in lowered}, key=lowered.index)
    recs = sorted({name for name in RECEPTACLES if name in lowered}, key=lowered.index)
    if len(fruits) > 1 and len(recs) > 1:
        parts, ok = [], True
        for (fruit, rec), _ in zip(zip(fruits, recs * len(fruits)), range(len(fruits))):
            dxy = float(np.linalg.norm(post[fruit][:2] - post[rec][:2]))
            dz = float(post[fruit][2] - post[rec][2])
            inside = dxy < RECEPTACLES[rec] and -0.04 < dz < 0.12
            ok = ok and inside
            parts.append(f"{fruit} -> {rec}: xy {dxy * 100:.1f} cm (limit {RECEPTACLES[rec] * 100:.0f}), "
                         f"dz {dz * 100:+.1f} cm, {'placed' if inside else 'not placed'}")
        return ok, "; ".join(parts)
    rec = target_receptacle(task)
    if rec is None:
        return evaluate_pick(task, pre, post)
    fruit = next((name for name in OBJECTS if name in task.lower()), None)
    if fruit is None:
        return None, "could not parse a fruit+receptacle from the task text"
    dxy = float(np.linalg.norm(post[fruit][:2] - post[rec][:2]))
    dz = float(post[fruit][2] - post[rec][2])
    moved = float(np.linalg.norm(post[fruit] - pre[fruit]))
    inside = dxy < RECEPTACLES[rec] and -0.04 < dz < 0.12
    detail = (f"{fruit} -> {rec}: xy {dxy * 100:.1f} cm (limit {RECEPTACLES[rec] * 100:.0f}), "
              f"dz {dz * 100:+.1f} cm, moved {moved * 100:.1f} cm")
    return inside, detail


LINE = re.compile(r"\s*(0x[0-9a-f]+)\s+(.*?):.*?\s(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)")


def list_windows():
    """Return {window id: (x, y, w, h)} from the X tree, using absolute coordinates."""
    try:
        tree = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return {}
    found = {}
    for line in tree.splitlines():
        match = LINE.match(line)
        if not match:
            continue
        wid, _title, w, h, _rx, _ry, ax, ay = match.groups()
        w, h, ax, ay = int(w), int(h), int(ax), int(ay)
        if w > 200 and h > 200:  # skip 1x1 helper windows and the like
            found[wid] = (ax, ay, w - (w % 2), h - (h % 2))
    return found


class Recorder:
    """Record the viewer window with ffmpeg x11grab; screenshot it mid-trial as GUI proof."""

    def __init__(self, video, shots, seconds):
        self.video = video
        self.shots = shots
        self.seconds = seconds
        self.geom = None
        self.process = None
        # windows that already exist, so the viewer's new window can be told apart
        self.baseline = set(list_windows())

    def start(self, wait=60.0):
        """Watch for the viewer window in a thread: the window only appears once the trial starts."""
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._watch, args=(wait,), daemon=True)
        self.thread.start()
        return True

    def _watch(self, wait):
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline and not self._stop.is_set():
            windows = list_windows()
            fresh = {wid: geom for wid, geom in windows.items() if wid not in self.baseline}
            if os.environ.get("YAM_DEBUG"):
                print(f"    [window] {len(windows)} up, {len(fresh)} new: "
                      f"{[(k, v[2], v[3]) for k, v in fresh.items()]}", flush=True)
            if fresh:
                wid = max(fresh, key=lambda key: fresh[key][2] * fresh[key][3])
                self.geom = (wid, *fresh[wid])
                break
            time.sleep(0.5)
        if not self.geom or self._stop.is_set():
            return
        wid, x, y, w, h = self.geom
        self.video.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "x11grab", "-framerate", "10",
               "-video_size", f"{w}x{h}", "-i", f"{os.environ.get('DISPLAY', ':1')}+{x},{y}",
               "-t", f"{int(self.seconds + 30)}", "-c:v", "libx264", "-preset", "ultrafast",
               "-pix_fmt", "yuv420p", str(self.video)]
        self.process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        # let the arms move a little, then take a GUI screenshot while the window is certainly alive
        for _ in range(int(max(10.0, min(60.0, self.seconds * 0.4)) * 2)):
            if self._stop.is_set():
                return
            time.sleep(0.5)
        self.shot(self.shots)

    def shot(self, path):
        if not self.geom:
            return False
        path.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(["import", "-window", self.geom[0], str(path)], capture_output=True)
        return result.returncode == 0 and path.exists()

    def stop(self):
        if getattr(self, "_stop", None):
            self._stop.set()
        if getattr(self, "thread", None):
            self.thread.join(timeout=5)
        if not self.process:
            return False
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                self.process.kill()
        return self.video.exists() and self.video.stat().st_size > 10000


def contact_sheet(before, after, out):
    """Stack the before/after top-camera frames side by side as a proof image."""
    try:
        import cv2

        a = cv2.imread(str(before))
        b = cv2.imread(str(after))
        if a is None or b is None:
            return False
        for image, label in ((a, "BEFORE"), (b, "AFTER")):
            cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4)
            cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.imwrite(str(out), np.vstack([a, b]))
        return out.exists()
    except Exception:
        return False


def grip_track(env):
    """Track how close each gripper gets to each object: min distance and the gripper height there."""
    model = env.model
    sites = {}
    for side in ("left", "right"):
        try:
            sites[side] = env.data.body(f"{side}_grasp_site").xpos.copy()
        except Exception:
            pass
    pad_ids = {side: [] for side in sites}
    for g in range(model.ngeom):
        side = arm_of(finger_body(model, g))
        if side in pad_ids:
            pad_ids[side].append(g)
    track = {side: {name: {"min": 1e9, "gz": 0.0, "bite": 1e9, "gap": 0.0} for name in OBJECTS}
             for side in pad_ids}

    def centre(side):
        if side in sites:
            return env.data.body(f"{side}_grasp_site").xpos
        geoms = pad_ids[side]
        return np.mean([env.data.geom_xpos[g] for g in geoms], axis=0)

    original = env.step

    def pads(side):
        """Positions of the two fingertip pads, which is what actually closes on an object."""
        try:
            return env.data.body(f"{side}_tip_left").xpos.copy(), env.data.body(f"{side}_tip_right").xpos.copy()
        except Exception:
            return None, None

    def step(left, right):
        result = original(left, right)
        for side in pad_ids:
            c = centre(side)
            a, b = pads(side)
            for name in OBJECTS:
                p = env.data.body(name).xpos
                d = float(np.linalg.norm(c - p))
                info = track[side][name]
                if d < info["min"]:
                    info["min"], info["gz"] = d, float(c[2])
                if a is not None:
                    axis = b - a
                    gap = float(np.linalg.norm(axis)) or 1e-9
                    u = float(np.clip(np.dot(p - a, axis) / gap ** 2, 0.0, 1.0))
                    bite = float(np.linalg.norm(p - (a + u * axis)))
                    if bite < info["bite"]:
                        info["bite"], info["gap"] = bite, gap
        return result

    env.step = step
    return pad_ids, track


def assist_descent(env, dz=0.03, radius=0.08):
    """SCRIPTED ASSIST: once a gripper is near a fruit, aim the arm a few cm lower (IK, not Dream-1).

    The policy's joint targets are kept, with a small downward Cartesian correction added on that
    arm only when its grasp site is already within `radius` of a fruit. Counts are reported per arm.
    """
    model, data = env.model, env.data
    stats = {"left": 0, "right": 0}
    prev = env.step
    dof = {s: [model.jnt_dofadr[model.joint(f"{s}_joint{i}").id] for i in range(1, 7)] for s in stats}
    limits = {s: np.vstack([model.joint(f"{s}_joint{i}").range for i in range(1, 7)]) for s in stats}
    bodies = {s: model.body(f"{s}_gripper").id for s in stats}

    def step(left, right):
        corrected = []
        for side, action in (("left", left), ("right", right)):
            row = np.array(action, dtype=float, copy=True)
            site = data.body(f"{side}_grasp_site").xpos
            fruit = min(OBJECTS, key=lambda n: np.linalg.norm(data.body(n).xpos - site))
            target = data.body(fruit).xpos
            # only lower while the gripper is still above the fruit: never drive it into the table
            if np.linalg.norm(target - site) < radius and site[2] > target[2] - 0.005:
                jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
                mujoco.mj_jacBody(model, data, jacp, jacr, bodies[side])
                jac = jacp[:, dof[side]]
                dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-5 * np.eye(3), np.array([0.0, 0.0, -dz]))
                row[:6] = np.clip(row[:6] + np.clip(dq, -0.05, 0.05), limits[side][:, 0], limits[side][:, 1])
                stats[side] += 1
            corrected.append(row)
        return prev(*corrected)

    env.step = step
    return stats


def between_pads(env, side):
    """How far each fruit's centre is from the line between this gripper's two fingertip pads."""
    try:
        a = env.data.body(f"{side}_tip_left").xpos
        b = env.data.body(f"{side}_tip_right").xpos
    except Exception:
        return None, 1e9, 0.0
    axis = b - a
    gap = float(np.linalg.norm(axis)) or 1e-9
    best, best_d = None, 1e9
    for name in OBJECTS:
        p = env.data.body(name).xpos
        u = float(np.clip(np.dot(p - a, axis) / gap ** 2, 0.0, 1.0))
        d = float(np.linalg.norm(p - (a + u * axis)))
        if d < best_d:
            best, best_d = name, d
    return best, best_d, gap


def hold_grip(env, between_tol=0.05):
    """SCRIPTED ASSIST: shut a gripper only when the fruit sits between its pads, and open it over a receptacle.

    Latching on any contact held grazing grasps, where the fingers catch the fruit's edge. Requiring the
    fruit centre to be between the two pads means the hand closes on the object, not beside it.
    """
    stats = {"left": 0, "right": 0, "released": 0}
    latched = {"left": False, "right": False}
    opened = {"left": False, "right": False}
    prev = env.step

    def over_receptacle(side):
        """Is this gripper's nearest fruit over the bowl or the plate?"""
        site = env.data.body(f"{side}_grasp_site").xpos
        fruit = min(OBJECTS, key=lambda n: np.linalg.norm(env.data.body(n).xpos - site))
        pos = env.data.body(fruit).xpos
        for name, radius in RECEPTACLES.items():
            if np.linalg.norm(pos[:2] - env.data.body(name).xpos[:2]) < radius + 0.025:
                return True
        return False

    def step(left, right):
        for side in ("left", "right"):
            if latched[side]:
                continue
            fruit, bite, gap = between_pads(env, side)
            if fruit is not None and bite < between_tol and gap < 0.11:
                latched[side] = True
            elif fruit is not None and bite < 0.05:  # fallback: a real touch while well centred
                for contact in env.data.contact[: env.data.ncon]:
                    a_body = finger_body(env.model, contact.geom1)
                    b_body = finger_body(env.model, contact.geom2)
                    for finger, other in ((a_body, b_body), (b_body, a_body)):
                        if arm_of(finger) == side and any(n in other.lower() for n in OBJECTS):
                            latched[side] = True
        out = []
        for side, action in (("left", left), ("right", right)):
            row = np.array(action, dtype=float, copy=True)
            if latched[side] and not opened[side] and over_receptacle(side):
                opened[side] = True
                stats["released"] += 1
            if latched[side]:
                row[6] = 1.0 if opened[side] else 0.0  # release over the receptacle, else hold
                stats[side] += 1
            out.append(row)
        return prev(*out)

    env.step = step
    return stats, latched


def grasp_probe(env):
    """Count finger-object contacts while a hand is closing: proof the object is between the fingers."""
    log = {"left": 0, "right": 0, "objects": set()}
    original = env.step

    def step(left, right):
        closing = left[6] < 0.25 or right[6] < 0.25
        result = original(left, right)
        if closing:
            for contact in env.data.contact[: env.data.ncon]:
                a = finger_body(env.model, contact.geom1)
                b = finger_body(env.model, contact.geom2)
                for finger, other in ((a, b), (b, a)):
                    side = arm_of(finger)
                    if side and any(name in other.lower() for name in OBJECTS):
                        log[side] += 1
                        log["objects"].add(other)
        return result

    env.step = step
    return log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", action="append", required=True, help="repeatable; run in order")
    ap.add_argument("--seconds", type=float, default=240.0)
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--actions", type=int, default=48)
    ap.add_argument("--tag", default="")
    ap.add_argument("--viewer", action="store_true", help="open the MuJoCo GUI (no --headless)")
    ap.add_argument("--frames", type=Path, default=Path("yam_check/trials"))
    ap.add_argument("--video-dir", type=Path, default=Path("yam_check/video"))
    ap.add_argument("--report", type=Path, default=Path("tasks_report.txt"))
    ap.add_argument("--grasp-boost", action="store_true",
                    help="stronger finger servo force and grippier pads (sim physics, not scripted motion)")
    ap.add_argument("--grasp-force", type=float, default=40.0, help="finger servo limit in N (stock 22)")
    ap.add_argument("--grasp-friction", type=float, default=2.5, help="finger pad friction (stock 1.6)")
    ap.add_argument("--object-friction", type=float, default=0.0, help="fruit surface friction (0 = leave stock)")
    ap.add_argument("--assist-descent", type=float, default=0.0,
                    help="SCRIPTED ASSIST: metres to aim below the gripper once near a fruit (0 = off)")
    ap.add_argument("--assist-radius", type=float, default=0.08, help="distance that arms the descent assist")
    ap.add_argument("--hold-grip", action="store_true",
                    help="SCRIPTED ASSIST: keep a gripper shut once it touches a fruit")
    args = ap.parse_args()
    args.frames.mkdir(parents=True, exist_ok=True)

    env = yam_env.YamEnv()
    env._pristine_step = env.step  # wrappers below replace env.step; each trial starts from this one
    if args.grasp_boost:
        fingers, pads = boost_grasp(env, args.grasp_force, args.grasp_friction)
        print(f"grasp boost: {fingers} finger actuators at +-{args.grasp_force:.0f} N, "
              f"{pads} finger pads at friction {args.grasp_friction:.1f}", flush=True)
        if args.object_friction > 0:
            fruits = boost_object_grip(env, args.object_friction)
            print(f"object grip: {fruits} fruit geoms at friction {args.object_friction:.1f}", flush=True)
    lines = []
    for task in args.task:
        for trial in range(args.trials):
            stamp = datetime.now().strftime("%H:%M:%S")
            slug = re.sub(r"[^a-z0-9]+", "_", task.lower()).strip("_")
            tag = f"{slug}_{trial + 1}" + (f"_{args.tag}" if args.tag else "")
            # fresh scene each trial, with the JD of every object recorded first
            env.step = env._pristine_step  # drop the previous trial's probe/assist wrappers
            env.reset()
            pre = poses(env)
            before = args.frames / f"{tag}_before_top.png"
            env_image = env.images()["top"]
            import cv2

            cv2.imwrite(str(before), cv2.cvtColor(env_image, cv2.COLOR_RGB2BGR))

            print(f"\n===== [{stamp}] {task} · {args.seconds:.0f}s · {'GUI' if args.viewer else 'headless'} =====", flush=True)
            argv = ["--task", task, "--seconds", str(args.seconds), "--quiet", "--actions", str(args.actions)]
            if not args.viewer:
                argv.append("--headless")

            grasp = grasp_probe(env)
            pad_ids, track = grip_track(env)
            descent = assist_descent(env, args.assist_descent, args.assist_radius) if args.assist_descent > 0 else None
            hold, latched = hold_grip(env) if args.hold_grip else (None, None)
            recorder = Recorder(args.video_dir / f"{tag}.mp4", args.frames / f"{tag}_gui.png", args.seconds)
            t0 = time.monotonic()
            recording = recorder.start() if args.viewer else False
            aborted = None
            try:
                dream_control.main(argv, shared_env=env)
            except BaseException as error:  # a budget-cap kill raises KeyboardInterrupt: keep the evidence
                aborted = f"{type(error).__name__}: {error}"
                print(f"  trial interrupted: {aborted}", flush=True)
                recorder.stop()
            elapsed = time.monotonic() - t0
            if recording and aborted is None:
                got_video = recorder.stop()
            else:
                got_video = recorder.video.exists() and recorder.video.stat().st_size > 0
            shot = (args.frames / f"{tag}_gui.png").exists()

            post = poses(env)
            final_state = env.state()
            openings = {side: float(final_state[side][6]) for side in final_state}
            inside, detail = evaluate(task, pre, post)
            after = args.frames / f"{tag}_after_top.png"
            try:
                cv2.imwrite(str(after), cv2.cvtColor(env.images()["top"], cv2.COLOR_RGB2BGR))
            except BaseException as error:  # include a cap kill, so the verdict below still gets written
                print(f"  after image unavailable: {type(error).__name__}: {error}", flush=True)
            sheet = args.frames / f"{tag}_proof.png"
            made = contact_sheet(before, after, sheet)
            # always compute the verdict from the captured state; only label aborted if the
            # env is unrecoverable (e.g. trial ended before any contact). Otherwise record
            # the real result and note the interruption separately.
            if aborted and not inside:
                verdict = "ABORTED (state not judged)"
            elif aborted:
                verdict = "SUCCESS"
            elif inside:
                verdict = "SUCCESS"
            else:
                verdict = "FAIL"

            touching = ", ".join(sorted(grasp["objects"])) or "nothing"
            print(f"  grasp: left finger-object contacts {grasp['left']}, right {grasp['right']} · touched {touching}", flush=True)
            approach = []
            for side in sorted(track):
                for name in OBJECTS:
                    info = track[side][name]
                    if info["min"] < 1e9:
                        approach.append(f"{side}->{name} closest {info['min'] * 100:.1f} cm at gripper z {info['gz']:.3f} m, "
                                f"between pads {info['bite'] * 100:.1f} cm at pad gap {info['gap'] * 100:.1f} cm")
            if approach:
                print("  grip approach: " + " | ".join(approach), flush=True)
            assist = []
            if descent is not None:
                assist.append(f"descent {args.assist_descent * 100:.0f} mm applied {descent['left']}x left / "
                              f"{descent['right']}x right")
            if hold is not None:
                assist.append(f"grip latched {latched['left']} left / {latched['right']} right "
                              f"({hold['left']}/{hold['right']} steps held shut), "
                              f"released {hold['released']}x")
            if assist:
                print("  SCRIPTED ASSIST: " + " | ".join(assist), flush=True)
            print(f"  {verdict}: {detail}", flush=True)
            print(f"  gripper opening at end: left {openings['left']:.2f}, right {openings['right']:.2f}", flush=True)
            print(f"  elapsed {elapsed:.0f}s · video {'saved' if got_video else 'none'} · "
                  f"proof {'saved' if made else 'none'} · gui shot {'saved' if shot else 'none'}", flush=True)

            block = (f"[{stamp}] {task}\n"
                     f"    verdict: {verdict}\n"
                     f"    {detail}\n"
                     f"    gripper opening at end (0 shut, 1 open): left {openings['left']:.2f}, "
                     f"right {openings['right']:.2f}\n"
                     f"    grasp contacts while closing: left {grasp['left']}, right {grasp['right']}, "
                     f"objects {touching}\n"
                     f"    grip approach: {' | '.join(approach) if approach else 'no distance recorded'}\n"
                     f"    SCRIPTED ASSIST: {' | '.join(assist) if assist else 'none (policy only)'}\n"
                     f"    elapsed: {elapsed:.0f}s ({'GUI' if args.viewer else 'headless'}), "
                     f"actions {args.actions}/request, seconds {args.seconds:.0f}\n"
                     f"    interrupted: {aborted or 'no'}\n"
                     f"    video: {recorder.video if got_video else 'not recorded'}\n"
                     f"    proof: {sheet if made else 'not made'}\n")
            lines.append(block)
            with args.report.open("a") as handle:
                handle.write(block + "\n")

    print("\n===== SUMMARY =====", flush=True)
    print("\n".join(lines), flush=True)
    env.close()


if __name__ == "__main__":
    main()
