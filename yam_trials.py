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
        if "finger" in body:
            model.geom_friction[g][0] = friction
            pads += 1
    return fingers, pads


def finger_body(model, gid):
    """Which arm a contact geom belongs to, or the object name for the other side of the contact."""
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or ""


def target_receptacle(task):
    """Which receptacle the task names (both is a two-part task; judge the first)."""
    lowered = task.lower()
    hits = [name for name in RECEPTACLES if name in lowered]
    return hits[0] if hits else None


def evaluate(task, pre, post):
    """Decide success from state: the named fruit within the receptacle radius and resting at its height."""
    rec = target_receptacle(task)
    fruit = next((name for name in OBJECTS if name in task.lower()), None)
    if rec is None or fruit is None:
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


def grasp_probe(env):
    """Count finger-object contacts while a hand is closing: proof the object is between the fingers."""
    log = {"left": 0, "right": 0, "objects": set()}
    original = getattr(env, "_probe_step", None) or env.step
    env._probe_step = original

    def step(left, right):
        closing = left[6] < 0.25 or right[6] < 0.25
        result = original(left, right)
        if closing:
            for contact in env.data.contact[: env.data.ncon]:
                a = finger_body(env.model, contact.geom1)
                b = finger_body(env.model, contact.geom2)
                for finger, other in ((a, b), (b, a)):
                    if "finger" in finger and any(name in other.lower() for name in OBJECTS):
                        log["left" if "left" in finger else "right"] += 1
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
    args = ap.parse_args()
    args.frames.mkdir(parents=True, exist_ok=True)

    env = yam_env.YamEnv()
    if args.grasp_boost:
        fingers, pads = boost_grasp(env)
        print(f"grasp boost: {fingers} finger actuators at +-40 N, {pads} finger pads at friction 2.5", flush=True)
    lines = []
    for task in args.task:
        for trial in range(args.trials):
            stamp = datetime.now().strftime("%H:%M:%S")
            slug = re.sub(r"[^a-z0-9]+", "_", task.lower()).strip("_")
            tag = f"{slug}_{trial + 1}" + (f"_{args.tag}" if args.tag else "")
            # fresh scene each trial, with the JD of every object recorded first
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
            recorder = Recorder(args.video_dir / f"{tag}.mp4", args.frames / f"{tag}_gui.png", args.seconds)
            t0 = time.monotonic()
            recording = recorder.start() if args.viewer else False
            try:
                dream_control.main(argv, shared_env=env)
            except Exception as error:
                print(f"  trial error: {type(error).__name__}: {error}", flush=True)
                recorder.stop()
                lines.append(f"[{stamp}] {task} | ERROR {type(error).__name__}: {error}")
                continue
            elapsed = time.monotonic() - t0
            got_video = recorder.stop() if recording else False
            shot = (args.frames / f"{tag}_gui.png").exists()

            post = poses(env)
            inside, detail = evaluate(task, pre, post)
            after = args.frames / f"{tag}_after_top.png"
            cv2.imwrite(str(after), cv2.cvtColor(env.images()["top"], cv2.COLOR_RGB2BGR))
            sheet = args.frames / f"{tag}_proof.png"
            made = contact_sheet(before, after, sheet)
            verdict = "SUCCESS" if inside else ("FAIL" if inside is False else "UNPARSED")

            touching = ", ".join(sorted(grasp["objects"])) or "nothing"
            print(f"  grasp: left finger-object contacts {grasp['left']}, right {grasp['right']} · touched {touching}", flush=True)
            print(f"  {verdict}: {detail}", flush=True)
            print(f"  elapsed {elapsed:.0f}s · video {'saved' if got_video else 'none'} · "
                  f"proof {'saved' if made else 'none'} · gui shot {'saved' if shot else 'none'}", flush=True)

            block = (f"[{stamp}] {task}\n"
                     f"    verdict: {verdict}\n"
                     f"    {detail}\n"
                     f"    grasp contacts while closing: left {grasp['left']}, right {grasp['right']}, "
                     f"objects {touching}\n"
                     f"    elapsed: {elapsed:.0f}s ({'GUI' if args.viewer else 'headless'}), "
                     f"actions {args.actions}/request, seconds {args.seconds:.0f}\n"
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
