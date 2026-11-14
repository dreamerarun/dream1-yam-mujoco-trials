#!/usr/bin/env python3
"""Upload trial artefacts to GitHub, skipping files that already exist."""
import base64
import json
import os
import sys
import time
import urllib.error
import urllib.request

TOKEN = os.environ.get("GH_TOKEN", "")
OWNER = "dreamerarun"
REPO = "dream1-yam-mujoco-trials"
API = f"https://api.github.com/repos/{OWNER}/{REPO}/contents/"


def gh(method, path, payload=None):
    url = API + path
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Authorization", "token " + TOKEN)
    req.add_header("Accept", "application/vnd.github+json")
    if data:
        req.add_header("Content-Type", "application/json")
    return urllib.request.urlopen(req, timeout=600)


def remote_sha(path):
    try:
        with gh("GET", path) as r:
            return json.load(r).get("sha")
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise


def upload(local, remote, attempts=4):
    size = os.path.getsize(local)
    for attempt in range(1, attempts + 1):
        try:
            sha = remote_sha(remote)
            with open(local, "rb") as f:
                content = base64.b64encode(f.read()).decode()
            payload = {"message": f"Add {os.path.basename(local)}", "content": content}
            if sha:
                payload["sha"] = sha
            with gh("PUT", remote, payload) as r:
                json.load(r)
            print(f"OK   {remote} ({size/1e6:.1f} MB)", flush=True)
            return True
        except urllib.error.HTTPError as e:
            body = e.read().decode(errors="replace")[:200]
            print(f"ERR  {remote} attempt {attempt}: HTTP {e.code} {body}", flush=True)
            if e.code == 422 and "sha" in body.lower():
                time.sleep(1)
                continue
            time.sleep(2 * attempt)
        except Exception as e:  # network / timeout
            print(f"ERR  {remote} attempt {attempt}: {e}", flush=True)
            time.sleep(2 * attempt)
    print(f"FAIL {remote}", flush=True)
    return False


def main():
    if not TOKEN:
        print("GH_TOKEN not set", file=sys.stderr)
        return 1

    jobs = []
    # source code, scripts, reports at repo root
    root_exts = (".py", ".sh", ".txt", ".md", ".json", ".usd", ".log")
    for name in sorted(os.listdir(".")):
        if os.path.isfile(name) and name.endswith(root_exts):
            jobs.append((name, name))
    # remaining trial images
    for name in sorted(os.listdir("yam_check/trials")):
        local = os.path.join("yam_check/trials", name)
        jobs.append((local, "yam_check/trials/" + name))
    # reference frames
    for name in sorted(os.listdir("yam_check/frames")):
        local = os.path.join("yam_check/frames", name)
        jobs.append((local, "yam_check/frames/" + name))
    # videos last (largest)
    for name in sorted(os.listdir("yam_check/video")):
        local = os.path.join("yam_check/video", name)
        jobs.append((local, "yam_check/video/" + name))

    # allow selecting a subset: python gh_upload.py video|trials|all
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    if which == "video":
        jobs = [j for j in jobs if "/video/" in j[1]]
    elif which == "trials":
        jobs = [j for j in jobs if "/trials/" in j[1] or "/frames/" in j[1]]
    elif which == "code":
        jobs = [j for j in jobs if "/" not in j[1]]
    elif which == "files":
        jobs = [j for j in jobs if "/video/" not in j[1]]

    ok = fail = 0
    for local, remote in jobs:
        if upload(local, remote):
            ok += 1
        else:
            fail += 1
        time.sleep(0.4)
    print(f"DONE: {ok} uploaded, {fail} failed", flush=True)
    return 0 if fail == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
