#!/usr/bin/env python3
"""Upload the pick-trial artefacts; re-encode videos that exceed the API body limit."""
import base64
import glob
import json
import os
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

TOKEN = os.environ["GH_TOKEN"]  # never store the token in a file: GitHub secret scanning blocks it
API = "https://api.github.com/repos/dreamerarun/dream1-yam-mujoco-trials/contents/"
LIMIT = 40e6  # keep base64 payload comfortably under the API limit


def gh(method, path, payload=None):
    req = urllib.request.Request(API + path, data=json.dumps(payload).encode() if payload else None, method=method)
    req.add_header("Authorization", "token " + TOKEN)
    req.add_header("Accept", "application/vnd.github+json")
    if payload:
        req.add_header("Content-Type", "application/json")
    return urllib.request.urlopen(req, timeout=600)


def sha(path):
    try:
        with gh("GET", path) as r:
            return json.load(r)["sha"]
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise


def upload(local, remote, message=None):
    payload = {"message": message or f"Add {os.path.basename(remote)}",
               "content": base64.b64encode(open(local, "rb").read()).decode()}
    s = sha(remote)
    if s:
        payload["sha"] = s
    with gh("PUT", remote, payload) as r:
        json.load(r)
    print(f"OK {remote} ({os.path.getsize(local)/1e6:.1f} MB)", flush=True)


def main():
    jobs = []
    for local in sorted(glob.glob("yam_check/video/*pick*.mp4")):
        jobs.append(local)
    for local in sorted(glob.glob("yam_check/trials/*pick*")):
        jobs.append(local)

    for local in jobs:
        remote = local.replace(os.sep, "/")
        name = os.path.basename(local)
        size = os.path.getsize(local)
        if local.endswith(".mp4") and size > LIMIT:
            out = os.path.join(tempfile.gettempdir(), "c_" + name)
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", local,
                            "-vf", "scale=1280:-2", "-c:v", "libx264", "-crf", "23",
                            "-preset", "veryfast", "-an", out], check=True)
            print(f"compressed {name}: {size/1e6:.1f} -> {os.path.getsize(out)/1e6:.1f} MB", flush=True)
            upload(out, remote, f"Add {name} (re-encoded 1280px CRF 23; original {size/1e6:.0f} MB over API limit)")
            os.unlink(out)
        else:
            upload(local, remote)
        time.sleep(0.4)


if __name__ == "__main__":
    main()
