"""Bridge between Isaac Sim (Python 3.11) and the Dream client (your ~/dream-env, Python 3.12).

Run in the dream-env terminal:
    python dream_bridge.py --mock                         # free: holds the arms still
    python dream_bridge.py --arm-file <arm.json> --cap-usd 8   # real calls, hard cap

Talks to run_dream1_isaac.py over localhost only (127.0.0.1) with a shared authkey.
"""
import argparse, os, sys, time
import numpy as np
from multiprocessing.connection import Listener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from budget import Ledger

AUTHKEY = b"dream-local-bridge"
ap = argparse.ArgumentParser()
ap.add_argument("--arm-file", help="arm JSON written by `dreamcli tcp` (needed for real calls)")
ap.add_argument("--arm", help="use a shipped Dream arm instead of an arm file, e.g. yam")
ap.add_argument("--model", default="dream-1-beta")
ap.add_argument("--cap-usd", type=float, default=5.0)
ap.add_argument("--port", type=int, default=6006)
ap.add_argument("--action-hz", type=float, default=30.0)
ap.add_argument("--max-calls", type=int, default=60, help="hard stop on paid calls")
ap.add_argument("--credit-usd", type=float, default=1.0, help="USD per platform credit (display only)")
ap.add_argument("--mock", action="store_true")
args = ap.parse_args()

ledger = Ledger(cap_usd=args.cap_usd, max_calls=args.max_calls, model=args.model,
                credit_usd=args.credit_usd,
                ledger_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "budget", "credit_ledger.json"))
ROWS = 48
client = None
if not args.mock:
    if not args.arm_file and not args.arm:
        sys.exit("real calls need --arm-file (create it with: dreamcli tcp) or --arm yam")
    import dream
    key = os.environ.get("DREAM_API_KEY")
    if not key:
        # read the key the CLI already stored, so no manual export is needed
        try:
            with open(os.path.expanduser("~/.dreamlabs/credentials")) as f:
                for line in f:
                    if line.strip().startswith("DREAM_API_KEY="):
                        key = line.split("=", 1)[1].strip()
                        break
        except OSError:
            pass
    if not key:
        sys.exit("no DREAM_API_KEY and no ~/.dreamlabs/credentials")
    client = dream.DreamClient(api_key=key)
    from embodiment_config import policy_config
    client.policy(policy_config(f"dreamlabs/{args.model}",
                                arm_file=args.arm_file or "",
                                action_hz=args.action_hz, timeout_ms=45000, arm=args.arm))
    print("[bridge] arm:", args.arm or f"file {args.arm_file}", flush=True)
print("[bridge]", "MOCK" if args.mock else f"model={args.model}", "|", ledger.summary(), flush=True)

last_state = {}
first_reply = True


def get_action(commanded):
    for _ in range(8):
        try:
            return client.get_action(commanded=commanded or None)
        except Exception as e:  # first call to a cold model is refused with pool_warming: wait, retry, not billed
            if "pool_warming" in str(e) or getattr(e, "reason", "") == "pool_warming":
                wait = float(getattr(e, "retry_after_s", 5) or 5)
                print(f"[bridge] model warming, retry in {wait:.0f}s (not billed)", flush=True)
                time.sleep(wait)
                continue
            raise
    raise RuntimeError("model still warming after 8 tries")


def handle(msg):
    global first_reply
    cmd = msg["cmd"]
    if cmd == "reset":
        if client: client.reset_episode()
        return {"ok": True}
    if cmd == "obs":
        last_state.update(msg["state"])
        if client:
            # the policy takes camera frames under the video.<role> names, not the bare role
            frames = {f"video.{k}": v for k, v in msg["frames"].items()}
            try:
                client.set_observation(frames, {**msg["state"], "prompt": msg["prompt"]},
                                       timestamp=msg["t"])
            except Exception as e:  # noqa: BLE001
                print("[bridge] observation rejected:", repr(e), flush=True)
                return {"error": repr(e)}
        return {"ok": True}
    if cmd == "act":
        if not ledger.can_afford():
            return {"error": f"BUDGET: {ledger.summary()}"}
        if args.mock:
            # Generate progressive mock joint positions so arms appear to move
            def make_rows(side):
                arm = np.asarray(last_state[f"state.{side}_arm"], float)
                grip = float(np.ravel(last_state[f"state.{side}_gripper"])[0])
                drift = np.clip(np.random.randn(7) * 0.1, -0.5, 0.5)
                moved = np.clip(arm + drift, -1.0, 1.0)
                row = list(moved) + [grip]
                return [row] * ROWS
            return {"q_r_chunk": make_rows("right"), "q_l_chunk": make_rows("left"),
                "hold": True, "budget": ledger.summary()}
        ledger.charge(msg.get("note", ""))                # count before the request: conservative
        out = get_action(msg.get("commanded"))
        if first_reply:
            print("[bridge] action keys:", {k: np.asarray(v).shape if not isinstance(v, dict) else list(v) for k, v in out.items()}, flush=True)
            pk = out.get("per_key") or {}
            for k in sorted(pk):
                a = np.asarray(pk[k], float)
                print(f"[bridge] per_key {k} shape={a.shape} row0={np.round(a[0], 4).tolist() if a.ndim == 2 else a}",
                      flush=True)
                if a.ndim == 2 and len(a) > 1:
                    print(f"[bridge] per_key {k} row-1={np.round(a[-1], 4).tolist()}", flush=True)
            first_reply = False
        if "q_r_chunk" not in out or "q_l_chunk" not in out:
            return {"error": f"reply has no q_r_chunk/q_l_chunk; keys={list(out)} (is `ik` active?)"}
        return {"q_r_chunk": np.asarray(out["q_r_chunk"]).tolist(), "q_l_chunk": np.asarray(out["q_l_chunk"]).tolist(),
                "budget": ledger.summary()}
    return {"error": f"unknown cmd {cmd}"}


with Listener(("127.0.0.1", args.port), authkey=AUTHKEY) as lst:
    print(f"[bridge] waiting for Isaac Sim on 127.0.0.1:{args.port}", flush=True)
    while True:
        conn = lst.accept()
        print("[bridge] Isaac connected", flush=True)
        try:
            while True:
                msg = conn.recv()
                if msg["cmd"] == "quit":
                    break
                try:
                    conn.send(handle(msg))
                except Exception as e:
                    print("[bridge] ERROR:", repr(e), flush=True)
                    conn.send({"error": repr(e)})
        except EOFError:
            pass
        print("[bridge] Isaac disconnected |", ledger.summary(), flush=True)
        if msg.get("cmd") == "quit":
            if client: client.close()
            break
