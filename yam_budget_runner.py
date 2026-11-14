#!/usr/bin/env python3
"""Budget-capped launcher for Dream's YAM MuJoCo sim:  dreamcli example yam-mujoco

The example has no spending cap of its own, so this wrapper adds one. It cannot read your credits, so it
bounds spend by TIME: every second the example process is alive is priced at --rate-per-hour (default $3/h,
Dream-1's list price). That is an UPPER BOUND on real spend (idle time at the prompt is counted too).
The total is saved in yam_ledger.json, so quitting and relaunching never resets the cap.

  python yam_budget_runner.py                      # launch the sim, $5 cap, ledger-backed
  python yam_budget_runner.py --status             # show used / remaining
  python yam_budget_runner.py --minutes 50         # also stop this session after 50 min
  python yam_budget_runner.py -- --verbose         # everything after -- goes to the example
Free self-test of this wrapper:  --cmd "sleep 100" --cap-usd 0.01
"""
import argparse, json, os, shlex, signal, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
LEDGER = os.path.join(HERE, "yam_ledger.json")

ap = argparse.ArgumentParser(add_help=True)
ap.add_argument("--cap-usd", type=float, default=5.0)
ap.add_argument("--rate-per-hour", type=float, default=3.0)
ap.add_argument("--minutes", type=float, default=None, help="optional session time limit")
ap.add_argument("--status", action="store_true")
ap.add_argument("--cmd", default="dreamcli example yam-mujoco", help="command to run (default: Dream's YAM sim)")
ap.add_argument("rest", nargs=argparse.REMAINDER)
args = ap.parse_args()
extra = [a for a in args.rest if a != "--"]

max_s = args.cap_usd / args.rate_per_hour * 3600.0


def load():
    try:
        return float(json.load(open(LEDGER))["seconds"])
    except Exception:
        return 0.0


def save(sec):
    tmp = LEDGER + ".tmp"
    json.dump({"seconds": sec, "cap_usd": args.cap_usd, "rate_per_hour": args.rate_per_hour,
               "est_spent_usd": round(sec / 3600 * args.rate_per_hour, 4)}, open(tmp, "w"))
    os.replace(tmp, LEDGER)


used = load()


def summary(u):
    return (f"est. spend <= ${u / 3600 * args.rate_per_hour:.3f} of ${args.cap_usd:.2f} "
            f"({u / 60:.1f} of {max_s / 60:.1f} min of runtime)")


print("[budget]", summary(used), flush=True)
if args.status:
    sys.exit(0)
if used >= max_s:
    sys.exit("[budget] cap already reached. Not launching. (Raise --cap-usd only if your dashboard says you have credit.)")

deadline = time.monotonic() + args.minutes * 60 if args.minutes else None
cmd = shlex.split(args.cmd) + extra
# the parent ignores Ctrl-C (the child, in the same foreground group, still gets it and handles it itself)
signal.signal(signal.SIGINT, signal.SIG_IGN)
child = subprocess.Popen(cmd, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
t0 = time.monotonic()
last_save = 0.0
reason = None
try:
    while child.poll() is None:
        now = time.monotonic()
        total = used + (now - t0)
        if sys.stdout.isatty():  # show the budget in the terminal TITLE so the app's own screen is untouched
            sys.stdout.write(f"\033]0;YAM sim | ${total / 3600 * args.rate_per_hour:.2f}/${args.cap_usd:.2f} | "
                             f"{(max_s - total) / 60:.0f} min left\007"); sys.stdout.flush()
        if now - last_save > 10:
            save(total); last_save = now
        if total >= max_s:
            reason = f"BUDGET CAP ${args.cap_usd:.2f} reached"
        elif deadline and now >= deadline:
            reason = f"session time limit ({args.minutes:.0f} min) reached"
        if reason:
            print(f"\n[budget] {reason}: stopping the sim.", flush=True)
            for sig, wait in ((signal.SIGINT, 4), (signal.SIGINT, 4), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
                if child.poll() is not None:
                    break
                child.send_signal(sig)
                try:
                    child.wait(timeout=wait)
                except subprocess.TimeoutExpired:
                    pass
            break
        time.sleep(1.0)
finally:
    total = used + (time.monotonic() - t0)
    save(total)
    print("\n[budget]", summary(total), flush=True)
sys.exit(child.returncode if child.returncode is not None else 1)
