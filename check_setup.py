"""Free pre-flight check for the Dream side of the app. Makes NO inference calls (no credits).

    cd ~/dream1_isaac && ~/dream-env/bin/python check_setup.py

It checks, in order:
  1. the dream library imports and which version it is;
  2. the API key in ~/.dreamlabs/credentials is accepted (GET /v1/models — free);
  3. the checkpoint we run is listed and servable, and which camera roles it reads;
  4. the arm file (from `dreamcli tcp`) parses, and the policy opens with our config;
  5. the IK solver for that arm builds (client.wait_for_ik) — free, local;
  6. the ledger state and the cost of one call.
Nothing here allocates a model serve, and no get_action() is called.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from budget import Ledger  # noqa: E402
from embodiment_config import DEFAULT_ARM_FILE, DEFAULT_CHECKPOINT, policy_config  # noqa: E402

ARM_FILE = os.environ.get("DREAM_ARM_FILE", DEFAULT_ARM_FILE)
CHECKPOINT = os.environ.get("DREAM_MODEL", DEFAULT_CHECKPOINT)
ROWS = []


def say(k, v=""):
    ROWS.append(f"{k}: {v}")
    print(f"{k}: {v}", flush=True)


try:
    import dream
except Exception as e:  # noqa: BLE001
    say("dream import", f"FAIL {e!r}")
    sys.exit(1)
say("dream library", f"{dream.__version__} ({dream.__file__})")

cred = os.path.expanduser("~/.dreamlabs/credentials")
say("credentials file", "present" if os.path.exists(cred) else "MISSING")

try:
    client = dream.DreamClient()
except Exception as e:  # noqa: BLE001
    say("client", f"FAIL {e!r}")
    sys.exit(1)

try:
    models = client.get_models()
except Exception as e:  # noqa: BLE001
    say("get_models (free)", f"FAIL {e!r}")
    sys.exit(1)

say("models available", ", ".join(f"{m.id}{'*' if m.servable else ''}" for m in models))
mine = next((m for m in models if m.id == CHECKPOINT), None)
if mine is None:
    say("checkpoint", f"FAIL {CHECKPOINT} is not listed for this key")
    sys.exit(1)
say("checkpoint", f"{mine.id} | owner={mine.owner} | embodiment={mine.embodiment} "
                  f"| servable={mine.servable} | cameras={mine.cameras}")

if not os.path.exists(ARM_FILE):
    say("arm file", f"FAIL missing {ARM_FILE} (create it with `dreamcli tcp`)")
    sys.exit(1)
with open(ARM_FILE) as f:
    arm = json.load(f)
say("arm file", f"{ARM_FILE} | base={arm.get('base')} flange={arm.get('flange')} tcp={arm.get('tcp')} "
                f"| calibrated={(arm.get('tcp_calibration') or {}).get('status')}")

cfg = policy_config(CHECKPOINT, ARM_FILE)
try:
    client.policy(cfg)
except Exception as e:  # noqa: BLE001
    say("policy()", f"FAIL {e!r}")
    sys.exit(1)
say("policy()", "opened (no request sent to the model yet)")
say("pacing", f"execution_horizon={client.execution_horizon} action_hz={client.action_hz} "
              f"mode={client.mode}")

try:
    ok = client.wait_for_ik(60.0)
    say("IK build", f"{'OK' if ok else 'TIMEOUT'} (local dream_ik, free)")
except Exception as e:  # noqa: BLE001
    say("IK build", f"FAIL {e!r}")

led = Ledger(cap_usd=5.0, max_calls=60, ledger_path=os.path.join(HERE, "budget/credit_ledger.json"),
             model=CHECKPOINT)
for line in led.display_lines():
    say("ledger", line)
say("credits spent by this check", f"$0.0000 (0 calls) — policy() only listed the model")

with open(os.path.join(HERE, "logs", "setup_check.txt"), "w") as f:
    f.write("\n".join(ROWS) + "\n")
client.close()
print("\nwrote logs/setup_check.txt")
