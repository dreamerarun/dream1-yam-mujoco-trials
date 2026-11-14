"""Persistent credit ledger for the Dream-1 API. Never resets on restart.

Pricing (dreamlabs.ai, Dream-1 launch post): $3 / hour of robot time, $2 / hour for
dream-1-flash. One get_action() returns 1.6 s of actions, so one hour is 2,250 calls and
one call is ~$0.00133 on Dream-1.

A "credit" on the platform is the prepaid billing unit; we treat 1 credit = $1 by default
(configurable) and display both the USD estimate and the credit estimate. Two independent
brakes are enforced before every request:
  * cap_usd      - estimated spend may not pass this
  * max_calls    - total paid calls may not pass this (protects us if a credit is NOT $1)
"""
import json
import os
import time

PRICE_PER_HOUR = {
    "dreamlabs/dream-1-beta": 3.0,
    "dream-1-beta": 3.0,
    "dreamlabs/dream-1": 3.0,
    "dream-1": 3.0,
    "dreamlabs/dream-1-flash": 2.0,
    "dream-1-flash": 2.0,
}
SECONDS_PER_CALL = 1.6          # Dream-1 returns 1.6 s of actions per call
CALLS_PER_HOUR = 3600.0 / SECONDS_PER_CALL


class BudgetExceeded(RuntimeError):
    pass


class Ledger:
    """Ledger of paid inference calls, persisted to JSON."""

    def __init__(self, cap_usd=5.0, max_calls=60, ledger_path="budget/credit_ledger.json",
                 model="dreamlabs/dream-1-beta", credit_usd=1.0):
        self.cap = float(cap_usd)
        self.max_calls = int(max_calls)
        self.path = ledger_path
        self.model = model
        self.credit_usd = float(credit_usd)
        self.price_per_hour = PRICE_PER_HOUR.get(model, 3.0)
        self.cost_per_call = self.price_per_hour / CALLS_PER_HOUR
        self.state = {"spent_usd": 0.0, "calls": 0, "model": model, "log": []}
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        if os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    loaded = json.load(f)
                if isinstance(loaded, dict):
                    self.state.update(loaded)
            except (OSError, ValueError):
                pass       # a corrupt ledger must not silently reset the count: keep zeros, but warn

    # -- accounting ------------------------------------------------------------------
    @property
    def spent(self):
        return float(self.state["spent_usd"])

    @property
    def calls(self):
        return int(self.state["calls"])

    @property
    def credits_used(self):
        return self.spent / self.credit_usd

    @property
    def credits_left(self):
        return max(0.0, (self.cap - self.spent) / self.credit_usd)

    def remaining_calls(self):
        by_money = int((self.cap - self.spent) / self.cost_per_call)
        return max(0, min(by_money, self.max_calls - self.calls))

    def estimate_usd(self, calls):
        return calls * self.cost_per_call

    def can_afford(self, calls=1):
        return (self.spent + calls * self.cost_per_call <= self.cap + 1e-12
                and self.calls + calls <= self.max_calls)

    def charge(self, note=""):
        """Call BEFORE a paid request. Conservative: counted even if the request fails."""
        if not self.can_afford(1):
            raise BudgetExceeded(
                f"stop: {self.calls} calls / ${self.spent:.4f} used, "
                f"cap ${self.cap:.2f} and {self.max_calls} calls")
        self.state["spent_usd"] = self.spent + self.cost_per_call
        self.state["calls"] = self.calls + 1
        self.state["model"] = self.model
        self.state["log"].append(
            [time.strftime("%Y-%m-%d %H:%M:%S"), round(self.cost_per_call, 6), note])
        self.state["log"] = self.state["log"][-500:]
        self.save()

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.state, f, indent=1)
        os.replace(tmp, self.path)

    # -- display ---------------------------------------------------------------------
    def summary(self):
        return (f"spent ${self.spent:.4f} ({self.credits_used:.3f} cr) / cap ${self.cap:.2f} "
                f"({self.cap / self.credit_usd:.2f} cr) | calls {self.calls}/{self.max_calls} "
                f"| ~{self.remaining_calls()} calls left")

    def display_lines(self):
        return [
            f"model: {self.model}  (${self.price_per_hour:.2f}/h, ${self.cost_per_call:.5f}/call)",
            f"calls: {self.calls} / hard limit {self.max_calls}",
            f"spent: ${self.spent:.4f}  (~{self.credits_used:.3f} credits @ ${self.credit_usd:.2f})",
            f"remaining: ${max(0.0, self.cap - self.spent):.4f}  (~{self.credits_left:.3f} credits)",
            f"next call costs ~${self.cost_per_call:.5f} (~{self.cost_per_call / self.credit_usd:.5f} credits)",
        ]


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="inspect the credit ledger")
    ap.add_argument("--path", default="budget/credit_ledger.json")
    ap.add_argument("--cap-usd", type=float, default=5.0)
    ap.add_argument("--max-calls", type=int, default=60)
    args = ap.parse_args()
    led = Ledger(cap_usd=args.cap_usd, max_calls=args.max_calls, ledger_path=args.path)
    print("\n".join(led.display_lines()))
