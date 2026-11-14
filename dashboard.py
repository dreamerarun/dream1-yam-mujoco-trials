"""Tk dashboard for the Dream-1 + dual-Franka app.

Runs in the *bridge* process (dream-env), so Isaac Sim's Python stays untouched.
It shows the three camera feeds, the robot state, the credit ledger, the inference status and
the logs, and the operator's buttons drive the bridge state that Isaac reads through the socket.
"""
import threading
import time
from collections import deque

import tkinter as tk
from tkinter import ttk

try:
    from PIL import Image, ImageTk
except Exception:  # noqa: BLE001
    Image = ImageTk = None

BG = "#14171c"
PANEL = "#1c2129"
FG = "#dfe4ea"
MUTED = "#8b95a3"
OK = "#4ec96b"
WARN = "#e8b23a"
BAD = "#e8544a"
ACCENT = "#61afef"
MODE_COLORS = {"MOCK": WARN, "SCRIPTED": WARN, "REAL DREAM-1 INFERENCE": OK,
               "DISCONNECTED": MUTED, "HOLD": ACCENT}


class Hub:
    """Thread-safe state shared by the socket thread and the Tk main loop."""

    def __init__(self, mode="MOCK", task="bimanual_bins", prompt=""):
        self.lock = threading.Lock()
        self.mode = mode
        self.task = task
        self.prompt = prompt
        self.running = False
        self.estop = False
        self.reset_request = False
        self.arm_enabled = {"left": True, "right": True}
        self.frames = {}
        self.frames_at = 0.0
        self.state = {}
        self.lines = deque(maxlen=600)
        self.info = {}
        self.connected = False
        self.log("dashboard up")

    def log(self, msg):
        stamp = time.strftime("%H:%M:%S")
        with self.lock:
            self.lines.append(f"{stamp}  {msg}")

    def push_frames(self, frames, state, prompt=None):
        with self.lock:
            self.frames = frames
            self.state = state or {}
            self.frames_at = time.time()
            if prompt:
                self.prompt = prompt

    def push_info(self, **kw):
        with self.lock:
            self.info.update(kw)

    def get_flags(self):
        with self.lock:
            return {"running": self.running, "estop": self.estop,
                    "reset": self.reset_request, "arm_enabled": dict(self.arm_enabled),
                    "mode": self.mode, "prompt": self.prompt, "task": self.task}

    def take_reset(self):
        with self.lock:
            was = self.reset_request
            self.reset_request = False
            return was


class Dashboard:
    def __init__(self, hub, ledger_fn, on_quit=None):
        self.hub = hub
        self.ledger_fn = ledger_fn
        self.on_quit = on_quit
        self.photos = {}
        self.root = tk.Tk()
        self.root.title("Dream-1 × Isaac Sim — dual Franka bimanual control")
        self.root.configure(bg=BG)
        self.root.geometry("1180x820+8+40")
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        try:      # keep the dashboard visible above the Isaac Sim window
            self.root.attributes("-topmost", True)
        except Exception:  # noqa: BLE001
            pass
        self._build()
        self.root.after(300, self._place)
        self.refresh()

    def _place(self):
        """Put the window where the operator can see it next to the Isaac Sim viewport."""
        try:
            self.root.geometry("1150x800+24+24")
            self.root.lift()
            self.root.attributes("-topmost", True)
        except Exception:  # noqa: BLE001
            pass

    # -- layout ----------------------------------------------------------------------
    def _build(self):
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(top, text="Dream-1 · dual Franka · Isaac Sim 5.1", bg=BG, fg=FG,
                 font=("DejaVu Sans", 14, "bold")).pack(side="left")
        self.mode_lbl = tk.Label(top, text="MODE: " + self.hub.mode, bg=MODE_COLORS.get(self.hub.mode, MUTED),
                                 fg="#101317", font=("DejaVu Sans", 12, "bold"), padx=12, pady=3)
        self.mode_lbl.pack(side="right")
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=10)

        # camera column
        cams = tk.Frame(body, bg=BG)
        cams.pack(side="left", fill="y")
        self.cam_lbls = {}
        for key, title in (("overhead", "overhead"), ("wrist_l", "wrist left"),
                           ("wrist_r", "wrist right")):
            box = tk.Frame(cams, bg=PANEL, padx=4, pady=4)
            box.pack(pady=4, anchor="w")
            tk.Label(box, text=title, bg=PANEL, fg=MUTED, font=("DejaVu Sans", 9)).pack(anchor="w")
            holder = tk.Frame(box, bg="#000000", width=320, height=176)
            holder.pack_propagate(False)          # pixel size, not text units
            holder.pack()
            lbl = tk.Label(holder, bg="#000000", text="NO FRAME", fg=BAD, font=("DejaVu Sans", 9))
            lbl.pack(fill="both", expand=True)
            self.cam_lbls[key] = lbl

        # right column
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(12, 0))

        self.status = tk.Text(right, height=11, width=72, bg=PANEL, fg=FG, font=("DejaVu Sans Mono", 9),
                              relief="flat", padx=8, pady=6)
        self.status.pack(fill="x")
        self.status.configure(state="disabled")

        ctl = tk.Frame(right, bg=PANEL, padx=8, pady=8)
        ctl.pack(fill="x", pady=8)
        row1 = tk.Frame(ctl, bg=PANEL)
        row1.pack(fill="x")
        self.btn_run = tk.Button(row1, text="▶  Run inference", command=self.toggle_run, width=16,
                                 bg="#22313f", fg=FG, relief="flat", font=("DejaVu Sans", 10, "bold"))
        self.btn_run.pack(side="left", padx=3)
        tk.Button(row1, text="■  E-STOP", command=self.estop, width=12, bg="#5c2320", fg=FG,
                  relief="flat", font=("DejaVu Sans", 10, "bold")).pack(side="left", padx=3)
        tk.Button(row1, text="⟲  Reset scene", command=self.reset, width=14, bg="#22313f", fg=FG,
                  relief="flat").pack(side="left", padx=3)

        row2 = tk.Frame(ctl, bg=PANEL)
        row2.pack(fill="x", pady=(8, 0))
        tk.Label(row2, text="Task:", bg=PANEL, fg=MUTED).pack(side="left")
        self.task_var = tk.StringVar(value=self.hub.task)
        self.task_box = ttk.Combobox(row2, textvariable=self.task_var, width=18, state="readonly")
        self.task_box.pack(side="left", padx=6)
        self.task_box.bind("<<ComboboxSelected>>", self.pick_task)
        self.arm_vars = {}
        for side in ("left", "right"):
            v = tk.BooleanVar(value=True)
            self.arm_vars[side] = v
            tk.Checkbutton(row2, text=f"{side} arm", variable=v, bg=PANEL, fg=FG, selectcolor=PANEL,
                           activebackground=PANEL, activeforeground=FG,
                           command=lambda s=side: self.toggle_arm(s)).pack(side="left", padx=6)

        self.prompt_lbl = tk.Label(right, text="", bg=BG, fg=ACCENT, wraplength=700, justify="left",
                                   font=("DejaVu Sans", 10, "italic"))
        self.prompt_lbl.pack(fill="x", pady=(4, 6))

        self.logbox = tk.Text(right, height=12, width=72, bg="#0f1216", fg="#c9d1d9",
                              font=("DejaVu Sans Mono", 9), relief="flat", padx=8, pady=6)
        self.logbox.pack(fill="both", expand=True)
        self.logbox.configure(state="disabled")

    # -- actions ---------------------------------------------------------------------
    def toggle_run(self):
        with self.hub.lock:
            self.hub.running = not self.hub.running
            running = self.hub.running
        self.hub.log("operator: RUN" if running else "operator: pause")
        self.btn_run.configure(text="❚❚  Pause" if running else "▶  Run inference")

    def estop(self):
        with self.hub.lock:
            self.hub.estop = True
            self.hub.running = False
        self.hub.log("operator: EMERGENCY STOP (arms hold)")
        self.btn_run.configure(text="▶  Run inference")

    def reset(self):
        with self.hub.lock:
            self.hub.estop = False
            self.hub.reset_request = True
        self.hub.log("operator: reset scene requested")

    def pick_task(self, _evt=None):
        with self.hub.lock:
            self.hub.task = self.task_var.get()
        self.hub.log(f"operator: task -> {self.task_var.get()}")

    def toggle_arm(self, side):
        with self.hub.lock:
            self.hub.arm_enabled[side] = bool(self.arm_vars[side].get())
        self.hub.log(f"operator: {side} arm {'enabled' if self.arm_vars[side].get() else 'disabled'}")

    def set_tasks(self, ids, titles):
        self.task_box["values"] = [f"{i}" for i in ids]
        self._task_titles = titles

    def quit(self):
        self.hub.log("operator: quit")
        if self.on_quit:
            try:
                self.on_quit()
            except Exception:  # noqa: BLE001
                pass
        self.root.quit()
        self.root.destroy()

    # -- periodic redraw -------------------------------------------------------------
    def refresh(self):
        flags = self.hub.get_flags()
        mode = flags["mode"]
        self.mode_lbl.configure(text="MODE: " + mode, bg=MODE_COLORS.get(mode, MUTED))

        if Image is not None:
            with self.hub.lock:
                frames = dict(self.hub.frames)
                age = time.time() - self.hub.frames_at
            for key, lbl in self.cam_lbls.items():
                b64 = frames.get(key)
                if not b64 or age > 3.0:
                    lbl.configure(image="", text="NO FRAME", fg=BAD, bg="#000000")
                    self.photos.pop(key, None)
                    continue
                try:
                    import base64
                    import io
                    img = Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")
                    if img.size != (320, 176):
                        img = img.resize((320, 176))
                    ph = ImageTk.PhotoImage(img)
                    self.photos[key] = ph
                    lbl.configure(image=ph, text="")
                except Exception:  # noqa: BLE001
                    lbl.configure(text="DECODE ERROR", fg=BAD)

        lines = [f"{k}: {v}" for k, v in self.hub.info.items()]
        lines += self.ledger_fn()
        self.status.configure(state="normal")
        self.status.delete("1.0", "end")
        self.status.insert("end", "\n".join(lines))
        self.status.configure(state="disabled")

        self.prompt_lbl.configure(text="instruction sent: " + (flags["prompt"] or "—"))

        with self.hub.lock:
            logs = list(self.hub.lines)[-200:]
        self.logbox.configure(state="normal")
        self.logbox.delete("1.0", "end")
        self.logbox.insert("end", "\n".join(logs))
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

        self.root.after(200, self.refresh)

    def run(self):
        self.root.mainloop()
