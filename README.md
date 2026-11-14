# Dream-1 × YAM MuJoCo — bimanual manipulation trials

Recorded trials of the **Dream-1** policy driving a **YAM bimanual arm** in MuJoCo, run through
Dream's own control loop with the MuJoCo GUI open and the viewer window captured to video.

Every trial is a **real model run**: Dream-1 receives camera observations and produces the joint
targets itself, at ~20 Hz, through `dream_control.main` (the same loop the interactive example
uses). Nothing in the motion is scripted. Success is decided afterwards from MuJoCo body poses,
not from anything the model says about itself.

---

## Results

| # | Task | Time | Verdict | Measurement |
|---|------|------|---------|-------------|
| 1 | put apple into the bowl | 240 s | **FAIL** | xy 11.6 cm (limit 6) |
| 2 | put banana on the plate | 240 s | **FAIL** | xy 20.9 cm (limit 7) |
| 3 | put orange into the bowl | 240 s | **SUCCESS** | xy 0.2 cm, dz +3.7 cm |
| 4 | put apple on the plate | 240 s | **FAIL** | xy 34.9 cm (limit 7) |
| 5 | put banana into the bowl | 300 s | **SUCCESS** | xy 2.7 cm, dz +6.0 cm |
| 6 | put orange on the plate | 240 s | **FAIL** | pose drift, xy 832 cm |
| 7 | apple→bowl + banana→plate | 300 s | **ERROR** | Dream API unreachable at the time |
| 8 | orange→plate + apple→bowl | 300 s | **FAIL** | apple xy 39.6 cm (limit 6) |

**2 successes of 8 trials.** Both successes are single-object placements into the bowl with good
horizontal alignment. Plate trials failed on centring; one combined trial failed on an API outage.

A later trial, **pick banana + apple, one per arm**, is recorded in `tasks_report.txt`.

## How a trial is measured

- `dxy` — horizontal distance between the fruit body and the receptacle body (metres → cm).
- `dz` — vertical offset from the receptacle surface.
- Success for a placement: `dxy < radius` and `-0.04 < dz < 0.12`
  (bowl radius 0.060 m, plate radius 0.070 m).
- Success for a pick: each named object raised clear of the table and held.
- Grasp probe — counts finger↔object contacts while a hand is closing.

## Artefacts

```
tasks_report.txt              verdicts, measurements, spend, notes (appended per trial)
yam_trials.py                 trial driver: GUI, threaded recorder, grasp boost, grasp probe
yam_budget_runner.py          hard budget gate (refuses to start past the cap)
budget.py                     spend accounting used by the bridge
yam_ledger.json               ledger: seconds run, cap, rate, estimated spend
yam_check/video/              one .mp4 per trial, GUI window capture (x11grab, 10 fps)
yam_check/trials/             <task>_before_top.png, _after_top.png, _proof.png, _gui.png
yam_check/frames/             reference left / right / top camera frames
yam_left/right/probe.usd      arm assets used by the scene
README.md, UPLOAD_NOTES.md    this file and upload notes
```

## Reproducing a trial

```bash
cd ~/dream1_isaac

# GUI + recording, one task, 240 s
export DISPLAY=:1
MUJOCO_GL=glx ~/dream-env/bin/python yam_trials.py \
    --viewer \
    --video-dir yam_check/video \
    --report tasks_report.txt \
    --task "put orange into the bowl" \
    --seconds 240
```

`--grasp-boost` raises the finger servo limit to ±40 N (stock ±22) and finger-pad friction to 2.5
(stock 1.6). This is a **physics setting only** — it strengthens the gripper, it does not move the
arm. The value used for each trial is printed in that trial's log line and in the report.

Budget: runs are charged at $3/h against a hard $5.00 cap enforced by `yam_budget_runner.py`;
`yam_ledger.json` is the source of truth for spend.

## Honest limitations

- Only 2 of 8 tasks succeeded; the model is good at bowl placement with a clean approach, weak at
  centring over the smaller plate target.
- The grasp probe usually counts 0 finger↔object contacts even when the object visibly moves, with
  or without the boost. Contact counting is not reliable evidence of a grasp here; the video is.
- Two-object tasks depend on the external Dream API being up; one trial errored for that reason.
- Trials run at $3/h are expensive; the ledger, not wall-clock estimates, is authoritative.

## Upload note

Large binaries were pushed through the GitHub Contents API. The 50 MB two-object video was
re-encoded (1280 px, CRF 23, 6 MB) because the original exceeded the API request limit; see
`UPLOAD_NOTES.md`.
