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

## Pick trials (5 attempts, after the placements)

Goal: pick the banana and the apple one at a time, one per arm, with a stronger grip.
Success = each named fruit raised more than 5 cm above its starting height.

| Attempt | Task wording | Grip settings | Result |
|---------|--------------|---------------|--------|
| 1 | banana + apple, one in each hand, 300 s | 40 N | FAIL, no lift |
| 2 | banana left arm / apple right arm, 180 s | 40 N | FAIL, no lift |
| 3 | banana then apple, 240 s | 60 N | FAIL, pad boost not applied yet (name bug) |
| 4 | banana then apple, 240 s | 60 N, pads 4.0 | FAIL; banana touched, +3.5 cm |
| 5 | banana then apple, 180 s | 80 N, pads 6.0, fruit 3.0 | FAIL; contacts left 87 / right 621 |

Attempt 5 shows the policy does reach the right objects: the left grasp site came within **2.1 cm** of
the apple and the right within **2.9 cm** of the banana, with sustained finger contact (87 and 621
closing-step contacts). It closes on them but never lifts — vertical change stayed under 2 cm even at
80 N finger force and 6.0 pad friction. Grip strength is not the limiting factor here; the arm produces
no upward motion after contact. Fixing that would mean scripting the motion, which would stop being
Dream-1's own behaviour.

**Probe bug, corrected:** earlier trials printed `contacts 0 / touched nothing`. The probe searched for
`"finger"` in gripper body names, but the bodies are `left_gripper` / `right_gripper` / `left_tip_*`, so
it never matched. The same bug meant the pad-friction half of the grasp boost silently did nothing in
those runs. Both are fixed; attempts 4-5 use the corrected probe and the friction really applies.

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

`--grasp-boost` raises the finger servo limit (default ±40 N, stock ±22) and finger-pad friction
(default 2.5, stock 1.6). `--grasp-force`, `--grasp-friction` and `--object-friction` override these.
This is a **physics setting only** — it strengthens the gripper, it does not move the arm. The values
actually applied are printed per trial (e.g. `286 finger pads at friction 6.0`).

Budget: runs are charged at $3/h against a hard $5.00 cap enforced by `yam_budget_runner.py`;
`yam_ledger.json` is the source of truth for spend.

## Honest limitations

- Only 2 of 8 placement tasks succeeded; the model is good at bowl placement with a clean approach,
  weak at centring over the smaller plate target.
- Pick-only tasks failed in all 5 attempts: the grippers reach and touch both fruits but never lift
  them. More finger force and pad friction did not change this.
- Earlier contact counts of 0 were a probe bug, not evidence about grasping; see the pick section.
- Two-object tasks depend on the external Dream API being up; one trial errored for that reason.
- Trials run at $3/h are expensive; the ledger, not wall-clock estimates, is authoritative.

## Upload note

Large binaries were pushed through the GitHub Contents API. The 50 MB two-object video was
re-encoded (1280 px, CRF 23, 6 MB) because the original exceeded the API request limit; see
`UPLOAD_NOTES.md`.
