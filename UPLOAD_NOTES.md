# Repository upload notes

Artefacts from the Dream-1 YAM MuJoCo bimanual trials, uploaded via the GitHub Contents API.

## Layout

- `tasks_report.txt` — verdicts and measured poses for all 8 tasks
- `yam_trials.py` — trial driver (threaded GUI recorder, `grasp_boost`, `grasp_probe`)
- `yam_budget_runner.py`, `budget.py` — hard budget enforcement (ledger)
- `yam_ledger.json` — spend ledger ($3.628 of $5.00)
- `yam_check/trials/` — before/after top-camera frames, contact sheets (`*_proof.png`), GUI screenshots
- `yam_check/frames/` — reference camera frames (left/right/top)
- `yam_check/video/` — one GUI screen recording per trial (x11grab)

## Note on one video

`yam_check/video/put_the_orange_on_the_plate_and_the_apple_into_the_bowl_1_a8.mp4`
is a re-encoded copy: the original 50 MB file exceeded the GitHub Contents API
request-size limit. The copy is 1280 px wide, H.264 CRF 23, no audio (6.0 MB).
The original remains local.

## Results summary

2 of 8 tasks succeeded: orange into the bowl, banana into the bowl (300 s).
The remaining single-object plate trials failed on horizontal centring; both
two-object trials failed (one on centring, one on a Dream API timeout).
