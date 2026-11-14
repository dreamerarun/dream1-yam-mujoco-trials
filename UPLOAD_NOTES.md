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

## Note on re-encoded videos

The GitHub Contents API rejects the request body for files much over ~40 MB, so these
videos are re-encoded copies (1280 px wide, H.264 CRF 23, no audio). The originals remain local.

| File | Original | Uploaded |
|------|----------|----------|
| `put_the_orange_on_the_plate_and_the_apple_into_the_bowl_1_a8.mp4` | 50 MB | 6.0 MB |
| `pick_up_the_banana_and_the_apple_one_in_each_hand_1_pick2arm.mp4` | 68 MB | 17 MB |
| `pick_up_the_banana_then_pick_up_the_apple_1_pick3.mp4` | 49 MB | 5.5 MB |
| `pick_up_the_banana_then_pick_up_the_apple_1_pick4.mp4` | 51 MB | 5.9 MB |

## Results summary

2 of 8 placement tasks succeeded: orange into the bowl, banana into the bowl (300 s).
The remaining single-object plate trials failed on horizontal centring; both
two-object trials failed (one on centring, one on a Dream API timeout).

All 5 pick attempts failed to lift: the grippers reach and contact the correct fruits
(2-3 cm from centre) but never raise them, even at 80 N finger force and 6.0 pad friction.
