# Dream-1 + Isaac Sim (dual Franka), budget-safe

Two processes, because Isaac Sim is Python 3.11 and Dream's IK libs live in ~/dream-env (3.12):
  Terminal 1 (dream-env):  dream_bridge.py   -> Dream API + Dream's IK, hard $ cap in budget.py
  Terminal 2 (Isaac):      run_dream1_isaac.py -> scene, 3 cameras, applies joint targets

ISAAC=~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64

## Step 1 (free): check camera views
    $ISAAC/python.sh run_dream1_isaac.py --dump-frames --headless      # look at frame_*.png

## Step 2 (free): whole loop with a do-nothing policy
    T1: cd ~/dream1_isaac && source ~/dream-env/bin/activate && python dream_bridge.py --mock
    T2: cd ~/dream1_isaac && $ISAAC/python.sh run_dream1_isaac.py --episodes 1 --max-calls-per-ep 2 --headless

## Step 3 (free): make the Franka arm file for Dream's IK
    source ~/dream-env/bin/activate && dreamcli tcp          # local page; Ctrl-C when done
    URDF: $ISAAC/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf
    You confirm base / flange / tcp / approach / jaw in that tool (do not guess them).

## Step 4 (spends credits, capped at $8): real run
    T1: source ~/dream-env/bin/activate
        export DREAM_API_KEY=$(grep '^DREAM_API_KEY=' ~/.dreamlabs/credentials | cut -d= -f2-)   # prints nothing
        cd ~/dream1_isaac && python dream_bridge.py --arm-file <file from step 3> --cap-usd 8
    T2: $ISAAC/python.sh run_dream1_isaac.py --episodes 1 --max-calls-per-ep 5 --headless    # first run: ~$0.007

Ledger: dream_ledger.json. Counts only successful calls (pool_warming refusals are retried and not counted).
Cost: $3/h => $0.00133 per call (48 rows = 1.6 s). 20 calls/episode ~ $0.027. $8 ~ 6000 calls.
