#!/usr/bin/env bash
# Runs Isaac Sim's python with a scrubbed environment, so conda / venv / PYTHONPATH / LD_LIBRARY_PATH
# from your shell cannot clash with Isaac's bundled libraries (a common cause of startup segfaults).
# Usage: ./run_isaac_clean.sh run_dream1_isaac.py --dump-frames --headless
ISAAC="${ISAAC:-$HOME/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64}"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LANG="${LANG:-C.UTF-8}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  DISPLAY="${DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
  "$ISAAC/python.sh" "$@"
