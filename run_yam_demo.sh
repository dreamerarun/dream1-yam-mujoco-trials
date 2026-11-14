#!/usr/bin/env bash
# Launch Dream's YAM sim through the $5 budget wrapper, on the NVIDIA GPU, inside dream-env.
source ~/dream-env/bin/activate
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
cd "$(dirname "$0")"
exec python yam_budget_runner.py --cap-usd 5 --minutes "${MINUTES:-55}" "$@"
