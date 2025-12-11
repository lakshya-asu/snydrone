#!/usr/bin/env bash
set -e

echo "[sim_bridge] Placeholder Isaac Sim + Pegasus launcher."
echo "Edit sim_bridge/scripts/start_isaac_pegasus.sh with your Isaac Sim path and Pegasus entrypoint."

# Example sketch (you will customise this on Linux):
#
# ISAAC_ROOT=~/isaac-sim
# PEGASUS_WS=~/pegasus_ws
#
# # Activate Isaac Sim environment (example, adjust for your install)
# source "${ISAAC_ROOT}/setup_conda_env.sh"
#
# # Launch Isaac Sim with Pegasus extension enabled
# "${ISAAC_ROOT}/python.sh" \
#   "${PEGASUS_WS}/pegasus_simulation/pegasus_simulation/scripts/pegasus_isaac.py"
#
# Or if Pegasus ships an 'isaac-run' helper:
# isaac-run pegasus_simulation/pegasus_simulation/scripts/pegasus_isaac.py
