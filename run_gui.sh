#!/usr/bin/env bash
set -e

# Resolve script root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${BUILD_WORKING_DIRECTORY:-$SCRIPT_DIR}"

cd "$WORKSPACE_ROOT"
exec python3 run_interactive_sim.py "$@"
