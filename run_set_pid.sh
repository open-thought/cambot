#!/bin/bash
# Write tested PID parameters to all CamBot servos
# Usage: ./run_set_pid.sh
#        ./run_set_pid.sh --dry-run
exec uv run cambot-set-pid "$@"
