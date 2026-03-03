#!/bin/bash
# PID auto-tuner for CamBot servos
# Usage: ./run_pid_tuning.sh --capture-poses
#        ./run_pid_tuning.sh --joints shoulder_pitch --verbose
#        ./run_pid_tuning.sh --restore-factory
exec uv run cambot-pid-tuning "$@"
