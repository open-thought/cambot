#!/bin/bash
# PID auto-tuner for StereoBot servos
# Usage: ./run_pid_tuning.sh --capture-poses
#        ./run_pid_tuning.sh --joints shoulder_pitch --verbose
#        ./run_pid_tuning.sh --restore-factory
exec uv run python -m cambot.tools.pid_tuning "$@"
