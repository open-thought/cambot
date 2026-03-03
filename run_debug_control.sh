#!/bin/bash
# Debug control TUI: joint mode, Cartesian/IK mode, PID tuning
# Options: --port /dev/ttyACM0 --servo-profile
exec uv run cambot-debug-control "$@"
