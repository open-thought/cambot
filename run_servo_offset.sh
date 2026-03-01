#!/bin/bash
# Servo homing offset / zero-point calibration
# Usage: ./run_servo_offset.sh read 1
#        ./run_servo_offset.sh set-home 3
#        ./run_servo_offset.sh clear-offset 3
exec uv run python -m cambot.tools.servo_offset "$@"
