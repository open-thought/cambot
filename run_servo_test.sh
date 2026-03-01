#!/bin/bash
# Interactive curses TUI for testing individual servos
exec uv run python -m cambot.tools.servo_test "$@"
