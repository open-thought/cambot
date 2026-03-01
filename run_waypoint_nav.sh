#!/bin/bash
# Record-and-replay waypoint navigation
# Usage: ./run_waypoint_nav.sh record [-o FILE]
#        ./run_waypoint_nav.sh replay FILE [--speed 2.0] [--interp smooth] [--loop]
exec uv run python -m cambot.tools.waypoint_nav "$@"
