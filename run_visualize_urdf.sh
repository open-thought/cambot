#!/bin/bash
# URDF visualization with viser (requires: uv pip install viser)
# Options: --live --port /dev/ttyACM0
exec uv run python -m cambot.tools.visualize_urdf "$@"
