#!/bin/bash
# Start the VR teleop server
# Options: --no-robot --no-camera --no-zed
exec uv run python -m cambot.teleop "$@"
