#!/bin/bash
# Joint limit calibration (passive / read-only)
exec uv run python -m cambot.tools.calibrate_limits "$@"
