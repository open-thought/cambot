#!/bin/bash
# Scan and set Feetech STS3215 servo IDs
# Options: --scan --set-id N --from-id N
exec uv run python -m cambot.tools.fix_servo_ids "$@"
