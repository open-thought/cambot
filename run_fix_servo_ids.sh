#!/bin/bash
# Scan and set Feetech STS3215 servo IDs
# Options: --scan --set-id N --from-id N
exec uv run cambot-fix-servo-ids "$@"
