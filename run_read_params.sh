#!/bin/bash
# Read/write Feetech STS3215 servo registers
# Options: --motors 1 2 --write MOTORS ADDR VALUE --eprom
exec uv run python -m cambot.tools.read_params "$@"
