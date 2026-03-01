#!/usr/bin/env python3
"""Feetech STS3215 servo offset / zero-point calibration tool.

Convention: Reported_Position = Raw_Encoder - Homing_Offset
  -> Raw_Encoder = Reported_Position + Homing_Offset

Commands:
  read          Show current offset, position, and position limits
  set-zero      Set homing offset so current position reads as 0 (no movement)
  set-home      Set homing offset so current position reads as 2048 (no movement)
  set-position  Set homing offset so current position reads as VALUE (no movement)
  clear-offset  Reset limits to 0-4095, clear offset, move motor to 2048
  reset-limits  Reset position limits to 0-4095

Examples:
  python -m cambot.tools.servo_offset read 1
  python -m cambot.tools.servo_offset set-zero 3
  python -m cambot.tools.servo_offset set-home 3
  python -m cambot.tools.servo_offset set-position 3 1000
  python -m cambot.tools.servo_offset clear-offset 3
  python -m cambot.tools.servo_offset reset-limits 4
"""

import argparse
import sys
import time

import scservo_sdk as scs

from cambot.servo import (
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    POS_MIN,
    POS_MAX,
    POS_SIGN_BIT,
    OFFSET_SIGN_BIT,
    OFFSET_MAX,
    ADDR_MIN_POSITION_LIMIT,
    ADDR_MAX_POSITION_LIMIT,
    ADDR_HOMING_OFFSET,
    ADDR_TORQUE_ENABLE,
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_TORQUE_LIMIT,
    ADDR_MOVING,
    ADDR_PRESENT_POSITION,
    decode_sm,
    encode_sm,
    connect,
    flush_serial,
    unlock_eprom,
    lock_eprom,
    write_eprom_u16,
)


# --- Local read helpers ---

def read_offset(ph, pkt, mid):
    """Return (raw, decoded) offset or (None, None) on error."""
    raw, res, _ = pkt.read2ByteTxRx(ph, mid, ADDR_HOMING_OFFSET)
    if res != scs.COMM_SUCCESS:
        return None, None
    return raw, decode_sm(raw, OFFSET_SIGN_BIT)


def read_position(ph, pkt, mid):
    """Return (raw, decoded) position or (None, None) on error."""
    raw, res, _ = pkt.read2ByteTxRx(ph, mid, ADDR_PRESENT_POSITION)
    if res != scs.COMM_SUCCESS:
        return None, None
    return raw, decode_sm(raw, POS_SIGN_BIT)


def read_position_limits(ph, pkt, mid):
    """Return (min_limit, max_limit) or (None, None) on error."""
    lo, res, _ = pkt.read2ByteTxRx(ph, mid, ADDR_MIN_POSITION_LIMIT)
    if res != scs.COMM_SUCCESS:
        return None, None
    hi, res, _ = pkt.read2ByteTxRx(ph, mid, ADDR_MAX_POSITION_LIMIT)
    if res != scs.COMM_SUCCESS:
        return None, None
    return lo, hi


def reset_position_limits(ph, pkt, mid):
    """Reset position limits to full range 0-4095."""
    unlock_eprom(ph, pkt, mid)
    flush_serial(ph)
    pkt.write2ByteTxRx(ph, mid, ADDR_MIN_POSITION_LIMIT, POS_MIN)
    time.sleep(0.05)
    flush_serial(ph)
    pkt.write2ByteTxRx(ph, mid, ADDR_MAX_POSITION_LIMIT, POS_MAX)
    time.sleep(0.05)
    lock_eprom(ph, pkt, mid)
    time.sleep(0.05)
    # Verify by reading back
    flush_serial(ph)
    lo, r1, _ = pkt.read2ByteTxRx(ph, mid, ADDR_MIN_POSITION_LIMIT)
    hi, r2, _ = pkt.read2ByteTxRx(ph, mid, ADDR_MAX_POSITION_LIMIT)
    ok = (r1 == scs.COMM_SUCCESS and r2 == scs.COMM_SUCCESS
          and lo == POS_MIN and hi == POS_MAX)
    if not ok:
        print(f"  DEBUG: limits readback: [{lo}, {hi}] (expected [{POS_MIN}, {POS_MAX}])")
    return ok


def wait_until_stopped(ph, pkt, mid, target, timeout=10.0, tolerance=15):
    """Wait for motor to reach *target* position (decoded)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        _, pos = read_position(ph, pkt, mid)
        if pos is not None and abs(pos - target) < tolerance:
            return True
        moving, res, _ = pkt.read1ByteTxRx(ph, mid, ADDR_MOVING)
        if res == scs.COMM_SUCCESS and moving == 0:
            _, pos = read_position(ph, pkt, mid)
            if pos is not None and abs(pos - target) < tolerance:
                return True
        time.sleep(0.05)
    return False


def print_status(ph, pkt, mid):
    """Print offset, position, limits. Returns (offset, position) decoded."""
    _, offset = read_offset(ph, pkt, mid)
    _, pos = read_position(ph, pkt, mid)
    lim_min, lim_max = read_position_limits(ph, pkt, mid)

    if offset is None or pos is None or lim_min is None:
        print("ERROR: Communication failure")
        return None, None

    raw_encoder = pos + offset
    print(f"Motor {mid}:")
    print(f"  Homing offset:    {offset:+d}")
    print(f"  Reported position:{pos:+d}")
    print(f"  Raw encoder:      {raw_encoder:+d}  (position + offset)")
    print(f"  Position limits:  [{lim_min}, {lim_max}]"
          f"{'  (full range)' if lim_min == POS_MIN and lim_max == POS_MAX else ''}")
    return offset, pos


# --- Commands ---

def cmd_read(ph, pkt, mid):
    """Print current offset, position, and position limits."""
    offset, pos = print_status(ph, pkt, mid)
    return offset is not None


def cmd_set_position(ph, pkt, mid, target_pos, yes=False):
    """Set the homing offset so that the current position reads as target_pos.

    The servo does NOT move.  Formula:
      new_offset = (raw_encoder - target_pos) mod 4096, wrapped to +/-2047
      where raw_encoder = reported_position + old_offset

    Position arithmetic wraps mod 4096, so we pick the smallest-magnitude
    offset that achieves the target.
    """
    offset, pos = print_status(ph, pkt, mid)
    if offset is None:
        return False

    raw_encoder = pos + offset
    naive_offset = raw_encoder - target_pos

    # Wrap to smallest magnitude: try naive, naive+4096, naive-4096
    candidates = [naive_offset, naive_offset + 4096, naive_offset - 4096]
    new_offset = min(candidates, key=abs)

    print(f"\n  Will set position to {target_pos:+d}  (currently {pos:+d})")
    print(f"  New offset: {new_offset:+d}")

    if abs(new_offset) > OFFSET_MAX:
        print(f"\n  ERROR: Required offset {new_offset:+d} exceeds register "
              f"range +/-{OFFSET_MAX}.")
        print(f"  (raw encoder is {raw_encoder}, target is {target_pos})")
        return False

    encoded = encode_sm(new_offset, OFFSET_SIGN_BIT)
    print(f"  Encoded register:  0x{encoded:04X}")

    if not yes:
        resp = input("\nWrite offset to EPROM? [y/N] ")
        if resp.lower() != "y":
            print("Aborted.")
            return False

    if not write_eprom_u16(ph, pkt, mid, ADDR_HOMING_OFFSET, encoded):
        print("ERROR: Failed to write homing offset")
        return False

    time.sleep(0.1)
    print()
    print_status(ph, pkt, mid)
    print("Done.")
    return True


def cmd_reset_limits(ph, pkt, mid, yes=False):
    """Reset position limits to full range 0-4095."""
    offset, pos = print_status(ph, pkt, mid)
    if offset is None:
        return False

    lim_min, lim_max = read_position_limits(ph, pkt, mid)
    if lim_min == POS_MIN and lim_max == POS_MAX:
        print("  Limits are already full range -- nothing to do.")
        return True

    print(f"\n  Will reset limits from [{lim_min}, {lim_max}] to "
          f"[{POS_MIN}, {POS_MAX}]")

    if not yes:
        resp = input("\nProceed? [y/N] ")
        if resp.lower() != "y":
            print("Aborted.")
            return False

    if not reset_position_limits(ph, pkt, mid):
        print("ERROR: Failed to write position limits")
        return False

    time.sleep(0.1)
    lim_min, lim_max = read_position_limits(ph, pkt, mid)
    print(f"\nVerify limits: [{lim_min}, {lim_max}]")
    print("Done.")
    return True


def cmd_clear_offset(ph, pkt, mid, yes=False):
    """Reset position limits to 0-4095, clear the homing offset, then drive
    the motor to position 2048 (mid-range, giving ~180 deg room in each direction).
    After this, reported position ~ 2048 with no offset.
    """
    offset, pos = print_status(ph, pkt, mid)
    if offset is None:
        return False

    lim_min, lim_max = read_position_limits(ph, pkt, mid)
    limits_need_reset = not (lim_min == POS_MIN and lim_max == POS_MAX)
    has_offset = offset != 0

    raw_encoder = pos + offset
    target = POS_MAX // 2  # 2047, effectively 2048 center
    step = 1
    print(f"\n  Plan:")
    if limits_need_reset:
        print(f"    {step}. Reset position limits [{lim_min}, {lim_max}] -> "
              f"[{POS_MIN}, {POS_MAX}]")
        step += 1
    if has_offset:
        print(f"    {step}. Clear offset to 0  "
              f"(position will jump from {pos:+d} to {raw_encoder:+d})")
        step += 1
    print(f"    {step}. Move motor to position {target}  (mid-range)")

    if not yes:
        resp = input("\nProceed? [y/N] ")
        if resp.lower() != "y":
            print("Aborted.")
            return False

    # Step 1: Reset position limits if needed
    if limits_need_reset:
        print(f"  Resetting position limits to [{POS_MIN}, {POS_MAX}]...")
        if not reset_position_limits(ph, pkt, mid):
            print("ERROR: Failed to reset position limits")
            return False
        time.sleep(0.05)

    # Step 2: Clear offset if needed (this disables torque)
    if has_offset:
        print("  Clearing offset to 0...")
        if not write_eprom_u16(ph, pkt, mid, ADDR_HOMING_OFFSET, 0):
            print("ERROR: Failed to clear offset")
            return False
        time.sleep(0.05)

        _, pos_after_clear = read_position(ph, pkt, mid)
        print(f"  Position after clearing offset: {pos_after_clear:+d}")

    # Step 3: Move to mid-range position
    flush_serial(ph)
    pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.02)
    flush_serial(ph)
    pkt.write1ByteTxRx(ph, mid, ADDR_ACCELERATION, 100)
    time.sleep(0.02)
    flush_serial(ph)
    pkt.write2ByteTxRx(ph, mid, ADDR_TORQUE_LIMIT, 800)
    time.sleep(0.02)

    print(f"  Goal position {target}")
    flush_serial(ph)
    res, _ = pkt.write2ByteTxRx(ph, mid, ADDR_GOAL_POSITION, target)
    if res != scs.COMM_SUCCESS:
        print("ERROR: Failed to write goal position")
        return False

    print("  Moving...", end="", flush=True)
    arrived_ok = wait_until_stopped(ph, pkt, mid, target, timeout=10.0)

    _, arrived = read_position(ph, pkt, mid)
    if arrived_ok:
        print(f" arrived at {arrived:+d}." if arrived is not None else " arrived.")
    else:
        pos_str = f"{arrived:+d}" if arrived is not None else "unknown"
        print(f" timeout at {pos_str} (target was {target}).")
        print("  WARNING: Motor did not reach target position.")

    time.sleep(0.1)
    print()
    print_status(ph, pkt, mid)
    print("Done.")
    return arrived_ok


# --- Main ---

def main():
    parser = argparse.ArgumentParser(
        description="Feetech STS3215 homing-offset / zero-point tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "command",
        choices=["read", "set-zero", "set-home", "set-position",
                 "clear-offset", "reset-limits"],
        help="read: show state | set-zero: offset to pos=0 | "
             "set-home: offset to pos=2048 | "
             "set-position: offset to pos=VALUE | "
             "clear-offset: reset limits + clear offset + move to 2048 | "
             "reset-limits: set limits to 0-4095",
    )
    parser.add_argument("motor_id", type=int, help="Motor ID (e.g. 1-6)")
    parser.add_argument("value", type=int, nargs="?", default=None,
                        help="Target position value (only for set-position)")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("-y", "--yes", action="store_true",
                        help="Skip confirmation prompt")
    args = parser.parse_args()

    ph, pkt = connect(args.port, args.baudrate)

    # Ping
    _, res, _ = pkt.ping(ph, args.motor_id)
    if res != scs.COMM_SUCCESS:
        print(f"ERROR: Motor {args.motor_id} not responding on {args.port}")
        sys.exit(1)

    ok = False
    try:
        if args.command == "read":
            ok = cmd_read(ph, pkt, args.motor_id)
        elif args.command == "set-zero":
            ok = cmd_set_position(ph, pkt, args.motor_id, 0, yes=args.yes)
        elif args.command == "set-home":
            ok = cmd_set_position(ph, pkt, args.motor_id, 2048, yes=args.yes)
        elif args.command == "set-position":
            if args.value is None:
                print("ERROR: set-position requires a value argument")
                print("Usage: python -m cambot.tools.servo_offset set-position MOTOR_ID VALUE")
                sys.exit(1)
            ok = cmd_set_position(ph, pkt, args.motor_id, args.value,
                                  yes=args.yes)
        elif args.command == "clear-offset":
            ok = cmd_clear_offset(ph, pkt, args.motor_id, yes=args.yes)
        elif args.command == "reset-limits":
            ok = cmd_reset_limits(ph, pkt, args.motor_id, yes=args.yes)
    finally:
        pkt.write1ByteTxRx(ph, args.motor_id, ADDR_TORQUE_ENABLE, 0)
        ph.closePort()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
