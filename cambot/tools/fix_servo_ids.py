#!/usr/bin/env python3
"""Utility to scan and set Feetech STS3215 servo IDs.

Usage examples:
  # Scan for all connected servos (IDs 0-253)
  python -m cambot.tools.fix_servo_ids --scan

  # Scan a smaller range
  python -m cambot.tools.fix_servo_ids --scan --max-id 10

  # Set the ID of a single connected servo (auto-detects current ID)
  python -m cambot.tools.fix_servo_ids --set-id 3

  # Change a servo from a known ID to a new ID
  python -m cambot.tools.fix_servo_ids --set-id 3 --from-id 1

  # Interactive mode (original behavior: fix servos stuck at ID 0)
  python -m cambot.tools.fix_servo_ids
"""

import argparse
import sys
import time

import scservo_sdk as scs

from cambot.servo import (
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    MOTOR_NAMES,
    ADDR_ID,
    ADDR_TORQUE_ENABLE,
    ADDR_LOCK,
    connect,
)


def scan_servos(port_handler, packet_handler, max_id):
    """Ping IDs 0..max_id and return list of responding IDs."""
    found = []
    for scan_id in range(max_id + 1):
        _, result, _ = packet_handler.ping(port_handler, scan_id)
        if result == scs.COMM_SUCCESS:
            found.append(scan_id)
    return found


def change_id(port_handler, packet_handler, old_id, new_id):
    """Change a servo's ID from old_id to new_id. Returns True on success."""
    # Confirm servo at old_id
    _, result, _ = packet_handler.ping(port_handler, old_id)
    if result != scs.COMM_SUCCESS:
        print(f"No servo responding at ID {old_id}.")
        return False

    # Disable torque
    packet_handler.write1ByteTxRx(port_handler, old_id, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.01)
    # Unlock EPROM
    packet_handler.write1ByteTxRx(port_handler, old_id, ADDR_LOCK, 0)
    time.sleep(0.01)
    # Write new ID (TX-only: servo changes ID immediately so it won't
    # respond on old_id anymore)
    result = packet_handler.write1ByteTxOnly(port_handler, old_id, ADDR_ID, new_id)
    time.sleep(0.01)
    if result != scs.COMM_SUCCESS:
        print(f"  Write failed: {packet_handler.getTxRxResult(result)}")
        return False

    # Lock EPROM (address the servo at its NEW id now)
    packet_handler.write1ByteTxRx(port_handler, new_id, ADDR_LOCK, 1)
    time.sleep(0.01)

    # Verify
    _, result, _ = packet_handler.ping(port_handler, new_id)
    if result == scs.COMM_SUCCESS:
        print(f"  SUCCESS! Servo now responds at ID {new_id}")
        return True
    else:
        print(f"  WARNING: Servo not responding at ID {new_id}, but write seemed OK.")
        print(f"  Try power-cycling the servo.")
        return False


def cmd_scan(args):
    """Scan for all connected servos and print results."""
    port_handler, packet_handler = connect(args.port, args.baud)
    print(f"Scanning for servos at IDs 0-{args.max_id} on {args.port} @ {args.baud}...")
    found = scan_servos(port_handler, packet_handler, args.max_id)
    port_handler.closePort()

    if not found:
        print("No servos found. Check wiring and power.")
        sys.exit(1)

    print(f"\nFound {len(found)} servo(s):")
    for sid in found:
        name = MOTOR_NAMES.get(sid, "")
        label = f"  ID {sid}"
        if name:
            label += f"  ({name})"
        print(label)


def cmd_set_id(args):
    """Set the ID of a connected servo."""
    new_id = args.set_id
    if new_id < 0 or new_id > 253:
        print("New ID must be 0-253.")
        sys.exit(1)

    port_handler, packet_handler = connect(args.port, args.baud)

    if args.from_id is not None:
        # User specified the current ID explicitly
        old_id = args.from_id
        print(f"Changing servo ID {old_id} -> {new_id} ...")
    else:
        # Auto-detect: scan and expect exactly one servo
        print(f"Scanning for a single connected servo (IDs 0-{args.max_id})...")
        found = scan_servos(port_handler, packet_handler, args.max_id)

        if len(found) == 0:
            print("No servos found. Check wiring and power.")
            port_handler.closePort()
            sys.exit(1)
        elif len(found) > 1:
            print(f"Multiple servos found at IDs: {found}")
            print("Connect only ONE servo, or use --from-id to specify which one to change.")
            port_handler.closePort()
            sys.exit(1)

        old_id = found[0]
        if old_id == new_id:
            print(f"Servo already at ID {new_id}, nothing to do.")
            port_handler.closePort()
            return
        print(f"Found servo at ID {old_id}. Changing ID {old_id} -> {new_id} ...")

    name = MOTOR_NAMES.get(new_id, "")
    if name:
        print(f"  (ID {new_id} = {name})")

    change_id(port_handler, packet_handler, old_id, new_id)
    port_handler.closePort()


def cmd_interactive(args):
    """Original interactive mode: fix servos stuck at ID 0."""
    port_handler, packet_handler = connect(args.port, args.baud)

    print("Scanning for servos at IDs 0-6...")
    found = scan_servos(port_handler, packet_handler, 6)

    if not found:
        print("\nNo servos found! Check wiring and power.")
        port_handler.closePort()
        sys.exit(1)

    print(f"\nServos found at IDs: {found}")

    if 0 not in found:
        print("No servo at ID 0 -- nothing to fix, or already recovered.")
        print("If all IDs 1-6 respond, you're good!")
        port_handler.closePort()
        return

    print("\n*** IMPORTANT ***")
    print("If multiple servos are at ID 0, you MUST connect only ONE at a time.")
    print("Disconnect all others from the daisy chain first.\n")

    print("Motor assignments:")
    for mid in sorted(MOTOR_NAMES):
        print(f"  ID {mid}: {MOTOR_NAMES[mid]}")

    while True:
        new_id_str = input("\nEnter new ID to assign (1-6), or 'q' to quit: ").strip()
        if new_id_str.lower() == 'q':
            break

        try:
            new_id = int(new_id_str)
        except ValueError:
            print("Invalid input.")
            continue

        if new_id < 1 or new_id > 6:
            print("ID must be 1-6.")
            continue

        print(f"Assigning ID 0 -> {new_id} ({MOTOR_NAMES[new_id]})...")
        change_id(port_handler, packet_handler, 0, new_id)

        # Check if another servo still at ID 0
        _, result, _ = packet_handler.ping(port_handler, 0)
        if result == scs.COMM_SUCCESS:
            print("  (Another servo still at ID 0 -- more to fix)")

    port_handler.closePort()
    print("Done.")


def main():
    parser = argparse.ArgumentParser(
        description="Scan and set Feetech STS3215 servo IDs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  %(prog)s --scan                  Scan for all connected servos
  %(prog)s --scan --max-id 10      Scan IDs 0-10 only
  %(prog)s --set-id 3              Set single connected servo to ID 3
  %(prog)s --set-id 3 --from-id 1  Change servo from ID 1 to ID 3
  %(prog)s                         Interactive mode (fix ID 0 servos)
""",
    )
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE,
                        help=f"baud rate (default: {DEFAULT_BAUDRATE})")
    parser.add_argument("--max-id", type=int, default=253,
                        help="max ID to scan (default: 253, use lower for speed)")
    parser.add_argument("--scan", action="store_true",
                        help="scan for connected servos and print their IDs")
    parser.add_argument("--set-id", type=int, metavar="NEW_ID",
                        help="set the servo to this ID (0-253)")
    parser.add_argument("--from-id", type=int, metavar="OLD_ID",
                        help="current ID of the servo to change (used with --set-id)")

    args = parser.parse_args()

    if args.from_id is not None and args.set_id is None:
        parser.error("--from-id requires --set-id")

    if args.scan and args.set_id is not None:
        parser.error("--scan and --set-id are mutually exclusive")

    if args.scan:
        cmd_scan(args)
    elif args.set_id is not None:
        cmd_set_id(args)
    else:
        cmd_interactive(args)


if __name__ == "__main__":
    main()
