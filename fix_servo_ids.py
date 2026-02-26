#!/usr/bin/env python3
"""Recovery script to fix servo IDs after accidental overwrite.

Since all servos are likely at ID 0, you must connect them ONE AT A TIME:
  1. Disconnect all servos from the daisy chain
  2. Connect only the first servo (shoulder_pan)
  3. Run this script and assign ID 1
  4. Disconnect that servo, connect the next one
  5. Repeat for each servo

The script will:
  - Ping ID 0 to confirm a servo is there
  - Write the new ID
  - Ping the new ID to confirm it worked
"""

import sys
import scservo_sdk as scs

PORT = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 0

ADDR_ID = 5        # 1 byte, EPROM
ADDR_LOCK = 55     # 1 byte, SRAM (0=unlock EPROM, 1=lock)
ADDR_TORQUE = 40   # 1 byte

MOTOR_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def main():
    port_handler = scs.PortHandler(PORT)
    packet_handler = scs.PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"Failed to open {PORT}")
        sys.exit(1)
    port_handler.setBaudRate(BAUDRATE)

    # First scan to see what IDs respond
    print("Scanning for servos at IDs 0-6...")
    found = []
    for scan_id in range(7):
        _, result, _ = packet_handler.ping(port_handler, scan_id)
        if result == scs.COMM_SUCCESS:
            found.append(scan_id)
            print(f"  Found servo at ID {scan_id}")

    if not found:
        print("\nNo servos found! Check wiring and power.")
        port_handler.closePort()
        sys.exit(1)

    print(f"\nServos found at IDs: {found}")

    if 0 not in found:
        print("No servo at ID 0 — nothing to fix, or already recovered.")
        print("If all IDs 1-6 respond, you're good!")
        port_handler.closePort()
        return

    # Check how many servos respond at ID 0
    # (We can't really count, but warn the user)
    print("\n*** IMPORTANT ***")
    print("If multiple servos are at ID 0, you MUST connect only ONE at a time.")
    print("Disconnect all others from the daisy chain first.\n")

    print("Motor assignments:")
    for mid, name in MOTOR_NAMES.items():
        print(f"  ID {mid}: {name}")

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

        # Confirm servo at ID 0
        _, result, _ = packet_handler.ping(port_handler, 0)
        if result != scs.COMM_SUCCESS:
            print("No servo at ID 0. Connect a servo and try again.")
            continue

        print(f"Assigning ID 0 -> {new_id} ({MOTOR_NAMES[new_id]})...")

        # Disable torque
        packet_handler.write1ByteTxRx(port_handler, 0, ADDR_TORQUE, 0)
        # Unlock EPROM
        packet_handler.write1ByteTxRx(port_handler, 0, ADDR_LOCK, 0)
        # Write new ID
        result, error = packet_handler.write1ByteTxRx(port_handler, 0, ADDR_ID, new_id)
        if result != scs.COMM_SUCCESS:
            print(f"  Write failed: {packet_handler.getTxRxResult(result)}")
            continue

        # Lock EPROM
        packet_handler.write1ByteTxRx(port_handler, new_id, ADDR_LOCK, 1)

        # Verify
        _, result, _ = packet_handler.ping(port_handler, new_id)
        if result == scs.COMM_SUCCESS:
            print(f"  SUCCESS! Servo now responds at ID {new_id}")
        else:
            print(f"  WARNING: Servo not responding at ID {new_id}, but write seemed OK.")
            print(f"  Try power-cycling the servo.")

        # Check ID 0 is gone
        _, result, _ = packet_handler.ping(port_handler, 0)
        if result == scs.COMM_SUCCESS:
            print("  (Another servo still at ID 0 — more to fix)")

    port_handler.closePort()
    print("Done.")


if __name__ == "__main__":
    main()
