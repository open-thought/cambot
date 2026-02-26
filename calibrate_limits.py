#!/usr/bin/env python3
"""Joint limit calibration for the StereoBot (passive / read-only).

The script never drives the motors. You move everything by hand.

Workflow:
  1. Pings all 6 motors, ensures torque is disabled
  2. Asks you to manually move the robot to the zero/home pose and press Enter
  3. Records the current positions as the zero reference
  4. You move each joint to its limits while the script tracks min/max
  5. Press Enter to finish — prints limits in encoder steps, degrees, and radians

Usage:
  ./calibrate_limits.py
  ./calibrate_limits.py --port /dev/ttyACM0
"""

import math
import sys
import time
import select

import scservo_sdk as scs

# --- Configuration ---
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1_000_000
PROTOCOL_VERSION = 0

MOTOR_NAMES = {
    1: "base_yaw",
    2: "shoulder_pitch",
    3: "elbow_pitch",
    4: "wrist_pitch",
    5: "wrist_yaw",
    6: "camera_yaw",
}
MOTOR_IDS = list(MOTOR_NAMES.keys())

# Register addresses
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56

# Conversion: 4096 encoder steps = 360 degrees = 2*pi radians
STEPS_PER_REV = 4096
STEPS_TO_DEG = 360.0 / STEPS_PER_REV
STEPS_TO_RAD = 2.0 * math.pi / STEPS_PER_REV

# Sign-magnitude bit for position register
POS_SIGN_BIT = 15


def decode_sm(raw, sign_bit):
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if (raw >> sign_bit) & 1 else magnitude


def connect(port, baudrate):
    ph = scs.PortHandler(port)
    pkt = scs.PacketHandler(PROTOCOL_VERSION)
    if not ph.openPort():
        print(f"ERROR: Cannot open {port}")
        sys.exit(1)
    if not ph.setBaudRate(baudrate):
        print(f"ERROR: Cannot set baud rate {baudrate}")
        sys.exit(1)
    return ph, pkt


def read_position(ph, pkt, mid):
    raw, res, _ = pkt.read2ByteTxRx(ph, mid, ADDR_PRESENT_POSITION)
    if res != scs.COMM_SUCCESS:
        return None
    return decode_sm(raw, POS_SIGN_BIT)


def stdin_ready():
    return select.select([sys.stdin], [], [], 0)[0]


def main():
    import argparse
    parser = argparse.ArgumentParser(description="StereoBot joint limit calibration")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    args = parser.parse_args()

    ph, pkt = connect(args.port, args.baudrate)

    # --- Ping all motors ---
    print("Pinging motors...")
    for mid in MOTOR_IDS:
        _, res, _ = pkt.ping(ph, mid)
        if res != scs.COMM_SUCCESS:
            print(f"  ERROR: Motor {mid} ({MOTOR_NAMES[mid]}) not responding!")
            ph.closePort()
            sys.exit(1)
        pos = read_position(ph, pkt, mid)
        print(f"  Motor {mid} ({MOTOR_NAMES[mid]}): position = {pos}")
    print()

    # --- Ensure torque is disabled (read-only mode) ---
    for mid in MOTOR_IDS:
        pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
    print("Torque disabled on all motors.\n")

    # --- Phase 1: User sets the zero/home pose manually ---
    print("Manually move the robot to the ZERO / HOME pose.")
    input("Press ENTER when the robot is in the home position... ")
    print()

    zero_pos = {}
    print("Zero reference recorded:")
    for mid in MOTOR_IDS:
        pos = read_position(ph, pkt, mid)
        if pos is None:
            print(f"  ERROR: Cannot read motor {mid} ({MOTOR_NAMES[mid]})")
            ph.closePort()
            sys.exit(1)
        zero_pos[mid] = pos
        print(f"  Motor {mid} ({MOTOR_NAMES[mid]}): {pos}")
    print()

    # --- Phase 2: Record min/max as user moves joints ---
    print("Now move each joint to its limits. The script records min/max.")
    print("Press ENTER to finish.\n")

    # Unwrapped position tracking: track deltas to handle encoder wrap at 4095/0
    prev_pos = dict(zero_pos)          # last raw reading per motor
    unwrapped = {mid: 0 for mid in MOTOR_IDS}  # accumulated displacement from zero
    min_pos = {mid: 0 for mid in MOTOR_IDS}
    max_pos = {mid: 0 for mid in MOTOR_IDS}

    # Build header
    header = ""
    for mid in MOTOR_IDS:
        name = MOTOR_NAMES[mid][:10]
        header += f" {name:>10}"
    print(f"{'':>12}{header}")

    try:
        while True:
            # Read all positions and update min/max
            cur_line = "  current: "
            min_line = "      min: "
            max_line = "      max: "
            rng_line = "    range: "

            for mid in MOTOR_IDS:
                raw = read_position(ph, pkt, mid)
                if raw is not None:
                    # Unwrap: detect wraps by checking if delta exceeds half a revolution
                    delta = raw - prev_pos[mid]
                    if delta > STEPS_PER_REV // 2:
                        delta -= STEPS_PER_REV
                    elif delta < -STEPS_PER_REV // 2:
                        delta += STEPS_PER_REV
                    unwrapped[mid] += delta
                    prev_pos[mid] = raw

                    if unwrapped[mid] < min_pos[mid]:
                        min_pos[mid] = unwrapped[mid]
                    if unwrapped[mid] > max_pos[mid]:
                        max_pos[mid] = unwrapped[mid]

                cur_line += f" {unwrapped[mid]:>10}" if raw is not None else f" {'ERR':>10}"
                min_line += f" {min_pos[mid]:>10}"
                max_line += f" {max_pos[mid]:>10}"
                rng_line += f" {max_pos[mid] - min_pos[mid]:>10}"

            # Print with carriage return to overwrite (4 lines)
            sys.stdout.write(f"\033[4A\033[J" if True else "")
            print(cur_line)
            print(min_line)
            print(max_line)
            print(rng_line)
            sys.stdout.flush()

            # Check for Enter key
            if stdin_ready():
                line = sys.stdin.readline()
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    print()

    # --- Phase 3: Print results ---
    print("=" * 80)
    print("JOINT LIMIT CALIBRATION RESULTS  (relative to recorded zero pose)")
    print("=" * 80)
    print()
    print(f"{'Joint':<18} {'Zero':>7} {'Min':>7} {'Max':>7}  {'Min (deg)':>10} {'Max (deg)':>10}  {'Min (rad)':>10} {'Max (rad)':>10}")
    print("-" * 88)

    for mid in MOTOR_IDS:
        name = MOTOR_NAMES[mid]
        lo = min_pos[mid]
        hi = max_pos[mid]
        lo_deg = lo * STEPS_TO_DEG
        hi_deg = hi * STEPS_TO_DEG
        lo_rad = lo * STEPS_TO_RAD
        hi_rad = hi * STEPS_TO_RAD
        print(f"{name:<18} {zero_pos[mid]:>7} {lo:>7} {hi:>7}  {lo_deg:>+10.2f} {hi_deg:>+10.2f}  {lo_rad:>+10.4f} {hi_rad:>+10.4f}")

    print()
    print("URDF joint limits (copy-paste):")
    print()
    for mid in MOTOR_IDS:
        name = MOTOR_NAMES[mid]
        lo_rad = min_pos[mid] * STEPS_TO_RAD
        hi_rad = max_pos[mid] * STEPS_TO_RAD
        print(f'  <!-- {name} -->')
        print(f'  <limit lower="{lo_rad:.4f}" upper="{hi_rad:.4f}" effort="1.5" velocity="3.0"/>')

    print()
    ph.closePort()


if __name__ == "__main__":
    # Print 4 blank lines so the cursor-up escape codes work on first iteration
    print("\n\n\n")
    main()
