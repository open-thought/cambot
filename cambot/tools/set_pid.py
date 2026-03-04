#!/usr/bin/env python3
"""Write tested PID parameters to all CamBot servos.

Writes recommended P/D/I coefficients (EPROM registers 21-23) to each motor,
with read-back verification.

Usage:
  python -m cambot.tools.set_pid            # write PID values to all motors
  python -m cambot.tools.set_pid --dry-run  # preview without writing
"""

import argparse
import sys
import time

import scservo_sdk as scs

from cambot.servo import (
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    JOINT_NAMES,
    MOTOR_IDS,
    MOTOR_NAMES,
    ADDR_P_COEFFICIENT,
    ADDR_D_COEFFICIENT,
    ADDR_I_COEFFICIENT,
    ADDR_MAX_TORQUE_LIMIT,
    ADDR_OVERLOAD_TORQUE,
    connect,
    flush_serial,
    unlock_eprom,
    lock_eprom,
)

# Tested PID values (P, D, I) per joint.
# C001 (1:345 gear ratio) joints need different tuning than C046 (1:147).
RECOMMENDED_PID = {
    "base_yaw":       (24, 48, 0),
    "shoulder_pitch":  (32, 16, 0),
    "elbow_pitch":     (64, 32, 0),
    "wrist_pitch":     (48, 32, 0),
    "wrist_yaw":       (32, 32, 0),
    "camera_roll":     (25, 32, 0),
}

PID_ADDRS = [
    ("P", ADDR_P_COEFFICIENT),
    ("D", ADDR_D_COEFFICIENT),
    ("I", ADDR_I_COEFFICIENT),
]

# Overload_Torque (EPROM addr 36) overrides per joint.
# Factory default is 25%, which is too low for load-bearing joints — the servo
# trips overload protection during normal operation under gravity.
# Value 95 = allow up to 95% of max torque before triggering protection.
RECOMMENDED_OVERLOAD_TORQUE = {
    "base_yaw": 95,
    "shoulder_pitch": 95,
    "elbow_pitch": 95,
    "wrist_pitch": 95,
    "wrist_yaw": 95,
    # camera_roll: left at factory default (25) — low torque joint
}

# Max_Torque_Limit (EPROM addr 16) overrides per joint.
# Factory default is 1000 (100%). Camera roll is capped for safety.
RECOMMENDED_MAX_TORQUE = {
    "camera_roll": 500,  # 50% — no need for full torque on camera rotation
}


def read_pid(pkt, ph, mid):
    """Read current P, D, I values from a motor. Returns (P, D, I) or None."""
    values = []
    for _, addr in PID_ADDRS:
        flush_serial(ph)
        val, result, _ = pkt.read1ByteTxRx(ph, mid, addr)
        if result != scs.COMM_SUCCESS:
            return None
        values.append(val)
    return tuple(values)


def write_pid(pkt, ph, mid, p, d, i, overload_torque=None, max_torque=None):
    """Write PID and torque parameters to a motor (EPROM).

    Returns True on success.
    """
    unlock_eprom(ph, pkt, mid)

    for val, (_, addr) in zip((p, d, i), PID_ADDRS):
        flush_serial(ph)
        result, _ = pkt.write1ByteTxRx(ph, mid, addr, val)
        if result != scs.COMM_SUCCESS:
            lock_eprom(ph, pkt, mid)
            return False
        time.sleep(0.01)

    if overload_torque is not None:
        flush_serial(ph)
        result, _ = pkt.write1ByteTxRx(ph, mid, ADDR_OVERLOAD_TORQUE, overload_torque)
        if result != scs.COMM_SUCCESS:
            lock_eprom(ph, pkt, mid)
            return False
        time.sleep(0.01)

    if max_torque is not None:
        flush_serial(ph)
        result, _ = pkt.write2ByteTxRx(ph, mid, ADDR_MAX_TORQUE_LIMIT, max_torque)
        if result != scs.COMM_SUCCESS:
            lock_eprom(ph, pkt, mid)
            return False
        time.sleep(0.01)

    lock_eprom(ph, pkt, mid)
    time.sleep(0.05)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Write tested PID parameters to CamBot servos.",
    )
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE,
                        help=f"baud rate (default: {DEFAULT_BAUDRATE})")
    parser.add_argument("--dry-run", action="store_true",
                        help="read and display current PID values without writing")
    args = parser.parse_args()

    ph, pkt = connect(args.port, args.baud)

    # Header
    if args.dry_run:
        print("DRY RUN — showing current vs recommended PID values\n")
    else:
        print("Writing recommended PID values to all motors\n")

    print(f"  {'Joint':<20} {'Old P':>5} {'Old D':>5} {'Old I':>5}"
          f"  {'New P':>5} {'New D':>5} {'New I':>5}  {'Status'}")
    print(f"  {'-'*20} {'-'*5} {'-'*5} {'-'*5}  {'-'*5} {'-'*5} {'-'*5}  {'-'*10}")

    errors = 0
    for name in JOINT_NAMES:
        mid = MOTOR_IDS[name]
        new_p, new_d, new_i = RECOMMENDED_PID[name]
        new_overload = RECOMMENDED_OVERLOAD_TORQUE.get(name)
        new_max_torque = RECOMMENDED_MAX_TORQUE.get(name)

        # Ping
        flush_serial(ph)
        _, result, _ = pkt.ping(ph, mid)
        if result != scs.COMM_SUCCESS:
            print(f"  {name:<20}  {'--- motor not responding ---':>40}  SKIP")
            errors += 1
            continue

        # Read current values
        old = read_pid(pkt, ph, mid)
        if old is None:
            old_str = "  ERR   ERR   ERR"
            errors += 1
        else:
            old_str = f"{old[0]:>5} {old[1]:>5} {old[2]:>5}"

        if args.dry_run:
            extras = []
            if new_overload is not None:
                extras.append(f"Overload_Torque->{new_overload}")
            if new_max_torque is not None:
                extras.append(f"Max_Torque->{new_max_torque}")
            extra = f"  ({', '.join(extras)})" if extras else ""
            print(f"  {name:<20} {old_str}  {new_p:>5} {new_d:>5} {new_i:>5}  --{extra}")
            continue

        # Write new values
        ok = write_pid(pkt, ph, mid, new_p, new_d, new_i, new_overload,
                       new_max_torque)
        if not ok:
            print(f"  {name:<20} {old_str}  {new_p:>5} {new_d:>5} {new_i:>5}  WRITE FAIL")
            errors += 1
            continue

        # Read back to verify PID
        verify = read_pid(pkt, ph, mid)
        if verify == (new_p, new_d, new_i):
            status = "OK"
        else:
            status = f"MISMATCH {verify}"
            errors += 1

        if new_overload is not None:
            flush_serial(ph)
            val, result, _ = pkt.read1ByteTxRx(ph, mid, ADDR_OVERLOAD_TORQUE)
            if result == scs.COMM_SUCCESS and val == new_overload:
                status += f" +OvlTq={new_overload}"
            else:
                status += f" +OvlTq MISMATCH({val})"
                errors += 1

        if new_max_torque is not None:
            flush_serial(ph)
            val, result, _ = pkt.read2ByteTxRx(ph, mid, ADDR_MAX_TORQUE_LIMIT)
            if result == scs.COMM_SUCCESS and val == new_max_torque:
                status += f" +MaxTq={new_max_torque}"
            else:
                status += f" +MaxTq MISMATCH({val})"
                errors += 1

        print(f"  {name:<20} {old_str}  {new_p:>5} {new_d:>5} {new_i:>5}  {status}")

    ph.closePort()

    if not args.dry_run:
        print(f"\nDone. {6 - errors}/6 motors updated successfully.")
    if errors:
        sys.exit(1)


if __name__ == "__main__":
    main()
