#!/usr/bin/env python3
"""Read and display all registers from Feetech STS3215 servos.

Also supports writing individual registers:
  python -m cambot.tools.read_params --write MOTOR_ID ADDR VALUE [--size 1|2] [--eprom]

Examples:
  # Read all registers for all motors
  python -m cambot.tools.read_params

  # Read registers for specific motors
  python -m cambot.tools.read_params --motors 1 2

  # Write Overload_Torque (EPROM addr 36) to 100 for motor 1
  python -m cambot.tools.read_params --write 1 36 100 --eprom

  # Write Overload_Torque=100 for all motors 1-5
  python -m cambot.tools.read_params --write 1,2,3,4,5 36 100 --eprom

  # Write SRAM Torque_Limit (addr 48) to 900 for motor 1 (2 bytes)
  python -m cambot.tools.read_params --write 1 48 900 --size 2
"""

import argparse
import sys
import time

import scservo_sdk as scs

from cambot.servo import (
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    MOTOR_NAMES,
    ADDR_TORQUE_ENABLE,
    ADDR_LOCK,
    ADDR_ID,
    EPROM_REGISTERS,
    SRAM_REGISTERS,
    FACTORY_REGISTERS,
    REGISTER_BY_ADDR,
    EPROM_ADDRS,
    decode_sm,
    flush_serial,
    connect,
)

# Map string decode keys from constants to actual functions
_DECODE_FUNCS = {
    "sm15": lambda v: decode_sm(v, 15),
    "sm11": lambda v: decode_sm(v, 11),
    "sm10": lambda v: decode_sm(v, 10),
}


def _resolve_decode(key):
    """Convert a decode key string to a callable, or return None."""
    if key is None:
        return None
    return _DECODE_FUNCS.get(key)


def parse_motor_ids(s):
    """Parse '1,2,3' or '1-5' or 'all' into a list of motor IDs."""
    if s.lower() == "all":
        return sorted(MOTOR_NAMES)
    ids = []
    for part in s.split(","):
        if "-" in part:
            lo, hi = part.split("-", 1)
            ids.extend(range(int(lo), int(hi) + 1))
        else:
            ids.append(int(part))
    return ids


def cmd_write(args):
    """Write a value to a register on one or more motors."""
    ph, pkt = connect(args.port, args.baud)

    motor_ids = parse_motor_ids(args.write[0])
    addr = int(args.write[1])
    value = int(args.write[2])

    # Determine register size
    if args.size:
        size = args.size
    elif addr in REGISTER_BY_ADDR:
        _, size = REGISTER_BY_ADDR[addr]
    else:
        size = 1

    # Determine if EPROM
    is_eprom = args.eprom or addr in EPROM_ADDRS

    reg_name = REGISTER_BY_ADDR.get(addr, (f"addr_{addr}", size))[0]

    print(f"Writing {reg_name} (addr {addr}, {size} byte{'s' if size > 1 else ''}"
          f"{', EPROM' if is_eprom else ''}) = {value}")

    # Safety check for ID register
    if addr == ADDR_ID:
        print("ERROR: Use fix_servo_ids to change motor IDs, not this tool.")
        ph.closePort()
        sys.exit(1)

    for mid in motor_ids:
        # Ping first
        flush_serial(ph)
        _, result, _ = pkt.ping(ph, mid)
        if result != scs.COMM_SUCCESS:
            print(f"  Motor {mid}: NOT RESPONDING, skipping")
            continue

        # Read current value
        flush_serial(ph)
        if size == 1:
            old_val, result, _ = pkt.read1ByteTxRx(ph, mid, addr)
        else:
            old_val, result, _ = pkt.read2ByteTxRx(ph, mid, addr)
        if result != scs.COMM_SUCCESS:
            old_str = "?"
        else:
            old_str = str(old_val)

        if is_eprom:
            # Disable torque and unlock EPROM
            flush_serial(ph)
            pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.05)
            flush_serial(ph)
            pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 0)
            time.sleep(0.05)

        # Write
        flush_serial(ph)
        if size == 1:
            result, _ = pkt.write1ByteTxRx(ph, mid, addr, value)
        else:
            result, _ = pkt.write2ByteTxRx(ph, mid, addr, value)
        time.sleep(0.05)

        if result != scs.COMM_SUCCESS:
            print(f"  Motor {mid}: WRITE FAILED ({pkt.getTxRxResult(result)})")
            continue

        if is_eprom:
            # Re-lock EPROM
            flush_serial(ph)
            pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 1)
            time.sleep(0.05)

        # Read back to verify
        flush_serial(ph)
        if size == 1:
            new_val, result, _ = pkt.read1ByteTxRx(ph, mid, addr)
        else:
            new_val, result, _ = pkt.read2ByteTxRx(ph, mid, addr)

        if result == scs.COMM_SUCCESS and new_val == value:
            name = MOTOR_NAMES.get(mid, "")
            print(f"  Motor {mid}{' ('+name+')' if name else ''}: {old_str} -> {new_val}  OK")
        elif result == scs.COMM_SUCCESS:
            print(f"  Motor {mid}: wrote {value} but read back {new_val}  MISMATCH")
        else:
            print(f"  Motor {mid}: wrote {value}, read-back failed")

    ph.closePort()


def cmd_read(args):
    """Read and display all registers."""
    ph, pkt = connect(args.port, args.baud)

    if args.motors:
        motor_ids = []
        for m in args.motors:
            motor_ids.extend(parse_motor_ids(m))
    else:
        motor_ids = sorted(MOTOR_NAMES)

    for mid in motor_ids:
        name = MOTOR_NAMES.get(mid, f"unknown_{mid}")
        print(f"\n{'='*70}")
        print(f"  Motor {mid}: {name}")
        print(f"{'='*70}")

        # Ping
        _, result, _ = pkt.ping(ph, mid)
        if result != scs.COMM_SUCCESS:
            print(f"  *** MOTOR NOT RESPONDING ***")
            continue

        for section_name, registers in [
            ("EPROM", EPROM_REGISTERS),
            ("SRAM", SRAM_REGISTERS),
            ("FACTORY", FACTORY_REGISTERS),
        ]:
            print(f"\n  --- {section_name} ---")
            for reg_name, addr, size, desc, decode_key in registers:
                if size == 1:
                    raw, result, error = pkt.read1ByteTxRx(ph, mid, addr)
                else:
                    raw, result, error = pkt.read2ByteTxRx(ph, mid, addr)

                if result != scs.COMM_SUCCESS:
                    print(f"  {reg_name:<30} addr={addr:<3} READ ERROR")
                    continue

                decode_fn = _resolve_decode(decode_key)
                decoded = decode_fn(raw) if decode_fn else raw
                if decode_fn:
                    print(f"  {reg_name:<30} addr={addr:<3} raw=0x{raw:04x} ({raw:>5})  decoded={decoded}")
                else:
                    print(f"  {reg_name:<30} addr={addr:<3} = {raw:>5}  (0x{raw:04x})  {desc}")

    # Summary table
    print(f"\n{'='*70}")
    print("  SUMMARY - Key Parameters")
    print(f"{'='*70}")
    header = f"  {'Parameter':<25}" + "".join(f"{'M'+str(m)+' '+MOTOR_NAMES.get(m,'?')[:6]:>12}" for m in motor_ids)
    print(header)
    print("  " + "-" * (25 + 12 * len(motor_ids)))

    key_params = [
        ("Max_Torque_Limit", 16, 2, None),
        ("P_Coefficient", 21, 1, None),
        ("D_Coefficient", 22, 1, None),
        ("I_Coefficient", 23, 1, None),
        ("Protection_Current", 28, 2, None),
        ("Protective_Torque", 34, 1, None),
        ("Protection_Time", 35, 1, None),
        ("Overload_Torque", 36, 1, None),
        ("Operating_Mode", 33, 1, None),
        ("Torque_Enable", 40, 1, None),
        ("Acceleration", 41, 1, None),
        ("Torque_Limit", 48, 2, None),
        ("Lock", 55, 1, None),
        ("Present_Load", 60, 2, "sm10"),
        ("Present_Voltage", 62, 1, None),
        ("Present_Temperature", 63, 1, None),
        ("Present_Current", 69, 2, None),
        ("Max_Acceleration", 85, 1, None),
    ]

    for param_name, addr, size, decode_key in key_params:
        row = f"  {param_name:<25}"
        decode_fn = _resolve_decode(decode_key)
        for mid in motor_ids:
            if size == 1:
                raw, result, _ = pkt.read1ByteTxRx(ph, mid, addr)
            else:
                raw, result, _ = pkt.read2ByteTxRx(ph, mid, addr)
            if result != scs.COMM_SUCCESS:
                row += f"{'ERR':>12}"
            else:
                val = decode_fn(raw) if decode_fn else raw
                row += f"{val:>12}"
        print(row)

    ph.closePort()


def main():
    parser = argparse.ArgumentParser(
        description="Read/write Feetech STS3215 servo registers.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  %(prog)s                                   Read all motors
  %(prog)s --motors 1 2                      Read motors 1 and 2 only
  %(prog)s --write 1 36 100 --eprom          Set Overload_Torque=100 on motor 1
  %(prog)s --write 1-5 36 100 --eprom        Set Overload_Torque=100 on motors 1-5
  %(prog)s --write all 36 100 --eprom        Set Overload_Torque=100 on all motors
  %(prog)s --write 1 48 900 --size 2         Set SRAM Torque_Limit=900 on motor 1
""",
    )
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE,
                        help=f"baud rate (default: {DEFAULT_BAUDRATE})")
    parser.add_argument("--motors", nargs="+",
                        help="motor IDs to read (default: all 1-6)")
    parser.add_argument("--write", nargs=3, metavar=("MOTORS", "ADDR", "VALUE"),
                        help="write VALUE to register ADDR on MOTORS (e.g. '1,2,3' or '1-5' or 'all')")
    parser.add_argument("--size", type=int, choices=[1, 2],
                        help="register size in bytes (auto-detected if known)")
    parser.add_argument("--eprom", action="store_true",
                        help="force EPROM unlock/lock sequence (auto-detected for known registers)")

    args = parser.parse_args()

    if args.write:
        cmd_write(args)
    else:
        cmd_read(args)


if __name__ == "__main__":
    main()
