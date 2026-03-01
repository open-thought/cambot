#!/usr/bin/env python3
"""Read and display all registers from Feetech STS3215 servos.

Also supports writing individual registers:
  python read_all_params.py --write MOTOR_ID ADDR VALUE [--size 1|2] [--eprom]

Examples:
  # Read all registers for all motors
  python read_all_params.py

  # Read registers for specific motors
  python read_all_params.py --motors 1 2

  # Write Overload_Torque (EPROM addr 36) to 100 for motor 1
  python read_all_params.py --write 1 36 100 --eprom

  # Write Overload_Torque=100 for all motors 1-5
  python read_all_params.py --write 1,2,3,4,5 36 100 --eprom

  # Write SRAM Torque_Limit (addr 48) to 900 for motor 1 (2 bytes)
  python read_all_params.py --write 1 48 900 --size 2
"""

import argparse
import sys
import time

import scservo_sdk as scs

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1_000_000
PROTOCOL_VERSION = 0

ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55


def flush_serial(port_handler):
    """Flush stale data from the serial buffer."""
    if hasattr(port_handler, 'ser') and port_handler.ser is not None:
        port_handler.ser.reset_input_buffer()
    time.sleep(0.02)

MOTOR_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

# Sign-magnitude decoding
def decode_sm(raw, sign_bit):
    direction = (raw >> sign_bit) & 1
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if direction else magnitude

# (name, address, size, description, decode_func or None)
EPROM_REGISTERS = [
    ("Firmware_Major", 0, 1, "Firmware major version", None),
    ("Firmware_Minor", 1, 1, "Firmware minor version", None),
    ("Model_Number", 3, 2, "Model number (777=STS3215)", None),
    ("ID", 5, 1, "Motor ID", None),
    ("Baud_Rate", 6, 1, "Baud rate index (0=1M)", None),
    ("Return_Delay_Time", 7, 1, "Response delay (x2 us)", None),
    ("Response_Status_Level", 8, 1, "Status return level", None),
    ("Min_Position_Limit", 9, 2, "Min position limit", None),
    ("Max_Position_Limit", 11, 2, "Max position limit", None),
    ("Max_Temperature_Limit", 13, 1, "Max temp limit (C)", None),
    ("Max_Voltage_Limit", 14, 1, "Max voltage limit (x0.1V)", None),
    ("Min_Voltage_Limit", 15, 1, "Min voltage limit (x0.1V)", None),
    ("Max_Torque_Limit", 16, 2, "Max torque (0-1000 = 0-100%)", None),
    ("Phase", 18, 1, "Phase", None),
    ("Unloading_Condition", 19, 1, "Unloading condition", None),
    ("LED_Alarm_Condition", 20, 1, "LED alarm condition", None),
    ("P_Coefficient", 21, 1, "PID P gain", None),
    ("D_Coefficient", 22, 1, "PID D gain", None),
    ("I_Coefficient", 23, 1, "PID I gain", None),
    ("Minimum_Startup_Force", 24, 2, "Min startup force", None),
    ("CW_Dead_Zone", 26, 1, "CW dead zone", None),
    ("CCW_Dead_Zone", 27, 1, "CCW dead zone", None),
    ("Protection_Current", 28, 2, "Protection current (mA)", None),
    ("Angular_Resolution", 30, 1, "Angular resolution", None),
    ("Homing_Offset", 31, 2, "Homing offset (sign-mag bit11)", lambda v: decode_sm(v, 11)),
    ("Operating_Mode", 33, 1, "Mode (0=pos, 1=speed, 3=step)", None),
    ("Protective_Torque", 34, 1, "Protective torque (%)", None),
    ("Protection_Time", 35, 1, "Protection time", None),
    ("Overload_Torque", 36, 1, "Overload torque (%)", None),
    ("Vel_P_Coeff", 37, 1, "Velocity P coefficient", None),
    ("Over_Current_Prot_Time", 38, 1, "Over-current protection time", None),
    ("Vel_I_Coeff", 39, 1, "Velocity I coefficient", None),
]

SRAM_REGISTERS = [
    ("Torque_Enable", 40, 1, "Torque enabled", None),
    ("Acceleration", 41, 1, "Acceleration", None),
    ("Goal_Position", 42, 2, "Goal position (sign-mag bit15)", lambda v: decode_sm(v, 15)),
    ("Goal_Time", 44, 2, "Goal time (ms)", None),
    ("Goal_Velocity", 46, 2, "Goal velocity (sign-mag bit15)", lambda v: decode_sm(v, 15)),
    ("Torque_Limit", 48, 2, "Runtime torque limit (0-1000)", None),
    ("Lock", 55, 1, "EPROM lock (1=locked)", None),
    ("Present_Position", 56, 2, "Present position (sign-mag bit15)", lambda v: decode_sm(v, 15)),
    ("Present_Velocity", 58, 2, "Present velocity (sign-mag bit15)", lambda v: decode_sm(v, 15)),
    ("Present_Load", 60, 2, "Present load (sign-mag bit10)", lambda v: decode_sm(v, 10)),
    ("Present_Voltage", 62, 1, "Voltage (x0.1V)", None),
    ("Present_Temperature", 63, 1, "Temperature (C)", None),
    ("Status", 65, 1, "Status flags", None),
    ("Moving", 66, 1, "Is moving", None),
    ("Present_Current", 69, 2, "Current (mA)", None),
]

FACTORY_REGISTERS = [
    ("Moving_Velocity_Threshold", 80, 1, "Moving velocity threshold", None),
    ("DTs", 81, 1, "DTs (ms)", None),
    ("Velocity_Unit_Factor", 82, 1, "Velocity unit factor", None),
    ("Hts", 83, 1, "Hts (ns)", None),
    ("Max_Velocity_Limit", 84, 1, "Max velocity limit", None),
    ("Maximum_Acceleration", 85, 1, "Max acceleration", None),
    ("Acceleration_Multiplier", 86, 1, "Accel multiplier (when accel=0)", None),
]

# Build a lookup by address for register names
ALL_REGISTERS = EPROM_REGISTERS + SRAM_REGISTERS + FACTORY_REGISTERS
REGISTER_BY_ADDR = {addr: (name, size) for name, addr, size, _, _ in ALL_REGISTERS}

# EPROM addresses (registers below addr 40)
EPROM_ADDRS = {addr for _, addr, _, _, _ in EPROM_REGISTERS}


def parse_motor_ids(s):
    """Parse '1,2,3' or '1-5' or 'all' into a list of motor IDs."""
    if s.lower() == "all":
        return list(MOTOR_NAMES.keys())
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
    port_handler = scs.PortHandler(args.port)
    packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
    if not port_handler.openPort():
        print(f"Failed to open {args.port}")
        sys.exit(1)
    port_handler.setBaudRate(args.baud)

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
    if addr == 5:
        print("ERROR: Use fix_servo_ids.py to change motor IDs, not this tool.")
        port_handler.closePort()
        sys.exit(1)

    for mid in motor_ids:
        # Ping first
        flush_serial(port_handler)
        _, result, _ = packet_handler.ping(port_handler, mid)
        if result != scs.COMM_SUCCESS:
            print(f"  Motor {mid}: NOT RESPONDING, skipping")
            continue

        # Read current value
        flush_serial(port_handler)
        if size == 1:
            old_val, result, _ = packet_handler.read1ByteTxRx(port_handler, mid, addr)
        else:
            old_val, result, _ = packet_handler.read2ByteTxRx(port_handler, mid, addr)
        if result != scs.COMM_SUCCESS:
            old_str = "?"
        else:
            old_str = str(old_val)

        if is_eprom:
            # Disable torque and unlock EPROM
            flush_serial(port_handler)
            packet_handler.write1ByteTxRx(port_handler, mid, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.05)
            flush_serial(port_handler)
            packet_handler.write1ByteTxRx(port_handler, mid, ADDR_LOCK, 0)
            time.sleep(0.05)

        # Write
        flush_serial(port_handler)
        if size == 1:
            result, _ = packet_handler.write1ByteTxRx(port_handler, mid, addr, value)
        else:
            result, _ = packet_handler.write2ByteTxRx(port_handler, mid, addr, value)
        time.sleep(0.05)

        if result != scs.COMM_SUCCESS:
            print(f"  Motor {mid}: WRITE FAILED ({packet_handler.getTxRxResult(result)})")
            continue

        if is_eprom:
            # Re-lock EPROM
            flush_serial(port_handler)
            packet_handler.write1ByteTxRx(port_handler, mid, ADDR_LOCK, 1)
            time.sleep(0.05)

        # Read back to verify
        flush_serial(port_handler)
        if size == 1:
            new_val, result, _ = packet_handler.read1ByteTxRx(port_handler, mid, addr)
        else:
            new_val, result, _ = packet_handler.read2ByteTxRx(port_handler, mid, addr)

        if result == scs.COMM_SUCCESS and new_val == value:
            name = MOTOR_NAMES.get(mid, "")
            print(f"  Motor {mid}{' ('+name+')' if name else ''}: {old_str} -> {new_val}  OK")
        elif result == scs.COMM_SUCCESS:
            print(f"  Motor {mid}: wrote {value} but read back {new_val}  MISMATCH")
        else:
            print(f"  Motor {mid}: wrote {value}, read-back failed")

    port_handler.closePort()


def cmd_read(args):
    """Read and display all registers."""
    port_handler = scs.PortHandler(args.port)
    packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
    if not port_handler.openPort():
        print(f"Failed to open {args.port}")
        sys.exit(1)
    port_handler.setBaudRate(args.baud)

    if args.motors:
        motor_ids = []
        for m in args.motors:
            motor_ids.extend(parse_motor_ids(m))
    else:
        motor_ids = list(MOTOR_NAMES.keys())

    for mid in motor_ids:
        name = MOTOR_NAMES.get(mid, f"unknown_{mid}")
        print(f"\n{'='*70}")
        print(f"  Motor {mid}: {name}")
        print(f"{'='*70}")

        # Ping
        _, result, _ = packet_handler.ping(port_handler, mid)
        if result != scs.COMM_SUCCESS:
            print(f"  *** MOTOR NOT RESPONDING ***")
            continue

        for section_name, registers in [
            ("EPROM", EPROM_REGISTERS),
            ("SRAM", SRAM_REGISTERS),
            ("FACTORY", FACTORY_REGISTERS),
        ]:
            print(f"\n  --- {section_name} ---")
            for reg_name, addr, size, desc, decode_fn in registers:
                if size == 1:
                    raw, result, error = packet_handler.read1ByteTxRx(port_handler, mid, addr)
                else:
                    raw, result, error = packet_handler.read2ByteTxRx(port_handler, mid, addr)

                if result != scs.COMM_SUCCESS:
                    print(f"  {reg_name:<30} addr={addr:<3} READ ERROR")
                    continue

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
        ("Present_Load", 60, 2, lambda v: decode_sm(v, 10)),
        ("Present_Voltage", 62, 1, None),
        ("Present_Temperature", 63, 1, None),
        ("Present_Current", 69, 2, None),
        ("Max_Acceleration", 85, 1, None),
    ]

    for param_name, addr, size, decode_fn in key_params:
        row = f"  {param_name:<25}"
        for mid in motor_ids:
            if size == 1:
                raw, result, _ = packet_handler.read1ByteTxRx(port_handler, mid, addr)
            else:
                raw, result, _ = packet_handler.read2ByteTxRx(port_handler, mid, addr)
            if result != scs.COMM_SUCCESS:
                row += f"{'ERR':>12}"
            else:
                val = decode_fn(raw) if decode_fn else raw
                row += f"{val:>12}"
        print(row)

    port_handler.closePort()


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
