#!/usr/bin/env python3
"""CamBot URDF visualization with viser.

Modes:
  Default:  Interactive sliders to move each joint manually.
  --live:   Reads Feetech servo positions and updates the model in real time.
            Move the physical robot by hand to verify joint directions match.

Usage:
  ./visualize_urdf.py                          # sliders only
  ./visualize_urdf.py --live                   # live from servos
  ./visualize_urdf.py --live --port /dev/ttyACM0
"""

import argparse
import math
import time
from pathlib import Path

import viser
from viser.extras import ViserUrdf

# --- Configuration ---
URDF_PATH = Path(__file__).parent / "urdf" / "cambot.urdf"

STEPS_PER_REV = 4096
STEPS_TO_RAD = 2.0 * math.pi / STEPS_PER_REV
POS_SIGN_BIT = 15

# Motor ID -> URDF joint name
MOTOR_JOINT_MAP = {
    1: "base_yaw",
    2: "shoulder_pitch",
    3: "elbow_pitch",
    4: "wrist_pitch",
    5: "wrist_yaw",
    6: "camera_roll",
}
MOTOR_IDS = list(MOTOR_JOINT_MAP.keys())


def decode_sm(raw, sign_bit):
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if (raw >> sign_bit) & 1 else magnitude


def connect_servos(port, baudrate):
    import scservo_sdk as scs
    ph = scs.PortHandler(port)
    pkt = scs.PacketHandler(0)
    if not ph.openPort():
        raise RuntimeError(f"Cannot open {port}")
    if not ph.setBaudRate(baudrate):
        raise RuntimeError(f"Cannot set baud rate {baudrate}")
    # Verify all motors respond
    for mid in MOTOR_IDS:
        _, res, _ = pkt.ping(ph, mid)
        if res != scs.COMM_SUCCESS:
            ph.closePort()
            raise RuntimeError(f"Motor {mid} ({MOTOR_JOINT_MAP[mid]}) not responding")
    return ph, pkt


def read_servo_positions_raw(ph, pkt):
    """Read all servo positions, return dict of {joint_name: steps}."""
    import scservo_sdk as scs
    cfg = {}
    for mid in MOTOR_IDS:
        raw, res, _ = pkt.read2ByteTxRx(ph, mid, 56)  # ADDR_PRESENT_POSITION
        if res == scs.COMM_SUCCESS:
            cfg[MOTOR_JOINT_MAP[mid]] = decode_sm(raw, POS_SIGN_BIT)
    return cfg


def main():
    parser = argparse.ArgumentParser(description="CamBot URDF visualizer")
    parser.add_argument("--live", action="store_true",
                        help="Read joint angles from Feetech servos")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()

    # --- Start viser server ---
    server = viser.ViserServer()
    server.scene.set_up_direction("+z")

    # Load URDF
    urdf_vis = ViserUrdf(server, URDF_PATH)

    # Get joint names and limits from URDF
    joint_names = urdf_vis.get_actuated_joint_names()
    joint_limits = urdf_vis.get_actuated_joint_limits()

    # Add GUI sliders
    sliders = {}
    for name in joint_names:
        lower, upper = joint_limits[name]
        sliders[name] = server.gui.add_slider(
            label=name,
            min=lower,
            max=upper,
            step=0.01,
            initial_value=0.0,
        )

    # Connect to servos if live mode
    ph = pkt = None
    if args.live:
        try:
            ph, pkt = connect_servos(args.port, args.baudrate)
            print(f"Connected to servos on {args.port}")
            # Disable torque so user can move by hand
            import scservo_sdk as scs
            for mid in MOTOR_IDS:
                pkt.write1ByteTxRx(ph, mid, 40, 0)  # ADDR_TORQUE_ENABLE
            print("Torque disabled — move joints by hand to verify directions.")
            # Capture current positions as zero reference and init unwrap state
            zero_raw = read_servo_positions_raw(ph, pkt)
            prev_raw = dict(zero_raw)
            unwrapped = {name: 0 for name in zero_raw}
            print("Zero reference captured:")
            for name, steps in zero_raw.items():
                print(f"  {name}: {steps} steps ({steps * STEPS_TO_RAD:.4f} rad)")
        except Exception as e:
            print(f"WARNING: Could not connect to servos: {e}")
            print("Falling back to slider-only mode.")
            args.live = False

    live_toggle = None
    if args.live:
        live_toggle = server.gui.add_checkbox("Live from servos", initial_value=True)

    print(f"Open http://localhost:8080 in your browser")

    try:
        while True:
            cfg = {}

            if args.live and live_toggle and live_toggle.value:
                # Read from servos, unwrap to handle encoder overflow
                raw = read_servo_positions_raw(ph, pkt)
                for name, steps in raw.items():
                    delta = steps - prev_raw.get(name, steps)
                    if delta > STEPS_PER_REV // 2:
                        delta -= STEPS_PER_REV
                    elif delta < -STEPS_PER_REV // 2:
                        delta += STEPS_PER_REV
                    unwrapped[name] = unwrapped.get(name, 0) + delta
                    prev_raw[name] = steps
                    angle = unwrapped[name] * STEPS_TO_RAD
                    cfg[name] = angle
                    if name in sliders:
                        sliders[name].value = angle
            else:
                # Read from sliders
                for name, slider in sliders.items():
                    cfg[name] = slider.value

            urdf_vis.update_cfg(cfg)
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        if ph is not None:
            ph.closePort()
        print("Done.")


if __name__ == "__main__":
    main()
