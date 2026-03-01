"""StereoBot servo controller.

Provides a clean controller class wrapping scservo_sdk for the 6-DOF arm.

Standalone usage:
    python -m cambot.servo.controller              # read/write round-trip test
    python -m cambot.servo.controller --save-home   # save current position as home
"""

from __future__ import annotations

import json
import math
import os
import time

import scservo_sdk as scs

from cambot import CALIBRATION_DIR
from cambot.servo.constants import (
    STEPS_PER_REV,
    STEPS_TO_RAD,
    RAD_TO_STEPS,
    POS_SIGN_BIT,
    JOINT_NAMES,
    MOTOR_IDS,
    ADDR_TORQUE_ENABLE,
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_GOAL_VELOCITY,
    ADDR_TORQUE_LIMIT,
    ADDR_PRESENT_POSITION,
    DEFAULT_ACCELERATION,
    DEFAULT_TORQUE_LIMIT,
)
from cambot.servo.protocol import decode_sm, encode_sm


class StereoBotServo:
    """Controller for the 6-DOF StereoBot arm servos."""

    JOINT_NAMES = JOINT_NAMES
    MOTOR_IDS = MOTOR_IDS

    def __init__(self):
        self.ph = None
        self.pkt = None
        self.is_connected = False
        self._goal_sync_write = None
        self._pos_sync_read = None
        # Delta-based encoder unwrapping state
        self._prev_raw: dict[str, int] = {}
        self._unwrapped: dict[str, int] = {}
        self._zero_raw: dict[str, int] = {}

    @classmethod
    def connect(
        cls,
        port: str = "/dev/ttyACM0",
        baudrate: int = 1_000_000,
        urdf_zero: dict[str, int] | None = None,
    ) -> "StereoBotServo":
        """Connect to servos and initialize SRAM registers.

        Args:
            urdf_zero: Absolute encoder positions (steps) corresponding to the
                URDF zero pose (all joints at 0 rad).  When provided,
                read/write_joint_angles operate relative to this reference so
                that ikpy joint limits are correct.  Falls back to connect-time
                readings when not provided.
        """
        self = cls()
        self.ph = scs.PortHandler(port)
        self.pkt = scs.PacketHandler(0)  # protocol version 0 for STS series

        if not self.ph.openPort():
            raise RuntimeError(f"Cannot open {port}")
        if not self.ph.setBaudRate(baudrate):
            self.ph.closePort()
            raise RuntimeError(f"Cannot set baud rate {baudrate}")

        # Verify all motors respond
        for name in cls.JOINT_NAMES:
            mid = cls.MOTOR_IDS[name]
            _, res, _ = self.pkt.ping(self.ph, mid)
            if res != scs.COMM_SUCCESS:
                self.ph.closePort()
                raise RuntimeError(f"Motor {mid} ({name}) not responding")

        # Init SRAM via SyncWrite (2 packets instead of 12 individual writes)
        all_ids = [cls.MOTOR_IDS[n] for n in cls.JOINT_NAMES]
        self._send_sync_write_raw(
            ADDR_ACCELERATION, 1,
            [(mid, DEFAULT_ACCELERATION) for mid in all_ids],
        )
        self._send_sync_write_raw(
            ADDR_TORQUE_LIMIT, 2,
            [(mid, DEFAULT_TORQUE_LIMIT & 0xFF, (DEFAULT_TORQUE_LIMIT >> 8) & 0xFF) for mid in all_ids],
        )

        # GroupSyncRead for bulk position reads (1 USB round-trip instead of 6)
        self._pos_sync_read = scs.GroupSyncRead(self.ph, self.pkt, ADDR_PRESENT_POSITION, 2)
        for mid in all_ids:
            self._pos_sync_read.addParam(mid)

        # Capture initial positions for unwrapping.
        # When urdf_zero is provided, use it as the zero reference so that
        # joint angles match the URDF convention (0 rad = resting pose).
        for name, steps in self._sync_read_positions().items():
            self._prev_raw[name] = steps
            if urdf_zero is not None and name in urdf_zero:
                self._zero_raw[name] = urdf_zero[name]
                delta = steps - urdf_zero[name]
                if delta > STEPS_PER_REV // 2:
                    delta -= STEPS_PER_REV
                elif delta < -STEPS_PER_REV // 2:
                    delta += STEPS_PER_REV
                self._unwrapped[name] = delta
            else:
                self._zero_raw[name] = steps
                self._unwrapped[name] = 0

        # Non-blocking writes for low-latency servo commands
        self.ph.ser.write_timeout = 0

        # Keep SDK SyncWrite for move_to_raw_position (non-latency-critical)
        self._goal_sync_write = scs.GroupSyncWrite(self.ph, self.pkt, ADDR_GOAL_POSITION, 2)

        self.is_connected = True
        return self

    def _send_sync_write_raw(self, start_addr: int, data_len: int,
                              motor_data: list[tuple]) -> None:
        """Send a SyncWrite packet directly to serial, bypassing SDK flush.

        Constructs the raw Feetech protocol v0 packet and writes it without
        calling tcdrain(), reducing latency from ~19ms to <1ms.

        motor_data: list of (motor_id, byte0, byte1, ...) tuples.
        """
        # Packet: FF FF FE LENGTH 83 START_ADDR DATA_LEN [ID D...]×N CHECKSUM
        n_motors = len(motor_data)
        n_params = 2 + n_motors * (1 + data_len)  # start_addr + data_len + per-motor
        length = n_params + 2  # instruction + params + checksum

        pkt = bytearray(4 + length)
        pkt[0] = 0xFF
        pkt[1] = 0xFF
        pkt[2] = 0xFE  # broadcast ID
        pkt[3] = length
        pkt[4] = 0x83  # SyncWrite instruction
        pkt[5] = start_addr
        pkt[6] = data_len

        idx = 7
        for entry in motor_data:
            for b in entry:
                pkt[idx] = b
                idx += 1

        # Checksum: ~(sum of bytes from ID to last param) & 0xFF
        checksum = 0
        for i in range(2, idx):
            checksum += pkt[i]
        pkt[idx] = (~checksum) & 0xFF

        # Write directly — no flush/tcdrain
        self.ph.ser.write(pkt)

    def _sync_read_positions(self) -> dict[str, int]:
        """Read all motor positions in a single USB round-trip via GroupSyncRead.

        Temporarily restores blocking write_timeout for the SyncRead request,
        then restores the previous (non-blocking) timeout.
        """
        old_timeout = self.ph.ser.write_timeout
        self.ph.ser.write_timeout = None  # blocking for SyncRead tx

        result = {}
        try:
            comm_result = self._pos_sync_read.txRxPacket()
            if comm_result != scs.COMM_SUCCESS:
                return result
            for name in self.JOINT_NAMES:
                mid = self.MOTOR_IDS[name]
                if not self._pos_sync_read.isAvailable(mid, ADDR_PRESENT_POSITION, 2):
                    continue
                raw = self._pos_sync_read.getData(mid, ADDR_PRESENT_POSITION, 2)
                result[name] = decode_sm(raw, POS_SIGN_BIT)
        finally:
            self.ph.ser.write_timeout = old_timeout
        return result

    def read_joint_angles(self) -> dict[str, float]:
        """Read current joint angles in radians (unwrapped from zero reference)."""
        positions = self._sync_read_positions()
        result = {}
        for name, steps in positions.items():
            # Delta-based unwrapping
            if name in self._prev_raw:
                delta = steps - self._prev_raw[name]
                if delta > STEPS_PER_REV // 2:
                    delta -= STEPS_PER_REV
                elif delta < -STEPS_PER_REV // 2:
                    delta += STEPS_PER_REV
                self._unwrapped[name] = self._unwrapped.get(name, 0) + delta
            self._prev_raw[name] = steps

            result[name] = self._unwrapped[name] * STEPS_TO_RAD
        return result

    def read_raw_positions(self) -> dict[str, int]:
        """Read raw encoder positions (steps, no unwrapping)."""
        return self._sync_read_positions()

    def write_joint_angles(self, angles: dict[str, float]) -> None:
        """Write joint angles in radians via raw SyncWrite (no flush, minimal latency).

        The angle is relative to the zero reference captured at connect time.
        """
        # Build motor data: [(id, low_byte, high_byte), ...]
        motor_data = []
        for name, rad in angles.items():
            if name not in self.MOTOR_IDS:
                continue
            mid = self.MOTOR_IDS[name]
            steps = self._zero_raw.get(name, 0) + int(round(rad * RAD_TO_STEPS))
            steps = max(0, min(4095, steps))
            motor_data.append((mid, steps & 0xFF, (steps >> 8) & 0xFF))
        if motor_data:
            self._send_sync_write_raw(ADDR_GOAL_POSITION, 2, motor_data)

    def raw_to_relative(self, raw_positions: dict[str, int]) -> dict[str, float]:
        """Convert raw encoder positions to radians relative to connect-time zero."""
        result = {}
        for name, raw in raw_positions.items():
            zero = self._zero_raw.get(name, raw)
            delta = raw - zero
            if delta > STEPS_PER_REV // 2:
                delta -= STEPS_PER_REV
            elif delta < -STEPS_PER_REV // 2:
                delta += STEPS_PER_REV
            result[name] = delta * STEPS_TO_RAD
        return result

    def write_raw_positions(self, positions: dict[str, int]) -> None:
        """Write raw encoder positions (steps) via raw SyncWrite (no flush)."""
        motor_data = []
        for name, steps in positions.items():
            if name not in self.MOTOR_IDS:
                continue
            mid = self.MOTOR_IDS[name]
            steps = max(0, min(4095, steps))
            motor_data.append((mid, steps & 0xFF, (steps >> 8) & 0xFF))
        if motor_data:
            self._send_sync_write_raw(ADDR_GOAL_POSITION, 2, motor_data)

    def _write_goal_with_velocity(self, target: dict[str, int], duration: float) -> None:
        """Write Goal_Position + Goal_Time=0 + Goal_Velocity via single SyncWrite.

        Computes per-motor velocity from distance/duration so all joints arrive
        at roughly the same time.  Uses velocity-limited profiling
        (Goal_Time=0, Goal_Velocity>0).
        """
        current = self._sync_read_positions()

        motor_data = []
        for name, steps in target.items():
            if name not in self.MOTOR_IDS:
                continue
            mid = self.MOTOR_IDS[name]
            steps = max(0, min(4095, steps))
            dist = abs(steps - current.get(name, steps))
            # steps/sec, minimum 1 to avoid zero (which means max speed)
            vel = max(1, int(round(dist / duration))) if duration > 0 else 1
            vel = min(vel, 4095)  # clamp to register range
            motor_data.append((
                mid,
                steps & 0xFF, (steps >> 8) & 0xFF,
                0, 0,  # time = 0
                vel & 0xFF, (vel >> 8) & 0xFF,
            ))
        if motor_data:
            self._send_sync_write_raw(ADDR_GOAL_POSITION, 6, motor_data)

    def _clear_goal_velocity(self) -> None:
        """Reset Goal_Velocity to 0 (max speed) on all motors.

        Must be called after a velocity-profiled move completes, otherwise
        the stale low velocity stays latched and throttles all subsequent
        position writes.
        """
        all_ids = [self.MOTOR_IDS[n] for n in self.JOINT_NAMES]
        self._send_sync_write_raw(
            ADDR_GOAL_VELOCITY, 2,
            [(mid, 0, 0) for mid in all_ids],
        )

    def move_to_raw_position(self, target: dict[str, int], duration: float = 2.0, rate_hz: float = 50.0, use_servo_time_profile: bool = False) -> None:
        """Interpolate smoothly from current position to target over duration.

        When use_servo_time_profile is False (default), uses host-side cosine
        easing at rate_hz, sending one SyncWrite per tick.

        When use_servo_time_profile is True, sends a single SyncWrite with
        per-motor velocity limits computed from distance/duration, offloading
        trajectory execution to the servo firmware.
        """
        if use_servo_time_profile:
            self._write_goal_with_velocity(target, duration)
            # Wait for the servo to execute the profile
            time.sleep(duration)
        else:
            current = self.read_raw_positions()
            n_steps = max(1, int(duration * rate_hz))
            dt = duration / n_steps
            t0 = time.monotonic()

            for i in range(1, n_steps + 1):
                # Cosine easing: zero velocity at start and end
                t = 0.5 * (1.0 - math.cos(math.pi * i / n_steps))
                interp = {}
                for name in self.JOINT_NAMES:
                    if name in target and name in current:
                        interp[name] = int(round(current[name] + t * (target[name] - current[name])))
                self.write_raw_positions(interp)
                # Sleep until next tick (accounts for serial I/O time)
                next_tick = t0 + i * dt
                remaining = next_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)

        # Settle loop: keep commanding target until servos converge or timeout
        settle_timeout = 1.0  # seconds
        settle_threshold = 10  # steps (~0.9 deg)
        settle_start = time.monotonic()
        while time.monotonic() - settle_start < settle_timeout:
            self.write_raw_positions(target)
            time.sleep(0.05)
            actual = self._sync_read_positions()
            max_err = max(
                abs(actual.get(n, 0) - target.get(n, 0))
                for n in self.JOINT_NAMES if n in target
            )
            if max_err <= settle_threshold:
                break

        if use_servo_time_profile:
            self._clear_goal_velocity()

    def set_zero_reference(self, zero_raw: dict[str, int]) -> None:
        """Update the URDF zero reference and reset unwrapped state to match."""
        current = self._sync_read_positions()
        for name, zero in zero_raw.items():
            self._zero_raw[name] = zero
            steps = current.get(name, zero)
            delta = steps - zero
            if delta > STEPS_PER_REV // 2:
                delta -= STEPS_PER_REV
            elif delta < -STEPS_PER_REV // 2:
                delta += STEPS_PER_REV
            self._unwrapped[name] = delta
            self._prev_raw[name] = steps

    def set_torque(self, enable: bool) -> None:
        """Enable or disable torque on all motors via single SyncWrite."""
        val = 1 if enable else 0
        all_ids = [self.MOTOR_IDS[n] for n in self.JOINT_NAMES]
        self._send_sync_write_raw(ADDR_TORQUE_ENABLE, 1, [(mid, val) for mid in all_ids])

    def disconnect(self) -> None:
        """Disable torque and close port."""
        if self.is_connected:
            self.set_torque(False)
            self.ph.closePort()
            self.is_connected = False


def save_home_position(servo: StereoBotServo, filepath: str) -> None:
    """Save current raw encoder positions as home position."""
    raw = servo.read_raw_positions()
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w") as f:
        json.dump(raw, f, indent=2)
        f.write("\n")
    print(f"Saved home position to {filepath}: {raw}")


def load_home_position(filepath: str) -> dict[str, float] | None:
    """Load home position from JSON file."""
    if not os.path.exists(filepath):
        return None
    with open(filepath) as f:
        return json.load(f)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="StereoBot servo interface test")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--save-home", action="store_true",
                        help="Save current position as home and exit")
    parser.add_argument("--save-resting", action="store_true",
                        help="Save current position as resting and exit")
    args = parser.parse_args()

    home_path = str(CALIBRATION_DIR / "home_position.json")
    resting_path = str(CALIBRATION_DIR / "resting_position.json")

    servo = StereoBotServo.connect(args.port, args.baudrate)
    print(f"Connected to {args.port}")

    try:
        if args.save_home:
            save_home_position(servo, home_path)
        elif args.save_resting:
            save_home_position(servo, resting_path)
        else:
            # Read/write round-trip test
            print("\n--- Read current positions ---")
            angles = servo.read_joint_angles()
            raw = servo.read_raw_positions()
            for name in StereoBotServo.JOINT_NAMES:
                r = angles.get(name, float("nan"))
                s = raw.get(name, "?")
                print(f"  {name:20s}: {math.degrees(r):+8.2f} deg  ({s} steps)")

            print("\n--- Writing same positions back (should not move) ---")
            servo.set_torque(True)
            servo.write_joint_angles(angles)
            time.sleep(0.5)

            angles2 = servo.read_joint_angles()
            max_err = 0.0
            for name in StereoBotServo.JOINT_NAMES:
                err = abs(angles.get(name, 0) - angles2.get(name, 0))
                max_err = max(max_err, err)
                print(f"  {name:20s}: {math.degrees(angles2.get(name, 0)):+8.2f} deg  (err={math.degrees(err):.2f} deg)")
            print(f"\nMax round-trip error: {math.degrees(max_err):.2f} deg")
            print("PASS" if max_err < 0.1 else "FAIL (>0.1 rad)")
    finally:
        servo.disconnect()
        print("Disconnected.")
