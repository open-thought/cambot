#!/usr/bin/env python3
"""Debug Control TUI for CamBot.

Two modes (Tab to switch):
  Joint Mode:    Select joint (1-6/Up/Down), move with Left/Right, +/- step size
  Cartesian Mode: WASD=X/Y, QE=Z, IJKL=pitch/yaw, UO=roll

Shared controls:
  t/T  Toggle torque (selected / all)
  h    Smooth move to home position
  0    Smooth move to URDF zero (resting position)
  Tab  Switch mode
  q/ESC Quit (ESC only in Cartesian mode, since Q=Z-up)

Usage:
    python -m cambot.tools.debug_control
    python -m cambot.tools.debug_control --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import curses
import json
import math
import os
import time

import numpy as np
import scservo_sdk as scs

from cambot import CALIBRATION_DIR
from cambot.servo import (
    JOINT_NAMES,
    STEPS_PER_REV,
    STEPS_TO_RAD,
    RAD_TO_STEPS,
    POS_SIGN_BIT,
    ADDR_TORQUE_ENABLE,
    ADDR_GOAL_POSITION,
    ADDR_PRESENT_POSITION,
    ADDR_P_COEFFICIENT,
    ADDR_D_COEFFICIENT,
    ADDR_I_COEFFICIENT,
    ADDR_LOCK,
    ADDR_PRESENT_VELOCITY,
    ADDR_PRESENT_LOAD,
    ADDR_PRESENT_VOLTAGE,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_CURRENT,
    PID_FACTORY_DEFAULTS,
    decode_sm,
    encode_sm,
)
from cambot.servo.controller import CamBotServo
from cambot.teleop.ik_solver import CamBotIK

# Extended sync read: addr 56, 15 bytes covers position(2) + speed(2) + load(2) + voltage(1) + temp(1) + pad(5) + current(2)
# Current is at addr 69 which is 56+13, so we need 15 bytes: 69-56+2 = 15
SYNC_READ_START = 56
SYNC_READ_LEN = 15  # covers addr 56-70

# Calibration file paths
HOME_PATH = str(CALIBRATION_DIR / "home_position.json")
RESTING_PATH = str(CALIBRATION_DIR / "resting_position.json")

MODE_JOINT = 0
MODE_CARTESIAN = 1
MODE_PID = 2
NUM_MODES = 3
MODE_NAMES = ["JOINT", "CARTESIAN/IK", "PID TUNING"]

PID_COEFF_NAMES = ["P", "D", "I"]
PID_COEFF_ADDRS = [ADDR_P_COEFFICIENT, ADDR_D_COEFFICIENT, ADDR_I_COEFFICIENT]

# Alias for ADDR_PRESENT_VELOCITY (previously named ADDR_PRESENT_SPEED locally)
ADDR_PRESENT_SPEED = ADDR_PRESENT_VELOCITY


def load_json(path: str) -> dict | None:
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def rodrigues_rotate(mat4: np.ndarray, axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Apply incremental rotation via Rodrigues formula to a 4x4 transform.

    Rotates the orientation (upper-left 3x3) of mat4 around `axis` (in world frame)
    by `angle_rad`. Position is unchanged.
    """
    result = mat4.copy()
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    R_inc = np.eye(3) * c + s * K + (1 - c) * np.outer(axis, axis)
    result[:3, :3] = R_inc @ mat4[:3, :3]
    return result


class DebugControlTUI:
    """Curses-based debug control interface."""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000, servo_time_profile: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.servo_time_profile = servo_time_profile
        self.servo: CamBotServo | None = None
        self.ik: CamBotIK | None = None

        # Mode
        self.mode = MODE_JOINT
        self.selected_joint = 0  # index into JOINT_NAMES
        self.joint_step = 20     # steps

        # Cartesian mode
        self.cart_trans_step = 5.0   # mm
        self.cart_rot_step = 2.0     # degrees
        self.target_4x4: np.ndarray | None = None
        self.workspace_min = np.full(3, np.inf)
        self.workspace_max = np.full(3, -np.inf)
        self.ik_success = True
        self.last_ik_angles: dict[str, float] = {}

        # Telemetry
        self.goal_positions: dict[str, int] = {}    # tracked locally
        self.actual_positions: dict[str, int] = {}
        self.speeds: dict[str, int] = {}
        self.loads: dict[str, int] = {}
        self.voltages: dict[str, float] = {}
        self.temperatures: dict[str, int] = {}
        self.currents: dict[str, int] = {}

        # PID coefficients (read from EPROM)
        self.pid: dict[str, tuple[int, int, int]] = {}  # name -> (P, D, I)

        # PID tuning state
        self.pid_selected_coeff = 0  # 0=P, 1=D, 2=I
        self.pid_step = 1            # adjustment step size

        # Torque state
        self.torque_on = False

        # Calibration data
        self.home_raw = load_json(HOME_PATH)
        self.resting_raw = load_json(RESTING_PATH)

        # Status message
        self.status_msg = ""
        self.status_time = 0.0

        # Extended GroupSyncRead
        self._ext_sync_read: scs.GroupSyncRead | None = None

    def connect(self):
        """Connect to servos and IK solver."""
        urdf_zero = self.resting_raw
        self.servo = CamBotServo.connect(self.port, self.baudrate, urdf_zero=urdf_zero)

        # Set up extended GroupSyncRead (addr 56, 15 bytes)
        self._ext_sync_read = scs.GroupSyncRead(
            self.servo.ph, self.servo.pkt, SYNC_READ_START, SYNC_READ_LEN
        )
        for name in JOINT_NAMES:
            mid = CamBotServo.MOTOR_IDS[name]
            self._ext_sync_read.addParam(mid)

        # Read PID coefficients (once)
        self._read_pid_coefficients()

        # Read initial positions as goals
        raw = self.servo.read_raw_positions()
        self.goal_positions = dict(raw)
        self.actual_positions = dict(raw)

        # Init IK solver
        self.ik = CamBotIK()

        # Init Cartesian target from current FK
        if self.resting_raw:
            angles = self.servo.raw_to_relative(raw)
            self.target_4x4 = self.ik.forward_kinematics(angles)

    def _read_pid_coefficients(self):
        """Read P, D, I coefficients from EPROM for all motors."""
        for name in JOINT_NAMES:
            mid = CamBotServo.MOTOR_IDS[name]
            p_val, _, _ = self.servo.pkt.read1ByteTxRx(self.servo.ph, mid, ADDR_P_COEFFICIENT)
            d_val, _, _ = self.servo.pkt.read1ByteTxRx(self.servo.ph, mid, ADDR_D_COEFFICIENT)
            i_val, _, _ = self.servo.pkt.read1ByteTxRx(self.servo.ph, mid, ADDR_I_COEFFICIENT)
            self.pid[name] = (p_val, d_val, i_val)

    def _write_pid_coefficient(self, name: str, coeff_idx: int, value: int):
        """Write a single PID coefficient to EPROM for one motor.

        EPROM write sequence: torque off -> Lock=0 -> write -> Lock=1.
        Torque is re-enabled afterward if self.torque_on is True.
        """
        value = max(0, min(255, value))
        mid = CamBotServo.MOTOR_IDS[name]
        ph, pkt = self.servo.ph, self.servo.pkt
        addr = PID_COEFF_ADDRS[coeff_idx]

        # Save/restore blocking timeout for reliable EPROM writes
        old_timeout = ph.ser.write_timeout
        ph.ser.write_timeout = None

        try:
            # 1. Torque off
            pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.01)
            # 2. Unlock EPROM
            pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 0)
            time.sleep(0.01)
            # 3. Write coefficient
            pkt.write1ByteTxRx(ph, mid, addr, value)
            time.sleep(0.01)
            # 4. Lock EPROM
            pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 1)
            time.sleep(0.01)
            # 5. Re-enable torque if it was on
            if self.torque_on:
                pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
            # 6. Flush rx buffer -- EPROM writes can leave stale bytes
            time.sleep(0.01)
            ph.ser.reset_input_buffer()
        finally:
            ph.ser.write_timeout = old_timeout

        # Update local state
        p, d, i = self.pid.get(name, (0, 0, 0))
        vals = [p, d, i]
        vals[coeff_idx] = value
        self.pid[name] = tuple(vals)

    def _write_pid_all_motors(self, coeff_idx: int, value: int):
        """Write a PID coefficient to all motors."""
        for name in JOINT_NAMES:
            self._write_pid_coefficient(name, coeff_idx, value)

    def _copy_pid_to_all(self, source_name: str):
        """Copy all PID values from source joint to all other joints."""
        p, d, i = self.pid.get(source_name, PID_FACTORY_DEFAULTS)
        for coeff_idx, val in enumerate((p, d, i)):
            for name in JOINT_NAMES:
                if name != source_name:
                    self._write_pid_coefficient(name, coeff_idx, val)

    def _read_extended_telemetry(self):
        """Read position, speed, load, voltage, temp, current for all motors in one packet."""
        old_timeout = self.servo.ph.ser.write_timeout
        self.servo.ph.ser.write_timeout = None

        try:
            comm_result = self._ext_sync_read.txRxPacket()
            if comm_result != scs.COMM_SUCCESS:
                return

            for name in JOINT_NAMES:
                mid = CamBotServo.MOTOR_IDS[name]
                if not self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_POSITION, 2):
                    continue

                # Position (addr 56, 2 bytes, sign-magnitude bit 15)
                raw_pos = self._ext_sync_read.getData(mid, ADDR_PRESENT_POSITION, 2)
                self.actual_positions[name] = decode_sm(raw_pos, POS_SIGN_BIT)

                # Speed (addr 58, 2 bytes, sign-magnitude bit 15)
                if self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_SPEED, 2):
                    raw_spd = self._ext_sync_read.getData(mid, ADDR_PRESENT_SPEED, 2)
                    self.speeds[name] = decode_sm(raw_spd, 15)

                # Load (addr 60, 2 bytes, sign-magnitude bit 10)
                if self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_LOAD, 2):
                    raw_load = self._ext_sync_read.getData(mid, ADDR_PRESENT_LOAD, 2)
                    self.loads[name] = decode_sm(raw_load, 10)

                # Voltage (addr 62, 1 byte, x0.1V)
                if self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_VOLTAGE, 1):
                    raw_v = self._ext_sync_read.getData(mid, ADDR_PRESENT_VOLTAGE, 1)
                    self.voltages[name] = round(raw_v * 0.1, 1)

                # Temperature (addr 63, 1 byte)
                if self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_TEMPERATURE, 1):
                    self.temperatures[name] = self._ext_sync_read.getData(mid, ADDR_PRESENT_TEMPERATURE, 1)

                # Current (addr 69, 2 bytes, unsigned mA)
                if self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_CURRENT, 2):
                    self.currents[name] = self._ext_sync_read.getData(mid, ADDR_PRESENT_CURRENT, 2)
        finally:
            self.servo.ph.ser.write_timeout = old_timeout

    def _set_status(self, msg: str):
        self.status_msg = msg
        self.status_time = time.monotonic()

    def _move_joint_step(self, direction: int):
        """Move selected joint by step in given direction (+1 or -1)."""
        name = JOINT_NAMES[self.selected_joint]
        current_goal = self.goal_positions.get(name, self.actual_positions.get(name, 2048))
        new_goal = max(0, min(4095, current_goal + direction * self.joint_step))
        self.goal_positions[name] = new_goal
        self.servo.write_raw_positions({name: new_goal})
        self._set_status(f"{name}: {current_goal} -> {new_goal} (step={self.joint_step})")

    def _move_cartesian(self, axis: str, direction: float):
        """Move Cartesian target along axis by step."""
        if self.target_4x4 is None:
            self._set_status("No Cartesian target (need resting_position.json)")
            return

        step_m = self.cart_trans_step / 1000.0  # mm to meters
        step_rad = math.radians(self.cart_rot_step)

        t = self.target_4x4

        if axis == "x":
            t[:3, 3] += np.array([1, 0, 0]) * direction * step_m
        elif axis == "y":
            t[:3, 3] += np.array([0, 1, 0]) * direction * step_m
        elif axis == "z":
            t[:3, 3] += np.array([0, 0, 1]) * direction * step_m
        elif axis == "pitch":
            t = rodrigues_rotate(t, np.array([0, 1, 0]), direction * step_rad)
        elif axis == "yaw":
            t = rodrigues_rotate(t, np.array([0, 0, 1]), direction * step_rad)
        elif axis == "roll":
            t = rodrigues_rotate(t, np.array([1, 0, 0]), direction * step_rad)

        self.target_4x4 = t

        # Update workspace bounds
        pos = t[:3, 3]
        self.workspace_min = np.minimum(self.workspace_min, pos)
        self.workspace_max = np.maximum(self.workspace_max, pos)

        # Solve IK with warm-start
        q_init = self.last_ik_angles if self.last_ik_angles else None
        angles, success = self.ik.solve(t, q_init)
        self.ik_success = success
        self.last_ik_angles = angles

        if success or not q_init:
            # Convert to raw and command servos
            raw = {}
            for name, rad in angles.items():
                zero = self.servo._zero_raw.get(name, 0)
                raw[name] = max(0, min(4095, zero + int(round(rad * RAD_TO_STEPS))))
            self.goal_positions.update(raw)
            self.servo.write_raw_positions(raw)

        label = axis.upper()
        self._set_status(f"IK {'OK' if success else 'FAIL'} | {label} {direction:+.0f} | pos=[{pos[0]*1000:.0f}, {pos[1]*1000:.0f}, {pos[2]*1000:.0f}]mm")

    def _smooth_move_to(self, target_raw: dict[str, int], label: str):
        """Smooth move to a target position."""
        self._set_status(f"Moving to {label}...")
        self.servo.set_torque(True)
        self.torque_on = True
        self.servo.move_to_raw_position(target_raw, duration=1.5, use_servo_time_profile=self.servo_time_profile)
        self.goal_positions.update(target_raw)

        # Update Cartesian target if in IK mode
        if self.ik and self.resting_raw:
            angles = self.servo.raw_to_relative(target_raw)
            self.target_4x4 = self.ik.forward_kinematics(angles)
            self.last_ik_angles = angles

        self._set_status(f"Reached {label}")

    def handle_input(self, key: int) -> bool:
        """Handle a keypress. Returns False to quit."""
        if key == -1:
            return True

        # --- Quit (ESC always, q/Q only in Joint mode) ---
        if key == 27:
            return False
        if key in (ord('q'), ord('Q')) and self.mode != MODE_CARTESIAN:
            return False

        # --- Tab: switch mode ---
        if key == 9:  # Tab
            self.mode = (self.mode + 1) % NUM_MODES
            self._set_status(f"Mode: {MODE_NAMES[self.mode]}")
            return True

        # --- Torque ---
        if key == ord('t'):
            name = JOINT_NAMES[self.selected_joint]
            mid = CamBotServo.MOTOR_IDS[name]
            self.torque_on = not self.torque_on
            self.servo.pkt.write1ByteTxRx(
                self.servo.ph, mid, ADDR_TORQUE_ENABLE, 1 if self.torque_on else 0
            )
            self._set_status(f"{name} torque {'ON' if self.torque_on else 'OFF'}")
            return True
        if key == ord('T'):
            self.torque_on = not self.torque_on
            self.servo.set_torque(self.torque_on)
            self._set_status(f"All torque {'ON' if self.torque_on else 'OFF'}")
            return True

        # --- Home / Zero ---
        if key == ord('h'):
            if self.home_raw:
                self._smooth_move_to(self.home_raw, "home")
            else:
                self._set_status("No home_position.json found")
            return True
        if key == ord('0'):
            if self.resting_raw:
                self._smooth_move_to(self.resting_raw, "URDF zero")
            else:
                self._set_status("No resting_position.json found")
            return True

        # --- Mode-specific ---
        if self.mode == MODE_JOINT:
            return self._handle_joint_input(key)
        elif self.mode == MODE_CARTESIAN:
            return self._handle_cartesian_input(key)
        else:
            return self._handle_pid_input(key)

    def _handle_joint_input(self, key: int) -> bool:
        # Select joint
        if ord('1') <= key <= ord('6'):
            self.selected_joint = key - ord('1')
            return True
        if key == curses.KEY_UP:
            self.selected_joint = (self.selected_joint - 1) % 6
            return True
        if key == curses.KEY_DOWN:
            self.selected_joint = (self.selected_joint + 1) % 6
            return True

        # Move joint
        if key == curses.KEY_RIGHT:
            self._move_joint_step(+1)
            return True
        if key == curses.KEY_LEFT:
            self._move_joint_step(-1)
            return True

        # Step size
        if key in (ord('+'), ord('=')):
            self.joint_step = min(200, self.joint_step + 5)
            self._set_status(f"Joint step: {self.joint_step}")
            return True
        if key in (ord('-'), ord('_')):
            self.joint_step = max(1, self.joint_step - 5)
            self._set_status(f"Joint step: {self.joint_step}")
            return True

        # Set current position as URDF zero
        if key == ord('Z'):
            raw = self.servo.read_raw_positions()
            # Save to resting_position.json
            os.makedirs(os.path.dirname(RESTING_PATH), exist_ok=True)
            with open(RESTING_PATH, "w") as f:
                json.dump(raw, f, indent=2)
                f.write("\n")
            # Update internal state
            self.resting_raw = dict(raw)
            self.servo.set_zero_reference(raw)
            # Re-init Cartesian target from new zero
            angles = self.servo.raw_to_relative(raw)
            self.target_4x4 = self.ik.forward_kinematics(angles)
            self._set_status(f"URDF zero saved & applied: {raw}")
            return True

        # Save current position as home
        if key == ord('H'):
            raw = self.servo.read_raw_positions()
            os.makedirs(os.path.dirname(HOME_PATH), exist_ok=True)
            with open(HOME_PATH, "w") as f:
                json.dump(raw, f, indent=2)
                f.write("\n")
            self.home_raw = dict(raw)
            self._set_status(f"Home position saved: {raw}")
            return True

        return True

    def _handle_cartesian_input(self, key: int) -> bool:
        # Translation: WASD = X/Y, QE = Z
        if key == ord('w'):
            self._move_cartesian("x", +1)
        elif key == ord('s'):
            self._move_cartesian("x", -1)
        elif key == ord('a'):
            self._move_cartesian("y", +1)
        elif key == ord('d'):
            self._move_cartesian("y", -1)
        elif key == ord('q'):
            self._move_cartesian("z", +1)
        elif key == ord('e'):
            self._move_cartesian("z", -1)

        # Rotation: IJKL = pitch/yaw, UO = roll
        elif key == ord('i'):
            self._move_cartesian("pitch", +1)
        elif key == ord('k'):
            self._move_cartesian("pitch", -1)
        elif key == ord('j'):
            self._move_cartesian("yaw", +1)
        elif key == ord('l'):
            self._move_cartesian("yaw", -1)
        elif key == ord('u'):
            self._move_cartesian("roll", -1)
        elif key == ord('o'):
            self._move_cartesian("roll", +1)

        # Step size
        elif key in (ord('+'), ord('=')):
            self.cart_trans_step = min(50.0, self.cart_trans_step + 1.0)
            self._set_status(f"Translation step: {self.cart_trans_step:.0f}mm")
        elif key in (ord('-'), ord('_')):
            self.cart_trans_step = max(1.0, self.cart_trans_step - 1.0)
            self._set_status(f"Translation step: {self.cart_trans_step:.0f}mm")
        elif key in (ord(']'),):
            self.cart_rot_step = min(20.0, self.cart_rot_step + 0.5)
            self._set_status(f"Rotation step: {self.cart_rot_step:.1f}deg")
        elif key in (ord('['),):
            self.cart_rot_step = max(0.5, self.cart_rot_step - 0.5)
            self._set_status(f"Rotation step: {self.cart_rot_step:.1f}deg")

        # Reset workspace bounds
        elif key == ord('r'):
            self.workspace_min = np.full(3, np.inf)
            self.workspace_max = np.full(3, -np.inf)
            self._set_status("Workspace bounds reset")

        return True

    def _handle_pid_input(self, key: int) -> bool:
        # Select joint
        if ord('1') <= key <= ord('6'):
            self.selected_joint = key - ord('1')
            return True
        if key == curses.KEY_UP:
            self.selected_joint = (self.selected_joint - 1) % 6
            return True
        if key == curses.KEY_DOWN:
            self.selected_joint = (self.selected_joint + 1) % 6
            return True

        # Select coefficient (Left/Right)
        if key == curses.KEY_LEFT:
            self.pid_selected_coeff = (self.pid_selected_coeff - 1) % 3
            self._set_status(f"Selected: {PID_COEFF_NAMES[self.pid_selected_coeff]}")
            return True
        if key == curses.KEY_RIGHT:
            self.pid_selected_coeff = (self.pid_selected_coeff + 1) % 3
            self._set_status(f"Selected: {PID_COEFF_NAMES[self.pid_selected_coeff]}")
            return True

        # Adjust value (+/-)
        if key in (ord('+'), ord('=')):
            name = JOINT_NAMES[self.selected_joint]
            p, d, i = self.pid.get(name, PID_FACTORY_DEFAULTS)
            vals = [p, d, i]
            new_val = min(255, vals[self.pid_selected_coeff] + self.pid_step)
            self._write_pid_coefficient(name, self.pid_selected_coeff, new_val)
            coeff = PID_COEFF_NAMES[self.pid_selected_coeff]
            self._set_status(f"{name} {coeff}: {vals[self.pid_selected_coeff]} -> {new_val}")
            return True
        if key in (ord('-'), ord('_')):
            name = JOINT_NAMES[self.selected_joint]
            p, d, i = self.pid.get(name, PID_FACTORY_DEFAULTS)
            vals = [p, d, i]
            new_val = max(0, vals[self.pid_selected_coeff] - self.pid_step)
            self._write_pid_coefficient(name, self.pid_selected_coeff, new_val)
            coeff = PID_COEFF_NAMES[self.pid_selected_coeff]
            self._set_status(f"{name} {coeff}: {vals[self.pid_selected_coeff]} -> {new_val}")
            return True

        # Step size ([/])
        if key == ord(']'):
            self.pid_step = min(50, self.pid_step + 1)
            self._set_status(f"PID step: {self.pid_step}")
            return True
        if key == ord('['):
            self.pid_step = max(1, self.pid_step - 1)
            self._set_status(f"PID step: {self.pid_step}")
            return True

        # Copy to all joints
        if key == ord('c'):
            name = JOINT_NAMES[self.selected_joint]
            self._copy_pid_to_all(name)
            self._set_status(f"Copied PID from {name} to all joints")
            return True

        # Re-read from EPROM
        if key == ord('r'):
            self._read_pid_coefficients()
            self._set_status("PID re-read from EPROM")
            return True

        # Reset selected joint to factory defaults
        if key == ord('f'):
            name = JOINT_NAMES[self.selected_joint]
            for coeff_idx, val in enumerate(PID_FACTORY_DEFAULTS):
                self._write_pid_coefficient(name, coeff_idx, val)
            self._set_status(f"{name} reset to factory defaults (P={PID_FACTORY_DEFAULTS[0]} D={PID_FACTORY_DEFAULTS[1]} I={PID_FACTORY_DEFAULTS[2]})")
            return True

        # Reset ALL to factory defaults
        if key == ord('F'):
            for name in JOINT_NAMES:
                for coeff_idx, val in enumerate(PID_FACTORY_DEFAULTS):
                    self._write_pid_coefficient(name, coeff_idx, val)
            self._set_status(f"ALL joints reset to factory defaults")
            return True

        return True

    def render(self, stdscr):
        """Render the TUI."""
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        y = 0

        # Title bar
        mode_str = MODE_NAMES[self.mode]
        title = f" CamBot Debug Control [{mode_str}] "
        torq_str = "TORQUE ON" if self.torque_on else "TORQUE OFF"
        header = f"{title}  |  {torq_str}  |  Tab=switch  q=quit"
        stdscr.addnstr(y, 0, header, w - 1, curses.A_BOLD | curses.color_pair(3))
        y += 1
        stdscr.addnstr(y, 0, "=" * min(len(header), w - 1), w - 1, curses.color_pair(3))
        y += 2

        # PID info
        y = self._render_pid(stdscr, y, w)
        y += 1

        if self.mode == MODE_JOINT:
            y = self._render_joint_table(stdscr, y, w)
            y += 1
            y = self._render_joint_controls(stdscr, y, w)
        elif self.mode == MODE_CARTESIAN:
            y = self._render_cartesian(stdscr, y, w)
            y += 1
            y = self._render_joint_table(stdscr, y, w)
            y += 1
            y = self._render_cartesian_controls(stdscr, y, w)
        else:
            y = self._render_pid_tuning(stdscr, y, w)
            y += 1
            y = self._render_joint_table(stdscr, y, w)
            y += 1
            y = self._render_pid_controls(stdscr, y, w)

        # Status bar
        y += 1
        if self.status_msg and (time.monotonic() - self.status_time) < 5.0:
            stdscr.addnstr(min(y, h - 1), 0, f" {self.status_msg}", w - 1, curses.color_pair(2))

        # Voltage warning
        any_v = next((v for v in self.voltages.values() if v), None)
        if any_v is not None and any_v < 6.0:
            vy = min(y + 1, h - 1)
            warn = f" WARNING: Voltage {any_v:.1f}V < 6.0V!"
            stdscr.addnstr(vy, 0, warn, w - 1, curses.A_BOLD | curses.color_pair(4))

        stdscr.refresh()

    def _render_pid(self, stdscr, y: int, w: int) -> int:
        """Render PID coefficient summary."""
        h = stdscr.getmaxyx()[0]
        if y >= h - 1:
            return y

        # Check if all I terms are zero
        all_i_zero = all(pid[2] == 0 for pid in self.pid.values())
        pid_label = "PD Coefficients"
        if all_i_zero:
            pid_label += "  ** PURE PD (I=0 on all joints) **"
            attr = curses.A_BOLD | curses.color_pair(4)
        else:
            pid_label = "PID Coefficients"
            attr = curses.A_BOLD

        stdscr.addnstr(y, 0, pid_label, w - 1, attr)
        y += 1

        # Compact one-line per joint
        hdr = f"  {'Joint':<18} {'P':>4} {'D':>4} {'I':>4}"
        if y < h:
            stdscr.addnstr(y, 0, hdr, w - 1, curses.A_DIM)
        y += 1

        for name in JOINT_NAMES:
            if y >= h - 1:
                break
            p, d, i = self.pid.get(name, (0, 0, 0))
            i_warn = " <--" if i == 0 else ""
            line = f"  {name:<18} {p:>4} {d:>4} {i:>4}{i_warn}"
            stdscr.addnstr(y, 0, line, w - 1)
            y += 1

        return y

    def _render_joint_table(self, stdscr, y: int, w: int) -> int:
        """Render the joint telemetry table."""
        h = stdscr.getmaxyx()[0]
        if y >= h - 1:
            return y

        hdr = f"  {'#':<2} {'Joint':<18} {'Goal':>6} {'Actual':>7} {'Err':>5} {'Err_d':>6} {'Load':>5} {'mA':>5} {'V':>5} {'C':>4} {'Spd':>5}"
        stdscr.addnstr(y, 0, hdr, w - 1, curses.A_BOLD)
        y += 1
        if y < h:
            stdscr.addnstr(y, 0, "  " + "-" * min(90, w - 3), w - 1, curses.A_DIM)
        y += 1

        for idx, name in enumerate(JOINT_NAMES):
            if y >= h - 1:
                break

            mid = CamBotServo.MOTOR_IDS[name]
            goal = self.goal_positions.get(name, 0)
            actual = self.actual_positions.get(name, 0)
            err_steps = goal - actual
            err_deg = err_steps * STEPS_TO_RAD * 180.0 / math.pi
            load = self.loads.get(name, 0)
            current = self.currents.get(name, 0)
            voltage = self.voltages.get(name, 0.0)
            temp = self.temperatures.get(name, 0)
            speed = self.speeds.get(name, 0)

            line = (
                f"  {mid:<2} {name:<18} "
                f"{goal:>6} {actual:>7} {err_steps:>+5} {err_deg:>+6.1f} "
                f"{load:>+5} {current:>5} {voltage:>5.1f} {temp:>4} {speed:>+5}"
            )

            is_selected = (self.mode == MODE_JOINT and idx == self.selected_joint)
            if is_selected:
                attr = curses.A_BOLD | curses.color_pair(1)
                marker = ">"
            else:
                attr = 0
                marker = " "

            stdscr.addnstr(y, 0, marker + line[1:], w - 1, attr)
            y += 1

        return y

    def _render_cartesian(self, stdscr, y: int, w: int) -> int:
        """Render Cartesian mode info: target pose, FK pose, workspace bounds."""
        h = stdscr.getmaxyx()[0]
        if y >= h - 1:
            return y

        label = "Cartesian/IK Mode"
        ik_str = "IK: OK" if self.ik_success else "IK: FAIL"
        ik_attr = curses.color_pair(1) if self.ik_success else curses.color_pair(4)
        stdscr.addnstr(y, 0, f"  {label}  ", w - 1, curses.A_BOLD | curses.color_pair(3))
        stdscr.addnstr(y, len(f"  {label}  "), ik_str, w - 1, curses.A_BOLD | ik_attr)
        y += 1

        if self.target_4x4 is not None:
            tp = self.target_4x4[:3, 3] * 1000  # meters to mm
            stdscr.addnstr(y, 0, f"  Target pos: X={tp[0]:+7.1f}  Y={tp[1]:+7.1f}  Z={tp[2]:+7.1f} mm", w - 1)
            y += 1

            # FK from current actual angles
            if self.ik and self.resting_raw:
                angles = self.servo.raw_to_relative(self.actual_positions)
                fk = self.ik.forward_kinematics(angles)
                fp = fk[:3, 3] * 1000
                stdscr.addnstr(y, 0, f"  Actual pos: X={fp[0]:+7.1f}  Y={fp[1]:+7.1f}  Z={fp[2]:+7.1f} mm", w - 1)
                y += 1

                # Position error
                perr = np.linalg.norm(tp - fp)
                stdscr.addnstr(y, 0, f"  Pos error:  {perr:.1f} mm", w - 1)
                y += 1

            # Workspace bounds
            y += 1
            if y < h:
                stdscr.addnstr(y, 0, "  Workspace bounds (mm):", w - 1, curses.A_BOLD)
            y += 1
            if np.isfinite(self.workspace_min[0]):
                for i, ax in enumerate(["X", "Y", "Z"]):
                    if y >= h - 1:
                        break
                    lo = self.workspace_min[i] * 1000
                    hi = self.workspace_max[i] * 1000
                    rng = hi - lo
                    stdscr.addnstr(y, 0, f"    {ax}: [{lo:+7.1f}, {hi:+7.1f}]  range={rng:.1f}", w - 1)
                    y += 1
            else:
                if y < h:
                    stdscr.addnstr(y, 0, "    (no data yet - move to record)", w - 1, curses.A_DIM)
                y += 1

            stdscr.addnstr(min(y, h - 1), 0, f"  Trans step: {self.cart_trans_step:.0f}mm  Rot step: {self.cart_rot_step:.1f}deg", w - 1, curses.color_pair(2))
            y += 1
        else:
            stdscr.addnstr(y, 0, "  (no target - missing resting_position.json)", w - 1, curses.A_DIM)
            y += 1

        return y

    def _render_joint_controls(self, stdscr, y: int, w: int) -> int:
        """Render joint mode controls help."""
        h = stdscr.getmaxyx()[0]
        if y >= h - 1:
            return y

        stdscr.addnstr(y, 0, "Controls:", w - 1, curses.A_BOLD | curses.color_pair(3))
        y += 1
        controls = [
            f"1-6/Up/Down: Select joint   Left/Right: Move   +/-: Step ({self.joint_step})",
            "t: Torque selected  T: Torque all   h: Home   0: URDF zero",
            "H: Save home   Z: Save URDF zero   Tab: Switch to Cartesian   q/ESC: Quit",
        ]
        for line in controls:
            if y >= h - 1:
                break
            stdscr.addnstr(y, 2, line, w - 3)
            y += 1
        return y

    def _render_cartesian_controls(self, stdscr, y: int, w: int) -> int:
        """Render Cartesian mode controls help."""
        h = stdscr.getmaxyx()[0]
        if y >= h - 1:
            return y

        stdscr.addnstr(y, 0, "Controls:", w - 1, curses.A_BOLD | curses.color_pair(3))
        y += 1
        controls = [
            "W/S: X+/-   A/D: Y+/-   Q/E: Z+/-   (translation)",
            "I/K: pitch  J/L: yaw    U/O: roll    (rotation)",
            "+/-: Trans step   [/]: Rot step   r: Reset workspace bounds",
            "t: Torque selected  T: Torque all   h: Home   0: URDF zero",
            "Tab: Switch to Joint    ESC: Quit",
        ]
        for line in controls:
            if y >= h - 1:
                break
            stdscr.addnstr(y, 2, line, w - 3)
            y += 1
        return y

    def _render_pid_tuning(self, stdscr, y: int, w: int) -> int:
        """Render PID tuning panel with editable values."""
        h = stdscr.getmaxyx()[0]
        if y >= h - 1:
            return y

        coeff_name = PID_COEFF_NAMES[self.pid_selected_coeff]
        stdscr.addnstr(y, 0, f"  PID Tuning  |  Editing: {coeff_name}  |  Step: {self.pid_step}",
                        w - 1, curses.A_BOLD | curses.color_pair(3))
        y += 1

        # Factory defaults reference
        if y < h:
            stdscr.addnstr(y, 0,
                f"  Factory defaults: P={PID_FACTORY_DEFAULTS[0]}  D={PID_FACTORY_DEFAULTS[1]}  I={PID_FACTORY_DEFAULTS[2]}",
                w - 1, curses.A_DIM)
        y += 1

        # Table header
        if y < h:
            hdr = f"  {'#':<2} {'Joint':<18} {'P':>6} {'D':>6} {'I':>6}   {'Notes'}"
            stdscr.addnstr(y, 0, hdr, w - 1, curses.A_BOLD)
        y += 1
        if y < h:
            stdscr.addnstr(y, 0, "  " + "-" * min(60, w - 3), w - 1, curses.A_DIM)
        y += 1

        for idx, name in enumerate(JOINT_NAMES):
            if y >= h - 1:
                break

            mid = CamBotServo.MOTOR_IDS[name]
            p, d, i = self.pid.get(name, PID_FACTORY_DEFAULTS)
            vals = [p, d, i]

            is_selected = (idx == self.selected_joint)

            # Build value strings with bracket around selected coefficient
            val_strs = []
            for ci, v in enumerate(vals):
                if is_selected and ci == self.pid_selected_coeff:
                    val_strs.append(f"[{v:>3}]")
                else:
                    val_strs.append(f" {v:>3} ")

            # Notes
            notes = ""
            if i == 0:
                notes = "no I term"
            if (p, d, i) == PID_FACTORY_DEFAULTS:
                notes = "factory" if not notes else notes + ", factory"
            elif (p, d, i) != PID_FACTORY_DEFAULTS:
                notes = notes + ", MODIFIED" if notes else "MODIFIED"

            line = f"  {mid:<2} {name:<18} {val_strs[0]} {val_strs[1]} {val_strs[2]}   {notes}"

            if is_selected:
                attr = curses.A_BOLD | curses.color_pair(1)
                marker = ">"
            else:
                attr = 0
                marker = " "

            stdscr.addnstr(y, 0, marker + line[1:], w - 1, attr)
            y += 1

        return y

    def _render_pid_controls(self, stdscr, y: int, w: int) -> int:
        """Render PID mode controls help."""
        h = stdscr.getmaxyx()[0]
        if y >= h - 1:
            return y

        stdscr.addnstr(y, 0, "Controls:", w - 1, curses.A_BOLD | curses.color_pair(3))
        y += 1
        controls = [
            "1-6/Up/Down: Select joint   Left/Right: Select P/D/I coefficient",
            f"+/-: Adjust value (step={self.pid_step})   [/]: Change step size",
            "c: Copy selected PID to all   r: Re-read from EPROM",
            "f: Factory reset selected   F: Factory reset ALL",
            "t: Torque selected  T: Torque all   Tab: Switch mode   q/ESC: Quit",
        ]
        for line in controls:
            if y >= h - 1:
                break
            stdscr.addnstr(y, 2, line, w - 3)
            y += 1
        return y

    def disconnect(self):
        if self.servo and self.servo.is_connected:
            self.servo.disconnect()


def _curses_main(stdscr):
    curses.curs_set(0)
    stdscr.timeout(50)  # 20Hz loop
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_CYAN, -1)
    curses.init_pair(4, curses.COLOR_RED, -1)

    tui = _curses_main.tui

    try:
        tui.connect()
        tui._set_status("Connected. Torque OFF. Press T to enable.")
    except Exception as e:
        stdscr.addstr(0, 0, f"Connection failed: {e}")
        stdscr.addstr(1, 0, "Press any key to exit...")
        stdscr.timeout(-1)
        stdscr.getch()
        return

    try:
        while True:
            tui._read_extended_telemetry()
            tui.render(stdscr)
            key = stdscr.getch()
            if not tui.handle_input(key):
                break
    finally:
        tui.disconnect()


def main():
    parser = argparse.ArgumentParser(description="CamBot Debug Control TUI")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Baud rate")
    parser.add_argument("--servo-profile", action="store_true",
                        help="Use servo-side Goal_Time profiling instead of host-side cosine interpolation")
    args = parser.parse_args()

    tui = DebugControlTUI(port=args.port, baudrate=args.baudrate, servo_time_profile=args.servo_profile)
    _curses_main.tui = tui
    curses.wrapper(_curses_main)


if __name__ == "__main__":
    main()
