#!/usr/bin/env python3
"""
TeleHead: VR Head Tracking -> CamBot IK Camera Control

Streams stereo video from ZED Mini on the CamBot 6-DOF arm to Meta Quest 3
via WebSocket, receives head orientation + position back, and drives the robot
via full inverse kinematics.

Usage:
    python -m cambot.teleop                     # full mode
    python -m cambot.teleop --no-robot          # camera streaming only
    python -m cambot.teleop --no-camera         # robot control only (IK console output)
    python -m cambot.teleop --no-robot --no-camera   # server only (for testing)
    python -m cambot.teleop --save-home         # save current position as home and exit
    python -m cambot.teleop --save-resting      # save current position as resting and exit
"""

import argparse
import asyncio
import json
import logging
import math
import os
import select
import signal
import ssl
import sys
import threading
import time

import numpy as np

from cambot import CALIBRATION_DIR
from cambot.servo.controller import load_home_position

logger = logging.getLogger(__name__)

# --- Quaternion utilities ---

def quat_multiply(q1, q2):
    """Multiply two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def quat_conjugate(q):
    """Conjugate (inverse for unit quaternion)."""
    return (-q[0], -q[1], -q[2], q[3])


def quat_to_euler(q):
    """Convert quaternion (x,y,z,w) to euler angles (yaw, pitch, roll) in degrees.

    Uses intrinsic Tait-Bryan: Y(yaw) X(pitch) Z(roll).
    Matches WebXR coordinate system (Y-up, -Z forward).
    """
    x, y, z, w = q

    siny_cosp = 2.0 * (w * y + x * z)
    cosy_cosp = 1.0 - 2.0 * (y * y + x * x)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    sinp = 2.0 * (w * x - z * y)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    sinr_cosp = 2.0 * (w * z + y * x)
    cosr_cosp = 1.0 - 2.0 * (x * x + z * z)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))


class TeleHead:
    """Connects VR head tracking to CamBot IK camera control."""

    def __init__(
        self,
        robot_port: str = "/dev/ttyACM0",
        use_robot: bool = True,
        use_ik: bool = True,
        smoothing: float = 0.08,
        max_joint_velocity: float = 20.0,  # rad/s per joint
        rate_hz: float = 100.0,
        home_path: str | None = None,
        resting_path: str | None = None,
        position_scale: float = 1.0,
        workspace_bounds: dict[str, tuple[float, float]] | None = None,
        max_position_delta: float | None = None,
        watchdog_timeout: float = 3.0,
    ):
        self.use_robot = use_robot
        self.use_ik = use_ik
        self.robot_port = robot_port
        self.smoothing = smoothing
        self.max_joint_velocity = max_joint_velocity
        self.rate_hz = rate_hz
        self.home_path = home_path
        self.resting_path = resting_path
        self.position_scale = position_scale
        self.workspace_bounds = workspace_bounds
        self.max_position_delta = max_position_delta
        self.watchdog_timeout = watchdog_timeout

        self.servo = None
        self.ik = None

        self._lock = threading.Lock()
        self._pose_event = threading.Event()
        self._latest_pose = None
        self._running = False
        self._moving_to_position = False
        self.position_tracking = False
        self.user_paused = False

        # Smoothed joint angles (radians)
        self._smoothed_q: dict[str, float] | None = None

        # IK failure tracking
        self._consecutive_failures = 0
        self._last_failure_log = 0.0

        # Camera lifecycle
        self._camera_config: dict | None = None  # {resolution, fps, quality, use_zed}
        self._cam_thread: threading.Thread | None = None
        self._event_loop: asyncio.AbstractEventLoop | None = None
        self._switching_resolution: bool = False

        # Watchdog: track last pose timestamp for timeout detection
        self._last_pose_time: float = 0.0
        self._watchdog_active: bool = False
        # Physical home position for watchdog return (set once, never recalibrated)
        self._watchdog_home_q: dict[str, float] | None = None

        # Telemetry (updated by control loop, read by server)
        self._telemetry = {
            'ik_ms': 0.0,
            'servo_ms': 0.0,
            'control_hz': 0.0,
            'ik_failures': 0,
        }

    def connect(self):
        """Connect to the robot and set up IK."""
        if self.use_robot:
            from cambot.servo.controller import CamBotServo

            # Use resting position as URDF zero reference so that joint angles
            # match the URDF convention and ikpy joint limits are correct.
            urdf_zero = None
            if self.resting_path and os.path.exists(self.resting_path):
                urdf_zero = load_home_position(self.resting_path)
                if urdf_zero:
                    logger.info(f"URDF zero from resting position: {self.resting_path}")

            self.servo = CamBotServo.connect(self.robot_port, urdf_zero=urdf_zero)
            logger.info(f"Robot connected on {self.robot_port}")

        if self.use_ik:
            from cambot.teleop.ik_solver import CamBotIK, JOINT_NAMES
            self.ik = CamBotIK(
                position_scale=self.position_scale,
                workspace_bounds=self.workspace_bounds,
                max_position_delta=self.max_position_delta,
            )
            logger.info(f"IK solver loaded ({self.ik.n_active} active joints)")

            # Set home position
            home = None
            if self.home_path and os.path.exists(self.home_path):
                raw_home = load_home_position(self.home_path)
                if raw_home and self.servo:
                    home = self.servo.raw_to_relative(raw_home)
                elif raw_home:
                    # No servo — treat raw values as radians (legacy format)
                    home = raw_home
                logger.info(f"Loaded home position from {self.home_path}")
            elif self.servo:
                home = self.servo.read_joint_angles()
                logger.info("Using current servo positions as home")
            else:
                home = {name: 0.0 for name in JOINT_NAMES}
                logger.info("Using zero position as home (no robot)")

            self.ik.set_home(home)
            self._smoothed_q = home.copy()
            if self._watchdog_home_q is None:
                self._watchdog_home_q = home.copy()

    def _move_to_named_position(self, filepath: str, label: str, duration: float = 2.0) -> bool:
        """Load a named position from JSON and move there smoothly.

        Returns True if the move was performed, False if skipped.
        """
        if not self.servo or not self.servo.is_connected:
            return False
        if not filepath or not os.path.exists(filepath):
            logger.info(f"No {label} position file at {filepath}, skipping move")
            return False

        raw = load_home_position(filepath)
        if not raw:
            return False

        current = self.servo.read_raw_positions()
        logger.info(f"Moving to {label} position ({duration:.1f}s): "
                     f"target={raw}, current={current}")
        self._moving_to_position = True
        try:
            self.servo.move_to_raw_position(raw, duration=duration)
        finally:
            self._moving_to_position = False
        actual = self.servo.read_raw_positions()
        logger.info(f"Reached {label} position: actual={actual}")
        return True

    def calibrate_neutral(self):
        """Move to home position, then capture VR head pose as neutral reference.

        Sequence:
        1. Pause control loop (_moving_to_position flag)
        2. Move robot to home position (blocks ~2s)
        3. Resync IK home to match actual servo positions
        4. Wait for a fresh VR pose to arrive
        5. Capture that pose as the neutral VR reference
        6. Resume control loop
        """
        with self._lock:
            pose = self._latest_pose
        if pose is None:
            logger.warning("No head pose data yet.")
            return False

        # Pause the control loop for the ENTIRE calibration sequence.
        # Without this, the control loop resumes between the home move and
        # neutral capture, processing poses with a stale/mismatched neutral
        # and causing a servo jump.
        self._moving_to_position = True
        try:
            # Move to home position (servo.move_to_raw_position blocks ~2s)
            if self.servo and self.servo.is_connected and self.home_path and os.path.exists(self.home_path):
                raw = load_home_position(self.home_path)
                if raw:
                    current = self.servo.read_raw_positions()
                    logger.info(f"Calibration: moving to home (2s): target={raw}, current={current}")
                    self.servo.move_to_raw_position(raw, duration=2.0)
                    logger.info("Calibration: reached home position")

                    # Resync smoothed angles and IK home
                    if self.ik:
                        self._smoothed_q = self.servo.raw_to_relative(raw)
                        self.ik.set_home(self._smoothed_q)

            if self.ik:
                # Wait for a fresh VR pose after the move completes.
                self._pose_event.clear()
                self._pose_event.wait(timeout=1.0)
                with self._lock:
                    pose = self._latest_pose
                if pose is None:
                    logger.warning("Lost head pose during calibration.")
                    return False
                q = pose.get("q", {})
                p = pose.get("p", {"x": 0, "y": 0, "z": 0})
                self.ik.calibrate_neutral_vr(q, p)

            logger.info("Neutral head pose captured.")
        finally:
            self._moving_to_position = False
        return True

    def pause(self):
        """Signal headset removed — trigger watchdog return-to-home immediately."""
        if self._last_pose_time > 0:
            self._last_pose_time = time.monotonic() - self.watchdog_timeout - 1.0
            logger.info("Pause: headset removed, watchdog will trigger immediately")

    def resume(self):
        """Signal headset back on. Next on_head_pose() naturally clears watchdog."""
        logger.info("Resume: waiting for VR pose data")

    def calibrate_soft(self) -> bool:
        """Instant recalibration: current position becomes new home,
        current VR pose becomes new neutral. No servo movement.

        Uses _smoothed_q (not servo reads) to avoid serial bus conflicts
        with the control loop thread.
        """
        if self.ik is None or self._smoothed_q is None:
            return False
        with self._lock:
            pose = self._latest_pose
        if pose is None:
            return False

        q_vr = pose.get("q")
        p_vr = pose.get("p", {"x": 0, "y": 0, "z": 0})
        if q_vr is None:
            return False

        self._moving_to_position = True  # gate control loop
        try:
            current_q = self._smoothed_q.copy()
            self.ik.set_home(current_q)
            self._smoothed_q = current_q.copy()
            self.ik.calibrate_neutral_vr(q_vr, p_vr)
        finally:
            self._moving_to_position = False
        logger.info("Soft recalibration complete (no movement)")
        return True

    def toggle_position_tracking(self):
        """Toggle position tracking with atomic soft recalibration.

        Holds _moving_to_position gate across BOTH the recalibration and
        the toggle so the control loop never sees a half-updated state.
        """
        self._moving_to_position = True
        try:
            # Soft recalibrate: current position → home, current VR → neutral
            if self.ik is not None and self._smoothed_q is not None:
                with self._lock:
                    pose = self._latest_pose
                if pose is not None:
                    q_vr = pose.get("q")
                    p_vr = pose.get("p", {"x": 0, "y": 0, "z": 0})
                    if q_vr is not None:
                        current_q = self._smoothed_q.copy()
                        self.ik.set_home(current_q)
                        self._smoothed_q = current_q.copy()
                        self.ik.calibrate_neutral_vr(q_vr, p_vr)
            self.position_tracking = not self.position_tracking
        finally:
            self._moving_to_position = False
        logger.info(f"Position tracking: {'ON' if self.position_tracking else 'OFF'}")

    def toggle_user_pause(self):
        """Toggle user-initiated pause (lock robot in place).

        When pausing: freezes the robot at its current position.
        When unpausing: performs soft recalibration so the robot resumes
        from the current position without jumping.
        """
        if self.user_paused:
            # Unpausing — soft recalibrate before resuming
            self.calibrate_soft()
            self.user_paused = False
            logger.info("User pause OFF — soft recalibrated, resuming control")
        else:
            self.user_paused = True
            logger.info("User pause ON — robot locked in place")

    def on_head_pose(self, pose: dict):
        """Callback for new head pose data from the server."""
        now = time.monotonic()
        pose["_recv_ts"] = now
        self._last_pose_time = now
        with self._lock:
            self._latest_pose = pose
        self._pose_event.set()

    def get_telemetry(self) -> dict:
        """Get a snapshot of control loop telemetry (thread-safe read of simple types)."""
        t = self._telemetry.copy()
        t['calibrated'] = (
            self.ik is not None
            and getattr(self.ik, '_vr_neutral_inv', None) is not None
        )
        t['position_tracking'] = self.position_tracking
        t['watchdog_active'] = self._watchdog_active
        t['user_paused'] = self.user_paused
        return t

    def _create_and_start_camera(self, resolution: str, use_zed: bool,
                                   quality: int, explicit_fps: int | None = None) -> bool:
        """Create a capture object, open it, bind the shared event, and start the thread.

        Updates server.camera_capture and server.current_resolution on success.
        Returns True on success.
        """
        from cambot.teleop import server as teleop_server
        from cambot.teleop.capture import create_capture

        fps_map = {"vga": 100, "720p": 60, "1080p": 30, "2k": 15}
        cam_fps = explicit_fps if explicit_fps is not None else fps_map.get(resolution, 60)

        if use_zed:
            cap = create_capture(use_zed=True, resolution=resolution, fps=cam_fps)
        else:
            res_map = {"vga": (672, 376), "720p": (1280, 720),
                       "1080p": (1920, 1080), "2k": (2208, 1242)}
            w, h = res_map.get(resolution, (1280, 720))
            cap = create_capture(use_zed=False, camera_index=0, width=w, height=h, fps=cam_fps)

        import cv2
        cap._encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]

        if not cap.open():
            logger.error(f"Failed to open camera at {resolution}")
            return False

        # Bind shared frame event
        if self._event_loop is not None and teleop_server.frame_ready_event is not None:
            cap.set_external_event(self._event_loop, teleop_server.frame_ready_event)

        teleop_server.camera_capture = cap
        teleop_server.current_resolution = resolution
        teleop_server.update_camera_demand()

        self._cam_thread = threading.Thread(target=cap.capture_loop, daemon=True)
        self._cam_thread.start()

        self._camera_config = {
            'resolution': resolution, 'fps': cam_fps,
            'quality': quality, 'use_zed': use_zed,
        }
        logger.info(f"Camera started: {resolution}@{cam_fps}fps q{quality} "
                     f"({'ZED' if use_zed else 'fallback'})")
        return True

    def switch_resolution(self, resolution: str) -> bool:
        """Switch camera resolution at runtime. Blocks during the swap (~1-2s).

        Returns True on success.
        """
        from cambot.teleop import server as teleop_server

        if self._switching_resolution:
            logger.warning("Resolution switch already in progress")
            return False

        cfg = self._camera_config
        if cfg is None:
            logger.warning("No camera configured, cannot switch")
            return False

        if cfg['resolution'] == resolution:
            logger.info(f"Already at {resolution}")
            return True

        self._switching_resolution = True
        prev_resolution = cfg['resolution']
        try:
            # 1. Detach capture so frame loops skip
            old_cap = teleop_server.camera_capture
            teleop_server.camera_capture = None

            # 2. Stop capture loop, wait for thread, then close device
            if old_cap is not None:
                old_cap.stop()
            if self._cam_thread is not None:
                self._cam_thread.join(3.0)
            if old_cap is not None:
                old_cap.close()

            # 3. Open new capture
            if self._create_and_start_camera(
                resolution, cfg['use_zed'], cfg['quality']
            ):
                logger.info(f"Resolution switched: {prev_resolution} -> {resolution}")
                return True
            else:
                # Attempt to restore previous resolution
                logger.warning(f"Switch to {resolution} failed, restoring {prev_resolution}")
                if self._create_and_start_camera(
                    prev_resolution, cfg['use_zed'], cfg['quality']
                ):
                    return False
                else:
                    logger.error("Failed to restore previous resolution!")
                    return False
        finally:
            self._switching_resolution = False

    def control_loop(self):
        """Main IK control loop. Event-driven with rate_hz ceiling."""
        from cambot.servo.constants import JOINT_NAMES

        self._running = True
        dt = 1.0 / self.rate_hz
        max_dq = self.max_joint_velocity * dt  # max change per tick
        # Watchdog return: fixed 1.0 rad/s (~57 deg/s) with exponential decel
        watchdog_return_speed = 1.0  # rad/s — visibly smooth over ~1.5s
        watchdog_max_dq = watchdog_return_speed * dt
        watchdog_approach_alpha = 0.03  # per tick → proportional decel near home

        # Timing stats (updated every 5 seconds)
        _t_ik_sum = 0.0
        _t_servo_sum = 0.0
        _t_loop_sum = 0.0
        _loop_count = 0
        _last_stats = time.monotonic()

        logger.info(f"Control loop started (max {self.rate_hz}Hz, event-driven)")

        while self._running:
            # Adaptive wait: idle longer when no VR data has been received
            if self._last_pose_time == 0:
                self._pose_event.wait(timeout=1.0)
            else:
                self._pose_event.wait(timeout=dt)
            self._pose_event.clear()

            loop_start = time.monotonic()

            with self._lock:
                pose = self._latest_pose

            if self._moving_to_position or self.user_paused:
                continue

            # Pose watchdog: if no VR pose received for watchdog_timeout seconds,
            # smoothly return toward home position to prevent unsafe drift.
            # Uses _watchdog_home_q (the physical home from startup) rather than
            # ik._home_joint_angles which gets overwritten by recalibration.
            if (self.watchdog_timeout > 0
                    and self._last_pose_time > 0
                    and self._smoothed_q is not None
                    and self._watchdog_home_q is not None):
                pose_age = loop_start - self._last_pose_time
                if pose_age > self.watchdog_timeout:
                    if not self._watchdog_active:
                        logger.warning(
                            f"Pose watchdog triggered ({pose_age:.1f}s without VR data) "
                            f"— returning to home")
                        self._watchdog_active = True
                    # Blend toward physical home with proportional deceleration:
                    # moves at watchdog_return_speed initially, then smoothly
                    # decelerates as it approaches home (exponential decay).
                    home_q = self._watchdog_home_q
                    new_q = {}
                    at_home = True
                    for name in JOINT_NAMES:
                        current_val = self._smoothed_q.get(name, 0.0)
                        home_val = home_q.get(name, 0.0)
                        delta = home_val - current_val
                        if abs(delta) > 0.001:
                            at_home = False
                        # Proportional step for smooth deceleration
                        step = delta * watchdog_approach_alpha
                        # Clamp to max return speed for safety
                        if abs(step) > watchdog_max_dq:
                            step = math.copysign(watchdog_max_dq, delta)
                        new_q[name] = current_val + step
                    self._smoothed_q = new_q
                    if not at_home and self.servo and self.servo.is_connected:
                        self.servo.write_joint_angles(new_q)
                    if at_home:
                        # Already at home — idle until new VR data
                        self._pose_event.wait(timeout=0.5)
                        self._pose_event.clear()
                    continue
                elif self._watchdog_active:
                    # Headset back on: recalibrate so robot starts from
                    # current (home) position with current VR pose as neutral.
                    self.calibrate_soft()
                    logger.info("Pose watchdog cleared — VR data resumed, recalibrated")
                    self._watchdog_active = False

            if pose is not None and self.ik is not None:
                q_vr = pose.get("q")
                p_vr = pose.get("p", {"x": 0, "y": 0, "z": 0})

                if q_vr is not None:
                    t0 = time.monotonic()
                    try:
                        if self.position_tracking:
                            # Full IK: orientation + position (all 6 joints)
                            target = self.ik.vr_pose_to_target(
                                q_vr, p_vr, position_tracking=True)
                            q_solved, success = self.ik.solve(target, self._smoothed_q)
                        else:
                            # Orientation only: direct wrist mapping (joints 4-6)
                            q_solved, success = self.ik.solve_orientation(q_vr, p_vr)
                    except Exception as e:
                        logger.warning(f"VR->joints failed: {e}")
                        q_solved, success = None, False

                    if q_solved is not None:
                        t_ik = time.monotonic() - t0

                        if success:
                            self._consecutive_failures = 0
                        else:
                            self._consecutive_failures += 1
                            now = time.monotonic()
                            if now - self._last_failure_log > 1.0:
                                logger.warning(
                                    f"IK failure (consecutive: {self._consecutive_failures})")
                                self._last_failure_log = now

                        # Apply IK result through EMA + velocity clamp
                        if self._smoothed_q is not None:
                            alpha = 1.0 - self.smoothing
                            new_q = {}
                            for name in JOINT_NAMES:
                                target_val = q_solved.get(name, 0.0)
                                current_val = self._smoothed_q.get(name, 0.0)

                                smoothed = current_val + alpha * (target_val - current_val)

                                delta = smoothed - current_val
                                if abs(delta) > max_dq:
                                    smoothed = current_val + math.copysign(max_dq, delta)

                                new_q[name] = smoothed

                            self._smoothed_q = new_q

                            # 2. Write to servos
                            t1 = time.monotonic()
                            if self.servo and self.servo.is_connected:
                                self.servo.write_joint_angles(new_q)
                            t_servo = time.monotonic() - t1

                            # Accumulate timing stats
                            _t_ik_sum += t_ik
                            _t_servo_sum += t_servo
                            _t_loop_sum += time.monotonic() - loop_start
                            _loop_count += 1

            # Print timing stats every 5 seconds
            now = time.monotonic()
            if _loop_count > 0 and now - _last_stats >= 5.0:
                n = _loop_count
                hz = n / (now - _last_stats)
                ik_ms = _t_ik_sum / n * 1000
                servo_ms = _t_servo_sum / n * 1000
                logger.info(
                    f"Control loop: {hz:.0f}Hz | "
                    f"IK={ik_ms:.1f}ms | "
                    f"servo={servo_ms:.1f}ms | "
                    f"total={_t_loop_sum / n * 1000:.1f}ms"
                )
                # Update telemetry for server to read
                self._telemetry = {
                    'ik_ms': ik_ms,
                    'servo_ms': servo_ms,
                    'control_hz': hz,
                    'ik_failures': self._consecutive_failures,
                }
                _t_ik_sum = _t_servo_sum = _t_loop_sum = 0.0
                _loop_count = 0
                _last_stats = now

            # Rate limiting: don't exceed rate_hz
            elapsed = time.monotonic() - loop_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def stop(self):
        """Stop the control loop."""
        self._running = False

    def disconnect(self):
        """Move to resting position, then disconnect from the robot."""
        self.stop()
        if self.servo is not None and self.servo.is_connected:
            self._move_to_named_position(self.resting_path, "resting")
            self.servo.disconnect()
            logger.info("Robot disconnected.")


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="TeleHead: VR head tracking -> CamBot IK camera control")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Robot serial port")
    parser.add_argument("--no-robot", action="store_true", help="Run without robot")
    parser.add_argument("--no-camera", action="store_true", help="Run without camera")
    parser.add_argument("--no-zed", action="store_true",
                        help="Use fallback camera instead of ZED SDK")
    parser.add_argument("--smoothing", type=float, default=0.08,
                        help="EMA smoothing (0=none, 0.99=max)")
    parser.add_argument("--rate", type=float, default=100.0, help="Control loop Hz")
    parser.add_argument("--max-joint-vel", type=float, default=20.0,
                        help="Max joint velocity (rad/s)")
    parser.add_argument("--position-scale", type=float, default=1.0,
                        help="Scale factor for VR head translation (default: 1.0)")
    parser.add_argument("--max-pos-delta", type=float, default=None,
                        help="Max position delta from home in meters (safety sphere radius, default: 0.15; "
                             "auto-disabled when --workspace-bounds is set)")
    parser.add_argument("--workspace-bounds", type=str, default=None,
                        help="Workspace bounding box as 'xmin,xmax,ymin,ymax,zmin,zmax' (meters, robot frame)")
    parser.add_argument("--watchdog-timeout", type=float, default=3.0,
                        help="Seconds without VR pose before returning to home (0=disabled, default: 3.0)")
    parser.add_argument("--resolution", default="720p", choices=["vga", "720p", "1080p", "2k"],
                        help="Camera resolution (default: 720p)")
    parser.add_argument("--camera-fps", type=int, default=None,
                        help="Camera FPS (default: auto based on resolution)")
    parser.add_argument("--jpeg-quality", type=int, default=85,
                        help="JPEG encoding quality 1-100 (default: 85)")
    parser.add_argument("--no-webrtc", action="store_true",
                        help="Disable WebRTC video (use WebSocket JPEG only)")
    parser.add_argument("--server-port", type=int, default=8080, help="HTTPS server port")
    parser.add_argument("--save-home", action="store_true",
                        help="Save current position as home and exit")
    parser.add_argument("--save-resting", action="store_true",
                        help="Save current position as resting and exit")
    parser.add_argument("--home", default=None,
                        help="Path to home_position.json")
    parser.add_argument("--resting", default=None,
                        help="Path to resting_position.json")
    args = parser.parse_args()

    home_path = args.home or str(CALIBRATION_DIR / "home_position.json")
    resting_path = args.resting or str(CALIBRATION_DIR / "resting_position.json")

    # --- Save position modes ---
    if args.save_home or args.save_resting:
        from cambot.servo.controller import CamBotServo, save_home_position
        servo = CamBotServo.connect(args.port)
        try:
            if args.save_home:
                save_home_position(servo, home_path)
            if args.save_resting:
                save_home_position(servo, resting_path)
        finally:
            servo.disconnect()
        return

    from cambot.teleop import server as teleop_server

    # Parse workspace bounds (format: "xmin,xmax,ymin,ymax,zmin,zmax")
    workspace_bounds = None
    if args.workspace_bounds:
        try:
            vals = [float(v) for v in args.workspace_bounds.split(",")]
            if len(vals) != 6:
                raise ValueError("need exactly 6 values")
            workspace_bounds = {
                'x': (vals[0], vals[1]),
                'y': (vals[2], vals[3]),
                'z': (vals[4], vals[5]),
            }
            logger.info(f"Workspace bounds: x=[{vals[0]:.3f},{vals[1]:.3f}] "
                        f"y=[{vals[2]:.3f},{vals[3]:.3f}] z=[{vals[4]:.3f},{vals[5]:.3f}]")
        except ValueError as e:
            logger.error(f"Invalid --workspace-bounds '{args.workspace_bounds}': {e}")
            sys.exit(1)

    # Resolve max_pos_delta: default 0.15m unless workspace_bounds is set
    max_pos_delta = args.max_pos_delta
    if max_pos_delta is None:
        if workspace_bounds is None:
            max_pos_delta = 0.15  # safe default
        # else: workspace_bounds provided without explicit --max-pos-delta → no sphere

    # --- TeleHead setup ---
    telehead = TeleHead(
        robot_port=args.port,
        use_robot=not args.no_robot,
        smoothing=args.smoothing,
        max_joint_velocity=args.max_joint_vel,
        rate_hz=args.rate,
        home_path=home_path,
        resting_path=resting_path,
        position_scale=args.position_scale,
        workspace_bounds=workspace_bounds,
        max_position_delta=max_pos_delta,
        watchdog_timeout=args.watchdog_timeout,
    )
    telehead.connect()

    # Wire telehead ref so server can read telemetry
    teleop_server.telehead_ref = telehead

    # Register head pose callback
    teleop_server.on_head_pose(telehead.on_head_pose)

    # Debug printer
    last_print = [0.0]
    pose_count = [0]

    def debug_print(pose):
        pose_count[0] += 1
        now = time.monotonic()
        if now - last_print[0] < 1.0:
            return
        last_print[0] = now
        q = pose.get("q", {})
        p = pose.get("p", {})
        q_tuple = (q.get("x", 0), q.get("y", 0), q.get("z", 0), q.get("w", 1))
        yaw, pitch, roll = quat_to_euler(q_tuple)
        ik_status = f"fail={telehead._consecutive_failures}" if telehead._consecutive_failures > 0 else "ok"

        if telehead.ik and telehead.ik._vr_neutral_inv is not None:
            pos_state = "ON" if telehead.position_tracking else "OFF"
            d = telehead.ik._last_p_delta_robot
            delta_str = f"pos={pos_state} delta=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f})m"
            cal = "CAL"
        else:
            delta_str = "uncalibrated(press Enter)"
            cal = "RAW"

        logger.info(
            f"VR[{cal}] yaw={yaw:+6.1f} pitch={pitch:+6.1f} roll={roll:+6.1f} "
            f"{delta_str} IK={ik_status} [{pose_count[0]}]"
        )

    teleop_server.on_head_pose(debug_print)

    # --- WebRTC setup ---
    webrtc_enabled = False
    if not args.no_webrtc:
        webrtc_enabled = teleop_server.init_webrtc()
    else:
        logger.info("WebRTC disabled (--no-webrtc)")

    # --- Camera setup ---
    if not args.no_camera:
        use_zed = not args.no_zed
        if not telehead._create_and_start_camera(
            args.resolution, use_zed, args.jpeg_quality, args.camera_fps
        ):
            logger.error("Failed to open camera")
            sys.exit(1)
    else:
        logger.info("Running without camera (--no-camera)")

    # --- Robot control loop ---
    if telehead.servo is not None:
        telehead.servo.set_torque(True)
        telehead._move_to_named_position(home_path, "home")
        # Re-read actual position and update IK home to match where the
        # robot physically settled (servos may not reach exact target under load)
        if telehead.ik:
            actual_home = telehead.servo.read_joint_angles()
            telehead.ik.set_home(actual_home)
            telehead._smoothed_q = actual_home.copy()
            telehead._watchdog_home_q = actual_home.copy()
            logger.info(f"IK home set from actual servo position: {actual_home}")
    control_thread = threading.Thread(target=telehead.control_loop, daemon=True)
    control_thread.start()

    # --- Calibration input thread ---
    def calibration_input_loop():
        while telehead._running:
            try:
                readable, _, _ = select.select([sys.stdin], [], [], 1.0)
                if readable:
                    line = sys.stdin.readline()
                    if line == "":
                        break
                    line = line.strip().lower()
                    if line == "" or line == "c":
                        if telehead.calibrate_neutral():
                            print("  >>> Homed. Move your head to control.")
                        else:
                            print("  >>> No head pose data yet. Connect Quest first.")
                    elif line == "p":
                        telehead.toggle_position_tracking()
                        state = "ON" if telehead.position_tracking else "OFF"
                        print(f"  >>> Position tracking: {state}")
                    elif line == "l":
                        telehead.toggle_user_pause()
                        state = "LOCKED" if telehead.user_paused else "UNLOCKED"
                        print(f"  >>> Robot: {state}")
            except (EOFError, KeyboardInterrupt):
                break

    cal_thread = threading.Thread(target=calibration_input_loop, daemon=True)
    cal_thread.start()

    # --- HTTPS server ---
    from aiohttp import web

    app = web.Application()
    app.router.add_get("/", teleop_server.index)
    app.router.add_get("/ws", teleop_server.websocket_stream)

    # Initialize shared frame event and bind to capture on startup
    async def on_startup(app):
        loop = asyncio.get_running_loop()
        telehead._event_loop = loop
        event = teleop_server.init_frame_event(loop)
        cap_ref = teleop_server.camera_capture
        if cap_ref is not None and hasattr(cap_ref, 'set_external_event'):
            cap_ref.set_external_event(loop, event)
    app.on_startup.append(on_startup)

    local_ip = teleop_server.get_local_ip()

    cert_dir = str(CALIBRATION_DIR)
    os.makedirs(cert_dir, exist_ok=True)
    cert_file = os.path.join(cert_dir, "cert.pem")
    key_file = os.path.join(cert_dir, "key.pem")
    teleop_server.generate_self_signed_cert(cert_file, key_file, local_ip)

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(cert_file, key_file)

    print("\n" + "=" * 60)
    print(f"  CamBot TeleHead: https://{local_ip}:{args.server_port}")
    print(f"  Robot: {'connected on ' + args.port if not args.no_robot else 'DISABLED'}")
    cam_info = 'DISABLED'
    if not args.no_camera and telehead._camera_config:
        cfg = telehead._camera_config
        cam_type = 'ZED Mini' if cfg['use_zed'] else 'fallback'
        cam_info = f"{cam_type} {cfg['resolution']}@{cfg['fps']}fps q{cfg['quality']}"
    print(f"  Camera: {cam_info}")
    webrtc_label = 'enabled (H.264)' if webrtc_enabled else 'disabled (WS JPEG only)'
    print(f"  WebRTC: {webrtc_label}")
    print(f"  IK: {'active' if telehead.ik else 'disabled'}")
    # Safety settings
    safety_parts = []
    if args.watchdog_timeout > 0:
        safety_parts.append(f"watchdog={args.watchdog_timeout:.1f}s")
    if args.max_pos_delta is not None:
        safety_parts.append(f"max_delta={args.max_pos_delta:.3f}m")
    if workspace_bounds:
        safety_parts.append("workspace_bounds=ON")
    print(f"  Safety: {', '.join(safety_parts) if safety_parts else 'defaults'}")
    print()
    print("  Open the URL on Quest 3, enter VR, then:")
    print("  Press ENTER to home (capture neutral head position).")
    print("  Press P+ENTER to toggle position tracking (default: OFF).")
    print("  Press L+ENTER to lock/unlock robot (pause control).")
    print("  Press Ctrl+C to quit.")
    print("=" * 60 + "\n")

    try:
        web.run_app(app, host="0.0.0.0", port=args.server_port,
                    ssl_context=ssl_ctx, shutdown_timeout=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Ignore further Ctrl+C during shutdown
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print("\nShutting down...")
        # Stop control loop and wait for thread to exit
        telehead.stop()
        control_thread.join(timeout=2.0)
        # Move to resting position and disconnect (synchronous, ~2s)
        telehead.disconnect()
        print("TeleHead stopped.")


if __name__ == "__main__":
    main()
