#!/usr/bin/env python3
"""
TeleHead: VR Head Tracking -> StereoBot IK Camera Control

Streams stereo video from ZED Mini on the StereoBot 6-DOF arm to Meta Quest 3
via WebSocket, receives head orientation + position back, and drives the robot
via full inverse kinematics.

Usage:
    python telehead.py                          # full mode
    python telehead.py --no-robot               # camera streaming only
    python telehead.py --no-camera              # robot control only (IK console output)
    python telehead.py --no-robot --no-camera   # server only (for testing)
    python telehead.py --save-home              # save current position as home and exit
    python telehead.py --save-resting           # save current position as resting and exit
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

from robot_control import load_home_position

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
    """Connects VR head tracking to StereoBot IK camera control."""

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

        self.servo = None
        self.ik = None

        self._lock = threading.Lock()
        self._pose_event = threading.Event()
        self._latest_pose = None
        self._running = False
        self._moving_to_position = False

        # Smoothed joint angles (radians)
        self._smoothed_q: dict[str, float] | None = None

        # IK failure tracking
        self._consecutive_failures = 0
        self._last_failure_log = 0.0

    def connect(self):
        """Connect to the robot and set up IK."""
        if self.use_robot:
            from robot_control import StereoBotServo

            # Use resting position as URDF zero reference so that joint angles
            # match the URDF convention and ikpy joint limits are correct.
            urdf_zero = None
            if self.resting_path and os.path.exists(self.resting_path):
                urdf_zero = load_home_position(self.resting_path)
                if urdf_zero:
                    logger.info(f"URDF zero from resting position: {self.resting_path}")

            self.servo = StereoBotServo.connect(self.robot_port, urdf_zero=urdf_zero)
            logger.info(f"Robot connected on {self.robot_port}")

        if self.use_ik:
            from ik_solver import StereoBotIK, JOINT_NAMES
            self.ik = StereoBotIK(position_scale=self.position_scale)
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

        logger.info(f"Moving to {label} position ({duration:.1f}s)...")
        self._moving_to_position = True
        try:
            self.servo.move_to_raw_position(raw, duration=duration)
        finally:
            self._moving_to_position = False
        logger.info(f"Reached {label} position")
        return True

    def calibrate_neutral(self):
        """Move to home position, then capture VR head pose as neutral reference."""
        with self._lock:
            pose = self._latest_pose
        if pose is None:
            logger.warning("No head pose data yet.")
            return False

        # Move to home position before capturing neutral
        if self._move_to_named_position(self.home_path, "home"):
            # Resync smoothed angles to match the home position
            if self.servo and self.ik:
                raw_home = load_home_position(self.home_path)
                if raw_home:
                    self._smoothed_q = self.servo.raw_to_relative(raw_home)
                    self.ik.set_home(self._smoothed_q)

        if self.ik:
            # Re-read the LATEST VR pose (operator may have moved during home move)
            with self._lock:
                pose = self._latest_pose
            if pose is None:
                logger.warning("Lost head pose during home move.")
                return False
            q = pose.get("q", {})
            p = pose.get("p", {"x": 0, "y": 0, "z": 0})
            self.ik.calibrate_neutral_vr(q, p)

        logger.info("Neutral head pose captured.")
        return True

    def on_head_pose(self, pose: dict):
        """Callback for new head pose data from the server."""
        pose["_recv_ts"] = time.monotonic()
        with self._lock:
            self._latest_pose = pose
        self._pose_event.set()

    def control_loop(self):
        """Main IK control loop. Event-driven with rate_hz ceiling."""
        from ik_solver import JOINT_NAMES

        self._running = True
        dt = 1.0 / self.rate_hz
        max_dq = self.max_joint_velocity * dt  # max change per tick

        # Timing stats (updated every 5 seconds)
        _t_ik_sum = 0.0
        _t_servo_sum = 0.0
        _t_loop_sum = 0.0
        _loop_count = 0
        _last_stats = time.monotonic()

        logger.info(f"Control loop started (max {self.rate_hz}Hz, event-driven)")

        while self._running:
            # Wait for new pose data OR timeout at rate_hz ceiling
            self._pose_event.wait(timeout=dt)
            self._pose_event.clear()

            loop_start = time.monotonic()

            with self._lock:
                pose = self._latest_pose

            if self._moving_to_position:
                continue

            if pose is not None and self.ik is not None:
                q_vr = pose.get("q")
                p_vr = pose.get("p", {"x": 0, "y": 0, "z": 0})

                if q_vr is not None:
                    # 1. Convert VR pose to robot target + IK solve
                    t0 = time.monotonic()
                    try:
                        target = self.ik.vr_pose_to_target(q_vr, p_vr)
                    except Exception as e:
                        logger.warning(f"VR->target failed: {e}")
                        target = None

                    if target is not None:
                        q_solved, success = self.ik.solve(target, self._smoothed_q)
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
                logger.info(
                    f"Control loop: {n / (now - _last_stats):.0f}Hz | "
                    f"IK={_t_ik_sum / n * 1000:.1f}ms | "
                    f"servo={_t_servo_sum / n * 1000:.1f}ms | "
                    f"total={_t_loop_sum / n * 1000:.1f}ms"
                )
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
        description="TeleHead: VR head tracking -> StereoBot IK camera control")
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
    parser.add_argument("--resolution", default="720p", choices=["vga", "720p", "1080p"],
                        help="Camera resolution (default: 720p)")
    parser.add_argument("--camera-fps", type=int, default=None,
                        help="Camera FPS (default: auto based on resolution)")
    parser.add_argument("--jpeg-quality", type=int, default=85,
                        help="JPEG encoding quality 1-100 (default: 85)")
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

    cal_dir = os.path.join(os.path.dirname(__file__), "..", "calibration")
    home_path = args.home or os.path.join(cal_dir, "home_position.json")
    resting_path = args.resting or os.path.join(cal_dir, "resting_position.json")

    # --- Save position modes ---
    if args.save_home or args.save_resting:
        from robot_control import StereoBotServo, save_home_position
        servo = StereoBotServo.connect(args.port)
        try:
            if args.save_home:
                save_home_position(servo, home_path)
            if args.save_resting:
                save_home_position(servo, resting_path)
        finally:
            servo.disconnect()
        return

    import server as teleop_server

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
    )
    telehead.connect()

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
            # Show position delta in robot frame (what IK actually receives)
            d = telehead.ik._last_p_delta_robot
            delta_str = f"robot_delta=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f})m"
            cal = "CAL"
        else:
            delta_str = "pos_transfer=OFF(press Enter)"
            cal = "RAW"

        logger.info(
            f"VR[{cal}] yaw={yaw:+6.1f} pitch={pitch:+6.1f} roll={roll:+6.1f} "
            f"{delta_str} IK={ik_status} [{pose_count[0]}]"
        )

    teleop_server.on_head_pose(debug_print)

    # --- Camera setup ---
    if not args.no_camera:
        from zed_capture import create_capture

        # Auto-select FPS based on resolution if not specified
        cam_fps = args.camera_fps
        if cam_fps is None:
            cam_fps = {"vga": 100, "720p": 60, "1080p": 30}[args.resolution]

        use_zed = not args.no_zed
        if use_zed:
            cap = create_capture(use_zed=True, resolution=args.resolution, fps=cam_fps)
        else:
            res_map = {"vga": (672, 376), "720p": (1280, 720), "1080p": (1920, 1080)}
            w, h = res_map[args.resolution]
            cap = create_capture(use_zed=False, camera_index=0, width=w, height=h, fps=cam_fps)

        import cv2
        cap._encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]

        if not cap.open():
            logger.error("Failed to open camera")
            sys.exit(1)

        teleop_server.camera_capture = cap

        # Start capture loop in daemon thread
        # WebSocket handler reads cap.latest_jpeg directly — no polling thread needed
        cam_thread = threading.Thread(target=cap.capture_loop, daemon=True)
        cam_thread.start()
        logger.info("Camera capture started")
    else:
        logger.info("Running without camera (--no-camera)")

    # --- Robot control loop ---
    if telehead.servo is not None:
        telehead.servo.set_torque(True)
        telehead._move_to_named_position(resting_path, "resting")
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
                    line = line.strip()
                    if line == "" or line.lower() == "c":
                        if telehead.calibrate_neutral():
                            print("  >>> Neutral position calibrated. Move your head to control.")
                        else:
                            print("  >>> No head pose data yet. Connect Quest first.")
            except (EOFError, KeyboardInterrupt):
                break

    cal_thread = threading.Thread(target=calibration_input_loop, daemon=True)
    cal_thread.start()

    # --- HTTPS server ---
    from aiohttp import web

    app = web.Application()
    app.router.add_get("/", teleop_server.index)
    app.router.add_get("/ws", teleop_server.websocket_stream)

    # Bind asyncio event loop to capture for event-driven frame streaming
    cap_ref = teleop_server.camera_capture
    if cap_ref is not None and hasattr(cap_ref, 'set_event_loop'):
        async def on_startup(app):
            cap_ref.set_event_loop(asyncio.get_running_loop())
        app.on_startup.append(on_startup)

    local_ip = teleop_server.get_local_ip()

    cert_dir = os.path.dirname(os.path.abspath(__file__))
    cert_file = os.path.join(cert_dir, "cert.pem")
    key_file = os.path.join(cert_dir, "key.pem")
    teleop_server.generate_self_signed_cert(cert_file, key_file, local_ip)

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(cert_file, key_file)

    print("\n" + "=" * 60)
    print(f"  StereoBot TeleHead: https://{local_ip}:{args.server_port}")
    print(f"  Robot: {'connected on ' + args.port if not args.no_robot else 'DISABLED'}")
    cam_info = 'DISABLED'
    if not args.no_camera:
        cam_type = 'ZED Mini' if not args.no_zed else 'fallback'
        cam_info = f"{cam_type} {args.resolution}@{cam_fps}fps q{args.jpeg_quality}"
    print(f"  Camera: {cam_info}")
    print(f"  IK: {'active' if telehead.ik else 'disabled'}")
    print()
    print("  Open the URL on Quest 3, enter VR, then:")
    print("  Press ENTER here to calibrate neutral head position.")
    print("  Press Ctrl+C to quit.")
    print("=" * 60 + "\n")

    try:
        web.run_app(app, host="0.0.0.0", port=args.server_port, ssl_context=ssl_ctx)
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
