#!/usr/bin/env python3
"""IK solver for StereoBot using ikpy + URDF.

Computes inverse kinematics for the 6-DOF arm to position/orient the
ZED Mini camera mount based on VR head tracking input.

Standalone usage:
    python ik_solver.py                    # FK→IK round-trip test
    python ik_solver.py --home FILE        # test with saved home position
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

URDF_PATH = Path(__file__).parent.parent / "urdf" / "stereobot.urdf"

JOINT_NAMES = [
    "base_yaw", "shoulder_pitch", "elbow_pitch",
    "wrist_pitch", "wrist_yaw", "camera_roll",
]

# VR (WebXR: Y-up, -Z forward) to Robot (Z-up, +X forward) rotation matrix.
# This is a fixed frame transform applied to VR deltas before IK.
# WebXR: +X=right, +Y=up, +Z=backward
# Robot:  +X=forward, +Y=left, +Z=up
# R maps VR axes to robot axes: robot_x = -vr_z, robot_y = -vr_x, robot_z = vr_y
R_VR_TO_ROBOT = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
], dtype=float)

# Fixed offset from camera_roll (zed_mini_mount) to camera sensor midpoint (meters).
# The IK chain ends at camera_roll; this tip vector places the end-effector
# at the midpoint between the two ZED Mini camera sensors (~3cm forward).
# This way the VR eye midpoint maps directly to the robot camera midpoint.
CAMERA_MIDPOINT_OFFSET = [-0.03, 0, 0]

# Offset from neck pivot to eye midpoint in the head's local frame (meters).
# Set to zero to disable the neck-pivot correction (use VR eye pose directly).
NECK_TO_EYE_LOCAL = np.array([0.0, 0.0, 0.0])


def quat_to_rotation_matrix(q: dict) -> np.ndarray:
    """Convert quaternion {x, y, z, w} to 3x3 rotation matrix."""
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def pose_to_4x4(rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    """Build 4x4 homogeneous transform from 3x3 rotation + 3-vector position."""
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = position
    return T


def vr_eye_to_neck(R_eye: np.ndarray, p_eye: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Transform VR eye-midpoint pose to neck-pivot pose.

    The VR headset reports the eye midpoint. When the head rotates around
    the neck, the eyes sweep an arc but the neck pivot stays fixed.
    This removes that arc so pure rotation produces zero position delta.
    """
    p_neck = p_eye - R_eye @ NECK_TO_EYE_LOCAL
    return R_eye, p_neck  # orientation is the same (rigid head)


class StereoBotIK:
    """IK solver for the StereoBot arm."""

    def __init__(self, urdf_path: str | Path | None = None, position_scale: float = 1.0,
                 validate_interval: int = 5):
        import warnings
        import ikpy.chain
        import scipy.optimize

        # Monkey-patch scipy.optimize.least_squares for real-time IK at 100Hz.
        #
        # ikpy calls scipy.optimize.least_squares internally but doesn't expose
        # solver parameters (tolerances, max evaluations). For real-time teleop,
        # the default tolerances (1e-8) and unlimited evaluations are far too
        # expensive. This patch sets looser tolerances (1e-4) and caps evaluations
        # at 80, reducing IK solve time from ~5ms to ~1ms.
        #
        # Trade-offs:
        # - Process-global: affects ALL scipy.optimize.least_squares calls in this
        #   process, not just ikpy. Acceptable since this is a dedicated teleop process.
        # - The _original reference is preserved on the patched function for testing
        #   or restoration: scipy.optimize.least_squares._original
        # - EMA smoothing + velocity clamping in the control loop absorb any
        #   slight accuracy loss from looser tolerances.
        if not hasattr(scipy.optimize.least_squares, '_patched'):
            _orig_ls = scipy.optimize.least_squares
            def _fast_ls(*args, **kwargs):
                kwargs.setdefault('max_nfev', 80)
                kwargs.setdefault('ftol', 1e-4)
                kwargs.setdefault('xtol', 1e-4)
                kwargs.setdefault('gtol', 1e-4)
                return _orig_ls(*args, **kwargs)
            _fast_ls._patched = True
            _fast_ls._original = _orig_ls
            scipy.optimize.least_squares = _fast_ls

        urdf_path = urdf_path or URDF_PATH

        # Load full URDF chain, then truncate at camera_roll and append a
        # fixed tip link at the camera sensor midpoint.  This makes the IK
        # end-effector the point between the two ZED Mini cameras, matching
        # the VR headset's eye midpoint directly.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            full_chain = ikpy.chain.Chain.from_urdf_file(
                str(urdf_path),
                base_elements=["base_link"],
                name="stereobot",
            )

        # Truncate: keep links up to and including camera_roll
        cam_idx = None
        for i, link in enumerate(full_chain.links):
            if hasattr(link, 'name') and link.name == 'camera_roll':
                cam_idx = i
                break
        if cam_idx is None:
            raise RuntimeError("camera_roll not found in URDF chain")

        from ikpy.link import URDFLink
        tip = URDFLink(
            name='camera_midpoint',
            origin_translation=CAMERA_MIDPOINT_OFFSET,
            origin_orientation=[0, 0, 0],
            joint_type='fixed',
        )
        truncated_links = list(full_chain.links[:cam_idx + 1]) + [tip]

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.chain = ikpy.chain.Chain(truncated_links, name="stereobot")

        # Build active_links_mask: only our 6 revolute joints are active.
        # ikpy index 0 = virtual base (fixed), 1-6 = revolute, 7 = fixed tip.
        n = len(self.chain.links)
        mask = [False] * n
        self._joint_indices = {}
        for i, link in enumerate(self.chain.links):
            if hasattr(link, 'name') and link.name in JOINT_NAMES:
                mask[i] = True
                self._joint_indices[link.name] = i

        self.chain.active_links_mask = np.array(mask)

        self.n_links = n
        self.n_active = sum(mask)

        # Extract joint limits from URDF chain for clamping
        self._joint_limits: dict[str, tuple[float, float]] = {}
        for name, idx in self._joint_indices.items():
            link = self.chain.links[idx]
            if hasattr(link, 'bounds') and link.bounds is not None:
                self._joint_limits[name] = (link.bounds[0], link.bounds[1])

        # Home pose (set via calibration)
        self._home_ee_pose: np.ndarray | None = None  # 4x4
        self._home_joint_angles: np.ndarray | None = None

        # VR neutral pose
        self._vr_neutral_inv: np.ndarray | None = None  # 4x4 inverse

        # Position scaling factor for VR translation
        self._position_scale = position_scale

        # Last valid solution for warm-starting
        self._last_valid_q: np.ndarray | None = None

        # FK validation interval: only validate every Nth solve to save CPU.
        # EMA smoothing + velocity clamping in telehead.py absorb occasional bad results.
        self._validate_interval = validate_interval
        self._solve_count = 0

        # Debug: last computed position delta in robot frame (meters)
        self._last_p_delta_robot: np.ndarray = np.zeros(3)

    def _angles_dict_to_array(self, angles: dict[str, float]) -> np.ndarray:
        """Convert joint angle dict to full ikpy array (with zeros for inactive links)."""
        q = np.zeros(self.n_links)
        for name, rad in angles.items():
            if name in self._joint_indices:
                q[self._joint_indices[name]] = rad
        return q

    def _array_to_angles_dict(self, q: np.ndarray) -> dict[str, float]:
        """Extract actuated joint angles from full ikpy array."""
        return {name: float(q[idx]) for name, idx in self._joint_indices.items()}

    def forward_kinematics(self, joint_angles: dict[str, float]) -> np.ndarray:
        """Compute FK. Returns 4x4 homogeneous transform of end-effector."""
        q = self._angles_dict_to_array(joint_angles)
        return self.chain.forward_kinematics(q)

    def inverse_kinematics(
        self, target_4x4: np.ndarray, q_init: dict[str, float] | None = None
    ) -> dict[str, float]:
        """Compute IK. Returns joint angles dict."""
        initial = self._angles_dict_to_array(q_init) if q_init else np.zeros(self.n_links)

        target_position = target_4x4[:3, 3]
        target_orientation = target_4x4[:3, :3]

        q_result = self.chain.inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation,
            orientation_mode="all",
            initial_position=initial,
        )
        return self._array_to_angles_dict(q_result)

    def set_home(self, q_home: dict[str, float]) -> None:
        """Record home joint angles and compute home FK pose."""
        self._home_joint_angles = q_home.copy()
        self._home_ee_pose = self.forward_kinematics(q_home)
        self._last_valid_q = q_home.copy()
        print(f"IK home set. EE position: {self._home_ee_pose[:3, 3]}")

    def calibrate_neutral_vr(self, vr_quat: dict, vr_pos: dict) -> None:
        """Capture current VR pose as the neutral (zero-delta) reference.

        Transforms from eye midpoint to neck pivot before storing,
        so deltas are computed at the neck — pure head rotation produces
        zero position delta.
        """
        R = quat_to_rotation_matrix(vr_quat)
        p = np.array([vr_pos["x"], vr_pos["y"], vr_pos["z"]])
        R_neck, p_neck = vr_eye_to_neck(R, p)
        T = pose_to_4x4(R_neck, p_neck)
        self._vr_neutral_inv = np.linalg.inv(T)
        print(f"IK VR neutral captured. Neck position: {p_neck}")

    def vr_pose_to_target(self, vr_quat: dict, vr_pos: dict,
                          position_tracking: bool = False) -> np.ndarray:
        """Convert VR head pose to robot end-effector target (4x4).

        Pipeline:
        1. Compute VR delta from neutral: delta = neutral_inv @ current
        2. Transform delta rotation to robot frame
        3. Apply to home EE pose: target = home @ R_mapped
        4. Optionally add position delta (when position_tracking=True)
        """
        if self._home_ee_pose is None:
            raise RuntimeError("Home pose not set. Call set_home() first.")

        # Build current VR pose at neck pivot (removes eye arc from rotation)
        R_vr = quat_to_rotation_matrix(vr_quat)
        p_vr = np.array([vr_pos["x"], vr_pos["y"], vr_pos["z"]])
        R_neck, p_neck = vr_eye_to_neck(R_vr, p_vr)
        T_vr = pose_to_4x4(R_neck, p_neck)

        if self._vr_neutral_inv is not None:
            # Relative pose: delta = neutral_inv * current
            T_delta = self._vr_neutral_inv @ T_vr
        else:
            T_delta = T_vr

        # Extract rotation delta and map to robot frame
        R_delta_vr = T_delta[:3, :3]
        R_delta_robot = R_VR_TO_ROBOT @ R_delta_vr @ R_VR_TO_ROBOT.T

        # Position delta (only when enabled and calibrated)
        if position_tracking and self._vr_neutral_inv is not None:
            p_delta_vr = T_delta[:3, 3]
            p_delta_robot = R_VR_TO_ROBOT @ (p_delta_vr * self._position_scale)
        else:
            p_delta_robot = np.zeros(3)

        self._last_p_delta_robot = p_delta_robot

        # Orientation: apply delta in world frame (left multiply).
        # Position: stays at home + translation delta only.  The rotation
        # happens at the camera, not at the robot base.
        target = self._home_ee_pose.copy()
        target[:3, :3] = R_delta_robot @ self._home_ee_pose[:3, :3]
        target[:3, 3] = self._home_ee_pose[:3, 3] + p_delta_robot

        return target

    def solve_orientation(self, vr_quat: dict, vr_pos: dict) -> tuple[dict[str, float], bool]:
        """Map VR head orientation directly to wrist joints 4-6.

        Decomposes the VR rotation delta (in robot frame) into ZYX Euler
        angles and maps them to the three wrist joints:
          - yaw   (Z) -> wrist_yaw   (axis -Z, negated)
          - pitch  (Y) -> wrist_pitch  (axis +Y, direct)
          - roll  (X) -> camera_roll  (axis -X, negated)

        Joints 1-3 (base_yaw, shoulder_pitch, elbow_pitch) stay at home.
        Returns (joint_angles_dict, success).
        """
        if self._home_joint_angles is None:
            raise RuntimeError("Home pose not set. Call set_home() first.")

        # Build VR delta rotation at neck pivot
        R_vr = quat_to_rotation_matrix(vr_quat)
        p_vr = np.array([vr_pos["x"], vr_pos["y"], vr_pos["z"]])
        R_neck, p_neck = vr_eye_to_neck(R_vr, p_vr)
        T_vr = pose_to_4x4(R_neck, p_neck)

        if self._vr_neutral_inv is not None:
            T_delta = self._vr_neutral_inv @ T_vr
        else:
            T_delta = T_vr

        R_delta_vr = T_delta[:3, :3]
        R_delta_robot = R_VR_TO_ROBOT @ R_delta_vr @ R_VR_TO_ROBOT.T

        # Decompose into ZYX Euler angles (yaw, pitch, roll)
        # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        pitch = math.asin(np.clip(-R_delta_robot[2, 0], -1.0, 1.0))
        cos_pitch = math.cos(pitch)
        if abs(cos_pitch) > 1e-6:
            yaw = math.atan2(R_delta_robot[1, 0], R_delta_robot[0, 0])
            roll = math.atan2(R_delta_robot[2, 1], R_delta_robot[2, 2])
        else:
            # Gimbal lock: pitch ≈ ±90°
            yaw = math.atan2(-R_delta_robot[0, 1], R_delta_robot[1, 1])
            roll = 0.0

        # Start from home, add VR delta to home wrist values.
        # At VR neutral (yaw=pitch=roll=0), joints stay at home orientation.
        q = dict(self._home_joint_angles)
        q["wrist_yaw"] = q.get("wrist_yaw", 0.0) + (-yaw)        # joint axis is -Z
        q["wrist_pitch"] = q.get("wrist_pitch", 0.0) + pitch      # joint axis is +Y
        q["camera_roll"] = q.get("camera_roll", 0.0) + (-roll)    # joint axis is -X

        # Clamp to joint limits
        for name in ("wrist_pitch", "wrist_yaw", "camera_roll"):
            if name in self._joint_limits:
                lo, hi = self._joint_limits[name]
                q[name] = float(np.clip(q[name], lo, hi))

        return q, True

    def solve(
        self, target: np.ndarray, q_init: dict[str, float] | None = None
    ) -> tuple[dict[str, float], bool]:
        """Run IK with periodic FK validation.

        Returns (joint_angles, success).
        On failure, returns last valid angles with success=False.

        FK validation only runs every `validate_interval` solves to reduce CPU.
        On non-validation ticks, the IK result is trusted directly.
        """
        init = q_init or self._last_valid_q or {}
        self._solve_count += 1

        try:
            q = self.inverse_kinematics(target, init)
        except Exception as e:
            import logging
            logging.getLogger(__name__).warning(f"IK exception: {e}")
            if self._last_valid_q is not None:
                return self._last_valid_q.copy(), False
            return {name: 0.0 for name in JOINT_NAMES}, False

        # Skip FK validation on non-validation ticks
        if self._solve_count % self._validate_interval != 0:
            self._last_valid_q = q.copy()
            return q, True

        # Validate: FK on result should be close to target
        fk = self.forward_kinematics(q)

        # Position error
        pos_err = np.linalg.norm(fk[:3, 3] - target[:3, 3])

        # Orientation error (angle of rotation difference)
        R_err = fk[:3, :3].T @ target[:3, :3]
        cos_angle = (np.trace(R_err) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        orient_err = abs(math.acos(cos_angle))

        # Thresholds: orientation is critical (camera gaze direction),
        # position is best-effort (small arm can't always reach).
        if pos_err > 0.20 or orient_err > 0.15:
            if self._last_valid_q is not None:
                return self._last_valid_q.copy(), False
            return q, False

        self._last_valid_q = q.copy()
        return q, True



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="StereoBot IK solver test")
    parser.add_argument("--home", default=None, help="Path to home_position.json")
    args = parser.parse_args()

    ik = StereoBotIK()
    print(f"Loaded URDF chain with {ik.n_links} links, {ik.n_active} active")
    print(f"Joint indices: {ik._joint_indices}")

    # Use home position or zeros
    if args.home and os.path.exists(args.home):
        import json
        with open(args.home) as f:
            home = json.load(f)
        print(f"\nHome from file: {home}")
    else:
        home = {name: 0.0 for name in JOINT_NAMES}
        print(f"\nUsing zero home position")

    # FK at home
    fk = ik.forward_kinematics(home)
    print(f"\nFK at home:")
    print(f"  Position: {fk[:3, 3]}")
    print(f"  Rotation:\n{fk[:3, :3]}")

    # IK back to same pose
    q_result = ik.inverse_kinematics(fk, home)
    print(f"\nIK result:")
    for name in JOINT_NAMES:
        orig = home.get(name, 0.0)
        solved = q_result.get(name, 0.0)
        err = abs(orig - solved)
        print(f"  {name:20s}: orig={math.degrees(orig):+8.2f}  solved={math.degrees(solved):+8.2f}  err={math.degrees(err):.4f} deg")

    # Verify FK of IK result
    fk2 = ik.forward_kinematics(q_result)
    pos_err = np.linalg.norm(fk[:3, 3] - fk2[:3, 3])
    R_err = fk[:3, :3].T @ fk2[:3, :3]
    cos_angle = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    orient_err = math.acos(cos_angle)
    print(f"\nRound-trip FK error:")
    print(f"  Position: {pos_err * 1000:.2f} mm")
    print(f"  Orientation: {math.degrees(orient_err):.4f} deg")
    print("PASS" if pos_err < 0.005 and orient_err < 0.05 else "FAIL")
