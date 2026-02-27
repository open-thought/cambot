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

# Eye-to-neck offset in VR frame (meters).
# When the head rotates around the neck, the eyes sweep an arc.
# This offset models the lever arm from neck pivot to eye position.
EYE_NECK_OFFSET_VR = np.array([0.0, 0.055, -0.055])


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


class StereoBotIK:
    """IK solver for the StereoBot arm."""

    def __init__(self, urdf_path: str | Path | None = None, position_scale: float = 1.0):
        import warnings
        import ikpy.chain
        import scipy.optimize

        # Speed up IK: ikpy doesn't expose scipy solver params, so we
        # monkey-patch least_squares to use looser tolerances and fewer evals.
        if not hasattr(scipy.optimize.least_squares, '_patched'):
            _orig_ls = scipy.optimize.least_squares
            def _fast_ls(*args, **kwargs):
                kwargs.setdefault('max_nfev', 80)
                kwargs.setdefault('ftol', 1e-4)
                kwargs.setdefault('xtol', 1e-4)
                kwargs.setdefault('gtol', 1e-4)
                return _orig_ls(*args, **kwargs)
            _fast_ls._patched = True
            scipy.optimize.least_squares = _fast_ls

        urdf_path = urdf_path or URDF_PATH

        # First load to discover chain structure (suppress ikpy fixed-link warnings)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            chain_tmp = ikpy.chain.Chain.from_urdf_file(
                str(urdf_path),
                base_elements=["base_link"],
                name="stereobot",
            )

        # Build active_links_mask: only our 6 revolute joints are active.
        # ikpy index 0 = virtual base (fixed), 1-6 = revolute, 7+ = trailing fixed.
        n = len(chain_tmp.links)
        mask = [False] * n
        self._joint_indices = {}
        for i, link in enumerate(chain_tmp.links):
            if hasattr(link, 'name') and link.name in JOINT_NAMES:
                mask[i] = True
                self._joint_indices[link.name] = i

        # Reload with correct mask (no warnings this time)
        self.chain = ikpy.chain.Chain.from_urdf_file(
            str(urdf_path),
            base_elements=["base_link"],
            active_links_mask=mask,
            name="stereobot",
        )

        self.n_links = n
        self.n_active = sum(mask)

        # Home pose (set via calibration)
        self._home_ee_pose: np.ndarray | None = None  # 4x4
        self._home_joint_angles: np.ndarray | None = None

        # VR neutral pose
        self._vr_neutral_inv: np.ndarray | None = None  # 4x4 inverse

        # Position scaling factor for VR translation
        self._position_scale = position_scale

        # Last valid solution for warm-starting
        self._last_valid_q: np.ndarray | None = None

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
        """Capture current VR pose as the neutral (zero-delta) reference."""
        R = quat_to_rotation_matrix(vr_quat)
        p = np.array([vr_pos["x"], vr_pos["y"], vr_pos["z"]])
        T = pose_to_4x4(R, p)
        self._vr_neutral_inv = np.linalg.inv(T)
        print(f"IK VR neutral captured. Position: {p}")

    def vr_pose_to_target(self, vr_quat: dict, vr_pos: dict) -> np.ndarray:
        """Convert VR head pose to robot end-effector target (4x4).

        Pipeline:
        1. Compute VR delta from neutral: delta = neutral_inv @ current
        2. Transform delta rotation to robot frame
        3. Apply to home EE pose: target = home @ R_mapped
        """
        if self._home_ee_pose is None:
            raise RuntimeError("Home pose not set. Call set_home() first.")

        # Build current VR pose
        R_vr = quat_to_rotation_matrix(vr_quat)
        p_vr = np.array([vr_pos["x"], vr_pos["y"], vr_pos["z"]])
        T_vr = pose_to_4x4(R_vr, p_vr)

        if self._vr_neutral_inv is not None:
            # Relative pose: delta = neutral_inv * current
            T_delta = self._vr_neutral_inv @ T_vr
        else:
            T_delta = T_vr

        # Extract rotation delta and map to robot frame
        R_delta_vr = T_delta[:3, :3]
        R_delta_robot = R_VR_TO_ROBOT @ R_delta_vr @ R_VR_TO_ROBOT.T

        # Position delta: linear head movement + rotational eye-neck arc
        p_delta_vr = T_delta[:3, 3]
        if self._vr_neutral_inv is not None:
            # Add eye-neck offset arc from rotation
            eye_arc_vr = R_delta_vr @ EYE_NECK_OFFSET_VR - EYE_NECK_OFFSET_VR
            p_total_vr = p_delta_vr + eye_arc_vr
            p_delta_robot = R_VR_TO_ROBOT @ (p_total_vr * self._position_scale)
        else:
            # Before calibration: only rotation, no position transfer
            p_delta_robot = np.zeros(3)

        self._last_p_delta_robot = p_delta_robot

        # Apply to home in WORLD frame (left multiply) so VR yaw/pitch/roll
        # map to robot world-frame yaw/pitch/roll regardless of home EE orientation.
        target = self._home_ee_pose.copy()
        target[:3, :3] = R_delta_robot @ self._home_ee_pose[:3, :3]
        target[:3, 3] = self._home_ee_pose[:3, 3] + p_delta_robot

        return target

    def solve(
        self, target: np.ndarray, q_init: dict[str, float] | None = None
    ) -> tuple[dict[str, float], bool]:
        """Run IK with validation.

        Returns (joint_angles, success).
        On failure, returns last valid angles with success=False.
        """
        init = q_init or self._last_valid_q or {}

        try:
            q = self.inverse_kinematics(target, init)
        except Exception as e:
            import logging
            logging.getLogger(__name__).warning(f"IK exception: {e}")
            if self._last_valid_q is not None:
                return self._last_valid_q.copy(), False
            return {name: 0.0 for name in JOINT_NAMES}, False

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
