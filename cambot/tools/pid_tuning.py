#!/usr/bin/env python3
"""PID Auto-Tuner for CamBot Servos.

Automatically finds optimal PID values for each joint by running step response
tests and optimizing a cost function that rewards fast, smooth, accurate settling.

Tuning poses are user-defined: physically position the robot to safe loaded
positions, capture them with --capture-poses, then tune using those poses.

Usage:
    python -m cambot.tools.pid_tuning --capture-poses
    python -m cambot.tools.pid_tuning --capture-poses --joints shoulder_pitch,elbow_pitch
    python -m cambot.tools.pid_tuning --joints shoulder_pitch --verbose
    python -m cambot.tools.pid_tuning --dry-run --verbose
    python -m cambot.tools.pid_tuning --restore-factory
"""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import dataclass, field

import scservo_sdk as scs

from cambot import CALIBRATION_DIR
from cambot.servo import (
    JOINT_NAMES,
    POS_SIGN_BIT,
    ADDR_TORQUE_ENABLE,
    ADDR_PRESENT_POSITION,
    ADDR_P_COEFFICIENT,
    ADDR_D_COEFFICIENT,
    ADDR_I_COEFFICIENT,
    ADDR_LOCK,
    ADDR_PRESENT_VELOCITY,
    PID_FACTORY_DEFAULTS,
    decode_sm,
)
from cambot.servo.controller import CamBotServo

# Calibration paths
POSES_PATH = str(CALIBRATION_DIR / "pid_tuning_poses.json")


def load_tuning_poses(path: str) -> dict[str, list[dict[str, int]]]:
    """Load per-joint tuning poses from JSON file.

    Returns dict mapping joint name -> list of full robot poses (all 6 joints).
    """
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return json.load(f)


def capture_poses(servo: CamBotServo, joints: list[str], path: str) -> None:
    """Interactive pose capture: disable torque, let user position robot, record."""
    print("=== Pose Capture Mode ===")
    print("Torque disabled. Manually position the robot for each pose.\n")
    servo.set_torque(False)

    # Load existing poses to allow incremental capture
    if os.path.exists(path):
        with open(path) as f:
            all_poses = json.load(f)
        print(f"Loaded existing poses from {path}")
    else:
        all_poses = {}

    for joint in joints:
        print(f"\n--- {joint} ---")
        poses = []
        n = 1
        while True:
            prompt = (f"  Position robot for {joint} pose {n}. "
                      f"Press Enter to capture, 's' to skip joint, 'd' to finish: ")
            resp = input(prompt).strip().lower()
            if resp == "s":
                print(f"  Skipped {joint}.")
                break
            if resp == "d":
                if len(poses) < 2:
                    print(f"  Need at least 2 poses (have {len(poses)}). "
                          "Capture more or 's' to skip.")
                    continue
                break
            # Enter pressed (or anything else) -> capture
            raw = servo.read_raw_positions()
            poses.append(raw)
            vals = "  ".join(f"{k}={v}" for k, v in raw.items())
            print(f"  Captured pose {n}: {vals}")
            n += 1
            if n > 2:
                print("  (Enter=capture more, 'd'=done with this joint)")

        if len(poses) >= 2:
            all_poses[joint] = poses
            print(f"  Saved {len(poses)} poses for {joint}.")

    if all_poses:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(all_poses, f, indent=2)
            f.write("\n")
        print(f"\nPoses saved to {path}")
    else:
        print("\nNo poses captured.")

# --- Cost function weights ---
W_SETTLING = 1.0
W_OVERSHOOT = 5.0
W_SSE = 20.0
W_OSCILLATION = 10.0

# Safety thresholds
SAFETY_OSCILLATION_STEPS = 100  # peak-to-peak
SAFETY_OSCILLATION_CYCLES = 5
SAFETY_POSITION_ERROR = 500     # steps from target
PENALTY_COST = 1e6


@dataclass
class StepResponseData:
    """Raw step response recording."""
    timestamps: list[float] = field(default_factory=list)
    positions: list[int] = field(default_factory=list)
    goal: int = 0
    start: int = 0


@dataclass
class ResponseMetrics:
    """Computed metrics from a step response."""
    settling_time: float = 0.0
    overshoot: float = 0.0
    steady_state_error: float = 0.0
    oscillation_energy: float = 0.0
    is_unstable: bool = False

    @classmethod
    def from_step_response(cls, data: StepResponseData) -> ResponseMetrics:
        """Compute metrics from recorded step response data."""
        if len(data.timestamps) < 10:
            return cls(is_unstable=True)

        step_size = abs(data.goal - data.start)
        if step_size == 0:
            return cls()

        direction = 1 if data.goal > data.start else -1
        t0 = data.timestamps[0]
        times = [t - t0 for t in data.timestamps]
        # Signed error: positive = past target (overshoot)
        errors = [(pos - data.goal) * direction for pos in data.positions]

        # --- Settling time: time until error stays within 5 steps ---
        settle_thresh = 5
        settling_time = times[-1]  # default: never settled
        for i in range(len(times) - 1, -1, -1):
            if abs(errors[i]) > settle_thresh:
                settling_time = times[min(i + 1, len(times) - 1)]
                break
        else:
            settling_time = 0.0  # always within threshold

        # --- Overshoot: max positive error as fraction of step ---
        max_overshoot_steps = max(errors) if errors else 0
        overshoot = max(0, max_overshoot_steps) / step_size

        # --- Steady-state error: mean |error| in last 0.5s ---
        last_05s = [abs(e) for t, e in zip(times, errors) if t > times[-1] - 0.5]
        sse = sum(last_05s) / max(len(last_05s), 1)

        # --- Oscillation energy: sum of squared zero-crossing amplitudes ---
        osc_energy = 0.0
        crossings = []
        for i in range(1, len(errors)):
            if errors[i - 1] * errors[i] < 0:  # sign change
                crossings.append(i)
        # Amplitude between successive crossings
        for j in range(len(crossings) - 1):
            segment = errors[crossings[j]:crossings[j + 1]]
            if segment:
                amp = max(abs(min(segment)), abs(max(segment)))
                osc_energy += amp ** 2

        # --- Stability check ---
        is_unstable = False
        # Check for sustained large oscillation
        if len(crossings) >= SAFETY_OSCILLATION_CYCLES * 2:
            # Check peak-to-peak in last few cycles
            late_crossings = crossings[-SAFETY_OSCILLATION_CYCLES * 2:]
            for j in range(len(late_crossings) - 1):
                seg = errors[late_crossings[j]:late_crossings[j + 1]]
                if seg:
                    pp = max(seg) - min(seg)
                    if pp > SAFETY_OSCILLATION_STEPS:
                        is_unstable = True
                        break

        return cls(
            settling_time=settling_time,
            overshoot=overshoot,
            steady_state_error=sse,
            oscillation_energy=osc_energy,
            is_unstable=is_unstable,
        )


def compute_cost(m: ResponseMetrics) -> float:
    """Compute scalar cost from response metrics."""
    if m.is_unstable:
        return PENALTY_COST
    return (
        W_SETTLING * m.settling_time
        + W_OVERSHOOT * max(0, m.overshoot - 0.02) ** 2
        + W_SSE * m.steady_state_error ** 2
        + W_OSCILLATION * m.oscillation_energy
    )


class PIDTuner:
    """Automated PID tuner for CamBot servos."""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000,
                 dry_run: bool = False, verbose: bool = False,
                 poses_path: str = POSES_PATH):
        self.port = port
        self.baudrate = baudrate
        self.dry_run = dry_run
        self.verbose = verbose
        self.poses_path = poses_path
        self.poses: dict[str, list[dict[str, int]]] = load_tuning_poses(poses_path)
        self.servo: CamBotServo | None = None
        self._ext_sync_read: scs.GroupSyncRead | None = None
        # Results accumulator
        self.results: dict = {}

    def connect(self):
        """Connect to servos and set up sync read."""
        self.servo = CamBotServo.connect(self.port, self.baudrate)
        # Extended GroupSyncRead for position (addr 56, 2 bytes)
        self._ext_sync_read = scs.GroupSyncRead(
            self.servo.ph, self.servo.pkt, ADDR_PRESENT_POSITION, 2
        )
        for name in JOINT_NAMES:
            mid = CamBotServo.MOTOR_IDS[name]
            self._ext_sync_read.addParam(mid)
        print(f"Connected to {self.port}")

    def disconnect(self):
        """Disable torque and close port."""
        if self.servo and self.servo.is_connected:
            self.servo.disconnect()
            print("Disconnected.")

    def read_pid(self, joint: str) -> tuple[int, int, int]:
        """Read current PID coefficients from a joint."""
        mid = CamBotServo.MOTOR_IDS[joint]
        ph, pkt = self.servo.ph, self.servo.pkt
        old_wt = ph.ser.write_timeout
        ph.ser.write_timeout = None
        try:
            p, _, _ = pkt.read1ByteTxRx(ph, mid, ADDR_P_COEFFICIENT)
            d, _, _ = pkt.read1ByteTxRx(ph, mid, ADDR_D_COEFFICIENT)
            i, _, _ = pkt.read1ByteTxRx(ph, mid, ADDR_I_COEFFICIENT)
        finally:
            ph.ser.write_timeout = old_wt
        return (p, d, i)

    def write_pid(self, joint: str, p: int, d: int, i: int):
        """Write PID coefficients to EPROM in a single unlock/lock cycle."""
        if self.dry_run:
            if self.verbose:
                print(f"  [dry-run] Would write {joint}: P={p} D={d} I={i}")
            return

        p = max(0, min(255, p))
        d = max(0, min(255, d))
        i = max(0, min(255, i))
        mid = CamBotServo.MOTOR_IDS[joint]
        ph, pkt = self.servo.ph, self.servo.pkt

        old_wt = ph.ser.write_timeout
        ph.ser.write_timeout = None
        try:
            # Torque off -> unlock -> write P,D,I -> lock -> torque on
            pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.01)
            pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 0)
            time.sleep(0.01)
            pkt.write1ByteTxRx(ph, mid, ADDR_P_COEFFICIENT, p)
            time.sleep(0.01)
            pkt.write1ByteTxRx(ph, mid, ADDR_D_COEFFICIENT, d)
            time.sleep(0.01)
            pkt.write1ByteTxRx(ph, mid, ADDR_I_COEFFICIENT, i)
            time.sleep(0.01)
            pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 1)
            time.sleep(0.01)
            pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
            time.sleep(0.01)
            ph.ser.reset_input_buffer()
        finally:
            ph.ser.write_timeout = old_wt

    def _read_position(self, joint: str) -> int:
        """Read a single joint's position via GroupSyncRead."""
        old_wt = self.servo.ph.ser.write_timeout
        self.servo.ph.ser.write_timeout = None
        try:
            comm = self._ext_sync_read.txRxPacket()
            if comm != scs.COMM_SUCCESS:
                return 0
            mid = CamBotServo.MOTOR_IDS[joint]
            if not self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_POSITION, 2):
                return 0
            raw = self._ext_sync_read.getData(mid, ADDR_PRESENT_POSITION, 2)
            return decode_sm(raw, POS_SIGN_BIT)
        finally:
            self.servo.ph.ser.write_timeout = old_wt

    def move_to_pose(self, pose: dict[str, int], duration: float = 1.0):
        """Smooth move to a full robot pose and settle."""
        self.servo.set_torque(True)
        self.servo.move_to_raw_position(pose, duration=duration)
        time.sleep(0.5)  # settle

    def record_step_response(self, joint: str, start: int, target: int,
                             duration: float = 3.0) -> StepResponseData:
        """Command a step change and record position at ~200Hz."""
        data = StepResponseData(goal=target, start=start)

        # Command the step
        self.servo.write_raw_positions({joint: target})

        t_end = time.monotonic() + duration
        mid = CamBotServo.MOTOR_IDS[joint]
        initial_distance = abs(start - target)
        safety_limit = initial_distance + SAFETY_POSITION_ERROR

        old_wt = self.servo.ph.ser.write_timeout
        self.servo.ph.ser.write_timeout = None
        try:
            while True:
                now = time.monotonic()
                if now >= t_end:
                    break
                comm = self._ext_sync_read.txRxPacket()
                if comm == scs.COMM_SUCCESS:
                    if self._ext_sync_read.isAvailable(mid, ADDR_PRESENT_POSITION, 2):
                        raw = self._ext_sync_read.getData(mid, ADDR_PRESENT_POSITION, 2)
                        pos = decode_sm(raw, POS_SIGN_BIT)
                        data.timestamps.append(now)
                        data.positions.append(pos)

                        # Safety: abort if servo diverges beyond start + margin
                        if abs(pos - target) > safety_limit:
                            if self.verbose:
                                print(f"  SAFETY: pos {pos} diverged from target {target} "
                                      f"(limit {safety_limit}), aborting")
                            break
        finally:
            self.servo.ph.ser.write_timeout = old_wt

        return data

    def evaluate_pid(self, joint: str, p: int, d: int, i: int) -> tuple[float, ResponseMetrics]:
        """Run step response tests with given PID, return (worst_cost, worst_metrics)."""
        poses = self.poses[joint]

        # Write PID
        self.write_pid(joint, p, d, i)

        worst_cost = 0.0
        worst_metrics = ResponseMetrics()

        for idx in range(len(poses) - 1):
            pose_a = poses[idx]
            pose_b = poses[idx + 1]

            # Drive full robot to pose_a
            self.move_to_pose(pose_a, duration=1.0)

            start_pos = self._read_position(joint)
            target_pos = pose_b[joint]

            # Forward step
            data_fwd = self.record_step_response(joint, start_pos, target_pos, duration=3.0)
            metrics_fwd = ResponseMetrics.from_step_response(data_fwd)
            cost_fwd = compute_cost(metrics_fwd)

            if metrics_fwd.is_unstable:
                if self.verbose:
                    print(f"    P={p:3d} D={d:3d} I={i:3d} -> UNSTABLE (fwd pair {idx})")
                return PENALTY_COST, metrics_fwd

            if cost_fwd > worst_cost:
                worst_cost, worst_metrics = cost_fwd, metrics_fwd

            time.sleep(0.3)

            # Reverse step
            actual = self._read_position(joint)
            data_rev = self.record_step_response(joint, actual, pose_a[joint], duration=3.0)
            metrics_rev = ResponseMetrics.from_step_response(data_rev)
            cost_rev = compute_cost(metrics_rev)

            if metrics_rev.is_unstable:
                if self.verbose:
                    print(f"    P={p:3d} D={d:3d} I={i:3d} -> UNSTABLE (rev pair {idx})")
                return PENALTY_COST, metrics_rev

            if cost_rev > worst_cost:
                worst_cost, worst_metrics = cost_rev, metrics_rev

        if self.verbose:
            print(f"    P={p:3d} D={d:3d} I={i:3d} -> cost={worst_cost:.3f}  "
                  f"settle={worst_metrics.settling_time:.2f}s  "
                  f"overshoot={worst_metrics.overshoot:.3f}  "
                  f"sse={worst_metrics.steady_state_error:.1f}  "
                  f"osc={worst_metrics.oscillation_energy:.1f}")

        return worst_cost, worst_metrics

    def optimize_joint(self, joint: str) -> dict:
        """Run coordinate descent optimization for one joint."""
        print(f"\n{'='*60}")
        print(f"Tuning: {joint}")
        print(f"{'='*60}")

        factory = self.read_pid(joint)
        n_evals = 0

        # --- Baseline ---
        print(f"  Baseline (factory P={factory[0]} D={factory[1]} I={factory[2]})...")
        factory_cost, factory_metrics = self.evaluate_pid(joint, *factory)
        n_evals += 1
        print(f"  Baseline cost: {factory_cost:.3f}")

        best_p, best_d, best_i = factory
        best_cost = factory_cost

        # --- Tune P ---
        print(f"\n  Phase 1: Tuning P (D={best_d}, I=0)...")
        p_values = [8, 16, 32, 48, 64, 96, 128, 192]
        best_p_cost = best_cost

        for p in p_values:
            cost, metrics = self.evaluate_pid(joint, p, best_d, 0)
            n_evals += 1
            if cost < best_p_cost:
                best_p_cost = cost
                best_p = p

        # Refine: try +/-25% around best P
        refine_p = [max(1, int(best_p * 0.75)), max(1, int(best_p * 1.25))]
        refine_p = [p for p in refine_p if p not in p_values and 1 <= p <= 255]
        for p in refine_p:
            cost, metrics = self.evaluate_pid(joint, p, best_d, 0)
            n_evals += 1
            if cost < best_p_cost:
                best_p_cost = cost
                best_p = p

        print(f"  Best P: {best_p} (cost={best_p_cost:.3f})")

        # --- Tune D ---
        print(f"\n  Phase 2: Tuning D (P={best_p}, I=0)...")
        d_values = [8, 16, 32, 48, 64, 96, 128]
        best_d_cost = best_p_cost

        for d in d_values:
            cost, metrics = self.evaluate_pid(joint, best_p, d, 0)
            n_evals += 1
            if cost < best_d_cost:
                best_d_cost = cost
                best_d = d

        print(f"  Best D: {best_d} (cost={best_d_cost:.3f})")

        # --- Tune I ---
        print(f"\n  Phase 3: Tuning I (P={best_p}, D={best_d})...")
        i_values = [1, 2, 4, 8, 12, 16, 24, 32]
        best_i_cost = best_d_cost
        best_i = 0
        best_i_metrics = None

        for i_val in i_values:
            cost, metrics = self.evaluate_pid(joint, best_p, best_d, i_val)
            n_evals += 1
            if cost < best_i_cost:
                best_i_cost = cost
                best_i = i_val
                best_i_metrics = metrics
            # Accept smallest I that brings SSE < 3 steps
            if best_i_metrics and best_i_metrics.steady_state_error < 3.0:
                break

        print(f"  Best I: {best_i} (cost={best_i_cost:.3f})")

        # --- Refinement ---
        print(f"\n  Phase 4: Refinement (P~{best_p}, D~{best_d}, I={best_i})...")
        best_final_cost = best_i_cost
        final_metrics = best_i_metrics

        refine_candidates = [
            (max(1, int(best_p * 0.9)), best_d, best_i),
            (min(255, int(best_p * 1.1)), best_d, best_i),
            (best_p, max(1, int(best_d * 0.9)), best_i),
            (best_p, min(255, int(best_d * 1.1)), best_i),
        ]
        for p, d, i in refine_candidates:
            cost, metrics = self.evaluate_pid(joint, p, d, i)
            n_evals += 1
            if cost < best_final_cost:
                best_final_cost = cost
                best_p, best_d, best_i = p, d, i
                final_metrics = metrics

        # Write final best PID
        self.write_pid(joint, best_p, best_d, best_i)

        # Get final metrics if we don't have them
        if final_metrics is None:
            _, final_metrics = self.evaluate_pid(joint, best_p, best_d, best_i)
            n_evals += 1

        print(f"\n  Result: P={best_p} D={best_d} I={best_i}  "
              f"cost={best_final_cost:.3f}  ({n_evals} evals)")
        print(f"  Settling: {final_metrics.settling_time:.2f}s  "
              f"Overshoot: {final_metrics.overshoot:.3f}  "
              f"SSE: {final_metrics.steady_state_error:.1f} steps")

        return {
            "pid": {"P": best_p, "D": best_d, "I": best_i},
            "factory": {"P": factory[0], "D": factory[1], "I": factory[2]},
            "metrics": {
                "settling_time": round(final_metrics.settling_time, 3),
                "overshoot": round(final_metrics.overshoot, 3),
                "sse": round(final_metrics.steady_state_error, 1),
                "oscillation_energy": round(final_metrics.oscillation_energy, 1),
            },
            "factory_metrics": {
                "settling_time": round(factory_metrics.settling_time, 3),
                "overshoot": round(factory_metrics.overshoot, 3),
                "sse": round(factory_metrics.steady_state_error, 1),
            },
            "cost": round(best_final_cost, 3),
            "factory_cost": round(factory_cost, 3),
            "evaluations": n_evals,
        }

    def restore_factory(self):
        """Reset all joints to factory PID defaults."""
        print("Restoring factory PID defaults...")
        for joint in JOINT_NAMES:
            self.write_pid(joint, *PID_FACTORY_DEFAULTS)
            readback = self.read_pid(joint)
            print(f"  {joint}: P={readback[0]} D={readback[1]} I={readback[2]}")
        print("Done.")

    def run(self, joints: list[str] | None = None):
        """Run full tuning session."""
        joints = joints or list(JOINT_NAMES)

        # Validate joint names
        for j in joints:
            if j not in JOINT_NAMES:
                print(f"Unknown joint: {j}")
                print(f"Valid joints: {', '.join(JOINT_NAMES)}")
                return

        # Validate poses exist for all requested joints
        missing = [j for j in joints if j not in self.poses or len(self.poses[j]) < 2]
        if missing:
            print(f"ERROR: No tuning poses (need >=2) for: {', '.join(missing)}")
            print(f"Run with --capture-poses first to define safe positions.")
            print(f"Poses file: {self.poses_path}")
            return

        # Read voltage for the record
        voltage = None
        try:
            old_wt = self.servo.ph.ser.write_timeout
            self.servo.ph.ser.write_timeout = None
            v_raw, _, _ = self.servo.pkt.read1ByteTxRx(
                self.servo.ph, CamBotServo.MOTOR_IDS["base_yaw"], 62
            )
            voltage = round(v_raw * 0.1, 1)
            self.servo.ph.ser.write_timeout = old_wt
            print(f"Supply voltage: {voltage}V")
        except Exception:
            pass

        if voltage and voltage < 6.0:
            print(f"WARNING: Voltage {voltage}V is low! Results may be unreliable.")

        # Enable torque
        self.servo.set_torque(True)

        print(f"\nTuning {len(joints)} joint(s): {', '.join(joints)}")
        if self.dry_run:
            print("DRY RUN: will record telemetry but not write PID values")

        t_start = time.monotonic()
        joint_results = {}

        for joint in joints:
            result = self.optimize_joint(joint)
            joint_results[joint] = result

        elapsed = time.monotonic() - t_start

        self.results = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "voltage": voltage,
            "elapsed_seconds": round(elapsed, 1),
            "dry_run": self.dry_run,
            "joints": joint_results,
        }

        # Summary
        print(f"\n{'='*60}")
        print(f"TUNING COMPLETE  ({elapsed:.0f}s)")
        print(f"{'='*60}")
        for joint, res in joint_results.items():
            pid = res["pid"]
            fac = res["factory"]
            m = res["metrics"]
            fm = res["factory_metrics"]
            improved = "IMPROVED" if res["cost"] < res["factory_cost"] else "no change"
            print(f"  {joint:20s}  "
                  f"P={pid['P']:3d} D={pid['D']:3d} I={pid['I']:3d}  "
                  f"(was P={fac['P']} D={fac['D']} I={fac['I']})  "
                  f"settle={m['settling_time']:.2f}s (was {fm['settling_time']:.2f}s)  "
                  f"sse={m['sse']:.1f} (was {fm['sse']:.1f})  "
                  f"[{improved}]")

    def save_results(self, path: str):
        """Save results to JSON."""
        if not self.results:
            print("No results to save.")
            return
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.results, f, indent=2)
            f.write("\n")
        print(f"Results saved to {path}")


def main():
    cal_dir = str(CALIBRATION_DIR)
    parser = argparse.ArgumentParser(
        description="PID Auto-Tuner for CamBot servos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  %(prog)s --capture-poses                     Capture tuning poses interactively
  %(prog)s --capture-poses --joints shoulder_pitch,elbow_pitch
  %(prog)s --joints shoulder_pitch --verbose    Tune one joint using saved poses
  %(prog)s --dry-run --verbose                  Measure without writing PID
  %(prog)s --verbose                            Full tuning, all joints
  %(prog)s --restore-factory                    Reset to factory defaults
""",
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--joints", type=str, default=None,
                        help="Comma-separated joints to tune (default: all)")
    parser.add_argument("--capture-poses", action="store_true",
                        help="Interactively capture tuning poses and exit")
    parser.add_argument("--poses-file", default=POSES_PATH,
                        help="Path to tuning poses JSON")
    parser.add_argument("--dry-run", action="store_true",
                        help="Measure only, don't write PID")
    parser.add_argument("--restore-factory", action="store_true",
                        help="Reset to factory defaults and exit")
    parser.add_argument("--output", default=os.path.join(cal_dir, "pid_results.json"),
                        help="Results JSON path")
    parser.add_argument("--verbose", action="store_true",
                        help="Print detailed metrics per evaluation")
    args = parser.parse_args()

    joints = None
    if args.joints:
        joints = [j.strip() for j in args.joints.split(",")]

    tuner = PIDTuner(
        port=args.port,
        baudrate=args.baudrate,
        dry_run=args.dry_run,
        verbose=args.verbose,
        poses_path=args.poses_file,
    )

    try:
        tuner.connect()

        if args.capture_poses:
            capture_joints = joints or list(JOINT_NAMES)
            # Validate joint names
            for j in capture_joints:
                if j not in JOINT_NAMES:
                    print(f"Unknown joint: {j}")
                    print(f"Valid joints: {', '.join(JOINT_NAMES)}")
                    return
            capture_poses(tuner.servo, capture_joints, args.poses_file)
            return

        if args.restore_factory:
            tuner.restore_factory()
            return

        tuner.run(joints)
        tuner.save_results(args.output)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        tuner.disconnect()


if __name__ == "__main__":
    main()
