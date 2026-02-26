# StereoBot

6-DOF camera arm for stereo vision, built with Feetech STS3215 servos and a ZED Mini stereo camera.

## Hardware

- **Servos:** 6x Feetech STS3215 (serial bus, 1 Mbaud, `/dev/ttyACM0`)
- **Camera:** ZED Mini (63mm baseline stereo)
- **Joints:** base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_yaw, camera_roll

## Project Structure

```
urdf/               URDF model and STL meshes (meters, +X fwd, +Z up)
3dprint/            STL files for 3D printing (mm scale)
visualize_urdf.py   Interactive 3D visualization (viser) with live servo input
calibrate_limits.py Passive joint limit calibration tool
servo_offset.py     Homing offset / zero-point calibration
servo_test.py       Interactive curses-based servo test (keyboard control)
waypoint_nav.py     Waypoint navigation
```

## Quick Start

**Visualize the URDF** (browser-based, no ROS needed):
```bash
pip install viser
./visualize_urdf.py              # slider mode
./visualize_urdf.py --live       # mirror real servo positions
```

**Calibrate joint limits** (passive, read-only):
```bash
./calibrate_limits.py
```

**Test servos interactively:**
```bash
./servo_test.py
```

## Dependencies

- Python 3.10+
- `scservo_sdk` (Feetech servo communication)
- `viser` + `yourdfpy` (URDF visualization)
