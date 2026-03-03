# CamBot

<p align="center">
  <img src="3dprint/CamBot_v2.png" alt="CamBot" width="600">
</p>

6-DOF camera arm for stereo vision, built with Feetech STS3215 servos and a ZED Mini stereo camera. Includes a VR teleop system for real-time head tracking from a Meta Quest 3.

**[Build & Assembly Guide](docs/BUILD_GUIDE.md)**

## Hardware

- **Servos:** 6x Feetech STS3215 (serial bus, 1 Mbaud, `/dev/ttyACM0`)
- **Camera:** ZED Mini (63mm baseline stereo)
- **Joints:** base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_yaw, camera_roll

## Project Structure

```
cambot/
  servo/              Shared servo communication layer
    constants.py      Register addresses, motor config, conversions
    protocol.py       Wire protocol (decode/encode, connect, EPROM helpers)
    controller.py     CamBotServo class, save/load calibration
  teleop/             VR teleop application
    app.py            Main entry point (TeleHead control loop)
    server.py         HTTPS + WebSocket server (aiohttp)
    ik_solver.py      IK solver (ikpy + URDF)
    capture.py        ZED Mini / fallback camera capture
    webrtc.py         WebRTC H.264 video track
    client/           Quest 3 VR viewer (Three.js + WebXR)
  tools/              CLI diagnostic and setup tools
    fix_servo_ids.py  Scan and assign servo IDs
    read_params.py    Register dump / read / write
    set_pid.py        Write tested PID parameters to servos
    debug_control.py  Debug TUI with IK visualization
    pid_tuning.py     PID auto-tuner
    visualize_urdf.py URDF visualization (viser)
calibration/          Saved positions and calibration data
urdf/                 URDF model and STL meshes
3dprint/              STL files for 3D printing (mm scale)
docs/                 Datasheets and manuals
```

## Setup

Tested on Linux (Ubuntu 24.04). Requires [uv](https://docs.astral.sh/uv/getting-started/installation/) for Python environment and dependency management.

```bash
uv pip install -e .              # install cambot package (editable)
uv pip install -e ".[viz]"       # + URDF visualization (viser)
uv pip install -e ".[webrtc]"    # + WebRTC streaming (aiortc)
```

For ZED Mini support, install pyzed from the ZED SDK (not PyPI):
```bash
/usr/local/zed/get_python_api.py
```

## VR Teleop

Stream stereo video to a Meta Quest 3 and control the robot arm with head tracking:

```bash
./run_teleop.sh                              # full mode
./run_teleop.sh --no-robot                   # camera streaming only
./run_teleop.sh --no-camera                  # robot control only
./run_teleop.sh --no-zed                     # use fallback camera
```

Open the displayed HTTPS URL on the Quest 3, then enter VR.

### Controls

**Terminal (host PC):**

| Key | Action |
|-----|--------|
| Enter | Home robot and calibrate neutral head pose |
| P + Enter | Toggle position tracking (orientation-only ↔ full 6-DOF) |
| L + Enter | Toggle pause (lock robot in place) |
| Ctrl+C | Quit (returns robot to resting pose) |

**Quest 3 controllers:**

| Button | Action |
|--------|--------|
| A / X | Home and calibrate (soft recalibrate if already homed) |
| B / Y | Toggle position tracking |
| Right squeeze | Toggle pause |
| Left squeeze | Toggle HUD overlay |
| Right thumbstick click | Cycle resolution (VGA → 720p → 1080p → 2K) |
| Left thumbstick click | Toggle transport (WebRTC H.264 ↔ WebSocket JPEG) |

The robot automatically pauses and returns to home when the headset is removed, and resumes when put back on.

### Safety features

Position tracking mode has built-in safety limits:

- **Position delta sphere** (default 15cm): limits end-effector displacement from home. `--max-pos-delta 0.20` for 20cm, or `--max-pos-delta 0` to disable.
- **Workspace bounding box**: hard axis-aligned limits in robot frame. `--workspace-bounds xmin,xmax,ymin,ymax,zmin,zmax` (meters). Auto-disables the default sphere.
- **Pose watchdog** (default 3s): smoothly returns to home if VR data stops (headset removed, connection lost). `--watchdog-timeout 5.0` or `--watchdog-timeout 0` to disable.

Additional safety: EMA smoothing, velocity clamping (20 rad/s), FK validation, torque limiting (90%).

## Tools

All tools have shell script wrappers in the project root:

```bash
./run_fix_servo_ids.sh           # scan/set servo IDs
./run_read_params.sh             # register dump
./run_set_pid.sh                 # write tested PID values to servos
./run_debug_control.sh           # debug TUI with IK
./run_pid_tuning.sh              # PID auto-tuner
./run_visualize_urdf.sh          # URDF 3D viewer (browser)
```

Or run directly via entry points: `uv run cambot-debug-control`

## Dependencies

- Python 3.12+
- `feetech-servo-sdk` (servo communication, imported as `scservo_sdk`)
- `numpy`, `scipy`, `ikpy` (IK solver)
- `aiohttp`, `opencv-python` (teleop server + video)
- Optional: `pyzed` (ZED SDK), `viser` (URDF viz), `aiortc` (WebRTC)

## Acknowledgements

- The [TRLC-DK1 Leader](https://github.com/robot-learning-co/trlc-dk1) arm design by Robot Learning Co. served as inspiration for CamBot.
- Special thanks to [Autodesk](https://www.autodesk.com/) for providing Fusion for personal use.
- Special thanks to [FeeTech](https://www.feetechrc.com/) for their great servo motors — I wish I already had these as a kid. ;-)

## License

Apache 2.0 — see [LICENSE](LICENSE) for details.
