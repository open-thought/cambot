# CamBot

<p align="center">
  <img src="3dprint/CamBot_v2.png" alt="CamBot" width="600">
</p>

6-DOF camera arm for stereo vision, built with Feetech STS3215 servos and a ZED Mini stereo camera. Includes a VR teleop system using WebXR for real-time head tracking from any compatible VR headset (tested with Meta Quest 3).

**[Build & Assembly Guide](docs/BUILD_GUIDE.md)**

## Hardware

- **Body:** 7x 3D printed parts (PLA)
- **Servos:** 6x Feetech STS3215 (serial bus)
- **Camera:** [StereoLabs ZED Mini](https://www.stereolabs.com/en-de/store/products/zed-mini) (63mm baseline stereo)
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
    client/           WebXR stereo viewer (Three.js)
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
docs/                 Build guide, datasheets, and manuals
```

## Setup

Tested on Linux (Ubuntu 24.04). Requires [uv](https://docs.astral.sh/uv/getting-started/installation/) for Python environment and dependency management.

```bash
uv pip install -e .              # install cambot package (editable)
uv pip install -e ".[viz]"       # + URDF visualization (viser)
```

For ZED Mini support, install the [ZED SDK](https://www.stereolabs.com/en-de/developers/release) and the [ZED Python API](https://www.stereolabs.com/docs/development/python/install) (not available on PyPI).

## VR Teleop

Stream stereo video to a WebXR-compatible VR headset and control the robot arm with head tracking:

```bash
./run_teleop.sh                              # full mode
./run_teleop.sh --no-robot                   # camera streaming only
./run_teleop.sh --no-camera                  # robot control only
./run_teleop.sh --no-zed                     # use fallback camera
```

<details>
<summary>All options</summary>

```
usage: cambot-teleop [-h] [--port PORT] [--no-robot] [--no-camera] [--no-zed]
                     [--smoothing SMOOTHING] [--rate RATE]
                     [--max-joint-vel MAX_JOINT_VEL]
                     [--position-scale POSITION_SCALE]
                     [--max-pos-delta MAX_POS_DELTA]
                     [--workspace-bounds WORKSPACE_BOUNDS]
                     [--watchdog-timeout WATCHDOG_TIMEOUT]
                     [--resolution {vga,720p,1080p,2k}]
                     [--camera-fps CAMERA_FPS] [--jpeg-quality JPEG_QUALITY]
                     [--no-webrtc] [--server-port SERVER_PORT]
                     [--save-home] [--save-resting]
                     [--home HOME] [--resting RESTING]

  --port PORT               Robot serial port (default: /dev/ttyACM0)
  --no-robot                Run without robot
  --no-camera               Run without camera
  --no-zed                  Use fallback camera instead of ZED SDK
  --smoothing SMOOTHING     EMA smoothing (0=none, 0.99=max)
  --rate RATE               Control loop Hz
  --max-joint-vel MAX_JOINT_VEL
                            Max joint velocity rad/s (default: 20.0)
  --position-scale POSITION_SCALE
                            Scale factor for VR head translation (default: 1.0)
  --max-pos-delta MAX_POS_DELTA
                            Safety sphere radius in meters (default: 0.15, 0=disable)
  --workspace-bounds WORKSPACE_BOUNDS
                            Bounding box 'xmin,xmax,ymin,ymax,zmin,zmax' (meters)
  --watchdog-timeout WATCHDOG_TIMEOUT
                            Return to home after N seconds without VR pose (default: 3.0, 0=disable)
  --resolution {vga,720p,1080p,2k}
                            Camera resolution (default: 720p)
  --camera-fps CAMERA_FPS   Camera FPS (default: auto based on resolution)
  --jpeg-quality JPEG_QUALITY
                            JPEG encoding quality 1-100 (default: 85)
  --no-webrtc               Disable WebRTC, use WebSocket JPEG only
  --server-port SERVER_PORT HTTPS server port (default: 8080)
  --save-home               Save current position as home and exit
  --save-resting            Save current position as resting and exit
  --home HOME               Path to home_position.json
  --resting RESTING         Path to resting_position.json
```

</details>

The server prints an HTTPS URL in the terminal — open it in the headset browser, then enter VR.

```
============================================================
  CamBot TeleHead: https://192.168.0.103:8080
  Robot: connected on /dev/ttyACM0
  Camera: ZED Mini 720p@60fps q85
  WebRTC: enabled (H.264)
  Safety: watchdog=3.0s

  Press ENTER to home (capture neutral head position).
  Press P+ENTER to toggle position tracking (default: OFF).
  Press L+ENTER to lock/unlock robot (pause control).
  Press Ctrl+C to quit.
============================================================
```

### Controls

**Terminal (host PC):**

| Key | Action |
|-----|--------|
| Enter | Home robot and calibrate neutral head pose |
| P + Enter | Toggle position tracking (orientation-only ↔ full 6-DOF) |
| L + Enter | Toggle pause (lock robot in place) |
| Ctrl+C | Quit (returns robot to resting pose) |

**VR controllers (WebXR gamepad):**

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
- `aiohttp`, `opencv-python`, `aiortc` (teleop server + video + WebRTC)
- Optional: `pyzed` (ZED SDK), `viser` (URDF viz)

## Acknowledgements

- The [TRLC-DK1 Leader](https://github.com/robot-learning-co/trlc-dk1) arm design by Robot Learning Co. served as inspiration for CamBot.
- The [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) project by The Robot Studio — CamBot uses the same servo platform and controller board.
- Special thanks to [Autodesk](https://www.autodesk.com/) for providing Fusion for personal use.
- Special thanks to [FeeTech](https://www.feetechrc.com/) for their great servo motors — I wish I already had these as a kid. ;-)

## License

Apache 2.0 — see [LICENSE](LICENSE) for details.

If you build a CamBot or use this project in your work, I'd love to hear about it! A mention or link back is always appreciated. And if you get your CamBot running, please share a photo or video on [X/Twitter](https://x.com) — tag [@neurosp1ke](https://x.com/neurosp1ke) so I can see it!
