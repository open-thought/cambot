#!/usr/bin/env python3
"""WebSocket server for StereoBot VR telepresence.

Streams stereo video (ZED Mini or fallback) to Quest 3 via WebSocket,
receives head pose data for IK-based robot control.

Adapted from HeadRobot/server/server.py.
"""

import asyncio
import json
import logging
import os
import struct
import subprocess
import time

from aiohttp import web

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --- Global state ---
connected_ws = set()
connected_ws_lock = asyncio.Lock()

# Head pose from VR headset
latest_head_pose = None
head_pose_lock = asyncio.Lock()
head_pose_callbacks = []

# Camera reference (set by main entry point)
camera_capture = None

# TeleHead reference (set by telehead.py after construction)
telehead_ref = None

# Frame header: magic(2) + header_len(2) + seq(4) + capture_ts(8) = 16 bytes
_FRAME_MAGIC = 0x5342  # 'SB'
_FRAME_HEADER_LEN = 16
_FRAME_HEADER_STRUCT = struct.Struct('<HHId')  # uint16, uint16, uint32, float64


def _pack_frame(jpeg: bytes, seq: int, capture_ts: float) -> bytes:
    """Prepend a 16-byte header to a JPEG frame."""
    header = _FRAME_HEADER_STRUCT.pack(_FRAME_MAGIC, _FRAME_HEADER_LEN, seq, capture_ts)
    return header + jpeg


async def broadcast(message, exclude=None):
    """Send a JSON string to all connected clients except the sender."""
    async with connected_ws_lock:
        targets = [c for c in connected_ws if c is not exclude]
    for c in targets:
        try:
            await c.send_str(message)
        except Exception:
            pass


def get_head_pose():
    """Get the latest head pose. Returns dict or None."""
    return latest_head_pose


def on_head_pose(callback):
    """Register a callback for head pose updates. callback(pose_dict)."""
    head_pose_callbacks.append(callback)


def _robot_state_msg(th) -> dict:
    """Build a robot_state message from the telehead instance."""
    calibrated = (
        th.ik is not None
        and getattr(th.ik, '_vr_neutral_inv', None) is not None
    )
    return {
        'type': 'robot_state',
        'calibrated': calibrated,
        'position_tracking': th.position_tracking,
    }


async def _ping_loop(ws, state):
    """Send periodic pings to measure RTT."""
    seq = 0
    try:
        while not ws.closed:
            await asyncio.sleep(2.0)
            if ws.closed:
                break
            seq += 1
            server_ts = time.monotonic()
            state['pending_pings'][seq] = server_ts
            try:
                await ws.send_str(json.dumps({
                    'type': 'ping',
                    'server_ts': server_ts,
                    'seq': seq,
                }))
            except (ConnectionResetError, ConnectionError):
                break
    except asyncio.CancelledError:
        pass


async def _telemetry_loop(ws, state):
    """Send periodic telemetry to client."""
    try:
        while not ws.closed:
            await asyncio.sleep(2.0)
            if ws.closed:
                break

            n = state['frame_count']
            drops = state['drop_count']
            avg_send = (state['send_time_sum'] / n * 1000) if n > 0 else 0.0

            msg = {
                'type': 'telemetry',
                'rtt_ms': round(state['rtt_ms'], 1),
                'frame': {
                    'fps_out': round(state['fps_out'], 1),
                    'drops': drops,
                    'budget_ms': round(state['frame_budget'] * 1000, 1),
                    'avg_send_ms': round(avg_send, 1),
                },
            }

            # Add control loop telemetry and robot state if available
            th = telehead_ref
            if th is not None:
                try:
                    t = th.get_telemetry()
                    msg['pose'] = {
                        'ik_ms': round(t.get('ik_ms', 0), 1),
                        'servo_ms': round(t.get('servo_ms', 0), 1),
                        'control_hz': round(t.get('control_hz', 0), 0),
                    }
                    msg['robot'] = {
                        'calibrated': t.get('calibrated', False),
                        'position_tracking': t.get('position_tracking', False),
                    }
                except Exception:
                    pass

            try:
                await ws.send_str(json.dumps(msg))
            except (ConnectionResetError, ConnectionError):
                break
    except asyncio.CancelledError:
        pass


async def _frame_send_loop(ws, state):
    """Send video frames with time-based dropping."""
    cap = camera_capture
    frame_event = getattr(cap, 'frame_event', None) if cap is not None else None

    frame_seq = 0
    last_sent_id = None
    last_send_complete = 0.0
    frame_budget = 1.0 / 60  # start at 60fps budget
    fps_count = 0
    fps_start = time.monotonic()

    try:
        while not ws.closed:
            # Wait for new frame (event-driven) or poll
            if frame_event is not None:
                try:
                    await asyncio.wait_for(frame_event.wait(), timeout=0.1)
                except asyncio.TimeoutError:
                    continue
                frame_event.clear()
            else:
                await asyncio.sleep(0.002)

            cap = camera_capture
            if cap is None:
                continue
            frame = cap.latest_jpeg
            capture_ts = getattr(cap, 'latest_capture_ts', 0.0)

            frame_id = id(frame) if frame is not None else None
            if frame is None or frame_id == last_sent_id:
                continue

            now = time.monotonic()

            # Time-based dropping: skip frame if we're still within budget
            if last_send_complete > 0 and (now - last_send_complete) < frame_budget:
                state['drop_count'] += 1
                continue

            # Pack frame with header
            frame_seq += 1
            payload = _pack_frame(frame, frame_seq, capture_ts)

            send_start = time.monotonic()
            try:
                await asyncio.wait_for(ws.send_bytes(payload), timeout=0.1)
            except asyncio.TimeoutError:
                state['drop_count'] += 1
                continue
            except (ConnectionResetError, ConnectionError):
                break
            send_end = time.monotonic()

            last_sent_id = frame_id
            last_send_complete = send_end
            send_duration = send_end - send_start

            # Adaptive frame budget
            if send_duration > 0.040:
                # Slow send: increase budget (back off)
                frame_budget = min(frame_budget * 1.5, 0.100)
            else:
                # Fast send: decrease budget (speed up)
                frame_budget = max(frame_budget * 0.9, 1.0 / 60)

            # Update telemetry state
            state['frame_count'] += 1
            state['send_time_sum'] += send_duration
            state['frame_budget'] = frame_budget

            # FPS tracking
            fps_count += 1
            fps_elapsed = send_end - fps_start
            if fps_elapsed >= 1.0:
                state['fps_out'] = fps_count / fps_elapsed
                fps_count = 0
                fps_start = send_end

    except asyncio.CancelledError:
        pass


async def websocket_stream(request):
    """WebSocket handler: streams video frames, receives head pose."""
    global latest_head_pose

    ws = web.WebSocketResponse(max_msg_size=0, heartbeat=20.0)
    await ws.prepare(request)

    send_video = request.query.get("no_video", "0") != "1"
    role = "viewer" if send_video else "control-only"
    logger.info(f"Client connected ({role})")

    async with connected_ws_lock:
        connected_ws.add(ws)

    # Shared telemetry state
    state = {
        'frame_count': 0,
        'drop_count': 0,
        'send_time_sum': 0.0,
        'frame_budget': 1.0 / 60,
        'fps_out': 0.0,
        'rtt_ms': 0.0,
        'pending_pings': {},
    }

    # Receive loop for head pose, settings, and pong responses
    async def receive_loop():
        global latest_head_pose
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        msg_type = data.get("type")
                        if msg_type == "head_pose":
                            latest_head_pose = data
                            for cb in head_pose_callbacks:
                                try:
                                    cb(data)
                                except Exception:
                                    pass
                        elif msg_type == "command":
                            action = data.get("action")
                            th = telehead_ref
                            if th is not None:
                                if action == "calibrate":
                                    loop = asyncio.get_event_loop()
                                    result = await loop.run_in_executor(
                                        None, th.calibrate_neutral)
                                    try:
                                        await ws.send_str(json.dumps(
                                            _robot_state_msg(th)))
                                    except Exception:
                                        pass
                                elif action == "toggle_position":
                                    th.position_tracking = not th.position_tracking
                                    try:
                                        await ws.send_str(json.dumps(
                                            _robot_state_msg(th)))
                                    except Exception:
                                        pass
                        elif msg_type == "pong":
                            # Compute RTT from ping response
                            seq = data.get('seq', 0)
                            sent_ts = state['pending_pings'].pop(seq, None)
                            if sent_ts is not None:
                                rtt = (time.monotonic() - sent_ts) * 1000
                                alpha = 0.3
                                if state['rtt_ms'] == 0:
                                    state['rtt_ms'] = rtt
                                else:
                                    state['rtt_ms'] += alpha * (rtt - state['rtt_ms'])
                        elif msg_type == "settings_sync":
                            await broadcast(msg.data, exclude=ws)
                    except (json.JSONDecodeError, KeyError):
                        pass
                elif msg.type == web.WSMsgType.ERROR:
                    break
        except (ConnectionResetError, ConnectionError, asyncio.CancelledError):
            pass

    tasks = [asyncio.ensure_future(receive_loop())]

    if send_video:
        tasks.append(asyncio.ensure_future(_frame_send_loop(ws, state)))
        tasks.append(asyncio.ensure_future(_ping_loop(ws, state)))
        tasks.append(asyncio.ensure_future(_telemetry_loop(ws, state)))

    try:
        if send_video:
            # Wait until any task finishes (usually means disconnect)
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        else:
            await tasks[0]  # just the receive loop
    except (ConnectionResetError, ConnectionError, asyncio.CancelledError):
        pass
    finally:
        for t in tasks:
            t.cancel()
        # Wait for cancellation to propagate
        await asyncio.gather(*tasks, return_exceptions=True)
        async with connected_ws_lock:
            connected_ws.discard(ws)

    logger.info("Client disconnected")
    return ws


async def index(request):
    """Serve the VR client HTML."""
    html_path = os.path.join(os.path.dirname(__file__), "client", "index.html")
    content = open(html_path, encoding="utf-8").read()
    return web.Response(content_type="text/html", text=content)


def get_local_ip():
    """Get the local IP address for LAN access."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def generate_self_signed_cert(cert_file, key_file, local_ip):
    """Generate a self-signed SSL certificate for HTTPS (required by WebXR).

    Reuses existing cert if it already exists and matches the current IP,
    so browser SSL exemptions survive server restarts.
    """
    # Check if existing cert matches current IP
    if os.path.exists(cert_file) and os.path.exists(key_file):
        try:
            result = subprocess.run(
                ["openssl", "x509", "-in", cert_file, "-noout", "-subject"],
                capture_output=True, text=True
            )
            if f"CN = {local_ip}" in result.stdout:
                logger.info(f"Reusing existing SSL certificate for {local_ip}")
                return
        except Exception:
            pass

    logger.info(f"Generating SSL certificate for {local_ip}...")
    subprocess.run([
        "openssl", "req", "-x509", "-newkey", "rsa:2048",
        "-keyout", key_file, "-out", cert_file,
        "-days", "365", "-nodes",
        "-subj", f"/CN={local_ip}",
        "-addext", f"subjectAltName=IP:{local_ip},DNS:localhost,IP:127.0.0.1",
    ], check=True, capture_output=True)
