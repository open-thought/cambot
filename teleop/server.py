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
import subprocess

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

    last_sent_id = None  # track by id() to avoid comparing large byte arrays

    # Receive loop for head pose and settings
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
                        elif msg_type == "settings_sync":
                            await broadcast(msg.data, exclude=ws)
                    except (json.JSONDecodeError, KeyError):
                        pass
                elif msg.type == web.WSMsgType.ERROR:
                    break
        except (ConnectionResetError, ConnectionError, asyncio.CancelledError):
            pass

    recv_task = asyncio.ensure_future(receive_loop())

    try:
        if send_video:
            while not ws.closed:
                # Read directly from capture object — no middleman thread
                cap = camera_capture
                frame = cap.latest_jpeg if cap is not None else None

                frame_id = id(frame) if frame is not None else None
                if frame is not None and frame_id != last_sent_id:
                    # Back-pressure: drop frame if TCP buffer backed up
                    transport = ws._req.transport
                    if transport is not None:
                        buf_size = transport.get_write_buffer_size()
                        if buf_size > 500_000:
                            await asyncio.sleep(0.002)
                            continue
                    try:
                        await ws.send_bytes(frame)
                    except ConnectionResetError:
                        break
                    last_sent_id = frame_id
                else:
                    await asyncio.sleep(0.002)
        else:
            await recv_task
    except (ConnectionResetError, ConnectionError, asyncio.CancelledError):
        pass
    finally:
        recv_task.cancel()
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
