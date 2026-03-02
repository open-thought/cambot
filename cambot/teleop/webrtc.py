"""WebRTC video track and peer connection manager.

Provides H.264 (preferred) or VP8 video streaming from the camera capture
to WebRTC clients.  Used alongside the existing WebSocket JPEG path as a
lower-latency alternative.

Requires: aiortc (``uv pip install aiortc``)
"""

from __future__ import annotations

import asyncio
import logging
import time
from fractions import Fraction
from typing import Callable

import numpy as np

from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCSessionDescription,
    RTCIceCandidate,
    RTCConfiguration,
    RTCIceServer,
)
from av import VideoFrame

VIDEO_CLOCK_RATE = 90000


# Raise aiortc's hard-coded bitrate ceilings so the encoder can use more
# bandwidth on fast links (LAN / 5 GHz Wi-Fi).  The constants are checked
# inside the target_bitrate setter, so we patch them before any encoder is
# created.  REMB feedback from the receiver still governs the actual rate.
RAISED_MAX_BITRATE = 20_000_000   # 20 Mbps
try:
    from aiortc.codecs import vpx, h264
    vpx.MAX_BITRATE = RAISED_MAX_BITRATE
    h264.MAX_BITRATE = RAISED_MAX_BITRATE
except Exception:
    pass  # codec modules may not be available

logger = logging.getLogger(__name__)


class CameraStreamTrack(MediaStreamTrack):
    """A video track that reads raw BGR frames from the camera capture."""

    kind = "video"

    # Maximum clock drift before resetting timing (seconds).  Prevents the
    # burst-catch-up problem where the encoder is flooded after any pause.
    MAX_DRIFT = 0.5

    def __init__(self, frame_getter: Callable[[], np.ndarray | None],
                 paused_getter: Callable[[], bool] | None = None,
                 fps: int = 30):
        super().__init__()
        self._frame_getter = frame_getter
        self._paused_getter = paused_getter
        self._fps = fps
        self._frame_count = 0
        self._null_count = 0
        self._drift_resets = 0
        self._pts = 0
        self._pts_increment = VIDEO_CLOCK_RATE // fps  # e.g. 3000 for 30fps
        self._time_base = Fraction(1, VIDEO_CLOCK_RATE)
        self._next_time: float = 0.0  # monotonic time for next frame
        self._interval = 1.0 / fps
        self._last_recv_time: float = 0.0  # for measuring actual delivery rate
        self._last_frame: np.ndarray | None = None  # reuse when paused

    async def recv(self) -> VideoFrame:
        # When video is paused (headset off), throttle to ~1fps with last frame
        # to keep the WebRTC connection alive without wasting CPU on encoding.
        paused = self._paused_getter() if self._paused_getter else False
        if paused:
            await asyncio.sleep(1.0)
            self._pts += self._pts_increment * self._fps  # advance PTS by ~1s
            self._next_time = 0.0  # reset timing for clean resume
            frame = self._last_frame
            if frame is None:
                frame = np.zeros((720, 2560, 3), dtype=np.uint8)
            video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
            video_frame.pts = self._pts
            video_frame.time_base = self._time_base
            return video_frame

        now = time.monotonic()

        if self._next_time == 0.0:
            # First frame — deliver immediately
            self._next_time = now + self._interval
        else:
            wait = self._next_time - now
            if wait < -self.MAX_DRIFT:
                # Clock drifted too far (event loop was busy / encoding stall).
                # Reset timing to avoid burst catch-up that floods the encoder.
                self._drift_resets += 1
                self._next_time = now + self._interval
            elif wait > 0:
                await asyncio.sleep(wait)
            self._next_time += self._interval

        self._pts += self._pts_increment

        frame = self._frame_getter()
        if frame is None:
            self._null_count += 1
            frame = np.zeros((720, 2560, 3), dtype=np.uint8)
        self._last_frame = frame

        self._frame_count += 1

        # Periodic logging: first 5 frames, then every 150 frames (~5s at 30fps)
        if self._frame_count <= 5 or self._frame_count % 150 == 0:
            dt = (now - self._last_recv_time) * 1000 if self._last_recv_time > 0 else 0
            logger.info(
                f"WebRTC track: frame={self._frame_count} "
                f"shape={frame.shape} dt={dt:.0f}ms "
                f"null={self._null_count} drift_resets={self._drift_resets}"
            )
        self._last_recv_time = now

        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = self._pts
        video_frame.time_base = self._time_base
        return video_frame


def _prefer_h264(pc: RTCPeerConnection) -> None:
    """Reorder codecs to prefer H.264 (Quest 3 has hardware decode)."""
    try:
        from aiortc import RTCRtpSender
        caps = RTCRtpSender.getCapabilities("video")
        h264 = [c for c in caps.codecs if c.mimeType == "video/H264"]
        others = [c for c in caps.codecs if c.mimeType != "video/H264"]
        if not h264:
            return
        preferred = h264 + others
        for transceiver in pc.getTransceivers():
            if transceiver.kind == "video":
                transceiver.setCodecPreferences(preferred)
                names = [c.mimeType.split("/")[1] for c in preferred]
                logger.info(f"Codec preference: {', '.join(names)}")
    except Exception as e:
        logger.warning(f"Could not set H.264 preference: {e}")


class WebRTCManager:
    """Manages WebRTC peer connections for video streaming."""

    def __init__(self, frame_getter: Callable[[], np.ndarray | None], fps: int = 30):
        self._frame_getter = frame_getter
        self._fps = fps
        self._peers: dict[int, RTCPeerConnection] = {}
        # Callback: (ws_id, active: bool) -> None
        self.on_connection_state: Callable[[int, bool], None] | None = None

    async def handle_offer(
        self, ws_id: int, sdp: str, sdp_type: str = "offer",
        paused_getter: Callable[[], bool] | None = None,
    ) -> dict:
        """Process a WebRTC offer and return an answer SDP."""
        # Close ALL existing peers — not just for this ws_id.  When a browser
        # tab refreshes, the old WebSocket (different ws_id) may still be open
        # until the heartbeat timeout, leaving a stale encoder running.
        # Only one viewer is supported, so close everything.
        await self.close_all()

        config = RTCConfiguration(iceServers=[
            RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
        ])
        pc = RTCPeerConnection(configuration=config)
        self._peers[ws_id] = pc

        # Create a fresh track for this peer
        track = CameraStreamTrack(
            self._frame_getter,
            paused_getter=paused_getter,
            fps=self._fps,
        )
        pc.addTrack(track)

        @pc.on("connectionstatechange")
        async def on_state_change():
            state = pc.connectionState
            logger.info(f"WebRTC peer {ws_id}: {state}")
            active = state in ("connected", "completed")
            if self.on_connection_state:
                self.on_connection_state(ws_id, active)
            if state == "failed":
                await self.close_peer(ws_id)

        # Set remote offer
        offer = RTCSessionDescription(sdp=sdp, type=sdp_type)
        await pc.setRemoteDescription(offer)

        _prefer_h264(pc)

        # Create and set local answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }

    async def handle_candidate(self, ws_id: int, candidate: dict) -> None:
        """Add an ICE candidate from the client."""
        pc = self._peers.get(ws_id)
        if pc is None:
            return

        try:
            c = RTCIceCandidate(
                sdpMid=candidate.get("sdpMid"),
                sdpMLineIndex=candidate.get("sdpMLineIndex"),
                candidate=candidate.get("candidate", ""),
            )
            await pc.addIceCandidate(c)
        except Exception as e:
            logger.warning(f"Failed to add ICE candidate for peer {ws_id}: {e}")

    async def close_peer(self, ws_id: int) -> None:
        """Close and clean up a peer connection."""
        pc = self._peers.pop(ws_id, None)
        if pc is not None:
            await pc.close()
            logger.info(f"WebRTC peer {ws_id} closed")

    async def close_all(self) -> None:
        """Close all peer connections."""
        for ws_id in list(self._peers):
            await self.close_peer(ws_id)
