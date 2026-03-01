#!/usr/bin/env python3
"""ZED Mini stereo camera capture module.

Provides side-by-side stereo JPEG frames for WebSocket streaming.
Uses the ZED SDK (pyzed) which provides already-rectified images.

Standalone usage:
    python zed_capture.py                  # capture and save a test image
    python zed_capture.py --fps 30         # set capture FPS
"""

from __future__ import annotations

import asyncio
import threading
import time

import cv2
import numpy as np


class ZedMiniCapture:
    """ZED Mini stereo camera capture using the ZED SDK."""

    def __init__(self, resolution: str = "720p", fps: int = 60):
        import pyzed.sl as sl

        self.sl = sl
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()

        # Resolution
        if resolution == "720p":
            self.init_params.camera_resolution = sl.RESOLUTION.HD720
        elif resolution == "1080p":
            self.init_params.camera_resolution = sl.RESOLUTION.HD1080
        elif resolution == "2k":
            self.init_params.camera_resolution = sl.RESOLUTION.HD2K
        elif resolution == "vga":
            self.init_params.camera_resolution = sl.RESOLUTION.VGA
        else:
            self.init_params.camera_resolution = sl.RESOLUTION.HD720

        self.init_params.camera_fps = fps
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE  # no depth needed

        self._left_image = sl.Mat()
        self._right_image = sl.Mat()
        self._runtime_params = sl.RuntimeParameters()

        # Thread-safe latest frame
        self._latest_jpeg: bytes | None = None
        self._latest_capture_ts: float = 0.0
        self._frame_lock = threading.Lock()
        self._opened = False
        self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]

        # Pre-allocated stereo buffer (set in open() after getting resolution)
        self._stereo_buffer: np.ndarray | None = None

        # Event signaling for async consumers (set via set_event_loop)
        self._frame_event: asyncio.Event | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def open(self) -> bool:
        """Open the ZED camera. Returns True on success."""
        err = self.zed.open(self.init_params)
        if err != self.sl.ERROR_CODE.SUCCESS:
            print(f"ZED open failed: {err}")
            return False

        info = self.zed.get_camera_information()
        res = info.camera_configuration.resolution
        fps = info.camera_configuration.fps
        print(f"ZED Mini opened: {res.width}x{res.height} @ {fps}fps")

        # Pre-allocate stereo frame buffer (eliminates per-frame allocation)
        self._stereo_buffer = np.empty((res.height, res.width * 2, 3), dtype=np.uint8)

        self._opened = True
        return True

    def grab_stereo_jpeg(self, quality: int = 85) -> bytes | None:
        """Grab a single stereo frame as side-by-side JPEG bytes.

        Returns None if grab fails.
        """
        sl = self.sl
        if self.zed.grab(self._runtime_params) != sl.ERROR_CODE.SUCCESS:
            return None

        self.zed.retrieve_image(self._left_image, sl.VIEW.LEFT)
        self.zed.retrieve_image(self._right_image, sl.VIEW.RIGHT)

        # Get numpy arrays (BGRA) and copy into pre-allocated buffer
        left_np = self._left_image.get_data()[:, :, :3]   # drop alpha
        right_np = self._right_image.get_data()[:, :, :3]

        w = left_np.shape[1]
        buf = self._stereo_buffer
        buf[:, :w] = left_np
        buf[:, w:] = right_np

        # Encode to JPEG
        self._encode_params[1] = quality
        _, jpeg = cv2.imencode('.jpg', buf, self._encode_params)
        return jpeg.tobytes()

    def capture_loop(self) -> None:
        """Continuous capture loop. Stores latest frame for streaming.

        Run this in a daemon thread.
        """
        while self._opened:
            jpeg = self.grab_stereo_jpeg()
            if jpeg is not None:
                ts = time.monotonic()
                with self._frame_lock:
                    self._latest_jpeg = jpeg
                    self._latest_capture_ts = ts
                # Wake async consumers waiting on frame_event
                if self._loop is not None and self._frame_event is not None:
                    try:
                        self._loop.call_soon_threadsafe(self._frame_event.set)
                    except RuntimeError:
                        pass  # event loop closed during shutdown

    def set_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Bind an asyncio event loop for frame-ready signaling."""
        self._loop = loop
        self._frame_event = asyncio.Event()

    @property
    def frame_event(self) -> asyncio.Event | None:
        """Asyncio event set when a new frame is available."""
        return self._frame_event

    @property
    def latest_jpeg(self) -> bytes | None:
        """Get the latest captured JPEG frame (thread-safe)."""
        with self._frame_lock:
            return self._latest_jpeg

    @property
    def latest_capture_ts(self) -> float:
        """Get the timestamp of the latest captured frame (thread-safe)."""
        with self._frame_lock:
            return self._latest_capture_ts

    def close(self) -> None:
        """Close the ZED camera."""
        self._opened = False
        self.zed.close()
        print("ZED camera closed.")


class FallbackCapture:
    """Fallback OpenCV capture for testing without ZED SDK.

    Uses a regular USB camera or generates test frames.
    """

    def __init__(self, camera_index: int = 0, width: int = 1280, height: int = 720, fps: int = 30):
        self._cap = None
        self._camera_index = camera_index
        self._width = width
        self._height = height
        self._fps = fps
        self._latest_jpeg: bytes | None = None
        self._latest_capture_ts: float = 0.0
        self._frame_lock = threading.Lock()
        self._opened = False
        self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]

        # Pre-allocated stereo buffer (set in open() after getting resolution)
        self._stereo_buffer: np.ndarray | None = None

        # Event signaling for async consumers (set via set_event_loop)
        self._frame_event: asyncio.Event | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def open(self) -> bool:
        """Open the camera."""
        self._cap = cv2.VideoCapture(self._camera_index)
        if not self._cap.isOpened():
            print(f"Fallback camera {self._camera_index} failed to open. Using test pattern.")
            self._cap = None
            self._opened = True
            return True

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Fallback camera opened: {w}x{h}")

        # Pre-allocate stereo frame buffer
        self._stereo_buffer = np.empty((h, w * 2, 3), dtype=np.uint8)

        self._opened = True
        return True

    def _generate_test_frame(self) -> bytes:
        """Generate a side-by-side test pattern."""
        h, w = 720, 1280
        frame = np.zeros((h, w * 2, 3), dtype=np.uint8)
        # Left eye: cyan
        frame[:, :w] = (200, 200, 0)
        # Right eye: magenta
        frame[:, w:] = (200, 0, 200)
        # Text
        cv2.putText(frame, "LEFT", (w // 2 - 100, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 3)
        cv2.putText(frame, "RIGHT", (w + w // 2 - 100, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 3)
        ts = time.strftime("%H:%M:%S")
        cv2.putText(frame, ts, (w - 150, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        _, jpeg = cv2.imencode('.jpg', frame, self._encode_params)
        return jpeg.tobytes()

    def capture_loop(self) -> None:
        """Continuous capture loop."""
        while self._opened:
            if self._cap is not None:
                ret, frame = self._cap.read()
                if ret:
                    # Duplicate for stereo effect into pre-allocated buffer
                    buf = self._stereo_buffer
                    if buf is not None:
                        w = frame.shape[1]
                        buf[:, :w] = frame
                        buf[:, w:] = frame
                        _, jpeg = cv2.imencode('.jpg', buf, self._encode_params)
                    else:
                        stereo = np.hstack([frame, frame])
                        _, jpeg = cv2.imencode('.jpg', stereo, self._encode_params)
                    ts = time.monotonic()
                    with self._frame_lock:
                        self._latest_jpeg = jpeg.tobytes()
                        self._latest_capture_ts = ts
                    if self._loop is not None and self._frame_event is not None:
                        try:
                            self._loop.call_soon_threadsafe(self._frame_event.set)
                        except RuntimeError:
                            pass  # event loop closed during shutdown
                continue
            else:
                jpeg = self._generate_test_frame()
                ts = time.monotonic()
                with self._frame_lock:
                    self._latest_jpeg = jpeg
                    self._latest_capture_ts = ts
                if self._loop is not None and self._frame_event is not None:
                    try:
                        self._loop.call_soon_threadsafe(self._frame_event.set)
                    except RuntimeError:
                        pass  # event loop closed during shutdown
                time.sleep(1.0 / 30)

    def set_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Bind an asyncio event loop for frame-ready signaling."""
        self._loop = loop
        self._frame_event = asyncio.Event()

    @property
    def frame_event(self) -> asyncio.Event | None:
        """Asyncio event set when a new frame is available."""
        return self._frame_event

    @property
    def latest_jpeg(self) -> bytes | None:
        with self._frame_lock:
            return self._latest_jpeg

    @property
    def latest_capture_ts(self) -> float:
        with self._frame_lock:
            return self._latest_capture_ts

    def close(self) -> None:
        self._opened = False
        if self._cap is not None:
            self._cap.release()
        print("Fallback camera closed.")


def create_capture(use_zed: bool = True, **kwargs) -> ZedMiniCapture | FallbackCapture:
    """Factory: try ZED SDK, fall back to OpenCV."""
    if use_zed:
        try:
            cap = ZedMiniCapture(**kwargs)
            return cap
        except ImportError:
            print("ZED SDK not available, falling back to OpenCV capture")
    return FallbackCapture(**kwargs)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="ZED Mini capture test")
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--resolution", default="720p", choices=["vga", "720p", "1080p", "2k"])
    parser.add_argument("--output", default="test_stereo.jpg")
    parser.add_argument("--no-zed", action="store_true", help="Use fallback capture")
    args = parser.parse_args()

    if args.no_zed:
        cap = FallbackCapture()
    else:
        try:
            cap = ZedMiniCapture(resolution=args.resolution, fps=args.fps)
        except ImportError:
            print("ZED SDK not available, using fallback")
            cap = FallbackCapture()

    if not cap.open():
        print("Failed to open camera")
        exit(1)

    print("Capturing test frame...")
    if isinstance(cap, ZedMiniCapture):
        jpeg = cap.grab_stereo_jpeg()
    else:
        # Use capture loop briefly
        cap._opened = True
        cap.capture_loop.__func__  # just get one frame
        jpeg = cap._generate_test_frame()

    if jpeg:
        with open(args.output, "wb") as f:
            f.write(jpeg)
        print(f"Saved {len(jpeg)} bytes to {args.output}")
    else:
        print("Failed to capture frame")

    cap.close()
