"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Menedżer Kamery (Camera Manager) dla Raspberry Pi 5 - Hybrid Mode.
Camera Manager for Raspberry Pi 5 - Hybrid Mode.

This module handles video input primarily via RTSP (from Mediamtx/rpicam-vid),
but also supports direct local camera access (Picamera2/Libcamera) as a fallback
or for low-latency AI input if RTSP is too slow.
"""

import logging
import os
import threading
import time
from typing import Any

import cv2
import numpy as np

try:
    from picamera2 import Picamera2

    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False

logger = logging.getLogger(__name__)


class CameraManager:
    """
    Menedżer kamery obsługujący strumień wideo.
    Camera Manager handling video stream.

    Modes:
    1. RTSP (Default): Connects to local RTSP server (mediamtx) which streams from rpicam-vid.
    2. Local (Fallback): Tries to open /dev/video0 or libcamerasrc directly.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        """
        Inicjalizuje CameraManager.
        Initializes the Camera Manager.

        Args:
            config (dict[str, Any]): Konfiguracja kamery.
        """
        self.config = config
        self.mode = config.get("mode", "rtsp")
        self.rtsp_url = config.get("rtsp_url", "rtsp://127.0.0.1:8554/camera_ai")
        self.local_device = config.get("local_device", "/dev/video0")

        # Picamera2 specific
        self.picam2: Any | None = None
        self.picam2_resolution = config.get("resolution", (640, 480))
        self.picam2_fps = config.get("fps", 30)

        self.is_running = False
        self.cap: cv2.VideoCapture | None = None
        self.last_frame: np.ndarray | None = None
        self.frame_lock = threading.Lock()
        self.thread: threading.Thread | None = None

        # Performance monitoring
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0

        logger.info(f"✅ CameraManager initialized (Mode: {self.mode})")

    def start(self) -> None:
        """
        Uruchamia wątek pobierania klatek.
        Starts the frame fetching thread.
        """
        if self.is_running:
            return

        self.is_running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info("✅ Camera Capture thread started")

    def _open_camera(self) -> bool:
        """
        Otwiera źródło wideo (RTSP lub Local).
        Opens video source (RTSP or Local).
        """
        try:
            if self.mode == "rtsp":
                # Force TCP for stability
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
                logger.info(f"🔗 Connecting to RTSP: {self.rtsp_url}")
                self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

            elif self.mode == "local":
                logger.info(f"🔗 Opening Local Device: {self.local_device}")
                # Try GStreamer pipeline for libcamera if available, else V4L2
                # GStreamer pipeline for RPi5/Libcamera:
                # libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink
                gst_pipeline = (
                    "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! "
                    "videoconvert ! appsink"
                )

                # Try GStreamer first
                self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                if not self.cap.isOpened():
                    logger.warning("GStreamer pipeline failed, falling back to V4L2")
                    # Fallback to V4L2 (might not work with libcamera stack without v4l2loopback)
                    self.cap = cv2.VideoCapture(0)  # Index 0 usually

            elif self.mode == "picamera2":
                if not PICAMERA2_AVAILABLE:
                    logger.error("❌ Picamera2 not available. Falling back to 'local'")
                    self.mode = "local"
                    return self._open_camera()

                logger.info("🔗 Opening Picamera2...")
                self.picam2 = Picamera2()
                config = self.picam2.create_video_configuration(
                    main={"size": tuple(self.picam2_resolution), "format": "BGR888"},
                    controls={"FrameRate": self.picam2_fps},
                )
                self.picam2.configure(config)
                self.picam2.start()
                logger.info(
                    f"✅ Picamera2 started: {self.picam2_resolution} @ {self.picam2_fps} FPS"
                )
                return True

            else:
                logger.error(f"Unknown camera mode: {self.mode}")
                return False

            if not self.cap.isOpened():
                logger.error("❌ Failed to open video source")
                if self.mode == "rtsp":
                    logger.warning(
                        "💡 DIAGNOSTIC: If you see '404 Not Found', ensure that 'mediamtx.service' "
                        "on the host RPi is running and has the path 'camera_ai' configured. "
                    )
                    # Add detailed hardware check
                    try:
                        import subprocess

                        v4l2_out = subprocess.check_output(
                            ["v4l2-ctl", "--list-devices"]
                        ).decode()
                        logger.info(f"🔍 AVAILABLE VIDEO DEVICES:\n{v4l2_out}")
                        if (
                            "unicam" not in v4l2_out.lower()
                            and "rpivid" not in v4l2_out.lower()
                        ):
                            logger.error(
                                "⚠️ CRITICAL: No RPi Camera hardware detected in V4L2. Check ribbon cable and 'config.txt'."
                            )
                    except Exception:
                        pass
                return False

            # Set buffer size to minimum to reduce latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            logger.info(f"✅ Camera Opened: {int(w)}x{int(h)} @ {fps:.1f} FPS")

            return True

        except Exception as e:
            logger.error(f"❌ Error opening camera: {e}")
            return False

    def _capture_loop(self) -> None:
        """
        Główna pętla wątku.
        Main thread loop.
        """
        retry_delay = 1.0
        while self.is_running:
            if self.mode == "picamera2" and self.picam2:
                frame = self.picam2.capture_array()
                if frame is not None:
                    with self.frame_lock:
                        self.last_frame = frame
                    self._update_fps()
                else:
                    time.sleep(0.01)
                continue

            if self.cap is None or not self.cap.isOpened():
                if not self._open_camera():
                    # Exponential Backoff for Reconnect
                    wait_time = min(retry_delay, 15.0)  # Max 15s wait
                    logger.warning(
                        f"⚠️ Camera open failed. Retrying in {wait_time:.1f}s..."
                    )
                    time.sleep(wait_time)
                    retry_delay *= 2.0
                    continue

            # Reset retry delay on success
            retry_delay = 1.0

            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.last_frame = frame
                self._update_fps()
            else:
                logger.warning("⚠️ Frame read failed")
                if self.cap:
                    self.cap.release()
                time.sleep(0.5)

    def _update_fps(self) -> None:
        """Oblicza klatki na sekundę."""
        self.frame_count += 1
        if time.time() - self.last_fps_time >= 5.0:
            self.current_fps = self.frame_count / 5.0
            self.frame_count = 0
            self.last_fps_time = time.time()

    def get_ai_frame(self) -> np.ndarray | None:
        """
        Zwraca kopię ostatniej klatki dla AI.
        Returns copy of last frame for AI.
        """
        if not self.is_running:
            return None
        with self.frame_lock:
            if self.last_frame is None:
                return None
            return self.last_frame.copy()

    def stop(self) -> None:
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
        if self.picam2:
            self.picam2.stop()
            self.picam2.close()
        logger.info("🛑 CameraManager stopped")
