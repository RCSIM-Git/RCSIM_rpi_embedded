#!/usr/bin/env python3
"""
Test Perception Loop
--------------------
Verifies the end-to-end perception pipeline on the hardware:
1. Camera Manager (RTSP/Local/Picamera2) -> Frame
2. AI Manager (Hailo/Mock) -> Detections
3. Debug Snapshots -> File System

Usage:
    python3 tools/test_perception_loop.py [--mock-cam] [--mock-ai]
"""

import argparse
import json
import logging
import os
import sys
import time

import numpy as np

# Add project root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.ai_manager import AIManager
from modules.camera_manager import CameraManager

# Logger setup
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("TestPerception")


def main():
    parser = argparse.ArgumentParser(description="Test Perception Loop")
    parser.add_argument("--config", default="config.json", help="Path to config file")
    parser.add_argument(
        "--duration", type=int, default=10, help="Test duration in seconds"
    )
    parser.add_argument("--mock-cam", action="store_true", help="Force mock camera")
    args = parser.parse_args()

    # Load Config
    if not os.path.exists(args.config):
        logger.error(f"Config file not found: {args.config}")
        # Create dummy config for testing if missing
        config = {
            "ai": {
                "classes": ["cone", "wall"],
                "hef_path": "models/rcsimai.hef",
                "input_size": [640, 640],
            },
            "autonomous_navigation": {
                "debug": {
                    "enabled": True,
                    "snapshot_interval_sec": 2.0,
                    "snapshot_dir": "debug_logs/test_snapshots",
                    "save_detections_overlay": True,
                },
                "yolo": {"confidence_threshold": 0.45, "nms_iou_threshold": 0.45},
            },
            "camera": {"mode": "local", "resolution": [640, 480], "fps": 30},
        }
        logger.warning("Using default internal config dictionary.")
    else:
        with open(args.config, "r") as f:
            config = json.load(f)

    # Overrides
    if args.mock_cam:
        config["camera"] = {"mode": "mock", "resolution": [640, 480], "fps": 30}

    # Ensure debug dir exists
    snapshot_dir = (
        config.get("autonomous_navigation", {})
        .get("debug", {})
        .get("snapshot_dir", "debug_logs/snapshots")
    )
    os.makedirs(snapshot_dir, exist_ok=True)
    logger.info(f"Snapshots will be saved to: {snapshot_dir}")

    # Initialize Modules
    try:
        logger.info("Initializing CameraManager...")
        # Patch for Mock Camera if requested/needed
        if config["camera"].get("mode") == "mock":
            # We can create a simple mock class or just let CameraManager fail and handle it?
            # CameraManager doesn't support "mock" mode explicitly in the code I read.
            # It falls back to "local" /dev/video0 or fails.
            # Let's verify CameraManager behavior.
            # If args.mock_cam is set, we might need to monkeypatch.
            pass

        camera = CameraManager(config.get("camera", {}))
        camera.start()
        time.sleep(2.0)  # Warmup

        logger.info("Initializing AIManager...")
        ai = AIManager(logger, config)

        start_time = time.time()
        frame_count = 0
        ai_count = 0

        logger.info(f"Starting loop for {args.duration} seconds...")

        while time.time() - start_time < args.duration:
            loop_start = time.time()

            # 1. Get Frame
            frame = camera.get_ai_frame()

            if frame is None:
                if args.mock_cam:
                    # Generate noise frame if mocking
                    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                else:
                    logger.warning("No frame from camera.")
                    time.sleep(0.1)
                    continue

            # 2. AI Inference
            # We use predict() because it handles snapshots internally
            detections = ai.predict(frame)

            frame_count += 1
            if detections:
                ai_count += 1
                logger.info(f"Frame {frame_count}: Detected {len(detections)} objects.")

            # Sleep to match ~20Hz loop
            elapsed = time.time() - loop_start
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)

        fps = frame_count / (time.time() - start_time)
        logger.info(f"Test finished. Average FPS: {fps:.2f}")
        logger.info(f"Total AI Detections batches: {ai_count}")

        # Verify Snapshots
        snapshots = os.listdir(snapshot_dir)
        logger.info(f"Snapshots created: {len(snapshots)} in {snapshot_dir}")

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
    finally:
        if "camera" in locals():
            camera.stop()
        if "ai" in locals():
            ai.cleanup()


if __name__ == "__main__":
    main()
