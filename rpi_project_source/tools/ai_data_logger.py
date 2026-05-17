# -*- coding: utf-8 -*-
"""
AI Data Logger - utility to collect real-world frames and YOLO detections.
Records to "tubs" for future training/fine-tuning.
"""

import json
import logging
import os
import time

import cv2
from modules.ai_manager import AIManager
from modules.camera_manager import CameraManager
from modules.postprocess_yolo import draw_detections

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DataLogger")


def run_logger(duration_s=60, output_dir="data/tubs"):
    # 1. Setup Config
    config = {
        "camera": {"mode": "local", "resolution": [640, 480], "fps": 15},
        "ai": {
            "hef_path": "models/yolov11n.he",
            "classes": ["cone", "wall", "person", "car"],
        },
        "autonomous_navigation": {
            "yolo_confidence_threshold": 0.4,
            "debug_snapshots_enabled": True,
        },
    }

    # 2. Initialize Managers
    cam = CameraManager(config["camera"])
    ai = AIManager(logger, config)

    os.makedirs(output_dir, exist_ok=True)

    cam.start()
    logger.info(f"Starting data collection for {duration_s}s...")

    start_time = time.time()
    count = 0

    try:
        while time.time() - start_time < duration_s:
            frame = cam.get_ai_frame()
            if frame is not None:
                # Run inference
                detections = ai.predict(frame)

                timestamp = int(time.time() * 1000)
                frame_filename = f"frame_{timestamp}.jpg"
                json_filename = f"record_{timestamp}.json"

                # Save Raw Image
                cv2.imwrite(os.path.join(output_dir, frame_filename), frame)

                # Save Overlay for easy review
                overlay = draw_detections(frame, detections)
                cv2.imwrite(
                    os.path.join(output_dir, f"overlay_{timestamp}.jpg"), overlay
                )

                # Save Metadata
                record = {
                    "timestamp": timestamp,
                    "frame": frame_filename,
                    "detections": detections,
                    "num_objects": len(detections),
                }

                with open(os.path.join(output_dir, json_filename), "w") as f:
                    json.dump(record, f)

                count += 1
                if count % 10 == 0:
                    logger.info(f"Recorded {count} frames...")

            time.sleep(0.1)  # 10 FPS collection

    except KeyboardInterrupt:
        logger.info("Collection interrupted by user.")
    finally:
        cam.stop()
        logger.info(f"Done. Collected {count} records in {output_dir}")


def export_for_labeling(tub_path: str, output_dir: str, class_names: list = None):
    """
    Exports collected frames and (optional) detections to YOLO format.
    Creates:
    - images/
    - labels/ (empty for manual annotation or with zero-shot boxes)
    - dataset.yaml
    """
    import shutil

    img_dir = os.path.join(output_dir, "images")
    lbl_dir = os.path.join(output_dir, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)

    if class_names is None:
        class_names = ["cone", "person", "car", "line", "barrier"]

    logger.info(f"Exporting data from {tub_path} to {output_dir}...")

    count = 0
    # Scan for JPG frames
    for file in os.listdir(tub_path):
        if file.startswith("frame_") and file.endswith(".jpg"):
            ts = file.replace("frame_", "").replace(".jpg", "")
            json_path = os.path.join(tub_path, f"record_{ts}.json")

            # Copy image
            new_img_name = f"obj_{ts}.jpg"
            shutil.copy2(
                os.path.join(tub_path, file), os.path.join(img_dir, new_img_name)
            )

            # Create label file (empty or with zero-shot detections as starting point)
            lbl_name = f"obj_{ts}.txt"
            lbl_path = os.path.join(lbl_dir, lbl_name)

            detections = []
            if os.path.exists(json_path):
                with open(json_path, "r") as f:
                    rec = json.load(f)
                    detections = rec.get("detections", [])

            with open(lbl_path, "w") as f:
                for det in detections:
                    # YOLO format: <class> <x_center> <y_center> <width> <height>
                    # Assuming bbox is [y1, x1, y2, x2] normalized from AIManager
                    # Convert to cx, cy, w, h
                    bbox = det.get("bbox")
                    class_name = det.get("class_name", det.get("label"))
                    if bbox and class_name in class_names:
                        cid = class_names.index(class_name)
                        y1, x1, y2, x2 = bbox
                        cx = (x1 + x2) / 2.0
                        cy = (y1 + y2) / 2.0
                        w = x2 - x1
                        h = y2 - y1
                        f.write(f"{cid} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")

            count += 1

    # Generate data.yaml
    yaml_content = {
        "train": "./images",
        "val": "./images",  # Using same for simple export
        "nc": len(class_names),
        "names": class_names,
    }

    import yaml

    with open(os.path.join(output_dir, "dataset.yaml"), "w") as f:
        yaml.dump(yaml_content, f)

    logger.info(f"Export complete. {count} frames exported to {output_dir}")
    return True


if __name__ == "__main__":
    run_logger()
