# -*- coding: utf-8 -*-
"""
Example of parsing raw YOLO-World / YOLOv11 output from Hailo-8L.
This shows how to convert the (1, 8400, 84) raw tensor into detections.
"""

import numpy as np


def parse_yolo_raw(raw_tensor, conf_threshold=0.5):
    """
    raw_tensor: np.ndarray of shape (1, 8400, 4 + num_classes)
    Returns: List of detections
    """
    # 1. Remove batch dimension
    # (8400, 4 + num_classes)
    predictions = raw_tensor[0]

    # 2. Extract BBox (first 4 elements) and Scores (rest)
    boxes = predictions[:, :4]  # [cx, cy, w, h]
    scores = predictions[:, 4:]  # Probabilities for each class

    # 3. Find best class for each anchor
    class_ids = np.argmax(scores, axis=1)
    confidences = np.max(scores, axis=1)

    # 4. Filter by confidence
    mask = confidences > conf_threshold

    detections = []
    for i in np.where(mask)[0]:
        det = {
            "bbox_cxcywh": boxes[i].tolist(),
            "confidence": float(confidences[i]),
            "class_id": int(class_ids[i]),
        }
        detections.append(det)

    return detections


if __name__ == "__main__":
    # Simulate a raw tensor for YOLO-World with 80 classes
    mock_raw = np.random.rand(1, 8400, 84).astype(np.float32)
    # Put a "fake" detection at index 100
    mock_raw[0, 100, 4] = 0.95  # High confidence for class 0

    results = parse_yolo_raw(mock_raw)
    print(f"Found {len(results)} detections above threshold.")
    if results:
        print(f"First detection: {results[0]}")
