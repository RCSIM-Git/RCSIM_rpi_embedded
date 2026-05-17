"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import cv2
import numpy as np


def non_max_suppression(
    boxes: np.ndarray, scores: np.ndarray, iou_thres: float = 0.45
) -> np.ndarray:
    """
    Wykonuje Non-Maximum Suppression (NMS) za pomocą cv2.dnn.NMSBoxes (C++).
    Performs Non-Maximum Suppression (NMS) using cv2.dnn.NMSBoxes (C++ backend).

    Args:
        boxes: Tablica [N, 4] współrzędnych (x1, y1, x2, y2).
        scores: Tablica [N] wyników pewności.
        iou_thres: Próg IoU do odrzucania nakładających się pudełek.

    Returns:
        np.ndarray: Indeksy pudełek do zachowania.
    """
    if len(boxes) == 0:
        return np.array([], dtype=int)

    # cv2.dnn.NMSBoxes wymaga formatu [x, y, w, h]
    # Konwersja xyxy -> xywh
    xywh = np.empty_like(boxes)
    xywh[:, 0] = boxes[:, 0]
    xywh[:, 1] = boxes[:, 1]
    xywh[:, 2] = boxes[:, 2] - boxes[:, 0]  # width
    xywh[:, 3] = boxes[:, 3] - boxes[:, 1]  # height

    # score_threshold=0.0 bo filtrowanie confidence jest wcześniej
    indices = cv2.dnn.NMSBoxes(
        xywh.tolist(), scores.tolist(), score_threshold=0.0, nms_threshold=iou_thres
    )

    if len(indices) == 0:
        return np.array([], dtype=int)

    return np.array(indices, dtype=int).flatten()


class SimpleTracker:
    """
    Prosty tracker euklidesowy dla detekcji.
    Simple Euclidean distance tracker for detections.
    """

    def __init__(self, max_disappeared: int = 5, max_distance: int = 50):
        self.next_object_id = 0
        self.objects = {}  # id -> center
        self.disappeared = {}  # id -> count
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def register(self, center):
        self.objects[self.next_object_id] = center
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1
        return self.next_object_id - 1

    def deregister(self, object_id):
        if object_id in self.objects:
            del self.objects[object_id]
            del self.disappeared[object_id]

    def update(self, detections: list[dict]) -> list[dict]:
        if not detections:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)
            return detections

        input_centers = np.array([d["center"] for d in detections])

        if not self.objects:
            for i in range(len(detections)):
                obj_id = self.register(input_centers[i])
                detections[i]["track_id"] = obj_id
            return detections

        object_ids = list(self.objects.keys())
        object_centers = np.array(list(self.objects.values()))

        # Calculate Euclidean distance
        D = np.linalg.norm(object_centers[:, np.newaxis] - input_centers, axis=2)

        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]

        used_rows = set()
        used_cols = set()

        for row, col in zip(rows, cols):
            if row in used_rows or col in used_cols:
                continue

            if D[row, col] > self.max_distance:
                continue

            object_id = object_ids[row]
            self.objects[object_id] = input_centers[col]
            self.disappeared[object_id] = 0
            detections[col]["track_id"] = object_id

            used_rows.add(row)
            used_cols.add(col)

        unused_rows = set(range(D.shape[0])) - used_rows
        unused_cols = set(range(D.shape[1])) - used_cols

        for row in unused_rows:
            object_id = object_ids[row]
            self.disappeared[object_id] += 1
            if self.disappeared[object_id] > self.max_disappeared:
                self.deregister(object_id)

        for col in unused_cols:
            obj_id = self.register(input_centers[col])
            detections[col]["track_id"] = obj_id

        return detections


# Global tracker instance
_tracker = SimpleTracker()


def postprocess_yolo(
    raw_output: np.ndarray,
    conf_thres: float = 0.45,
    iou_thres: float = 0.45,
    class_names: list[str] | None = None,
    max_dets: int = 50,
    tracker_params: dict | None = None,
) -> list[dict]:
    """
    raw_output: [1, num_anchors, 4 + 1 + num_classes] lub [1, num_anchors, 4 + num_classes]
    Zakładamy standard YOLOv8/v11: xywh + obj_conf + class_probs
    """
    # Support for dictionary input (if passed directly from AIManager result)
    if isinstance(raw_output, dict):
        raw_output = raw_output.get("raw_output")
        if raw_output is None:
            return []

    if raw_output.ndim != 3:
        if raw_output.ndim == 2:
            raw_output = np.expand_dims(raw_output, axis=0)
        else:
            return []

    output = raw_output[0]  # usuń batch dim

    # Transpose if features < anchors (standard Hailo/ONNX format [84, 8400])
    if output.shape[0] < output.shape[1] and output.shape[0] < 1000:
        output = output.transpose()

    boxes = output[:, :4]  # xywh

    # Detection of objectness presence
    num_features = output.shape[1]
    num_classes = len(class_names) if class_names else (num_features - 4)

    if num_features == 4 + 1 + num_classes:
        confs = output[:, 4]  # objectness
        class_probs = output[:, 5:]
    else:
        # If no objectness (World models or certain exports)
        class_probs = output[:, 4:]
        confs = np.max(class_probs, axis=1)  # Use max class prob as confidence

    # Confidence filter
    mask = confs > conf_thres
    boxes = boxes[mask]
    confs = confs[mask]
    class_probs = class_probs[mask]

    if len(boxes) == 0:
        return []

    # Best class + final score
    class_scores = class_probs * confs[:, None]
    class_ids = np.argmax(class_scores, axis=1)
    final_confs = np.max(class_scores, axis=1)

    # xywh -> xyxy
    xyxy = np.zeros_like(boxes)
    xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    # NMS
    keep = non_max_suppression(xyxy, final_confs, iou_thres)
    xyxy = xyxy[keep]
    final_confs = final_confs[keep]
    class_ids = class_ids[keep]

    # Create detection list
    detections = []
    for i in range(len(xyxy)):
        x1, y1, x2, y2 = xyxy[i]
        detections.append(
            {
                "class_id": int(class_ids[i]),
                "class_name": (
                    class_names[class_ids[i]]
                    if class_names and class_ids[i] < len(class_names)
                    else f"class_{class_ids[i]}"
                ),
                "con": float(final_confs[i]),
                "bbox": [float(x1), float(y1), float(x2), float(y2)],  # xyxy
                "center": (float((x1 + x2) / 2), float((y1 + y2) / 2)),
            }
        )

    # Sort & Limit
    detections.sort(key=lambda d: d["con"], reverse=True)
    detections = detections[:max_dets]

    # Tracking integration (maintaining feature from previous step)
    if tracker_params:
        _tracker.max_disappeared = tracker_params.get(
            "max_disappeared", _tracker.max_disappeared
        )
        _tracker.max_distance = tracker_params.get(
            "max_distance", _tracker.max_distance
        )

    detections = _tracker.update(detections)

    return detections


def draw_detections(
    frame: np.ndarray, detections: list[dict], thickness: int = 2
) -> np.ndarray:
    """
    Rysuje detekcje na kopii klatki obrazu.
    Draws detections on a copy of the video frame.

    Args:
        frame: Obraz BGR (numpy array).
        detections: Lista detekcji z postprocess_yolo.
        thickness: Grubość linii.

    Returns:
        np.ndarray: Obraz z narysowanymi ramkami.
    """
    img = frame.copy()

    for det in detections:
        x1, y1, x2, y2 = map(int, det["bbox"])
        conf = det["con"]
        label = f"{det['class_name']} {conf:.2f}"

        # Kolor zależny od pewności: Zielony > 0.6, Żółty < 0.6
        color = (0, 255, 0) if conf > 0.6 else (0, 255, 255)

        # Rysowanie ramki
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

        # Tło dla tekstu
        (text_w, text_h), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        cv2.rectangle(img, (x1, y1 - text_h - 10), (x1 + text_w, y1), color, -1)

        # Tekst
        cv2.putText(
            img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
        )

    return img
