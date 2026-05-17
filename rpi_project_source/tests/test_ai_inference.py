# -*- coding: utf-8 -*-
import logging
import time
import unittest

import numpy as np
from modules.ai_manager import AIManager
from modules.postprocess_yolo import postprocess_yolo


class TestAIInference(unittest.TestCase):
    def setUp(self):
        self.config = {
            "autonomous_navigation": {
                "hef_path": "models/yolov11n.he",
                "debug_snapshots_enabled": False,
            },
            "yolo_params": {"conf_thres": 0.4, "iou_thres": 0.45},
        }
        self.logger = logging.getLogger("TestAI")
        self.ai = AIManager(self.logger, self.config)

    def test_mock_inference(self):
        """Testuje czy tryb mock zwraca detekcje i czy tracker działa."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        # Pierwsza ramka / First frame
        detections1 = self.ai.predict(img)
        self.assertTrue(len(detections1) > 0)
        self.assertIn("track_id", detections1[0])

        id1 = detections1[0]["track_id"]

        # Druga ramka (krótka chwila później) / Second frame
        time.sleep(0.05)
        detections2 = self.ai.predict(img)
        id2 = detections2[0]["track_id"]

        # W trybie mock bboxy się poruszają, ale powinny zachować ID jeśli są blisko
        self.assertEqual(id1, id2, "Tracker should maintain ID for moving mock objects")

    def test_nms_logic(self):
        """Testuje logikę NMS na syntetycznym wyjściu."""
        # [x, y, w, h, conf, c1, c2...]
        # Tworzymy dwa niemal identyczne bboxy
        fake_output = np.zeros((1, 10, 6))
        # Bbox 1
        fake_output[0, 0, :4] = [100, 100, 50, 50]
        fake_output[0, 0, 4:] = [0.9, 0.1]  # Class 0
        # Bbox 2 (overlaps Bbox 1)
        fake_output[0, 1, :4] = [105, 105, 52, 52]
        fake_output[0, 1, 4:] = [0.85, 0.1]

        class_names = ["cone", "wall"]
        detections = postprocess_yolo(
            fake_output, conf_thres=0.5, iou_thres=0.45, class_names=class_names
        )

        # NMS should keep only 1
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]["class_name"], "cone")


if __name__ == "__main__":
    unittest.main()
