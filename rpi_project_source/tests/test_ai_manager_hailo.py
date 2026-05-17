# -*- coding: utf-8 -*-
import logging
import os
import sys
import unittest

import numpy as np

# Add project root to path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from modules.ai_manager import AIManager


class TestAIManagerHailo(unittest.TestCase):
    def setUp(self):
        self.config = {
            "autonomous_navigation": {
                "hef_path": "models/yolov11n.he",
                "classes": ["cone", "wall", "person"],
            }
        }
        self.logger = logging.getLogger("TestAI")
        # Force mock mode for PC testing
        self.ai = AIManager(self.logger, self.config)

    def test_raw_inference_mock(self):
        """Testuje czy infer() zwraca poprawny słownik z surowym tensorem."""
        mock_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        result = self.ai.infer(mock_frame)

        # Mandatory checks from USER_REQUEST
        self.assertIn("raw_output", result, "Result must contain 'raw_output'")
        self.assertIn("shape", result, "Result must contain 'shape'")

        raw_output = result["raw_output"]
        shape = result["shape"]

        print(f"Mock Inference Shape: {shape}")

        # Verify shape (1, 8400, 4 + classes)
        num_classes = len(self.config["autonomous_navigation"]["classes"])
        self.assertEqual(shape[0], 1)
        self.assertEqual(shape[1], 8400)
        self.assertEqual(shape[2], 4 + num_classes)
        self.assertEqual(shape, raw_output.shape)

        if result.get("mock"):
            print("Successfully verified Mock Raw Inference")

    def test_preprocess(self):
        """Testuje czy obraz jest poprawnie przygotowany do wejścia."""
        mock_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # We need to ensure we have input vstream info mock or real
        if self.ai.hailo_input_vstreams:
            input_data = self.ai._preprocess(mock_frame)
            self.assertTrue(len(input_data) > 0)
            tensor = list(input_data.values())[0]
            self.assertEqual(len(tensor.shape), 4)  # Batch dimension
        else:
            print("Skipping real preprocess test (no vstreams in mock mode)")


if __name__ == "__main__":
    unittest.main()
