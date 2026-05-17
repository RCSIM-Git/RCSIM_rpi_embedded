import logging
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

# Add rpi_project_source to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.ai_manager import AIManager


class TestAIManagerInference(unittest.TestCase):
    def setUp(self):
        self.logger = logging.getLogger("TestAI")
        self.logger = logging.getLogger("TestAI")
        self.logger.setLevel(logging.DEBUG)

        # Ensure hailort is mocked
        if "hailort" not in sys.modules:
            sys.modules["hailort"] = MagicMock()

        self.config = {
            "model_path": "dummy.he",
            "hef_path": "dummy.he",
            "input_size": [640, 640],
            "ai": {"classes": ["cone", "person", "car"]},
            "autonomous_navigation": {
                "yolo": {
                    "confidence_threshold": 0.45,
                    "nms_iou_threshold": 0.45,
                    "max_detections": 50,
                    "debug_overlay": True,
                }
            },
        }

    def tearDown(self):
        if os.path.exists("dummy.he"):
            os.remove("dummy.he")

    def test_mock_inference(self):
        """Test inference when Hailo is not available (mock mode)"""
        # Patch HAILO_AVAILABLE to False within the module
        with patch("modules.ai_manager.HAILO_AVAILABLE", False):
            manager = AIManager(self.logger, self.config)

            # verify it initialized in mock mode
            self.assertIsNone(manager.hailo_device)

            # Create dummy frame
            frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

            # Run inference
            result = manager.infer(frame)

            # Verify result structure
            self.assertIn("raw_output", result)
            self.assertIn("shape", result)
            self.assertTrue(result.get("mock", False))

            # Verify shape (1, 8400, 4+1+num_classes or 4+num_classes)
            shape = result["shape"]
            num_classes = len(self.config["ai"]["classes"])
            self.assertEqual(shape[0], 1)
            self.assertEqual(shape[1], 8400)
            self.assertIn(shape[2], [4 + num_classes, 5 + num_classes])

    def test_infer_and_postprocess(self):
        """Test inference + postprocessing (Mock mode)"""
        with patch("modules.ai_manager.HAILO_AVAILABLE", False):
            manager = AIManager(self.logger, self.config)
            frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

            result = manager.infer_and_postprocess(frame)

            self.assertIn("detections", result)
            detections = result["detections"]

            # Our updated mock creates exactly 3 valid detections
            self.assertEqual(
                len(detections), 3, "Mock mode should produce 3 detections"
            )

            # Verify first detection structure
            det = detections[0]
            self.assertIn("class_name", det)
            self.assertIn("con", det)
            self.assertIn("bbox", det)
            self.assertIn("center", det)

            # Verify specific mock values
            self.assertIn(det["class_name"], self.config["ai"]["classes"])
            self.assertGreater(det["con"], 0.6)

            # Verify overlay generation (if enabled in config)
            # By default config in setUp has debug_overlay: True
            if result["frame_with_overlay"] is not None:
                self.assertEqual(result["frame_with_overlay"].shape, frame.shape)

    @patch("modules.ai_manager.HAILO_AVAILABLE", True)
    @patch("modules.ai_manager.VDevice")
    @patch("modules.ai_manager.InferVStreams")
    @patch("modules.ai_manager.HailoStreamInterface")
    def test_hailo_loading(
        self, mock_hailo_interface, mock_infer_vstreams, mock_vdevice
    ):
        """Test Hailo model loading (mocked libraries)"""

        # Mock HailoStreamInterface enum
        mock_hailo_interface.PCIe = "PCIe"

        # Setup mocks
        mock_vdevice_instance = MagicMock()
        mock_vdevice.return_value = mock_vdevice_instance

        mock_vstreams_instance = MagicMock()
        mock_infer_vstreams.return_value = mock_vstreams_instance

        mock_vdevice_instance.configure.return_value = [mock_vstreams_instance]

        # Setup input/output streams mocks
        input_stream = MagicMock()
        input_stream.shape = (640, 640, 3)
        input_stream.name = "input_layer"
        input_stream.format.type = "UINT8"

        output_stream = MagicMock()
        output_stream.name = "output_layer"

        mock_vstreams_instance.get_input_vstreams.return_value = [input_stream]
        mock_vstreams_instance.get_output_vstreams.return_value = [output_stream]

        # Mock infer return
        mock_vstreams_instance.infer.return_value = {
            "output_layer": np.zeros((1, 8400, 84))
        }

        # Create fake HEF file to pass existence check
        with open("dummy.he", "w") as f:
            f.write("dummy content")

        manager = AIManager(self.logger, self.config)

        # Verify initialization
        # Note: load_model is called in __init__, so we check state
        self.assertTrue(manager.hailo_device is not None)
        self.assertFalse(manager.is_multimodal)  # Only 1 input stream mocked

        # Test inference call
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        print(f"DEBUG: Manager Mock Mode: {manager.use_mock}")
        print(f"DEBUG: Manager Initialized: {manager.is_initialized}")

        result = manager.infer(frame)

        # Verify result
        self.assertIn("raw_output", result)
        # Check that infer was called on the mocked vstreams
        mock_vstreams_instance.infer.assert_called()


if __name__ == "__main__":
    unittest.main()
