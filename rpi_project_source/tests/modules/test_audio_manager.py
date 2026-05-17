import unittest
from queue import Queue
from unittest.mock import MagicMock, patch

import numpy as np
from modules.audio_manager import AudioManager


class TestAudioManager(unittest.TestCase):
    def setUp(self):
        # Mock PyAudio before initializing AudioManager
        self.patcher = patch("pyaudio.PyAudio")
        self.mock_pyaudio = self.patcher.start()

        # Setup mock stream
        self.mock_stream = MagicMock()
        self.mock_pyaudio.return_value.open.return_value = self.mock_stream

        self.manager = AudioManager(threshold_rms=500)

    def tearDown(self):
        self.patcher.stop()

    def test_initialization(self):
        self.assertTrue(self.manager.is_enabled)
        self.assertEqual(self.manager.threshold, 500)
        self.assertIsInstance(self.manager.event_queue, Queue)

    def test_get_latest_event_empty(self):
        event = self.manager.get_latest_event()
        self.assertIsNone(event)

    def test_get_latest_event_with_data(self):
        # Manually put an event into the queue
        test_event = {"type": "LOUD_NOISE", "rms": 600.0, "timestamp": 123.456}
        self.manager.event_queue.put(test_event)

        event = self.manager.get_latest_event()
        self.assertEqual(event, test_event)

        # Verify queue is empty again
        self.assertIsNone(self.manager.get_latest_event())

    def test_rms_calculation_and_event_trigger(self):
        # Create silent and loud audio buffers (paInt16)
        # CHUNK = 1024
        silent_data = np.zeros(1024, dtype=np.int16).tobytes()

        # Loud data: Square wave exceeding 500 RMS
        # RMS of square wave with amplitude A is A.
        loud_val = 1000
        loud_data = np.array([loud_val] * 1024, dtype=np.int16).tobytes()

        # Mock stream.read to return silent then loud then raise exception to stop loop
        self.mock_stream.read.side_effect = [
            silent_data,
            loud_data,
            Exception("Stop loop"),
        ]

        # We need to run _capture_loop manually or in a very short-lived way
        # Since it's a loop, we rely on the Exception to break it
        try:
            self.manager.stream = self.mock_stream
            self.manager._capture_loop()
        except Exception as e:
            if str(e) != "Stop loop":
                raise

        # Check if one event was triggered (for the 1000 amplitude loop)
        event = self.manager.get_latest_event()
        self.assertIsNotNone(event)
        self.assertEqual(event["type"], "LOUD_NOISE")
        self.assertGreater(event["rms"], 500)


if __name__ == "__main__":
    unittest.main()
