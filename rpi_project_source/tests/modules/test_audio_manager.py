"""PL: Testuje kolejkę zdarzeń i detekcję głośnego dźwięku w AudioManager.
EN: Tests event queuing and loud sound detection in AudioManager.
"""

import unittest
from queue import Queue
from unittest.mock import MagicMock, patch

import numpy as np
from modules.audio_manager import AudioManager


class TestAudioManager(unittest.TestCase):
    """PL: Testy audio z atrapą biblioteki PyAudio.
    EN: Audio tests using a PyAudio mock.
    """

    def setUp(self):
        """PL: Przygotowuje strumień audio i menedżera z progiem RMS.
        EN: Prepares an audio stream and a manager with an RMS threshold.
        """
        # Mock PyAudio before initializing AudioManager
        self.patcher = patch("pyaudio.PyAudio")
        self.mock_pyaudio = self.patcher.start()

        # Setup mock stream
        self.mock_stream = MagicMock()
        self.mock_pyaudio.return_value.open.return_value = self.mock_stream

        self.manager = AudioManager(threshold_rms=500)

    def tearDown(self):
        """PL: Zatrzymuje atrapę PyAudio po teście.
        EN: Stops the PyAudio mock after the test.
        """
        self.patcher.stop()

    def test_initialization(self):
        """PL: Sprawdza stan, próg i kolejkę po inicjalizacji.
        EN: Checks enabled state, threshold, and queue after initialization.
        """
        self.assertTrue(self.manager.is_enabled)
        self.assertEqual(self.manager.threshold, 500)
        self.assertIsInstance(self.manager.event_queue, Queue)

    def test_get_latest_event_empty(self):
        """PL: Sprawdza wynik odczytu pustej kolejki zdarzeń.
        EN: Checks the result when reading an empty event queue.
        """
        event = self.manager.get_latest_event()
        self.assertIsNone(event)

    def test_get_latest_event_with_data(self):
        """PL: Sprawdza pobranie zdarzenia i usunięcie go z kolejki.
        EN: Checks that an event is retrieved and removed from the queue.
        """
        # Manually put an event into the queue
        test_event = {"type": "LOUD_NOISE", "rms": 600.0, "timestamp": 123.456}
        self.manager.event_queue.put(test_event)

        event = self.manager.get_latest_event()
        self.assertEqual(event, test_event)

        # Verify queue is empty again
        self.assertIsNone(self.manager.get_latest_event())

    def test_rms_calculation_and_event_trigger(self):
        """PL: Sprawdza, że próbka o RMS ponad progiem wywołuje zdarzenie.
        EN: Checks that a sample above the RMS threshold triggers an event.
        """
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
