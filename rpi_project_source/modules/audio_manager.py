"""Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Przechwytuje dźwięk z mikrofonu i wykrywa głośne zdarzenia.
EN: Captures microphone audio and detects loud events.
"""

import logging
import threading
import time
from queue import Empty, Queue

import numpy as np

try:
    import pyaudio

    PYAUDIO_AVAILABLE = True
    # Parametry strumienia audio / Audio stream parameters
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK = 1024
except ImportError:
    PYAUDIO_AVAILABLE = False
    # Placeholders to prevent NameError if referenced
    FORMAT = None
    CHANNELS = 1
    RATE = 16000
    CHUNK = 1024


logger = logging.getLogger(__name__)


class AudioManager:
    """PL: Prowadzi strumień audio i umieszcza wykryte zdarzenia w kolejce.
    EN: Runs the audio stream and queues detected events.
    """

    def __init__(self, threshold_rms: int = 500):
        """PL: Inicjalizuje przechwytywanie audio z podanym progiem RMS.
        EN: Initializes audio capture with the specified RMS threshold.
        """
        if not PYAUDIO_AVAILABLE:
            logger.warning(
                "Biblioteka PyAudio nie jest zainstalowana. AudioManager jest wyłączony."
            )
            self.is_enabled = False
            return

        self.is_enabled = True
        self.threshold = threshold_rms
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.event_queue = Queue(maxsize=10)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)

    def start(self) -> bool:
        """PL: Otwiera wejście audio i uruchamia wątek przechwytujący.
        EN: Opens the audio input and starts the capture thread.
        """
        if not self.is_enabled:
            return False

        try:
            self.stream = self.audio.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
            )
            logger.info("Strumień audio został otwarty.")
        except Exception as e:
            logger.error(
                f"Nie można otworzyć strumienia audio. Sprawdź podłączenie mikrofonu: {e}"
            )
            self.is_enabled = False
            return False

        self._stop_event.clear()
        self._thread.start()
        logger.info("Wątek AudioManager został uruchomiony.")
        return True

    def stop(self):
        """PL: Zatrzymuje przechwytywanie i zwalnia zasoby PyAudio.
        EN: Stops capture and releases PyAudio resources.
        """
        if not self.is_enabled or not self._thread.is_alive():
            return

        self._stop_event.set()
        self._thread.join(timeout=2)

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            logger.info("Strumień audio został zamknięty.")

        self.audio.terminate()
        logger.info("AudioManager został zatrzymany.")

    def _capture_loop(self):
        """PL: Czyta próbki audio i kolejkuje zdarzenia przekraczające próg.
        EN: Reads audio samples and queues events that exceed the threshold.
        """
        while not self._stop_event.is_set():
            try:
                data = self.stream.read(CHUNK, exception_on_overflow=False)
                audio_data = np.frombuffer(data, dtype=np.int16)

                # Prosta analiza: obliczenie RMS
                # Simple analysis: RMS calculation
                rms = np.sqrt(np.mean(audio_data.astype(float) ** 2))

                if rms > self.threshold:
                    event = {"type": "LOUD_NOISE", "rms": rms, "timestamp": time.time()}
                    if not self.event_queue.full():
                        self.event_queue.put_nowait(event)
                    logger.debug(f"Wykryto głośny dźwięk: RMS = {rms:.2f}")

            except IOError as e:
                # Błąd ten może wystąpić, jeśli bufor jest przepełniony
                # This error can occur if the buffer is full
                logger.warning(f"Błąd odczytu strumienia audio: {e}")
            except Exception as e:
                logger.error(f"Nieoczekiwany błąd w pętli audio: {e}", exc_info=True)
                self._stop_event.set()  # Zatrzymaj pętlę w przypadku poważnego błędu / Stop loop on serious error

    def get_latest_event(self):
        """PL: Pobiera następne zdarzenie audio bez blokowania.
        EN: Retrieves the next audio event without blocking.
        """
        try:
            return self.event_queue.get_nowait()
        except Empty:
            return None
