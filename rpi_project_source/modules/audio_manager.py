"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Moduł AudioManagera (Audio Manager Module) dla RPi.
Audio Manager Module for RPi.

Ten moduł jest odpowiedzialny za przechwytywanie i analizę dźwięku
z podłączonego mikrofonu USB. Umożliwia detekcję prostych zdarzeń
dźwiękowych, które mogą być wykorzystane jako triggery w logice
autonomicznej pojazdu.
This module is responsible for capturing and analyzing sound
from a connected USB microphone. It enables the detection of simple
sound events that can be used as triggers in the vehicle's
autonomous logic.

Kluczowe funkcjonalności / Key features:
-   **Przechwytywanie dźwięku / Audio Capture:** Używa biblioteki PyAudio do nagrywania
    dźwięku w osobnym, nieblokującym wątku.
    Uses the PyAudio library to record sound in a separate, non-blocking thread.
-   **Wykrywanie zdarzeń / Event Detection:** Implementuje prostą detekcję głośnych dźwięków
    poprzez analizę wartości RMS (Root Mean Square) strumienia audio.
    Stanowi to podstawę pod przyszłe, bardziej zaawansowane modele klasyfikacji dźwięku.
    Implements simple loud sound detection by analyzing the RMS (Root Mean Square)
    value of the audio stream. This serves as a foundation for future,
    more advanced sound classification models.
-   **Odporność na błędy / Fault Tolerance:** Moduł jest zaprojektowany tak, aby działać
    poprawnie nawet w przypadku braku mikrofonu lub biblioteki PyAudio,
    nie powodując awarii całej aplikacji.
    The module is designed to work correctly even if the microphone or PyAudio
    library is missing, without causing a complete application crash.
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
    """
    Zarządza przechwytywaniem i prostą analizą dźwięku z mikrofonu.
    Manages audio capture and simple analysis from the microphone.
    """

    def __init__(self, threshold_rms: int = 500):
        """
        Inicjalizuje AudioManager.
        Initializes the Audio Manager.

        Args:
            threshold_rms (int): Próg RMS, powyżej którego dźwięk jest uznawany za "głośne zdarzenie".
                                 RMS threshold above which sound is considered a "loud event".
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
        """
        Uruchamia wątek przechwytywania dźwięku.
        Starts the audio capture thread.

        Returns:
            bool: True, jeśli udało się uruchomić, False w przeciwnym razie.
                  True if started successfully, else False.
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
        """
        Zatrzymuje wątek przechwytywania dźwięku i zwalnia zasoby.
        Stops the audio capture thread and releases resources.
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
        """
        Główna pętla wątku, która odczytuje i analizuje dane z mikrofonu.
        Main thread loop that reads and analyzes microphone data.
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
        """
        Pobiera ostatnie wykryte zdarzenie dźwiękowe z kolejki.
        Retrieves the latest detected audio event from the queue.

        Returns:
            dict or None: Słownik z danymi zdarzenia lub None, jeśli kolejka jest pusta.
                          Dictionary with event data or None if queue is empty.
        """
        try:
            return self.event_queue.get_nowait()
        except Empty:
            return None
