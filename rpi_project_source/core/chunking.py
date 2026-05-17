"""
Narzędzie do minifikowanej fragmentacji i składania pakietów komunikacyjnych.
Utility for minified fragmentation and assembly of communication packets.
Zgodne z limitem MTU=1100. (Industrial grade protocol).
"""

import json
import logging
import time


class MessageChunker:
    """Rozbija duże payloady na ramki o rozmiarze <= max_size."""

    @staticmethod
    def chunk_message(
        payload: dict, max_size: int = 1100, msg_id: int | None = None
    ) -> list[str]:
        """
        Zwraca listę "gotowych do wysłania" ciągów znaków (JSON string).
        Jeśli payload jest mały, zwróci po prostu zawartość (1-elementowa lista).
        Wpp dodaje metadane chunkingu (__c, id, i, n, d).
        """
        try:
            encoded = json.dumps(payload)
        except Exception as e:
            # Re-raise or fallback if serialization fails
            raise e

        if len(encoded) <= max_size:
            return [encoded]

        if msg_id is None:
            # Losowy identyfikator bazujący na timestampie (bez gwarancji atomowości)
            msg_id = int(time.time() * 1000) % 1000000

        # Zapas 100 znaków na nagłówek JSON {"__c":1,"id":123456,"i":999,"n":999,"d":""}
        effective_max = max_size - 100
        if effective_max < 100:
            effective_max = 100  # Fallback minimum

        chunks = []
        total = (len(encoded) + effective_max - 1) // effective_max
        for i in range(total):
            start = i * effective_max
            end = start + effective_max
            chunk_data = encoded[start:end]
            chunk_packet = {"__c": 1, "id": msg_id, "i": i, "n": total, "d": chunk_data}
            chunks.append(json.dumps(chunk_packet))

        return chunks


class MessageAssembler:
    """Skleja przesyłane fragmenty, udostępnia interfejs nasłuchowy."""

    def __init__(self, logger: logging.Logger | None = None):
        self._buffers: dict[int, dict] = {}
        self.logger = logger or logging.getLogger(__name__)

    def add_chunk(self, packet: dict) -> dict | None:
        """
        Dodaje mniejszy fragment do bufora. Jeśli zmontuje cały pakiet, przesyła oryginalny słownik.
        Wymaga, aby wejściowy słownik był formacie {"__c": 1, ...}
        Zwraca skompletowany dictionary lub None.
        """
        if packet.get("__c") != 1:
            return None

        msg_id = packet.get("id")
        i = packet.get("i")
        n = packet.get("n")
        d = packet.get("d")

        if msg_id is None or i is None or n is None or d is None:
            return None

        if msg_id not in self._buffers:
            self._buffers[msg_id] = {"chunks": {}, "total": n, "last_seen": time.time()}

        buffer = self._buffers[msg_id]
        buffer["chunks"][i] = d
        buffer["last_seen"] = time.time()

        if len(buffer["chunks"]) == buffer["total"]:
            # Zakończono składnię: łączymy chronologicznie
            ordered_chunks = [buffer["chunks"][idx] for idx in range(buffer["total"])]
            full_str = "".join(ordered_chunks)
            del self._buffers[msg_id]

            try:
                return json.loads(full_str)
            except json.JSONDecodeError as e:
                self.logger.error(
                    f"MessageAssembler - błąd dekodowania JSON po re-assemblingu: {e}"
                )
                return None

        return None

    def cleanup(self, timeout: float = 5.0) -> None:
        """
        Zwalnia martwe bufory. Należy wywoływać to asynchronicznie.
        """
        now = time.time()
        expired = [
            m_id
            for m_id, b in self._buffers.items()
            if (now - b["last_seen"]) > timeout
        ]
        for m_id in expired:
            del self._buffers[m_id]
            self.logger.warning(
                f"MessageAssembler - Bufor {m_id} wygasł, usuwam zawartość."
            )
