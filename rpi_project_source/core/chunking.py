"""PL: Dzieli duże komunikaty na fragmenty i składa je ponownie.

Moduł obsługuje pakiety JSON z metadanymi fragmentacji i domyślnym limitem
rozmiaru 1100 znaków.

EN: Splits large messages into chunks and reassembles them.

The module handles JSON packets with chunk metadata and a default size limit
of 1100 characters.
"""

import json
import logging
import time


class MessageChunker:
    """PL: Dzieli duże ładunki JSON na ramki nieprzekraczające limitu.

    EN: Splits large JSON payloads into frames that do not exceed the size limit.
    """

    @staticmethod
    def chunk_message(
        payload: dict, max_size: int = 1100, msg_id: int | None = None
    ) -> list[str]:
        """PL: Serializuje ładunek do JSON i dzieli go na ramki, gdy przekracza limit.

        Dla małego ładunku zwraca jedną ramkę bez metadanych. Dla większego
        dodaje metadane ``__c``, ``id``, ``i``, ``n`` i ``d`` do każdej części.

        EN: Serializes the payload to JSON and chunks it when it exceeds the limit.

        A small payload is returned as one frame without metadata. Larger payloads
        receive ``__c``, ``id``, ``i``, ``n``, and ``d`` metadata on each chunk.
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
    """PL: Buforuje i składa fragmenty komunikatów JSON.

    EN: Buffers and reassembles chunks of JSON messages.
    """

    def __init__(self, logger: logging.Logger | None = None):
        """PL: Tworzy pusty bufor fragmentów z opcjonalnym loggerem.

        EN: Creates an empty chunk buffer with an optional logger.
        """
        self._buffers: dict[int, dict] = {}
        self.logger = logger or logging.getLogger(__name__)

    def add_chunk(self, packet: dict) -> dict | None:
        """PL: Buforuje fragment i zwraca oryginalny obiekt po jego złożeniu.

        Pakiet musi mieć format fragmentu z polem ``__c`` równym ``1``.

        EN: Buffers a chunk and returns the original object once it is reassembled.

        The packet must use the chunk format with ``__c`` set to ``1``.
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
        """PL: Usuwa bufory fragmentów nieaktualizowane dłużej niż podany limit.

        EN: Removes chunk buffers that have not been updated within the timeout.
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
