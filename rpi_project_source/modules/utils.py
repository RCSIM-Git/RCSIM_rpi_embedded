"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Narzędzia pomocnicze.
Utility functions.
"""

import logging
from typing import Any

logger = logging.getLogger(__name__)


def sanitize_payload(obj: Any) -> Any:
    """
    Rekursywnie usuwa niedozwolone typy danych z payloadu.
    Recursively removes non-serializable data types from the payload.

    Funkcja przechodzi przez zagnieżdżone struktury (słowniki, listy)
    i konwertuje typy, które nie są serializowalne przez JSON, na bezpieczne
    odpowiedniki.

    The function traverses nested structures (dictionaries, lists)
    and converts types that are not JSON serializable into safe
    equivalents.

    Args:
        obj (Any): Obiekt do oczyszczenia (np. słownik, lista, wartość). / Object to sanitize (e.g., dict, list, value).

    Returns:
        Any: Oczyszczony obiekt, bezpieczny do serializacji. / Sanitized object, safe for serialization.
    """
    if isinstance(obj, dict):
        return {k: sanitize_payload(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [sanitize_payload(elem) for elem in obj]
    if isinstance(obj, (bytes, bytearray)):
        logger.warning("Wykryto i usunięto niedozwolony typ 'bytes/bytearray'.")
        return None
    if isinstance(obj, set):
        logger.warning("Wykryto i przekonwertowano 'set' na listę.")
        return [sanitize_payload(elem) for elem in obj]
    if isinstance(obj, complex):
        logger.warning("Wykryto i usunięto niedozwolony typ 'complex'.")
        return None
    if not isinstance(obj, (type(None), str, int, float, bool)):
        if hasattr(obj, "to_dict") and callable(getattr(obj, "to_dict")):
            return sanitize_payload(obj.to_dict())
        if hasattr(obj, "__dict__"):
            return sanitize_payload(obj.__dict__)
        logger.warning(
            f"Wykryto nieznany typ: {type(obj).__name__}. Zastąpiono przez None."
        )
        return None
    return obj
