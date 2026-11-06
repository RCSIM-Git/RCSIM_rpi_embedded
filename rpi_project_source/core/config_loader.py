"""PL: Ładuje, przechowuje i zapisuje konfigurację aplikacji w formacie JSON.

EN: Loads, stores, and saves the application configuration in JSON format.

Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import json
import logging
import os
from typing import Any


class ConfigManager:
    """PL: Zarządza wczytywaniem, kontrolą i odświeżaniem konfiguracji.

    EN: Manages configuration loading, validation, and hot reloading.
    """

    def __init__(self, config_path: str):
        """PL: Inicjalizuje manager dla wskazanego pliku i od razu go wczytuje.

        Args:
            config_path: Ścieżka do pliku JSON z konfiguracją.

        EN: Initializes the manager for the given file and loads it immediately.

        Args:
            config_path: Path to the JSON configuration file.
        """
        self.config_path = config_path
        self.last_mtime = 0.0
        self.config: dict[str, Any] = {}
        self.logger = logging.getLogger("ConfigManager")

        # Load initial config
        self.load_config()

    def load_config(self) -> dict[str, Any]:
        """PL: Wczytuje konfigurację i zachowuje poprzednią przy błędzie odczytu.

        Returns:
            Wczytany słownik albo bieżąca konfiguracja, jeśli wystąpi błąd.

        EN: Loads the configuration and preserves the previous one on read errors.

        Returns:
            The loaded dictionary, or the current configuration if loading fails.
        """
        try:
            if not os.path.exists(self.config_path):
                self.logger.error(f"Config file not found: {self.config_path}")
                return {}

            mtime = os.path.getmtime(self.config_path)

            with open(self.config_path, "r") as f:
                new_config = json.load(f)

            # Simple validation (ensure essential keys exist)
            if "autonomous_navigation" not in new_config:
                self.logger.warning("Config missing 'autonomous_navigation' section.")

            self.config = new_config
            self.last_mtime = mtime
            self.logger.info("Configuration loaded/reloaded successfully.")
            return self.config

        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to decode JSON config: {e}")
            return self.config  # Return old config on error
        except Exception as e:
            self.logger.error(f"Error loading config: {e}")
            return self.config

    def check_for_updates(self) -> dict[str, Any] | None:
        """PL: Sprawdza czas modyfikacji pliku i wczytuje zmiany, jeśli istnieją.

        Returns:
            Nową konfigurację po zmianie pliku albo ``None`` bez zmian.

        EN: Checks the file modification time and reloads it when it has changed.

        Returns:
            The reloaded configuration when changed, or ``None`` otherwise.
        """
        try:
            if not os.path.exists(self.config_path):
                return None

            current_mtime = os.path.getmtime(self.config_path)
            if current_mtime > self.last_mtime:
                self.logger.info("Config file modification detected. Reloading...")
                return self.load_config()

            return None
        except Exception as e:
            self.logger.error(f"Error checking for updates: {e}")
            return None

    def get(self, key: str, default: Any = None) -> Any:
        """PL: Zwraca wartość konfiguracji albo podaną wartość domyślną.

        Args:
            key: Klucz szukanej wartości.
            default: Wartość zwracana, gdy klucz nie istnieje.

        EN: Returns a configuration value or the provided default.

        Args:
            key: Key of the requested value.
            default: Value to return when the key is absent.
        """
        return self.config.get(key, default)

    def save_config(self, new_config: dict[str, Any]) -> bool:
        """PL: Zapisuje konfigurację atomowo i aktualizuje stan managera.

        Args:
            new_config: Słownik konfiguracji do zapisania.

        Returns:
            ``True`` po udanym zapisie, w przeciwnym razie ``False``.

        EN: Atomically saves the configuration and updates the manager state.

        Args:
            new_config: Configuration dictionary to write.

        Returns:
            ``True`` if saving succeeds, otherwise ``False``.
        """
        try:
            # Atomic write pattern
            tmp_path = self.config_path + ".tmp"
            with open(tmp_path, "w") as f:
                json.dump(new_config, f, indent=4)

            os.replace(tmp_path, self.config_path)

            # Update internal state to avoid reloading our own change
            self.config = new_config
            self.last_mtime = os.path.getmtime(self.config_path)

            self.logger.info("Configuration saved successfully.")
            return True
        except Exception as e:
            self.logger.error(f"Failed to save config: {e}")
            return False
