"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import json
import logging
import os
from typing import Any


class ConfigManager:
    """
    Manages configuration loading, validation, and hot-reloading.
    Zarządza ładowaniem, walidacją i dynamicznym odświeżaniem konfiguracji.
    """

    def __init__(self, config_path: str):
        self.config_path = config_path
        self.last_mtime = 0.0
        self.config: dict[str, Any] = {}
        self.logger = logging.getLogger("ConfigManager")

        # Load initial config
        self.load_config()

    def load_config(self) -> dict[str, Any]:
        """
        Loads the configuration from the file.
        Ładuje konfigurację z pliku.
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
        """
        Checks if the configuration file has been modified.
        Returns new config if changed, None otherwise.
        Sprawdza, czy plik konfiguracyjny został zmodyfikowany.
        Zwraca nową konfigurację jeśli nastąpiła zmiana, w przeciwnym razie None.
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
        """
        Safe getter for config values.
        Bezpieczny getter dla wartości konfiguracyjnych.
        """
        return self.config.get(key, default)

    def save_config(self, new_config: dict[str, Any]) -> bool:
        """
        Saves the configuration to the file.
        Zapisuje konfigurację do pliku.
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
