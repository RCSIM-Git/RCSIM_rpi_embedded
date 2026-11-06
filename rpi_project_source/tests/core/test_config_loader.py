"""PL: Testuje wczytywanie, odczyt, wykrywanie zmian i atomowy zapis konfiguracji.
EN: Tests configuration loading, access, change detection, and atomic saving.
"""

import json
from unittest.mock import patch

import pytest
from core.config_loader import ConfigManager


@pytest.fixture
def temp_config_file(tmp_path):
    """PL: Tworzy przykładowy plik konfiguracji w katalogu tymczasowym.
    EN: Creates a sample configuration file in a temporary directory.
    """
    config_data = {
        "comm_mode": "WEBRTC",
        "hardware": {"pca_address": 0x40},
        "autonomous_navigation": True,
    }
    config_file = tmp_path / "config.json"
    config_file.write_text(json.dumps(config_data))
    return str(config_file)


def test_config_manager_load_success(temp_config_file):
    """PL: Sprawdza poprawne wczytanie pól konfiguracji.
    EN: Checks that configuration fields load successfully.
    """
    manager = ConfigManager(temp_config_file)
    assert manager.config["comm_mode"] == "WEBRTC"
    assert manager.config["hardware"]["pca_address"] == 0x40
    assert manager.config["autonomous_navigation"] is True


def test_config_manager_get_existing_key(temp_config_file):
    """PL: Sprawdza odczyt istniejących kluczy konfiguracji.
    EN: Checks retrieval of existing configuration keys.
    """
    manager = ConfigManager(temp_config_file)
    assert manager.get("comm_mode") == "WEBRTC"
    assert manager.get("autonomous_navigation") is True


def test_config_manager_get_missing_key_with_default(temp_config_file):
    """PL: Sprawdza zwracanie wartości domyślnej dla brakującego klucza.
    EN: Checks the default value returned for a missing key.
    """
    manager = ConfigManager(temp_config_file)
    assert manager.get("non_existent_key", "default_value") == "default_value"


def test_config_manager_get_missing_key_no_default(temp_config_file):
    """PL: Sprawdza, że brakujący klucz bez wartości domyślnej daje None.
    EN: Checks that a missing key without a default returns None.
    """
    manager = ConfigManager(temp_config_file)
    assert manager.get("another_missing_key") is None


def test_config_manager_load_file_not_found():
    """PL: Sprawdza pustą konfigurację po braku pliku wejściowego.
    EN: Checks for an empty configuration when the input file is missing.
    """
    manager = ConfigManager("non_existent_file.json")
    assert manager.config == {}


def test_config_manager_load_invalid_json(tmp_path):
    """PL: Sprawdza obsługę pliku z niepoprawnym składniowo JSON-em.
    EN: Checks handling of a file containing malformed JSON.
    """
    invalid_file = tmp_path / "invalid_config.json"
    invalid_file.write_text("{ invalid json: [ }")

    # Inicjalnie ładowany jest pusty słownik przy błędzie
    manager = ConfigManager(str(invalid_file))
    assert manager.config == {}


def test_config_manager_load_json_decode_error_preserves_old_config(temp_config_file):
    """PL: Weryfikuje zachowanie poprzedniej konfiguracji po błędzie dekodowania.
    EN: Verifies the previous configuration is retained after a decode error.
    """
    manager = ConfigManager(temp_config_file)
    original_config = manager.config.copy()

    # Ręcznie psujemy plik
    with open(temp_config_file, "w") as f:
        f.write("{ broken }")

    # Próba przeładowania
    result = manager.load_config()
    assert result == original_config
    assert manager.config == original_config


@patch("os.path.getmtime")
def test_config_manager_check_for_updates(mock_mtime, temp_config_file):
    """PL: Sprawdza wykrycie zmiany czasu modyfikacji pliku.
    EN: Checks detection of a changed file modification time.
    """
    # Ustawiamy początkową datę modyfikacji jako liczbę
    mock_mtime.return_value = 100.0
    manager = ConfigManager(temp_config_file)

    # Symulacja nowszej daty modyfikacji
    mock_mtime.return_value = 101.0

    with patch.object(manager, "load_config") as mock_load:
        mock_load.return_value = {"updated": True}
        updated_config = manager.check_for_updates()
        assert updated_config == {"updated": True}
        mock_load.assert_called_once()


def test_config_manager_save_config_atomic(temp_config_file):
    """PL: Weryfikuje atomowy zapis konfiguracji na dysku.
    EN: Verifies atomic configuration saving to disk.
    """
    manager = ConfigManager(temp_config_file)
    new_data = {"new_key": "new_val"}

    success = manager.save_config(new_data)
    assert success is True
    assert manager.config == new_data

    # Sprawdzenie czy plik na dysku został zaktualizowany
    with open(temp_config_file, "r") as f:
        saved_data = json.load(f)
    assert saved_data == new_data
