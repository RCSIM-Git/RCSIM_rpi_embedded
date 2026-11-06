"""PL: Testuje budowę telemetrii i obsługę danych oraz poleceń przez usługę.
EN: Tests telemetry construction and service handling of data and commands.
"""

import json
import logging
import os
import sys
import time
from unittest.mock import MagicMock, patch

import pytest
from core.main_service import TelemetryWorker

# Add project root to the Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, project_root)


@pytest.fixture
def config_mock():
    """PL: Zwraca minimalną konfigurację workera telemetrycznego.
    EN: Returns a minimal configuration for the telemetry worker.
    """
    return {
        "hardware": {},
        "main_loop_freq_hz": 30,
        "video": {"engine": "native"},
        "pc_ip": "127.0.0.1",
    }


@pytest.fixture
def telemetry_worker(config_mock) -> TelemetryWorker:
    """PL: Tworzy worker z atrapami zależności zewnętrznych.
    EN: Creates a worker with mocked external dependencies.
    """
    worker = TelemetryWorker(config_mock)
    # Mock internal components that are not part of the logic being tested
    worker.hw_manager = MagicMock()
    worker.nav_manager = MagicMock()
    worker.control_selector = MagicMock()
    worker.data_service = MagicMock()
    worker.video_service = MagicMock()
    return worker


# --- Istniejący test (bez zmian) ---


def test_prepare_telemetry_packet_structure(telemetry_worker: TelemetryWorker):
    """PL: Sprawdza strukturę i podstawowe pola pakietu telemetrii.
    EN: Checks the structure and key fields of a telemetry packet.
    """
    # Mocking frame_data expected by TelemetryBuilder
    frame_data = {
        "imu": {"roll": 0.0, "pitch": 0.0},
        "manual_controls": {"steering": 0.0, "throttle": 0.0},
        "pose": (0.0, 0.0, 0.0),
        "link_status": {"webrtc_dead": False, "elrs_dead": False},
        "system": {"cpu_temp": 45.0}
    }
    start_time = time.time()
    packet = telemetry_worker.telemetry_builder.prepare_telemetry(frame_data)

    assert isinstance(packet, dict)
    assert packet["t"] == "telemetry"
    assert packet["im"] == frame_data["imu"]
    assert packet["mo"] == "MANUAL"
    assert packet["ts"] >= start_time


# --- Nowe testy dla on_data_received i handle_command ---


def test_on_data_received_updates_last_control_input(telemetry_worker: TelemetryWorker):
    """PL: Sprawdza aktualizację sterowania po odebraniu pakietu kanałów.
    EN: Checks control state updates after receiving a channel packet.
    """
    # Arrange
    # CommandDispatcher expects "type": "control" and "channels": [steering_pwm, throttle_pwm]
    # We need to mock hw_manager to allow scaling
    telemetry_worker.pca_armed = True
    telemetry_worker.hw_manager.actuators.steering_range = (1000, 2000)
    telemetry_worker.hw_manager.actuators.throttle_range = (1000, 2000)
    
    control_data = {"type": "control", "channels": [1500, 1500]} 
    json_data = json.dumps(control_data)

    # Act
    telemetry_worker.command_dispatcher.on_data_received(json_data)

    # Assert
    # 1500 is center for (1000, 2000) range -> 0.0
    assert telemetry_worker.last_control_input["manual_controls"]["steering"] == 0.0
    assert telemetry_worker.last_control_input["manual_controls"]["throttle"] == 0.0


def test_on_data_received_handles_command(telemetry_worker: TelemetryWorker):
    """PL: Sprawdza przekazanie odebranej komendy do jej obsługi.
    EN: Checks that a received command is forwarded to its handler.
    """
    # Arrange
    command_data = {"type": "command", "command": "set_mode", "mode": "RTH"}
    json_data = json.dumps(command_data)
    with patch.object(telemetry_worker.command_dispatcher, "handle_command") as mock_handle_command:
        # Act
        telemetry_worker.command_dispatcher.on_data_received(json_data)
        # Assert
        mock_handle_command.assert_called_once_with(command_data)


def test_on_data_received_logs_warning_on_invalid_json(
    telemetry_worker: TelemetryWorker, caplog
):
    """PL: Sprawdza ostrzeżenie logowane dla niepoprawnego JSON-u.
    EN: Checks that invalid JSON produces a warning log entry.
    """
    # Arrange
    invalid_json = "this is not json"

    # Act
    with caplog.at_level(logging.WARNING):
        telemetry_worker.command_dispatcher.on_data_received(invalid_json)

    # Assert
    assert "Data error (JSON)" in caplog.text


# --- Testy dla handle_command ---


def test_handle_command_set_mode(telemetry_worker: TelemetryWorker):
    """PL: Sprawdza zmianę trybu workera po poprawnej komendzie.
    EN: Checks that a valid command changes the worker's mode.
    """
    # Arrange
    command = {"command": "set_mode", "mode": "RTH"}
    assert telemetry_worker.current_mode == "MANUAL"  # Stan początkowy

    # Act
    telemetry_worker.command_dispatcher.handle_command(command)

    # Assert
    assert telemetry_worker.current_mode == "RTH"


def test_handle_command_set_mode_invalid_mode(
    telemetry_worker: TelemetryWorker, caplog
):
    """PL: Sprawdza, że nieprawidłowy tryb nie zmienia stanu workera.
    EN: Checks that an invalid mode does not change the worker state.
    """
    # Arrange
    command = {"command": "set_mode", "mode": ""} # Empty mode or invalid

    # Act
    with caplog.at_level(logging.INFO): # changed log level check
        telemetry_worker.command_dispatcher.handle_command(command)

    # Assert
    assert telemetry_worker.current_mode == "MANUAL"


def test_handle_command_set_home(telemetry_worker: TelemetryWorker):
    """PL: Sprawdza zapis pozycji domowej po odebraniu komendy.
    EN: Checks that the home position is stored after receiving a command.
    """
    # Arrange
    home_pos = {"lat": 52.0, "lon": 21.0}
    command = {"command": "set_home", "position": home_pos}
    assert telemetry_worker.home_position is None  # Stan początkowy

    # Act
    telemetry_worker.command_dispatcher.handle_command(command)

    # Assert
    assert telemetry_worker.home_position == home_pos
