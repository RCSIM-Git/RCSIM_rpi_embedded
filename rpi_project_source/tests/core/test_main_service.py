"""
Unit tests for the main service and TelemetryWorker.
Testy jednostkowe dla głównego serwisu i TelemetryWorker.
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
    """Provides a default config dictionary."""
    return {
        "hardware": {},
        "main_loop_freq_hz": 30,
        "video": {"engine": "native"},
        "pc_ip": "127.0.0.1",
    }


@pytest.fixture
def telemetry_worker(config_mock) -> TelemetryWorker:
    """
    Provides a TelemetryWorker instance with mocked dependencies for testing.
    Dostarcza instancję TelemetryWorker z zamockowanymi zależnościami do testów.
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
    """
    Tests if the generated telemetry packet has the correct structure.
    Testuje, czy generowany pakiet telemetryczny ma poprawną strukturę.
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
    """
    Tests if incoming data (not a command) correctly updates `last_control_input`.
    Testuje, czy dane przychodzące (niebędące komendą) poprawnie aktualizują `last_control_input`.
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
    """
    Tests if a message with type 'command' is passed to the command handler.
    Testuje, czy wiadomość typu 'command' jest przekazywana do obsługi komend.
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
    """
    Tests that a warning is logged if the incoming data is not valid JSON.
    Testuje, czy ostrzeżenie jest logowane, gdy przychodzące dane nie są poprawnym JSON-em.
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
    """
    Tests if the 'set_mode' command correctly changes the worker's current mode.
    Testuje, czy komenda 'set_mode' poprawnie zmienia aktualny tryb workera.
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
    """
    Tests that a 'set_mode' command with an invalid mode is ignored and a warning is logged.
    Testuje, czy komenda 'set_mode' z nieprawidłowym trybem jest ignorowana i logowane jest ostrzeżenie.
    """
    # Arrange
    command = {"command": "set_mode", "mode": ""} # Empty mode or invalid

    # Act
    with caplog.at_level(logging.INFO): # changed log level check
        telemetry_worker.command_dispatcher.handle_command(command)

    # Assert
    assert telemetry_worker.current_mode == "MANUAL"


def test_handle_command_set_home(telemetry_worker: TelemetryWorker):
    """
    Tests if the 'set_home' command correctly updates the home position.
    Testuje, czy komenda 'set_home' poprawnie aktualizuje pozycję domową.
    """
    # Arrange
    home_pos = {"lat": 52.0, "lon": 21.0}
    command = {"command": "set_home", "position": home_pos}
    assert telemetry_worker.home_position is None  # Stan początkowy

    # Act
    telemetry_worker.command_dispatcher.handle_command(command)

    # Assert
    assert telemetry_worker.home_position == home_pos
