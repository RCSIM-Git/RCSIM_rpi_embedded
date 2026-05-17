"""
Unit tests for the SupervisorService.
Testy jednostkowe dla SupervisorService.
"""

import json
import os
import sys
from unittest.mock import MagicMock, mock_open, patch

import pytest
from core.supervisor import SupervisorService

# Add project root to the Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, project_root)


@pytest.fixture
def supervisor_service():
    """
    Provides a SupervisorService instance with a mocked socket for each test.
    Dostarcza instancję SupervisorService z zamockowanym socketem dla każdego testu.
    """
    with patch("socket.socket"):
        service = SupervisorService()
        service.sock = MagicMock()
        # Mock the config path to a predictable value for testing
        service.config_path = "/tmp/test_config.json"
        yield service


def test_handle_ping_command(supervisor_service: SupervisorService):
    """
    Tests if the 'PING' command returns a valid status response.
    Testuje, czy komenda 'PING' zwraca poprawną odpowiedź statusową.
    """
    # Arrange
    with patch("core.supervisor.get_board_info", return_value={"model_name": "TestPi"}):
        with patch.object(supervisor_service, "is_service_active", return_value=True):
            command = {"cmd": "PING"}
            addr = ("127.0.0.1", 12345)

            # Act
            supervisor_service.handle_command(command, addr)

            # Assert
            supervisor_service.sock.sendto.assert_called_once()
            response_data = json.loads(supervisor_service.sock.sendto.call_args[0][0])
            assert response_data["status"] == "PONG"
            assert response_data["model"] == "TestPi"
            assert response_data["service_active"] is True

    # The assertion was duplicated and incorrect outside the with block.
    # It has been removed. The correct assertions are inside the with block.


@patch("subprocess.run")
def test_handle_reboot_command(
    mock_subprocess_run: MagicMock, supervisor_service: SupervisorService
):
    """
    Tests if the 'REBOOT' command correctly calls the reboot command.
    Testuje, czy komenda 'REBOOT' poprawnie wywołuje komendę reboot.
    """
    # Arrange
    command = {"cmd": "REBOOT"}
    addr = ("127.0.0.1", 12345)

    # Act
    supervisor_service.handle_command(command, addr)

    # Assert
    mock_subprocess_run.assert_called_once_with(["sudo", "reboot"], check=True)
    response_data = json.loads(supervisor_service.sock.sendto.call_args[0][0])
    assert response_data["status"] == "OK"


def test_handle_get_config_command(supervisor_service: SupervisorService):
    """
    Tests if the 'GET_CONFIG' command correctly reads and returns the config file.
    Testuje, czy komenda 'GET_CONFIG' poprawnie odczytuje i zwraca plik konfiguracyjny.
    """
    # Arrange
    mock_config_data = {"setting": "value"}
    m = mock_open(read_data=json.dumps(mock_config_data))
    with patch("builtins.open", m):
        command = {"cmd": "GET_CONFIG"}
        addr = ("127.0.0.1", 12345)

        # Act
        supervisor_service.handle_command(command, addr)

        # Assert
        m.assert_called_once_with(supervisor_service.config_path, "r", encoding="utf-8")
        response_data = json.loads(supervisor_service.sock.sendto.call_args[0][0])
        assert response_data["status"] == "OK"
        assert response_data["config"] == mock_config_data


@patch("os.access", return_value=True)
@patch("os.replace")
@patch("builtins.open", new_callable=mock_open)
@patch("json.dump")
def test_handle_set_config_command_atomic_write(
    mock_json_dump,
    mock_file_open,
    mock_os_replace,
    mock_os_access,
    supervisor_service: SupervisorService,
):
    """
    Tests if 'SET_CONFIG' performs an atomic write by mocking json.dump.
    Testuje, czy 'SET_CONFIG' wykonuje atomowy zapis, mockując json.dump.
    """
    # Arrange
    new_config = {"new_setting": "new_value"}
    command = {"cmd": "SET_CONFIG", "config": new_config}
    addr = ("127.0.0.1", 12345)

    # Act
    supervisor_service.handle_command(command, addr)

    # Assert
    # Verify file was opened in write mode for the temp path
    mock_file_open.assert_called_once_with(
        supervisor_service.config_path + ".tmp", "w", encoding="utf-8"
    )
    # Verify that json.dump was called with the correct data and file handle
    handle = mock_file_open()
    mock_json_dump.assert_called_once_with(new_config, handle, indent=4)
    # Verify the atomic rename was performed
    mock_os_replace.assert_called_once_with(
        supervisor_service.config_path + ".tmp", supervisor_service.config_path
    )

    response_data = json.loads(supervisor_service.sock.sendto.call_args[0][0])
    assert response_data["status"] == "OK"


def test_handle_set_config_invalid_data(supervisor_service: SupervisorService):
    """
    Tests that 'SET_CONFIG' returns an error if 'config' data is missing or not a dict.
    Testuje, czy 'SET_CONFIG' zwraca błąd, gdy brakuje danych 'config' lub nie są one słownikiem.
    """
    # Arrange
    command = {"cmd": "SET_CONFIG", "config": "not-a-dict"}
    addr = ("127.0.0.1", 12345)

    # Act
    supervisor_service.handle_command(command, addr)

    # Assert
    response_data = json.loads(supervisor_service.sock.sendto.call_args[0][0])
    assert response_data["status"] == "ERROR"
    assert "Invalid or missing" in response_data["message"]


def test_handle_unknown_command(supervisor_service: SupervisorService):
    """
    Tests that an unknown command returns an appropriate error message.
    Testuje, czy nieznana komenda zwraca odpowiedni komunikat o błędzie.
    """
    # Arrange
    command = {"cmd": "UNKNOWN_COMMAND"}
    addr = ("127.0.0.1", 12345)

    # Act
    supervisor_service.handle_command(command, addr)

    # Assert
    response_data = json.loads(supervisor_service.sock.sendto.call_args[0][0])
    assert response_data["status"] == "ERROR"
    assert response_data["message"] == "Unknown command"
