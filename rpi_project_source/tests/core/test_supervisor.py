"""PL: Testuje odpowiedzi SupervisorService na polecenia administracyjne.
EN: Tests SupervisorService responses to administrative commands.
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
    """PL: Tworzy usługę supervisor z atrapą gniazda dla każdego testu.
    EN: Creates a supervisor service with a socket mock for each test.
    """
    with patch("socket.socket"):
        service = SupervisorService()
        service.sock = MagicMock()
        # Mock the config path to a predictable value for testing
        service.config_path = "/tmp/test_config.json"
        yield service


def test_handle_ping_command(supervisor_service: SupervisorService):
    """PL: Sprawdza status, model i aktywność usługi w odpowiedzi PING.
    EN: Checks service status, model, and activity in the PING response.
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
    """PL: Sprawdza wywołanie polecenia restartu i odpowiedź OK.
    EN: Checks that reboot is invoked and an OK response is returned.
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
    """PL: Sprawdza odczyt i zwrócenie zawartości konfiguracji.
    EN: Checks that configuration contents are read and returned.
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
    """PL: Weryfikuje zapis tymczasowy, atomową podmianę i odpowiedź OK.
    EN: Verifies the temporary write, atomic replacement, and OK response.
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
    """PL: Sprawdza błąd odpowiedzi dla nieprawidłowych danych konfiguracji.
    EN: Checks for an error response when configuration data is invalid.
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
    """PL: Sprawdza błąd odpowiedzi dla nieobsługiwanej komendy.
    EN: Checks the error response for an unsupported command.
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
