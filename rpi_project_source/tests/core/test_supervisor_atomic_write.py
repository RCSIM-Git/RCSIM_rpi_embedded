# --- Supervisor Tests ---


def test_supervisor_set_config_atomic_write(fs):
    """
    Tests if the Supervisor's _set_config method performs an atomic write
    (write to .tmp file, then rename).
    """
    # Arrange
    from core.supervisor import SupervisorService

    config_path = "/app/config.json"
    tmp_path = config_path + ".tmp"
    # Create a fake file system and a placeholder config file
    fs.create_file(config_path, contents='{"old": "data"}')

    service = SupervisorService()
    # Point the service to the fake config file
    service.config_path = config_path
    new_config = {"new": "data"}

    # Act
    result = service._set_config(new_config)

    # Assert
    assert result["status"] == "OK"
    # Check that the temp file was created and then removed
    assert not fs.exists(tmp_path)
    # Check that the final config file has the new content
    with open(config_path, "r") as f:
        import json

        content = json.load(f)
    assert content == new_config


def test_supervisor_set_config_io_error_during_write(fs):
    """
    Tests if the Supervisor correctly handles an I/O error during the
    initial write to the .tmp file and cleans up the partial file.
    """
    # Arrange
    from core.supervisor import SupervisorService

    config_path = "/app/config.json"
    tmp_path = config_path + ".tmp"
    fs.create_file(config_path, contents='{"old": "data"}')

    service = SupervisorService()
    service.config_path = config_path
    new_config = {"new": "data"}

    # Act
    # Simulate an I/O error by making the directory read-only
    fs.chmod("/app", 0o555)
    result = service._set_config(new_config)
    # Restore permissions for cleanup check
    fs.chmod("/app", 0o755)

    # Assert
    assert result["status"] == "ERROR"
    assert "Permission denied" in result["message"]
    # Ensure the potentially corrupt .tmp file was removed
    assert not fs.exists(tmp_path)
    # Ensure the original config file is untouched
    with open(config_path, "r") as f:
        import json

        content = json.load(f)
    assert content == {"old": "data"}
