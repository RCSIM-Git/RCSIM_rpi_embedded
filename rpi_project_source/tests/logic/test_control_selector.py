from unittest.mock import MagicMock

import pytest

# --- ControlSelector Tests ---


@pytest.fixture
def mock_nav_manager():
    """Provides a mock NavigationManager instance."""
    return MagicMock()


@pytest.fixture
def mock_ai_manager():
    """Provides a mock AIManager instance."""
    return MagicMock()


def test_control_selector_failsafe_mode(mock_nav_manager, mock_ai_manager):
    """Tests if FAILSAFE mode always returns (0, 0)."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {"manual_controls": {"steering": 1.0, "throttle": 1.0}}

    # Act
    steering, throttle = selector.process_frame("FAILSAFE", frame_data)

    # Assert
    assert steering == 0.0
    assert throttle == 0.0


def test_control_selector_manual_mode(mock_nav_manager, mock_ai_manager):
    """Tests if MANUAL mode returns direct manual control inputs."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {"manual_controls": {"steering": 0.75, "throttle": -0.5}}

    # Act
    steering, throttle = selector.process_frame("MANUAL", frame_data)

    # Assert
    assert steering == 0.75
    assert throttle == -0.5


def test_control_selector_rth_mode_success(mock_nav_manager, mock_ai_manager):
    """Tests RTH mode when all data is available."""
    # Arrange
    from logic.control_selector import ControlSelector

    mock_nav_manager.update_rth.return_value = (0.5, 0.2, False)  # steer, thr, arrived
    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {
        "gps": {"lat": 10, "lon": 10, "fix_quality": 4},
        "imu": {"heading": 90},
        "home_position": {"lat": 11, "lon": 11},
        "dt": 0.1,
    }

    # Act
    steering, throttle = selector.process_frame("RTH", frame_data)

    # Assert
    mock_nav_manager.update_rth.assert_called_once()
    assert steering == 0.5
    assert throttle == 0.2  # RTH throttle is constant


def test_control_selector_rth_mode_arrived(mock_nav_manager, mock_ai_manager):
    """Tests RTH mode when the vehicle has arrived."""
    # Arrange
    from logic.control_selector import ControlSelector

    mock_nav_manager.update_rth.return_value = (0.0, 0.0, True)  # arrived = True
    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {
        "gps": {"lat": 10, "lon": 10, "fix_quality": 4},
        "imu": {"heading": 90},
        "home_position": {"lat": 10, "lon": 10},
    }

    # Act
    steering, throttle = selector.process_frame("RTH", frame_data)

    # Assert
    assert steering == 0.0
    assert throttle == 0.0


def test_control_selector_rth_mode_no_data(mock_nav_manager, mock_ai_manager):
    """Tests RTH mode when required sensor data is missing."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {}  # Missing gps, imu, home_position

    # Act
    steering, throttle = selector.process_frame("RTH", frame_data)

    # Assert
    assert steering == 0.0
    assert throttle == 0.0
    mock_nav_manager.update_rth.assert_not_called()


def test_control_selector_ai_mode_success(mock_nav_manager, mock_ai_manager):
    """Tests AI mode when the AI manager and image are available."""
    # Arrange
    from logic.control_selector import ControlSelector

    mock_ai_manager.predict.return_value = (0.9, 0.6)
    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {"image": "mock_image_data"}

    # Act
    steering, throttle = selector.process_frame("AI_AUTOPILOT", frame_data)

    # Assert
    mock_ai_manager.predict.assert_called_once_with("mock_image_data", frame_data)
    assert steering == 0.9
    assert throttle == 0.6


def test_control_selector_ai_mode_no_manager(mock_nav_manager):
    """Tests AI mode when the AI manager is not provided."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager, ai_manager=None)
    frame_data = {"image": "mock_image_data"}

    # Act
    steering, throttle = selector.process_frame("AI_AUTOPILOT", frame_data)

    # Assert
    assert steering == 0.0
    assert throttle == 0.0


# --- ACC (Adaptive Cruise Control) Tests ---


def test_acc_no_lidar_scan(mock_nav_manager):
    """Tests that ACC returns a full multiplier (1.0) if no LiDAR data is present."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager)
    frame_data = {"manual_controls": {"steering": 0.0, "throttle": 0.8}}

    # Act
    steering, throttle = selector.process_frame("MANUAL", frame_data)

    # Assert
    assert throttle == 0.8  # No change


def test_acc_obstacle_far_away(mock_nav_manager):
    """Tests that ACC returns a full multiplier if the nearest obstacle is far."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager)
    # Obstacle at 5m (5000mm)
    frame_data = {
        "manual_controls": {"steering": 0.0, "throttle": 0.8},
        "lidar": [(0, 5000)],
    }

    # Act
    steering, throttle = selector.process_frame("MANUAL", frame_data)

    # Assert
    assert throttle == 0.8  # No change


def test_acc_obstacle_in_range_proportional_reduction(mock_nav_manager):
    """Tests if ACC proportionally reduces throttle within its active range."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager)
    # Obstacle at 1.25m (1250mm), halfway between 0.5m and 2.0m
    # Expected multiplier = (1.25 - 0.5) / 1.5 = 0.5
    frame_data = {
        "manual_controls": {"steering": 0.0, "throttle": 0.8},
        "lidar": [(5, 1250)],
    }

    # Act
    steering, throttle = selector.process_frame("MANUAL", frame_data)

    # Assert
    assert throttle == pytest.approx(0.8 * 0.5)


def test_acc_obstacle_too_close(mock_nav_manager):
    """Tests if ACC reduces throttle to zero when an obstacle is too close."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager)
    # Obstacle at 0.4m (400mm)
    frame_data = {
        "manual_controls": {"steering": 0.0, "throttle": 0.8},
        "lidar": [(-5, 400)],
    }

    # Act
    steering, throttle = selector.process_frame("MANUAL", frame_data)

    # Assert
    assert throttle == 0.0


def test_acc_is_ignored_when_reversing(mock_nav_manager):
    """Tests that ACC does not engage if the vehicle is reversing."""
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager)
    # Obstacle very close, but throttle is negative
    frame_data = {
        "manual_controls": {"steering": 0.0, "throttle": -0.5},
        "lidar": [(-5, 400)],
    }

    # Act
    steering, throttle = selector.process_frame("MANUAL", frame_data)

    # Assert
    assert throttle == -0.5  # No change
