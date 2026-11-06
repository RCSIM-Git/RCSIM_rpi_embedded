"""PL: Testuje wybór trybu sterowania, RTH, AI i ograniczenia ACC.
EN: Tests control mode selection, RTH, AI, and ACC constraints.
"""

from unittest.mock import MagicMock

import pytest

# --- ControlSelector Tests ---


@pytest.fixture
def mock_nav_manager():
    """PL: Zwraca atrapę menedżera nawigacji.
    EN: Returns a mock navigation manager.
    """
    return MagicMock()


@pytest.fixture
def mock_ai_manager():
    """PL: Zwraca atrapę menedżera AI.
    EN: Returns a mock AI manager.
    """
    return MagicMock()


def test_control_selector_failsafe_mode(mock_nav_manager, mock_ai_manager):
    """PL: Sprawdza wyzerowanie skrętu i gazu w trybie FAILSAFE.
    EN: Checks that steering and throttle are zero in FAILSAFE mode.
    """
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
    """PL: Sprawdza przekazanie ręcznych wartości sterowania.
    EN: Checks passthrough of manual control values.
    """
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
    """PL: Sprawdza sterowanie RTH, gdy dostępne są wymagane dane.
    EN: Checks RTH controls when the required data is available.
    """
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
    """PL: Sprawdza zatrzymanie po dotarciu do pozycji domowej.
    EN: Checks that the vehicle stops after reaching the home position.
    """
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
    """PL: Sprawdza bezpieczne zatrzymanie przy braku danych RTH.
    EN: Checks safe stopping when RTH sensor data is missing.
    """
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
    """PL: Sprawdza użycie predykcji AI dla dostępnego obrazu.
    EN: Checks AI prediction use when an image is available.
    """
    # Arrange
    from logic.control_selector import ControlSelector

    mock_ai_manager.predict.return_value = ([], {"steering": 0.9, "throttle": 0.6})
    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {"image": "mock_image_data"}

    # Act
    steering, throttle = selector.process_frame("AI_AUTOPILOT", frame_data)

    # Assert
    mock_ai_manager.predict.assert_called_once_with(
        "mock_image_data", sensor_data=frame_data
    )
    assert steering == 0.9
    assert throttle == 0.6


def test_control_selector_reuses_precomputed_ai_controls(mock_nav_manager, mock_ai_manager):
    """PL: Sprawdza użycie gotowych sterowań AI bez ponownej predykcji.
    EN: Checks reuse of precomputed AI controls without another prediction.
    """
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {
        "image": "mock_image_data",
        "ai_controls": {"steering": -0.4, "throttle": 0.7},
        "planner_cmd": (-0.4, 0.7, 1.0),
    }

    assert selector.process_frame("FULL_AUTOPILOT", frame_data) == (-0.4, 0.7)
    mock_ai_manager.predict.assert_not_called()


def test_control_selector_ai_steer_only_keeps_manual_throttle(
    mock_nav_manager, mock_ai_manager
):
    """PL: Sprawdza zachowanie ręcznego gazu w trybie AI-only-steering.
    EN: Checks that manual throttle is retained in AI steering-only mode.
    """
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {
        "image": "mock_image_data",
        "ai_controls": {"steering": -0.4, "throttle": 0.7},
        "manual_controls": {"steering": 0.2, "throttle": 0.3},
    }

    assert selector.process_frame("AI_STEER_ONLY", frame_data) == (-0.4, 0.3)


def test_control_selector_stops_on_missing_ai_controls(mock_nav_manager, mock_ai_manager):
    """PL: Sprawdza zatrzymanie, gdy predykcja AI nie zawiera sterowań.
    EN: Checks for stopping when AI prediction provides no controls.
    """
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager, mock_ai_manager)
    frame_data = {"image": "mock_image_data", "ai_controls": None}

    assert selector.process_frame("FULL_AUTOPILOT", frame_data) == (0.0, 0.0)
    mock_ai_manager.predict.assert_not_called()


def test_control_selector_ai_mode_no_manager(mock_nav_manager):
    """PL: Sprawdza bezpieczne sterowanie AI bez menedżera AI.
    EN: Checks safe AI mode behavior without an AI manager.
    """
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
    """PL: Sprawdza brak ograniczenia gazu bez skanu LiDAR.
    EN: Checks that throttle is not limited when LiDAR data is absent.
    """
    # Arrange
    from logic.control_selector import ControlSelector

    selector = ControlSelector(mock_nav_manager)
    frame_data = {"manual_controls": {"steering": 0.0, "throttle": 0.8}}

    # Act
    steering, throttle = selector.process_frame("MANUAL", frame_data)

    # Assert
    assert throttle == 0.8  # No change


def test_acc_obstacle_far_away(mock_nav_manager):
    """PL: Sprawdza brak ograniczenia gazu dla odległej przeszkody.
    EN: Checks that a distant obstacle does not limit throttle.
    """
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
    """PL: Sprawdza proporcjonalne ograniczenie gazu w zakresie ACC.
    EN: Checks proportional throttle reduction within the ACC range.
    """
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
    """PL: Sprawdza wyzerowanie gazu dla przeszkody w małej odległości.
    EN: Checks that throttle is zeroed for a nearby obstacle.
    """
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
    """PL: Sprawdza pominięcie ACC podczas cofania pojazdu.
    EN: Checks that ACC is skipped while the vehicle is reversing.
    """
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
