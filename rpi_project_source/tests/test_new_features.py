"""
Unit tests for new features in the RPi application.
"""

import os
import sys
from unittest.mock import MagicMock

import numpy as np
import pytest

# Mock hardware modules before any application code is imported.
sys.modules["smbus2"] = MagicMock()
sys.modules["serial"] = MagicMock()
sys.modules["RPi"] = MagicMock()
sys.modules["RPi.GPIO"] = MagicMock()
sys.modules["picamera2"] = MagicMock()
sys.modules["picamera2.picamera2"] = MagicMock()

# Ensure the application's source code is on the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from logic.control_selector import ControlSelector
# We need to import the modules under test AFTER mocking is set up
from logic.navigation_manager import NavigationManager
from modules.ai_manager import AIManager

# -- Tests for NavigationManager --


@pytest.fixture
def nav_manager():
    """Provides a fresh NavigationManager instance for each test."""
    return NavigationManager(kp=0.5, ki=0.1, kd=0.05)


@pytest.mark.parametrize(
    "point_a, point_b, expected_bearing",
    [
        ((52.2297, 21.0122), (52.4064, 21.0122), 0.0),  # Warsaw -> North
        ((52.2297, 21.0122), (52.2297, 21.2), 89.9),  # Warsaw -> East
        ((34.0522, -118.2437), (36.7783, -119.4179), 341.0),  # LA -> Central CA
        ((51.5074, -0.1278), (48.8566, 2.3522), 148.1),  # London -> Paris
    ],
)
def test_calculate_bearing(nav_manager, point_a, point_b, expected_bearing):
    """Test the bearing calculation between two GPS points."""
    lat1, lon1 = point_a
    lat2, lon2 = point_b
    bearing = nav_manager._calculate_bearing(lat1, lon1, lat2, lon2)
    assert bearing == pytest.approx(expected_bearing, abs=0.2)


@pytest.mark.parametrize(
    "point_a, point_b, expected_distance_m",
    [
        ((40.7128, -74.0060), (34.0522, -118.2437), 3935710),  # NYC to LA
        ((51.5074, -0.1278), (48.8566, 2.3522), 343513),  # London to Paris
        ((0, 0), (0, 0), 0),  # Same point
    ],
)
def test_haversine_distance(nav_manager, point_a, point_b, expected_distance_m):
    """Test the haversine distance calculation."""
    lat1, lon1 = point_a
    lat2, lon2 = point_b
    distance = nav_manager._haversine_distance(lat1, lon1, lat2, lon2)
    assert distance == pytest.approx(expected_distance_m, rel=0.01)


def test_calculate_steering_clipping(nav_manager):
    """Test if steering value is correctly clipped to [-1.0, 1.0]."""
    # Force a large error that would result in steering > 1.0
    nav_manager.kp = 100.0  # Exaggerate Kp
    # Target is East (90), current is North (0) -> error is 90
    steering = nav_manager.calculate_steering(0, 0, 0, 0, 1, 0.1)
    assert steering == 1.0

    # Target is West (270), current is North (0) -> error is -90
    steering_neg = nav_manager.calculate_steering(0, 0, 0, 0, -1, 0.1)
    assert steering_neg == -1.0


def test_update_rth_arrived_home(nav_manager):
    """Test RTH when the vehicle has arrived at the home position."""
    home = {"lat": 52.2297, "lon": 21.0122}
    # Current position is within the 2.0m arrival threshold
    current = {"lat": 52.229701, "lon": 21.012201}
    steering, throttle, arrived = nav_manager.update_rth(
        True, home, current["lat"], current["lon"], 0, 0.1
    )
    assert steering == 0.0
    assert throttle == 0.0
    assert arrived


def test_pid_error_wrapping(nav_manager):
    """Test that the PID error correctly wraps around the -180/180 degree boundary."""
    # Target is ~354 deg, current is 10 deg. Error should be -15.7, not 344.
    nav_manager.calculate_steering(0, 0, 10, 0.1, -0.01, 0.1)  # Bearing is ~354.3 deg
    assert nav_manager.previous_error == pytest.approx(-15.7, abs=1.0)

    # Target is ~5.7 deg, current is 350. Error should be 15.7, not -344.3.
    nav_manager.calculate_steering(0, 0, 350, 0.1, 0.01, 0.1)  # Bearing is ~5.7 deg
    assert nav_manager.previous_error == pytest.approx(15.7, abs=1.0)


def test_pid_integral_clamping(nav_manager):
    """Test that the integral term is correctly clamped."""
    nav_manager.integral_max = 0.5
    nav_manager.integral_min = -0.5
    nav_manager.ki = 1.0
    # Force a large error over a long time to build up the integral term
    # Target is East (90), current is North (0) -> error = 90
    for _ in range(100):
        nav_manager.calculate_steering(0, 0, 0, 0, 1, 0.1)
    assert nav_manager.integral == 0.5  # Should be clamped to max

    nav_manager.integral = 0  # Reset
    # Target is West (270), current is North (0) -> error = -90
    for _ in range(100):
        nav_manager.calculate_steering(0, 0, 0, 0, -1, 0.1)
    assert nav_manager.integral == -0.5  # Should be clamped to min


def test_pid_dt_zero(nav_manager):
    """Test that the derivative term is zero when dt is zero to prevent division by zero."""
    nav_manager.calculate_steering(0, 0, 0, 1, 0, 0.1)  # Initial step
    # Second step with dt=0
    steering = nav_manager.calculate_steering(0, 0, 0, 1, 0, 0)
    # P-term should be there, I-term might be, but D-term must be 0
    # We can't directly check d_term, but we can ensure it doesn't crash.
    assert isinstance(steering, float)


# -- Tests for ControlSelector --


@pytest.fixture
def mock_nav_manager():
    return MagicMock(spec=NavigationManager)


@pytest.fixture
def mock_ai_manager():
    return MagicMock(spec=AIManager)


@pytest.fixture
def control_selector(mock_nav_manager, mock_ai_manager):
    """Provides a ControlSelector instance with mocked dependencies."""
    return ControlSelector(nav_manager=mock_nav_manager, ai_manager=mock_ai_manager)


# Tests for ACC (_apply_acc)
def test_acc_no_lidar_or_throttle(control_selector):
    assert control_selector._apply_acc(None, 1.0) == 1.0
    assert control_selector._apply_acc([], -0.5) == 1.0


def test_acc_clear_path(control_selector):
    lidar_scan = [(0, 300.0)]  # 3 meters away
    assert control_selector._apply_acc(lidar_scan, 0.8) == 1.0


def test_acc_warning_zone(control_selector):
    # Object at 1.25m -> (1.25 - 0.5) / 1.5 = 0.5 multiplier
    lidar_scan = [(0, 125.0)]  # 1.25 meters
    assert control_selector._apply_acc(lidar_scan, 0.8) == pytest.approx(0.5)


def test_acc_danger_zone(control_selector):
    lidar_scan = [(0, 40.0)]  # 0.4 meters
    assert control_selector._apply_acc(lidar_scan, 0.8) == 0.0


def test_acc_irrelevant_points(control_selector):
    """Points are outside the -10 to 10 degree cone."""
    lidar_scan = [(30, 50.0), (-45, 60.0)]
    assert control_selector._apply_acc(lidar_scan, 0.8) == 1.0


# Tests for main logic (process_frame)
def test_process_frame_failsafe_mode(control_selector):
    steering, throttle = control_selector.process_frame("FAILSAFE", {})
    assert steering == 0.0
    assert throttle == 0.0


def test_process_frame_manual_mode(control_selector):
    frame_data = {"manual_controls": {"steering": 0.7, "throttle": -0.5}}
    steering, throttle = control_selector.process_frame("MANUAL", frame_data)
    assert steering == 0.7
    assert throttle == -0.5


def test_process_frame_manual_mode_with_acc(control_selector):
    """ACC should reduce throttle in manual mode if moving forward."""
    frame_data = {
        "manual_controls": {"steering": 0.2, "throttle": 0.8},
        "lidar_scan": [(0, 40.0)],  # 0.4m -> ACC stop
    }
    steering, throttle = control_selector.process_frame("MANUAL", frame_data)
    assert steering == 0.2
    assert throttle == 0.0  # 0.8 * 0.0 = 0.0


def test_process_frame_rth_mode(control_selector, mock_nav_manager):
    mock_nav_manager.update_rth.return_value = (
        0.5,
        0.2,
        False,
    )  # Steering, throttle, not arrived
    frame_data = {
        "gps_data": {"lat": 1, "lon": 1},
        "imu_data": {"heading": 90},
        "home_position": {"lat": 2, "lon": 2},
        "dt": 0.1,
    }
    steering, throttle = control_selector.process_frame("RTH", frame_data)
    mock_nav_manager.update_rth.assert_called_once()
    assert steering == 0.5
    assert throttle == pytest.approx(0.2)  # Default RTH throttle


def test_process_frame_rth_arrived(control_selector, mock_nav_manager):
    mock_nav_manager.update_rth.return_value = (0.0, 0.0, True)  # Arrived
    frame_data = {"gps_data": {}, "imu_data": {}, "home_position": {}}
    steering, throttle = control_selector.process_frame("RTH", frame_data)
    assert steering == 0.0
    assert throttle == 0.0


def test_process_frame_rth_missing_data(control_selector, mock_nav_manager):
    steering, throttle = control_selector.process_frame("RTH", {})  # No data
    assert not mock_nav_manager.update_rth.called
    assert steering == 0.0
    assert throttle == 0.0


def test_process_frame_ai_mode(control_selector, mock_ai_manager):
    mock_ai_manager.predict.return_value = (0.9, 0.6)
    image_mock = np.zeros((120, 160, 3), dtype=np.uint8)
    frame_data = {"image": image_mock}
    steering, throttle = control_selector.process_frame("AI_AUTOPILOT", frame_data)
    mock_ai_manager.predict.assert_called_once()
    assert steering == 0.9
    assert throttle == 0.6


def test_process_frame_ai_mode_no_ai_manager(mock_nav_manager):
    # Create a selector without an AI manager
    selector = ControlSelector(nav_manager=mock_nav_manager, ai_manager=None)
    image_mock = np.zeros((120, 160, 3), dtype=np.uint8)
    frame_data = {"image": image_mock}
    steering, throttle = selector.process_frame("AI_AUTOPILOT", frame_data)
    assert steering == 0.0
    assert throttle == 0.0


def test_update_rth_not_active(nav_manager):
    """Test RTH when it is not active."""
    steering, throttle, arrived = nav_manager.update_rth(
        is_rth_active=False,
        home_position={"lat": 1, "lon": 1},
        current_lat=0,
        current_lon=0,
        current_heading=0,
        dt=0.1,
    )
    assert steering == 0.0
    assert throttle == 0.0
    assert not arrived


def test_process_frame_ai_mode_no_image_data(control_selector, mock_ai_manager):
    """Test AI mode when the camera image is missing."""
    frame_data = {"image": None}  # No image
    steering, throttle = control_selector.process_frame("AI_AUTOPILOT", frame_data)
    assert not mock_ai_manager.predict.called
    assert steering == 0.0
    assert throttle == 0.0
