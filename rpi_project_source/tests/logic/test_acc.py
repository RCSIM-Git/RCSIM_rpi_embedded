"""
Unit tests for the ACC functionality in ControlSelector.
"""

import pytest
from logic.control_selector import ControlSelector


@pytest.fixture
def control_selector():
    """Provides a ControlSelector instance with mocked dependencies."""
    return ControlSelector(nav_manager=None, ai_manager=None)


def test_acc_no_lidar_data(control_selector: ControlSelector):
    """Tests that ACC returns a multiplier of 1.0 when no LiDAR data is available."""
    multiplier = control_selector._apply_acc(lidar_scan=None, throttle_input=1.0)
    assert multiplier == 1.0


def test_acc_throttle_is_zero_or_negative(control_selector: ControlSelector):
    """Tests that ACC is not applied when the vehicle is reversing or stationary."""
    lidar_scan = [(-5, 30), (0, 30), (5, 30)]  # Obstacle at 30cm
    # Reversing
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=-0.5)
    assert multiplier == 1.0
    # Stationary
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=0.0)
    assert multiplier == 1.0


def test_acc_obstacle_far_away(control_selector: ControlSelector):
    """Tests that the multiplier is 1.0 when the nearest obstacle is far away."""
    lidar_scan = [(-5, 250), (0, 300), (5, 280)]  # Nearest obstacle at 2.5m
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 1.0


def test_acc_obstacle_in_linear_zone(control_selector: ControlSelector):
    """Tests the linear throttle reduction zone."""
    # Obstacle exactly at 2.0m -> multiplier should be 1.0
    lidar_scan = [(-2, 200), (3, 210)]
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == pytest.approx(1.0)

    # Obstacle exactly at 0.5m -> multiplier should be 0.0
    lidar_scan = [(-8, 60), (0, 50), (8, 55)]
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == pytest.approx(0.0)

    # Obstacle in the middle (1.25m) -> multiplier should be 0.5
    lidar_scan = [(0, 125)]
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == pytest.approx(0.5)


def test_acc_obstacle_too_close(control_selector: ControlSelector):
    """Tests that the multiplier is 0.0 when an obstacle is too close."""
    lidar_scan = [(-5, 40), (0, 30), (5, 45)]  # Nearest obstacle at 30cm
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 0.0


def test_acc_filters_angles(control_selector: ControlSelector):
    """Tests that only obstacles within the -10 to +10 degree arc are considered."""
    # Obstacle at 30cm but outside the arc, should be ignored
    lidar_scan = [(-15, 30), (15, 30)]
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 1.0

    # Obstacle at 30cm just inside the arc, should be detected
    lidar_scan = [(-15, 100), (10, 30)]
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 0.0
