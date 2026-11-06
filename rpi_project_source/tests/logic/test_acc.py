"""PL: Testuje adaptacyjny tempomat i reakcję na odległość z LiDAR.
EN: Tests adaptive cruise control and responses to LiDAR distance.
"""

import pytest
from logic.control_selector import ControlSelector


@pytest.fixture
def control_selector():
    """PL: Tworzy selektor sterowania bez zewnętrznych menedżerów.
    EN: Creates a control selector without external managers.
    """
    return ControlSelector(nav_manager=None, ai_manager=None)


def test_acc_no_lidar_data(control_selector: ControlSelector):
    """PL: Sprawdza brak redukcji gazu bez danych LiDAR.
    EN: Checks that throttle is not reduced when LiDAR data is absent.
    """
    multiplier = control_selector._apply_acc(lidar_scan=None, throttle_input=1.0)
    assert multiplier == 1.0


def test_acc_throttle_is_zero_or_negative(control_selector: ControlSelector):
    """PL: Sprawdza pominięcie ACC podczas cofania i postoju.
    EN: Checks that ACC is skipped while reversing or stationary.
    """
    lidar_scan = [(-5, 30), (0, 30), (5, 30)]  # Obstacle at 30cm
    # Reversing
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=-0.5)
    assert multiplier == 1.0
    # Stationary
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=0.0)
    assert multiplier == 1.0


def test_acc_obstacle_far_away(control_selector: ControlSelector):
    """PL: Sprawdza brak ograniczenia gazu dla odległej przeszkody.
    EN: Checks that a distant obstacle does not limit throttle.
    """
    lidar_scan = [(-5, 250), (0, 300), (5, 280)]  # Nearest obstacle at 2.5m
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 1.0


def test_acc_obstacle_in_linear_zone(control_selector: ControlSelector):
    """PL: Sprawdza liniowe ograniczanie gazu w aktywnym zakresie ACC.
    EN: Checks linear throttle reduction within the ACC range.
    """
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
    """PL: Sprawdza wyzerowanie mnożnika dla zbyt bliskiej przeszkody.
    EN: Checks that the multiplier becomes zero for a very close obstacle.
    """
    lidar_scan = [(-5, 40), (0, 30), (5, 45)]  # Nearest obstacle at 30cm
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 0.0


def test_acc_filters_angles(control_selector: ControlSelector):
    """PL: Sprawdza uwzględnianie tylko przeszkód w przednim kącie skanu.
    EN: Checks that only obstacles within the forward scan angle are considered.
    """
    # Obstacle at 30cm but outside the arc, should be ignored
    lidar_scan = [(-15, 30), (15, 30)]
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 1.0

    # Obstacle at 30cm just inside the arc, should be detected
    lidar_scan = [(-15, 100), (10, 30)]
    multiplier = control_selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 0.0


def test_acc_disabled():
    """PL: Sprawdza pominięcie ACC po wyłączeniu go w konfiguracji.
    EN: Checks that ACC is bypassed when disabled in configuration.
    """
    config = {"safety": {"acc_enabled": False, "acc_min_dist_m": 0.5, "acc_max_dist_m": 2.0}}
    selector = ControlSelector(nav_manager=None, ai_manager=None, config=config)
    lidar_scan = [(0, 30)]  # 30cm obstacle directly in front
    multiplier = selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 1.0


def test_acc_custom_ranges():
    """PL: Sprawdza użycie skonfigurowanych progów odległości ACC.
    EN: Checks that ACC uses configured distance thresholds.
    """
    # Custom config: min_dist = 1.0m, max_dist = 3.0m
    config = {"safety": {"acc_enabled": True, "acc_min_dist_m": 1.0, "acc_max_dist_m": 3.0}}
    selector = ControlSelector(nav_manager=None, ai_manager=None, config=config)

    # Obstacle at 3.0m -> multiplier should be 1.0
    lidar_scan = [(0, 300)]
    multiplier = selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == pytest.approx(1.0)

    # Obstacle at 1.0m -> multiplier should be 0.0
    lidar_scan = [(0, 100)]
    multiplier = selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == pytest.approx(0.0)

    # Obstacle at 2.0m -> multiplier should be 0.5 (exactly in middle of [1.0, 3.0])
    lidar_scan = [(0, 200)]
    multiplier = selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == pytest.approx(0.5)


def test_acc_custom_ranges_edge_case():
    """PL: Sprawdza awaryjne ograniczenie przy nieprawidłowym zakresie ACC.
    EN: Checks safe limiting when the ACC distance range is invalid.
    """
    config = {"safety": {"acc_enabled": True, "acc_min_dist_m": 2.0, "acc_max_dist_m": 1.0}}
    selector = ControlSelector(nav_manager=None, ai_manager=None, config=config)
    lidar_scan = [(0, 150)]  # 1.5m obstacle
    multiplier = selector._apply_acc(lidar_scan=lidar_scan, throttle_input=1.0)
    assert multiplier == 0.0
