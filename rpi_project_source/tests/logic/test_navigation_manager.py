"""PL: Testuje obliczenia geograficzne, regulator PID i logikę powrotu RTH.
EN: Tests geographic calculations, PID control, and return-to-home behavior.
"""

from unittest.mock import MagicMock

import pytest
from logic.navigation_manager import NavigationManager, GPSPoint
from modules.map_utils import calculate_bearing, haversine_distance

# --- Testy dla funkcji pomocniczych (teraz z map_utils) ---


def test_calculate_bearing_new_york_to_los_angeles():
    """PL: Sprawdza azymut na trasie z Nowego Jorku do Los Angeles.
    EN: Checks the bearing from New York to Los Angeles.
    """
    lat1, lon1 = 40.7128, -74.0060  # New York
    lat2, lon2 = 34.0522, -118.2437  # Los Angeles

    bearing = calculate_bearing(lat1, lon1, lat2, lon2)

    assert (bearing + 360) % 360 == pytest.approx(273.7, abs=1.0)


def test_haversine_distance_london_to_paris():
    """PL: Sprawdza odległość Haversine między Londynem a Paryżem.
    EN: Checks the Haversine distance between London and Paris.
    """
    lat1, lon1 = 51.5072, -0.1276  # London
    lat2, lon2 = 48.8566, 2.3522  # Paris

    distance = haversine_distance(lat1, lon1, lat2, lon2)

    # Oczekiwana odległość to ok. 344 km
    assert distance == pytest.approx(344000, abs=1000)


# --- Testy dla logiki PID (calculate_steering) ---


def test_pid_proportional_term(monkeypatch):
    """PL: Sprawdza człon proporcjonalny i ograniczenie sterowania do zakresu.
    EN: Checks the proportional term and steering output clamping.
    """
    monkeypatch.setattr(
        "logic.navigation_manager.calculate_bearing", lambda *args: 90.0
    )
    nav_manager = NavigationManager(kp=0.1, ki=0.0, kd=0.0)  # Kp = 0.1
    # Target 90 stopni, aktualny kurs 70 stopni -> błąd = 20 -> sterowanie = 0.1 * 20 = 2.0 -> obcięte do 1.0
    steering = nav_manager.calculate_steering(GPSPoint(0, 0), 70, GPSPoint(0, 0), 1.0)
    assert steering == pytest.approx(1.0)

    # Test z ujemnym błędem i obcięciem
    nav_manager.kp = -0.1
    steering = nav_manager.calculate_steering(GPSPoint(0, 0), 70, GPSPoint(0, 0), 1.0)
    assert steering == pytest.approx(-1.0)


def test_pid_error_wrapping(monkeypatch):
    """PL: Sprawdza normalizację błędu kursu przez granicę ±180 stopni.
    EN: Checks heading error normalization across the ±180 degree boundary.
    """
    monkeypatch.setattr(
        "logic.navigation_manager.calculate_bearing", lambda *args: 10.0
    )
    nav_manager = NavigationManager(kp=0.05, ki=0.0, kd=0.0)  # Kp = 0.05
    # Target 10 stopni, aktualny kurs 350 stopni -> błąd = 10 - 350 = -340 -> zawinięty do 20
    steering = nav_manager.calculate_steering(GPSPoint(0, 0), 350, GPSPoint(0, 0), 1.0)
    assert steering == pytest.approx(1.0)


def test_pid_derivative_term_with_dt_zero(monkeypatch):
    """PL: Sprawdza zerowanie członu D przy zerowym kroku czasu.
    EN: Checks that the derivative term is zero when the time step is zero.
    """
    monkeypatch.setattr(
        "logic.navigation_manager.calculate_bearing", lambda *args: 90.0
    )
    nav_manager = NavigationManager(kp=0.0, ki=0.0, kd=0.1)
    nav_manager.calculate_steering(GPSPoint(0, 0), 70, GPSPoint(0, 0), 1.0)
    steering = nav_manager.calculate_steering(GPSPoint(0, 0), 70, GPSPoint(0, 0), 0.0)
    assert steering == 0.0


def test_pid_integral_anti_windup(monkeypatch):
    """PL: Sprawdza ograniczenie całki PID przez zabezpieczenie anti-windup.
    EN: Checks PID integral clamping through anti-windup protection.
    """
    monkeypatch.setattr(
        "logic.navigation_manager.calculate_bearing", lambda *args: 90.0
    )
    nav_manager = NavigationManager(kp=0.0, ki=0.1, kd=0.0)
    nav_manager.integral_max = 0.5

    for _ in range(100):
        nav_manager.calculate_steering(GPSPoint(0, 0), 70, GPSPoint(0, 0), 0.1)  # Błąd = 20, dt=0.1

    assert nav_manager.integral == pytest.approx(0.5)
    steering = nav_manager.calculate_steering(GPSPoint(0, 0), 70, GPSPoint(0, 0), 0.1)
    assert steering == pytest.approx(0.1 * 0.5)


# --- Testy dla logiki RTH (update_rth) ---


def test_update_rth_inactive(monkeypatch):
    """PL: Sprawdza neutralne sterowanie, gdy RTH jest nieaktywny.
    EN: Checks neutral controls when RTH is inactive.
    """
    nav_manager = NavigationManager()
    steering, throttle, arrived = nav_manager.update_rth(
        False, {"lat": 0, "lon": 0}, 1, 1, 90, 1.0
    )
    assert steering == 0.0
    assert throttle == 0.0
    assert not arrived


def test_update_rth_arrived_at_home(monkeypatch):
    """PL: Sprawdza sygnalizację dotarcia do domu w granicy tolerancji.
    EN: Checks the arrival flag when the vehicle is within home tolerance.
    """
    monkeypatch.setattr(
        "logic.navigation_manager.haversine_distance", lambda *args: 1.5
    )
    nav_manager = NavigationManager()

    steering, throttle, arrived = nav_manager.update_rth(
        True, {"lat": 0, "lon": 0}, 0, 0, 90, 1.0
    )

    assert steering == 0.0
    assert throttle == 0.0
    assert arrived


def test_update_rth_calculates_steering_when_active(monkeypatch):
    """PL: Sprawdza wywołanie regulatora skrętu podczas aktywnego RTH.
    EN: Checks steering controller invocation while RTH is active.
    """
    monkeypatch.setattr(
        "logic.navigation_manager.haversine_distance", lambda *args: 100
    )
    mock_steering = MagicMock(return_value=0.5)
    monkeypatch.setattr(NavigationManager, "calculate_steering", mock_steering)

    nav_manager = NavigationManager()
    home_pos = {"lat": 10, "lon": 10}

    steering, throttle, arrived = nav_manager.update_rth(True, home_pos, 1, 1, 90, 0.1)

    mock_steering.assert_called_once_with(
        GPSPoint(1, 1), 90, GPSPoint(home_pos["lat"], home_pos["lon"]), 0.1
    )
    assert steering == 0.5
    assert throttle == 0.2
    assert not arrived


def test_update_rth_deceleration_near_home(monkeypatch):
    """PL: Sprawdza redukcję gazu w pobliżu pozycji domowej.
    EN: Checks throttle reduction near the home position.
    """
    monkeypatch.setattr(
        "logic.navigation_manager.haversine_distance", lambda *args: 5.0
    )
    monkeypatch.setattr(NavigationManager, "calculate_steering", lambda *args: 0.1)

    nav_manager = NavigationManager()
    steering, throttle, arrived = nav_manager.update_rth(
        True, {"lat": 0, "lon": 0}, 0.00004, 0.00004, 0, 0.1
    )
    assert not arrived
    assert throttle == pytest.approx(0.1)

