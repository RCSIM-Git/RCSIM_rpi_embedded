import pytest

# --- NavigationManager Tests ---


def test_navigation_manager_calculate_steering_pid(monkeypatch):
    """
    Tests the PID controller in NavigationManager with a given heading error.
    """
    # Arrange
    from logic.navigation_manager import NavigationManager

    # Mock the bearing calculation to produce a constant target of 90 degrees
    monkeypatch.setattr(NavigationManager, "_calculate_bearing", lambda *args: 90.0)
    nav_manager = NavigationManager(kp=0.1, ki=0.01, kd=0.05)
    nav_manager.previous_error = 0
    nav_manager.integral = 0

    # Act
    # Current heading is 80, target is 90 -> error = 10
    steering = nav_manager.calculate_steering(0, 0, 80, 0, 0, dt=0.1)

    # Assert
    # P-term = 0.1 * 10 = 1.0
    # I-term = 0.01 * (0 + 10 * 0.1) = 0.01
    # D-term = 0.05 * (10 - 0) / 0.1 = 5.0
    # Total = 1.0 + 0.01 + 5.0 = 6.01 -> clipped to 1.0
    assert steering == pytest.approx(1.0)
    assert nav_manager.integral == pytest.approx(1.0)
    assert nav_manager.previous_error == pytest.approx(10.0)


def test_navigation_manager_rth_arrived(monkeypatch):
    """
    Tests the RTH logic to confirm it returns 'arrived' when close to home.
    """
    # Arrange
    from logic.navigation_manager import NavigationManager

    # Mock the distance calculation to return a value less than the threshold (2.0m)
    monkeypatch.setattr(NavigationManager, "_haversine_distance", lambda *args: 1.5)
    nav_manager = NavigationManager()

    # Act
    steering, throttle, arrived = nav_manager.update_rth(
        is_rth_active=True,
        home_position={"lat": 1, "lon": 1},
        current_lat=1,
        current_lon=1,
        current_heading=0,
        dt=0.1,
    )

    # Assert
    assert arrived is True
    assert steering == 0.0
    assert throttle == 0.0


def test_navigation_manager_rth_not_arrived(monkeypatch):
    """
    Tests the RTH logic when still far from home, expecting steering output.
    """
    # Arrange
    from logic.navigation_manager import NavigationManager

    monkeypatch.setattr(NavigationManager, "_haversine_distance", lambda *args: 10.0)
    # Mock calculate_steering to isolate the logic of this method
    monkeypatch.setattr(NavigationManager, "calculate_steering", lambda *args: -0.8)
    nav_manager = NavigationManager()

    # Act
    steering, throttle, arrived = nav_manager.update_rth(
        is_rth_active=True,
        home_position={"lat": 2, "lon": 2},
        current_lat=1,
        current_lon=1,
        current_heading=0,
        dt=0.1,
    )

    # Assert
    assert arrived is False
    assert steering == -0.8  # Should return the value from calculate_steering
    assert throttle == 0.2  # RTH mode has a constant throttle
