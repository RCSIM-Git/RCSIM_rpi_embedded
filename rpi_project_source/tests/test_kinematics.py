import pytest
import math
from modules.utils.kinematics import estimate_motion, ControlInput, KinematicConfig

def test_estimate_motion_straight():
    # Test straight line kinematics
    control = ControlInput(throttle=0.5, steering=0.0)
    config = KinematicConfig(max_speed_mps=2.0, max_steer_rad=0.5, wheelbase_m=0.25)
    
    dx, dy, dyaw = estimate_motion(control=control, dt=0.1, config=config)
    
    # 0.5 * 2.0 = 1.0 m/s velocity. 1.0 * 0.1s = 0.1m dx
    assert dx == pytest.approx(0.1)
    assert dy == 0.0
    assert dyaw == 0.0

def test_estimate_motion_turn_no_imu():
    # Test turn with model-based dyaw calculation
    control = ControlInput(throttle=1.0, steering=0.5)
    config = KinematicConfig(max_speed_mps=2.0, max_steer_rad=0.5, wheelbase_m=0.2)
    
    # steer_angle = 0.5 * 0.5 = 0.25 rad
    # v = 1.0 * 2.0 = 2.0 m/s
    # dyaw_expected = (2.0 / 0.2) * tan(0.25) * 0.1
    dyaw_expected = (2.0 / 0.2) * math.tan(0.25) * 0.1
    
    dx, dy, dyaw = estimate_motion(control=control, dt=0.1, config=config)
    assert dx == pytest.approx(0.2)
    assert dy == 0.0
    assert dyaw == pytest.approx(dyaw_expected)

def test_estimate_motion_turn_with_imu():
    # Test gyro prioritization for dyaw
    control = ControlInput(throttle=1.0, steering=0.5)
    config = KinematicConfig(max_speed_mps=2.0, max_steer_rad=0.5, wheelbase_m=0.2)
    imu_data = {"gz": 15.0} # 15 deg/s
    
    # dyaw should be 15 deg to radians * 0.1s
    dyaw_expected = math.radians(15.0) * 0.1
    
    dx, dy, dyaw = estimate_motion(control=control, dt=0.1, config=config, imu_data=imu_data)
    assert dx == pytest.approx(0.2)
    assert dy == 0.0
    assert dyaw == pytest.approx(dyaw_expected)
