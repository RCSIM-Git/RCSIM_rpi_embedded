"""PL: Testuje dynamiczne progi zatrzymania i unikania przeszkód.
EN: Tests dynamic stopping and obstacle avoidance thresholds.
"""

import os
import sys
import unittest

# Add project root to sys.path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from core.safety_supervisor import SafetyState, SafetySupervisor


class TestSafetyDynamic(unittest.TestCase):
    """PL: Testy zależności progów bezpieczeństwa od prędkości.
    EN: Tests for safety thresholds that depend on vehicle speed.
    """

    def setUp(self):
        """PL: Tworzy supervisor z progami bezpieczeństwa testowymi.
        EN: Creates a safety supervisor with test thresholds.
        """
        self.config = {
            "safety": {
                "emergency_stop_distance_m": 0.3,
                "avoid_dist_m": 0.8,
                "speed_factor": 0.4,
                "impact_g_threshold": 5.0,
                "critical_battery_v": 10.5,
            }
        }
        self.supervisor = SafetySupervisor(self.config)

    def test_static_stop_distance(self):
        """PL: Sprawdza zatrzymanie przy przeszkodzie w bazowym zasięgu.
        EN: Checks stopping for an obstacle within the base distance.
        """
        # Speed 0 m/s -> Stop dist should be base (0.3m)
        sensor_data = {"lidar": [(0, 250)]}  # 250mm = 0.25m < 0.3m
        self.supervisor.update(sensor_data, current_speed_mps=0.0)
        self.assertEqual(self.supervisor.state, SafetyState.STOP)

    def test_dynamic_stop_distance_low_speed(self):
        """PL: Sprawdza rozszerzony dystans zatrzymania przy małej prędkości.
        EN: Checks the expanded stopping distance at low speed.
        """
        # Speed 1 m/s -> Stop dist = 0.3 + (1.0 * 0.4) = 0.7m
        # Obstacle at 0.6m -> Should STOP
        sensor_data = {"lidar": [(0, 600)]}  # 600mm = 0.6m < 0.7m
        self.supervisor.update(sensor_data, current_speed_mps=1.0)
        self.assertEqual(self.supervisor.state, SafetyState.STOP)
        self.assertIn("DynLimit", self.supervisor.reason)

    def test_dynamic_stop_distance_high_speed(self):
        """PL: Sprawdza rozszerzony dystans zatrzymania przy dużej prędkości.
        EN: Checks the expanded stopping distance at high speed.
        """
        # Speed 5 m/s -> Stop dist = 0.3 + (5.0 * 0.4) = 2.3m
        # Obstacle at 2.0m -> Should STOP
        sensor_data = {"lidar": [(0, 2000)]}  # 2000mm = 2.0m < 2.3m
        self.supervisor.update(sensor_data, current_speed_mps=5.0)
        self.assertEqual(self.supervisor.state, SafetyState.STOP)

    def test_dynamic_avoid_distance(self):
        """PL: Sprawdza wejście w strefę unikania przy dynamicznym progu.
        EN: Checks entry into the avoidance zone using a dynamic threshold.
        """
        # Speed 2 m/s -> Stop dist = 0.3 + 0.8 = 1.1m
        # Avoid dist = max(0.8, 1.1 + 0.5) = 1.6m
        # Obstacle at 1.5m -> Should AVOID
        sensor_data = {"lidar": [(0, 1500)]}  # 1.5m
        self.supervisor.update(sensor_data, current_speed_mps=2.0)
        self.assertEqual(self.supervisor.state, SafetyState.AVOID)

    def test_safe_distance_high_speed(self):
        """PL: Sprawdza normalny stan przy bezpiecznej odległości i dużej prędkości.
        EN: Checks the normal state at a safe distance and high speed.
        """
        # Speed 5 m/s -> Stop dist = 2.3m
        # Obstacle at 3.0m -> Should be NORMAL
        sensor_data = {"lidar": [(0, 3000)]}
        self.supervisor.update(sensor_data, current_speed_mps=5.0)
        self.assertEqual(self.supervisor.state, SafetyState.NORMAL)


if __name__ == "__main__":
    unittest.main()
