"""PL: Szczegółowe testy przyczyn i resetowania stanów bezpieczeństwa.

EN: Detailed tests for safety-state causes and reset behavior.
"""

import unittest

from core.safety_supervisor import SafetyState, SafetySupervisor


class TestSafetySupervisorRefined(unittest.TestCase):
    """PL: Sprawdza przyczyny stanów i wstrzykiwanie usterek.

    EN: Verify safety-state reasons and fault injection.
    """

    def setUp(self):
        """PL: Tworzy nadzorcę z progami używanymi w testach.

        EN: Create the supervisor with thresholds used by the tests.
        """
        self.config = {
            "safety": {
                "emergency_stop_dist_m": 0.3,
                "avoid_dist_m": 0.8,
                "impact_g_threshold": 5.0,
                "critical_battery_v": 10.5,
            }
        }
        self.supervisor = SafetySupervisor(self.config)

    def test_normal_state(self):
        """PL: Sprawdza stan normalny przy bezpiecznych danych.

        EN: Verify the normal state with safe input data.
        """
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [],  # No obstacles
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.NORMAL)
        self.assertEqual(self.supervisor.reason, "Normal operation")

    def test_impact_detection(self):
        """PL: Sprawdza wykrywanie uderzenia i przejście do STOP.

        EN: Verify impact detection and transition to STOP.
        """
        sensor_data = {
            "imu": {"ax": 70.0, "ay": 0.1, "az": 9.81},  # Impact > 5g
            "lidar": [],
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)
        self.assertIn("Impact detected", self.supervisor.reason)

    def test_obstacle_avoidance(self):
        """PL: Sprawdza przejście do omijania przy przeszkodzie.

        EN: Verify transition to avoidance when an obstacle is nearby.
        """
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [(0, 500)],  # 0.5m forward
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.AVOID)
        self.assertIn("Obstacle nearby", self.supervisor.reason)

    def test_obstacle_stop(self):
        """PL: Sprawdza zatrzymanie przed zbyt bliską przeszkodą.

        EN: Verify stopping before an obstacle that is too close.
        """
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [(0, 200)],  # 0.2m forward
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)
        self.assertIn("Obstacle too close", self.supervisor.reason)

    def test_low_battery_rth(self):
        """PL: Sprawdza ograniczenie baterii przy krytycznym napięciu.

        EN: Verify battery limiting at a critical voltage.
        """
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [],
            "battery": {"voltage": 10.0},  # Critical < 10.5
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.BATTERY_LIMIT)
        self.assertIn("CRITICAL Battery", self.supervisor.reason)

    def test_fault_injection(self):
        """PL: Sprawdza zatrzymanie po wstrzyknięciu usterki i reset.

        EN: Verify fault-injected stop behavior and reset handling.
        """
        self.supervisor.inject_fault("force_stop", True)
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [],
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)
        self.assertIn("FAULT_INJECTION", self.supervisor.reason)

        # Test Reset
        self.supervisor.reset_impact()
        self.assertEqual(self.supervisor.state, SafetyState.NORMAL)
        self.assertFalse(self.supervisor.faults["force_stop"])


if __name__ == "__main__":
    unittest.main()
