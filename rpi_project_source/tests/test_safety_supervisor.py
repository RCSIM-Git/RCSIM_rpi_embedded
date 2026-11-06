"""PL: Testy stanów bezpieczeństwa i ograniczania sterowania.

EN: Tests for safety states and control limiting.
"""

import unittest

from core.safety_supervisor import SafetyState, SafetySupervisor


class TestSafetySupervisor(unittest.TestCase):
    """PL: Sprawdza reakcje nadzorcy bezpieczeństwa na sensory.

    EN: Verify safety supervisor responses to sensor inputs.
    """

    def setUp(self):
        """PL: Tworzy supervisor z progami bezpieczeństwa testu.

        EN: Create the supervisor with test safety thresholds.
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

    def test_normal_operation(self):
        """PL: Sprawdza normalną pracę przy bezpiecznych odczytach.

        EN: Verify normal operation with safe sensor readings.
        """
        sensor_data = {
            "lidar": [(0, 2000), (10, 2500)],  # far away
            "imu": {"ax": 0.1, "ay": 0.0, "az": 9.81},  # normal gravity (1g)
            "battery": {"voltage": 12.0},  # healthy
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.NORMAL)

        s, t = self.supervisor.process_controls(0.5, 0.5)
        self.assertEqual(s, 0.5)
        self.assertEqual(t, 0.5)

    def test_emergency_stop_lidar(self):
        """PL: Sprawdza awaryjne zatrzymanie przy bliskiej przeszkodzie.

        EN: Verify emergency stop for a nearby obstacle.
        """
        sensor_data = {
            "lidar": [(0, 250)],  # 25cm < 30cm
            "imu": {"ax": 0.1, "ay": 0.0, "az": 9.81},
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)

        s, t = self.supervisor.process_controls(0.5, 0.5)
        self.assertEqual(t, 0.0)  # Throttle must be 0

    def test_avoid_lidar(self):
        """PL: Sprawdza ograniczenie gazu w strefie omijania przeszkody.

        EN: Verify throttle limiting in the obstacle-avoidance zone.
        """
        sensor_data = {
            "lidar": [(0, 500)],  # 50cm > 30cm, but < 80cm
            "imu": {"ax": 0.1, "ay": 0.0, "az": 9.81},
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.AVOID)

        s, t = self.supervisor.process_controls(0.5, 0.8)
        self.assertLessEqual(t, 0.3)  # Throttle should be limited

    def test_impact_stop(self):
        """PL: Sprawdza zatrzymanie po wykryciu silnego uderzenia.

        EN: Verify stop behavior after a strong impact is detected.
        """
        sensor_data = {
            "lidar": [(0, 2000)],
            "imu": {"ax": 70.0, "ay": 0.0, "az": 9.81},  # ~7.1g -> impact ~6.1g > 5g
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)
        self.assertTrue(self.supervisor.impact_detected)

    def test_battery_rth(self):
        """PL: Sprawdza ograniczenie pracy przy krytycznym napięciu baterii.

        EN: Verify operation is limited at critical battery voltage.
        """
        sensor_data = {
            "lidar": [(0, 2000)],
            "imu": {"ax": 0.1, "ay": 0.0, "az": 1.0},
            "battery": {"voltage": 10.0},  # < 10.5V
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.BATTERY_LIMIT)


if __name__ == "__main__":
    unittest.main()
