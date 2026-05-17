import unittest

from core.safety_supervisor import SafetyState, SafetySupervisor


class TestSafetySupervisorRefined(unittest.TestCase):
    def setUp(self):
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
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [],  # No obstacles
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.NORMAL)
        self.assertEqual(self.supervisor.reason, "Normal operation")

    def test_impact_detection(self):
        sensor_data = {
            "imu": {"ax": 70.0, "ay": 0.1, "az": 9.81},  # Impact > 5g
            "lidar": [],
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)
        self.assertIn("Impact detected", self.supervisor.reason)

    def test_obstacle_avoidance(self):
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [(0, 500)],  # 0.5m forward
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.AVOID)
        self.assertIn("Obstacle nearby", self.supervisor.reason)

    def test_obstacle_stop(self):
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [(0, 200)],  # 0.2m forward
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)
        self.assertIn("Obstacle too close", self.supervisor.reason)

    def test_low_battery_rth(self):
        sensor_data = {
            "imu": {"ax": 0.1, "ay": 0.1, "az": 9.81},
            "lidar": [],
            "battery": {"voltage": 10.0},  # Critical < 10.5
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.RTH)
        self.assertIn("Low battery", self.supervisor.reason)

    def test_fault_injection(self):
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
