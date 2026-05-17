import os
import sys
import unittest

# Add the project source directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from core.safety_supervisor import SafetyState, SafetySupervisor


class TestSafetySupervisor(unittest.TestCase):
    def setUp(self):
        self.config = {
            "safety": {
                "emergency_stop_dist_m": 0.5,
                "avoid_dist_m": 1.0,
                "impact_g_threshold": 2.0,
                "critical_battery_v": 10.0,
            }
        }
        self.supervisor = SafetySupervisor(self.config)

    def test_init(self):
        """Test initialization and config loading."""
        self.assertEqual(self.supervisor.stop_dist_m, 0.5)
        self.assertEqual(self.supervisor.state, SafetyState.NORMAL)

    def test_update_normal(self):
        """Test update with no threats."""
        sensor_data = {
            "imu": {
                "ax": 0.0,
                "ay": 0.0,
                "az": 9.81,
            },  # Stationary on level ground (1g in Z)
            "lidar": [(0, 2000), (90, 2000)],  # Far away
            "battery": {"voltage": 12.0},
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.NORMAL)

    def test_impact_detection(self):
        """Test impact detection (High G-force)."""
        # Threshold is 2.0g. Data in m/s^2 (BMX160 format)
        # Simulating: ax=29.4 m/s^2 (≈3g), ay=0, az=9.81 m/s^2 (stationary Z-axis)
        # net_acc = sqrt(29.4^2 + 0 + 0^2) = 29.4 m/s^2 = 3.0g > 2.0g threshold
        sensor_data = {
            "imu": {"ax": 29.4, "ay": 0.0, "az": 9.81}  # Raw acceleration in m/s^2
        }
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)
        self.assertTrue(self.supervisor.impact_detected)
        self.assertIn("Impact detected", self.supervisor.reason)

    def test_no_impact_on_level_ground(self):
        """Test that stationary robot on level ground does NOT trigger impact."""
        # Robot on level ground: ax=0, ay=0, az=1g (9.81 m/s^2)
        # net_acc = sqrt(0^2 + 0^2 + (9.81-9.81)^2) = 0 m/s^2 = 0g < 2.0g → NO IMPACT
        sensor_data = {"imu": {"ax": 0.0, "ay": 0.0, "az": 9.81}}
        state = self.supervisor.update(sensor_data)
        self.assertNotEqual(
            state, SafetyState.STOP, "Robot on level ground should NOT trigger impact"
        )
        self.assertFalse(self.supervisor.impact_detected)

    def test_obstacle_avoid_zone(self):
        """Test obstacle entering avoidance zone."""
        # Avoid dist is 1.0m (1000mm). Stop is 0.5m.
        # Object at 0.8m (800mm)
        sensor_data = {"lidar": [(0, 800)]}
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.AVOID)

    def test_obstacle_stop_zone(self):
        """Test obstacle entering stop zone."""
        # Stop dist is 0.5m (500mm).
        # Object at 0.4m (400mm)
        sensor_data = {"lidar": [(0, 400)]}
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.STOP)

    def test_low_battery(self):
        """Test low battery triggers RTH/Warning."""
        # Critical is 10.0V
        sensor_data = {"battery": {"voltage": 9.5}}
        state = self.supervisor.update(sensor_data)
        self.assertEqual(state, SafetyState.RTH)

    def test_process_controls_normal(self):
        """Test control passthrough in NORMAL state."""
        self.supervisor.state = SafetyState.NORMAL
        steering, throttle = self.supervisor.process_controls(0.5, 0.8)
        self.assertEqual(steering, 0.5)
        self.assertEqual(throttle, 0.8)

    def test_process_controls_stop(self):
        """Test control override in STOP state."""
        self.supervisor.state = SafetyState.STOP
        steering, throttle = self.supervisor.process_controls(0.5, 0.8)
        self.assertEqual(
            steering, 0.5
        )  # Steering might remain? Logic says "return steering, 0.0"
        self.assertEqual(throttle, 0.0)

    def test_process_controls_avoid(self):
        """Test control limiting in AVOID state."""
        self.supervisor.state = SafetyState.AVOID
        # Throttle should be capped (logic says min(throttle, 0.25))
        steering, throttle = self.supervisor.process_controls(0.5, 0.8)
        self.assertEqual(steering, 0.5)
        self.assertEqual(throttle, 0.25)

        # If throttle is lower than cap, should pass through
        steering, throttle = self.supervisor.process_controls(0.5, 0.1)
        self.assertEqual(throttle, 0.1)

    def test_reset_impact(self):
        """Test resetting impact state."""
        self.supervisor.state = SafetyState.STOP
        self.supervisor.impact_detected = True

        self.supervisor.reset_impact()

        self.assertEqual(self.supervisor.state, SafetyState.NORMAL)
        self.assertFalse(self.supervisor.impact_detected)

    def test_fault_injection(self):
        """Test fault injection mechanisms."""
        self.supervisor.inject_fault("force_stop", True)
        state = self.supervisor.update({})
        self.assertEqual(state, SafetyState.STOP)
        self.assertIn("FAULT_INJECTION", self.supervisor.reason)


if __name__ == "__main__":
    unittest.main()
