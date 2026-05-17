import os
import sys
import unittest
from unittest.mock import MagicMock

# Add the project source directory to sys.path to allow imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from logic.actuator_manager import ActuatorManager


class TestActuatorManager(unittest.TestCase):
    def setUp(self):
        # Mock the HardwareManager
        self.mock_hw_manager = MagicMock()
        self.actuator_manager = ActuatorManager(self.mock_hw_manager)

    def test_init(self):
        """Test initialization of ActuatorManager."""
        self.assertEqual(self.actuator_manager.hw_manager, self.mock_hw_manager)

    def test_set_gripper_open(self):
        """Test setting gripper to open position (-1.0)."""
        # Expected pulse: 1500 + (-1.0 * 500) = 1000
        self.actuator_manager.set_gripper(-1.0)
        self.mock_hw_manager.set_servo_pulse.assert_called_with(2, 1000)

    def test_set_gripper_closed(self):
        """Test setting gripper to closed position (1.0)."""
        # Expected pulse: 1500 + (1.0 * 500) = 2000
        self.actuator_manager.set_gripper(1.0)
        self.mock_hw_manager.set_servo_pulse.assert_called_with(2, 2000)

    def test_set_gripper_neutral(self):
        """Test setting gripper to neutral position (0.0)."""
        # Expected pulse: 1500 + (0.0 * 500) = 1500
        self.actuator_manager.set_gripper(0.0)
        self.mock_hw_manager.set_servo_pulse.assert_called_with(2, 1500)

    def test_set_gripper_no_hw_method(self):
        """Test set_gripper when HardwareManager lacks set_servo_pulse."""
        # Remove set_servo_pulse from the mock
        del self.mock_hw_manager.set_servo_pulse

        # Should not raise exception, just log warning (which we won't assert here, but ensure no crash)
        try:
            self.actuator_manager.set_gripper(0.5)
        except Exception as e:
            self.fail(f"set_gripper raised {e} unexpectedly when method missing")

    def test_cleanup(self):
        """Test cleanup method."""
        try:
            self.actuator_manager.cleanup()
        except Exception as e:
            self.fail(f"cleanup raised {e} unexpectedly")


if __name__ == "__main__":
    unittest.main()
