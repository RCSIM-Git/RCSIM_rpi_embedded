"""PL: Testuje sterowanie pozycją chwytaka przez ActuatorManager.
EN: Tests gripper positioning through the ActuatorManager.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

# Add the project source directory to sys.path to allow imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from logic.actuator_manager import ActuatorManager


class TestActuatorManager(unittest.TestCase):
    """PL: Testy inicjalizacji, sterowania i zamykania menedżera aktywatorów.
    EN: Tests actuator manager initialization, control, and cleanup.
    """

    def setUp(self):
        """PL: Tworzy menedżera z atrapą sprzętową.
        EN: Creates the manager with a hardware mock.
        """
        # Mock the HardwareManager
        self.mock_hw_manager = MagicMock()
        self.actuator_manager = ActuatorManager(self.mock_hw_manager)

    def test_init(self):
        """PL: Sprawdza zapis referencji do menedżera sprzętu.
        EN: Checks that the hardware manager reference is stored.
        """
        self.assertEqual(self.actuator_manager.hw_manager, self.mock_hw_manager)

    def test_set_gripper_open(self):
        """PL: Sprawdza impuls odpowiadający otwartemu chwytakowi.
        EN: Checks the pulse corresponding to an open gripper.
        """
        # Expected pulse: 1500 + (-1.0 * 500) = 1000
        self.actuator_manager.set_gripper(-1.0)
        self.mock_hw_manager.set_servo_pulse.assert_called_with(2, 1000)

    def test_set_gripper_closed(self):
        """PL: Sprawdza impuls odpowiadający zamkniętemu chwytakowi.
        EN: Checks the pulse corresponding to a closed gripper.
        """
        # Expected pulse: 1500 + (1.0 * 500) = 2000
        self.actuator_manager.set_gripper(1.0)
        self.mock_hw_manager.set_servo_pulse.assert_called_with(2, 2000)

    def test_set_gripper_neutral(self):
        """PL: Sprawdza impuls neutralnej pozycji chwytaka.
        EN: Checks the pulse for the neutral gripper position.
        """
        # Expected pulse: 1500 + (0.0 * 500) = 1500
        self.actuator_manager.set_gripper(0.0)
        self.mock_hw_manager.set_servo_pulse.assert_called_with(2, 1500)

    def test_set_gripper_no_hw_method(self):
        """PL: Sprawdza bezpieczne działanie bez metody impulsu serwa.
        EN: Checks safe behavior when the servo pulse method is unavailable.
        """
        # Remove set_servo_pulse from the mock
        del self.mock_hw_manager.set_servo_pulse

        # Should not raise exception, just log warning (which we won't assert here, but ensure no crash)
        try:
            self.actuator_manager.set_gripper(0.5)
        except Exception as e:
            self.fail(f"set_gripper raised {e} unexpectedly when method missing")

    def test_cleanup(self):
        """PL: Sprawdza, że zamknięcie menedżera nie zgłasza wyjątku.
        EN: Checks that manager cleanup completes without an exception.
        """
        try:
            self.actuator_manager.cleanup()
        except Exception as e:
            self.fail(f"cleanup raised {e} unexpectedly")


if __name__ == "__main__":
    unittest.main()
