import os
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

# We need to mock the imports before importing pca9685
sys.modules["modules.drivers.native.native_pca9685"] = MagicMock()
sys.modules["modules.drivers.native.native_i2c"] = MagicMock()
sys.modules["board"] = MagicMock()
sys.modules["busio"] = MagicMock()
sys.modules["adafruit_pca9685"] = MagicMock()

from modules.pca9685 import PCA9685


class TestPCA9685(unittest.TestCase):
    def setUp(self):
        # Reset mocks
        sys.modules["modules.drivers.native.native_pca9685"].NativePCA9685.reset_mock()
        sys.modules["modules.drivers.native.native_i2c"].I2CWrapper.reset_mock()

    def test_init_native(self):
        """Test initialization with Native driver."""
        # Ensure NATIVE_AVAILABLE is True in the module (it might be False if imports failed in the module script itself)
        # But since we mocked imports in sys.modules *before* importing pca9685, they should be available.
        # However, the module checks for ImportError on import level.
        # Let's verify if we can force the path.

        # We need to reload the module or patch constants if possible.
        # Easier to checking if PCA9685 logic picks up our mocked modules.

        with patch("modules.pca9685.NATIVE_AVAILABLE", True):
            pca = PCA9685(init_neutral=False)
            self.assertEqual(pca.driver_type, "Native")
            self.assertIsNotNone(pca.pca)

    def test_set_servo_pulse_native(self):
        """Test setting servo pulse with Native driver."""
        with patch("modules.pca9685.NATIVE_AVAILABLE", True):
            pca = PCA9685(init_neutral=False)
            pca.pca.set_us = MagicMock()  # Mock set_us method

            # Test valid range
            pca.set_servo_pulse(0, 1500)
            pca.pca.set_us.assert_called_with(0, 1500)

            # Test clamping
            pca.set_servo_pulse(0, 900)
            pca.pca.set_us.assert_called_with(0, 1000)  # Min

            pca.set_servo_pulse(0, 2100)
            pca.pca.set_us.assert_called_with(0, 2000)  # Max

            # Test force
            pca.set_servo_pulse(0, 2100, force=True)
            pca.pca.set_us.assert_called_with(0, 2100)

    def test_set_all_channels_neutral(self):
        """Test setting all channels to neutral."""
        with patch("modules.pca9685.NATIVE_AVAILABLE", True):
            pca = PCA9685(init_neutral=False)
            pca.pca.set_us = MagicMock()

            pca.set_all_channels_neutral()
            self.assertEqual(pca.pca.set_us.call_count, 16)
            pca.pca.set_us.assert_any_call(0, 1500)
            pca.pca.set_us.assert_any_call(15, 1500)

    def test_disable_all_channels(self):
        """Test disabling all channels."""
        with patch("modules.pca9685.NATIVE_AVAILABLE", True):
            pca = PCA9685(init_neutral=False)
            pca.pca.set_pwm = MagicMock()  # Native uses set_pwm(i, 0, 0) for disable

            pca.disable_all_channels()
            self.assertEqual(pca.pca.set_pwm.call_count, 16)
            pca.pca.set_pwm.assert_any_call(0, 0, 0)


if __name__ == "__main__":
    unittest.main()
