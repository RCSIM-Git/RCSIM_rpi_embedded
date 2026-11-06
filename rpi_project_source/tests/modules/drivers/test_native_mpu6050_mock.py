"""PL: Testuje inicjalizację i przeliczenie odczytów sterownika MPU6050.
EN: Tests MPU6050 driver initialization and sensor reading conversion.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
)

from modules.drivers.native.native_mpu6050 import NativeMPU6050


class TestNativeMPU6050(unittest.TestCase):
    """PL: Testy natywnego sterownika MPU6050 z atrapą magistrali I2C.
    EN: Native MPU6050 driver tests using an I2C bus mock.
    """

    def setUp(self):
        """PL: Tworzy atrapę magistrali I2C.
        EN: Creates an I2C bus mock.
        """
        self.mock_i2c = MagicMock()

    def test_init_success(self):
        """PL: Sprawdza inicjalizację po odczycie prawidłowego identyfikatora.
        EN: Checks initialization after reading the expected chip identifier.
        """
        # Mock MPU6050 chip ID (0x68)
        self.mock_i2c.read_byte_data.return_value = 0x68

        mpu = NativeMPU6050(self.mock_i2c)

        self.assertEqual(mpu.address, 0x68)
        self.mock_i2c.read_byte_data.assert_called_with(0x68, 0x75)  # WHO_AM_I
        self.mock_i2c.write_byte_data.assert_called_with(0x68, 0x6B, 0x00)  # Wake up

    def test_init_fail_chip_id(self):
        """PL: Sprawdza błąd inicjalizacji przy nieprawidłowym identyfikatorze.
        EN: Checks initialization failure for an unexpected chip identifier.
        """
        self.mock_i2c.read_byte_data.return_value = 0x00  # Wrong ID

        with self.assertRaises(RuntimeError):
            NativeMPU6050(self.mock_i2c)

    def test_read_raw_and_scaled(self):
        """PL: Sprawdza skalowanie surowych próbek przyspieszenia i temperatury.
        EN: Checks scaling of raw acceleration and temperature samples.
        """
        self.mock_i2c.read_byte_data.return_value = 0x68
        mpu = NativeMPU6050(self.mock_i2c)

        # Mock block data return
        # 14 bytes: Ax, Ay, Az, Temp, Gx, Gy, Gz (2 bytes each, big endian)
        # Value 16384 for Ax = 1G (if scale is 2G)
        # Value 0 for others
        # Temp = 0 -> (0/340 + 36.53) = 36.53 C
        # 16384 = 0x4000 -> 0x40, 0x00

        # Let's verify _bytes_to_int logic too with negative numbers.
        # -1 = 0xFFFF -> 255, 255

        # Data:
        # Ax: 16384 (0x40, 0x00)
        # Ay: 0
        # Az: -16384 (-1G) (0xC0, 0x00) -> 2s complement of 16384 is ... wait.
        # 16384 = 0100 0000 0000 0000
        # -16384: 1011 1111 1111 1111 + 1 = 1100 0000 0000 0000 = 0xC000

        data = [
            0x40,
            0x00,  # Ax
            0x00,
            0x00,  # Ay
            0xC0,
            0x00,  # Az
            0x00,
            0x00,  # Temp
            0x00,
            0x00,  # Gx
            0x00,
            0x00,  # Gy
            0x00,
            0x00,  # Gz
        ]

        self.mock_i2c.read_i2c_block_data.return_value = data

        scaled = mpu.read_scaled()

        self.assertAlmostEqual(scaled["ax"], 1.0)
        self.assertAlmostEqual(scaled["ay"], 0.0)
        self.assertAlmostEqual(scaled["az"], -1.0)
        self.assertAlmostEqual(scaled["temp"], 36.53)
        self.assertEqual(scaled["mx"], 0.0)


if __name__ == "__main__":
    unittest.main()
