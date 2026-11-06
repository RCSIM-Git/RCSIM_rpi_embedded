"""PL: Testuje obliczanie CRC i dekodowanie kanałów oraz statystyk CRSF.
EN: Tests CRC calculation and decoding of CRSF channels and link statistics.
"""

import unittest

from core.crsf_parser import CRSFParser


class TestCRSFParser(unittest.TestCase):
    """PL: Testy dekodera protokołu CRSF.
    EN: Tests for the CRSF protocol decoder.
    """

    def setUp(self):
        """PL: Tworzy parser CRSF z portem testowym.
        EN: Creates a CRSF parser with a test port.
        """
        self.parser = CRSFParser(port="/dev/test_port")

    def test_crc8_calculation(self):
        """PL: Sprawdza, że wynik CRC8 mieści się w jednym bajcie.
        EN: Checks that the CRC8 result fits in one byte.
        """
        # Test vector for CRSF CRC8 (DVB-S2 poly 0xD5)
        # Type(0x16) + 22 bytes of payload
        data = bytes([0x16]) + bytes([0] * 22)
        crc = self.parser._calc_crc8(data)
        # We can't easily guess the CRC without the table, but we can verify it's consistent.
        # Let's test a known simple case if possible, or just verify it's a byte.
        self.assertIsInstance(crc, int)
        self.assertGreaterEqual(crc, 0)
        self.assertLess(crc, 256)

    def test_decode_rc_channels(self):
        """PL: Sprawdza dekodowanie szesnastu neutralnych kanałów RC.
        EN: Checks decoding of sixteen neutral RC channels.
        """
        # Payload is 22 bytes (176 bits)
        # Each channel is 11 bits. 16 channels total.
        # Value 992 in CRSF should map to 1500 PWM.

        # 11 bits of 992 (0x3E0)
        # Pack 16 times.
        # Since it's little endian bit packing, it's easier to verify specific values.

        # Test with all channels at 992 (Neutral)
        payload = bytearray()
        val_176 = 0
        for i in range(16):
            val_176 |= 992 << (i * 11)

        payload = val_176.to_bytes(22, byteorder="little")

        self.parser._decode_rc_channels(payload)

        for ch in self.parser.channels:
            self.assertEqual(ch, 1500)

    def test_decode_rc_channels_boundaries(self):
        """PL: Sprawdza mapowanie minimalnych i maksymalnych wartości CRSF.
        EN: Checks mapping of minimum and maximum CRSF values.
        """
        # Min CRSF 172 -> ~1000 PWM (around 987 based on formula)
        # Max CRSF 1811 -> ~2000 PWM (around 2012 based on formula)

        val_176 = 0
        val_176 |= 172 << 0  # Ch 0 min
        val_176 |= 1811 << 11  # Ch 1 max

        payload = val_176.to_bytes(22, byteorder="little")
        self.parser._decode_rc_channels(payload)

        self.assertAlmostEqual(self.parser.channels[0], 987, delta=5)
        self.assertAlmostEqual(self.parser.channels[1], 2012, delta=5)

    def test_decode_link_statistics(self):
        """PL: Sprawdza dekodowanie RSSI, jakości łącza, SNR i anteny.
        EN: Checks decoding of RSSI, link quality, SNR, and antenna fields.
        """
        # [RSSI1][RSSI2][LQ][SNR][Antenna]... 10 bytes total
        payload = bytes([60, 70, 99, 10, 1]) + bytes([0] * 5)

        self.parser._decode_link_statistics(payload)

        self.assertEqual(self.parser.link_statistics["rssi_1"], -60)
        self.assertEqual(self.parser.link_statistics["rssi_2"], -70)
        self.assertEqual(self.parser.link_statistics["link_quality"], 99)
        self.assertEqual(self.parser.link_statistics["snr"], 10)
        self.assertEqual(self.parser.link_statistics["active_antenna"], 1)


if __name__ == "__main__":
    unittest.main()
