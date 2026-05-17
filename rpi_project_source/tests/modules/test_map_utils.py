import os
import sys
import unittest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from modules import map_utils


class TestMapUtils(unittest.TestCase):
    def test_deg_to_num(self):
        # Known value: lat=0, lon=0, zoom=0 -> x=0, y=0 (1 tile total)
        x, y = map_utils.deg_to_num(0, 0, 0)
        self.assertEqual(x, 0)
        self.assertEqual(y, 0)

        # Test zoom 1 (2x2 tiles)
        # lat=0, lon=0 is center, should be top-left of bottom-right tile or similar boundary
        # Let's use specific coordinates. Berlin approx 52.5, 13.4
        # zoom 10
        x, y = map_utils.deg_to_num(52.5200, 13.4050, 10)
        # Calculated via other tools or simple logic:
        # n = 2^10 = 1024
        # x = (13.405 + 180) / 360 * 1024 = 193.405 / 360 * 1024 = ~550
        # lat_rad = 0.9166
        # y = (1 - asinh(tan(0.9166))/pi)/2 * 1024 = ~336

        self.assertEqual(x, 550)
        self.assertEqual(y, 335)

    def test_haversine_distance(self):
        # Distance between two points on equator, 1 degree apart
        # 1 degree lat/lon is approx 111km
        dist = map_utils.haversine_distance(0, 0, 0, 1)
        self.assertAlmostEqual(dist, 111195.0, delta=100)  # approx 111.195 km

        # Distance between same point
        dist = map_utils.haversine_distance(50, 10, 50, 10)
        self.assertEqual(dist, 0)

    def test_calculate_bearing(self):
        # East
        b = map_utils.calculate_bearing(0, 0, 0, 1)
        self.assertAlmostEqual(b, 90.0, delta=0.1)

        # North
        b = map_utils.calculate_bearing(0, 0, 1, 0)
        self.assertAlmostEqual(b, 0.0, delta=0.1)

        # West
        b = map_utils.calculate_bearing(0, 0, 0, -1)
        self.assertAlmostEqual(b, 270.0, delta=0.1)

        # South
        b = map_utils.calculate_bearing(0, 0, -1, 0)
        self.assertAlmostEqual(b, 180.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
