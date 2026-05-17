import os
import sys
import unittest

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from modules.planners.global_planner import GlobalPlanner


class TestGlobalPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = GlobalPlanner(resolution=0.1, inflation_radius=0.0)
        # Create a 10x10 map (100x100 pixels @ 0.1m/px = 10x10m area?)
        # Wait, resolution is m/px. So 100px * 0.1 = 10m.
        self.grid = np.zeros((100, 100), dtype=np.uint8)

    def test_straight_path(self):
        # Start (1.0, 1.0) -> Goal (1.0, 5.0) - movement along Y
        # Center is (50, 50) corresponding to (0,0)m
        # Wait, my GlobalPlanner assumes center-based coordinates?
        # Let's check implementation.
        # Yes: start_c = int(center_c + start_pose[0] / self.resolution)

        start = (0.0, 0.0)
        goal = (0.0, 4.0)  # 4 meters away

        path = self.planner.plan_path(self.grid, start, goal)
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 0)

        # Check start/end approximation
        self.assertTrue(np.allclose(path[0], start, atol=0.1))
        self.assertTrue(np.allclose(path[-1], goal, atol=0.1))

    def test_obstacle_avoidance(self):
        # Place obstacle in the middle
        # Start (0,0), Goal (0, 4.0)
        # Obstacle at (0, 2.0)
        rows, cols = self.grid.shape
        center_r, center_c = rows // 2, cols // 2

        obs_x_m = 0.0
        obs_y_m = 2.0

        # Convert to grid
        # x -> col ?? NO. My implementation:
        # start_c = int(center_c + start_pose[0] / self.resolution) -> x is column
        # start_r = int(center_r + start_pose[1] / self.resolution) -> y is row

        obs_c = int(center_c + obs_x_m / 0.1)
        obs_r = int(center_r + obs_y_m / 0.1)

        # Add wall
        self.grid[obs_r, obs_c - 5 : obs_c + 5] = 255  # Wall 1m wide

        path = self.planner.plan_path(self.grid, (0, 0), (0, 4.0))
        self.assertIsNotNone(path)

        # Check if any point in path hits obstacle
        for x, y in path:
            c = int(center_c + x / 0.1)
            r = int(center_r + y / 0.1)
            self.assertLess(self.grid[r, c], 128, f"Path hits obstacle at {x},{y}")


if __name__ == "__main__":
    unittest.main()
