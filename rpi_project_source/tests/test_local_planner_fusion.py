import logging
import os
import sys
import unittest

import numpy as np

# Add rpi_project_source to path
sys.path.append(os.path.abspath("RCSIMDEPLOY/rpi_project_source"))

from modules.planners.local_planner import LocalPlanner

# Configure logging
logging.basicConfig(level=logging.INFO)


class TestLocalPlannerFusion(unittest.TestCase):
    def setUp(self):
        self.planner = LocalPlanner(grid_size=200, resolution=0.05)
        self.camera_extrinsics = {"x": 0.1, "y": 0.0, "z": 0.15, "pitch": 0.0}

    def test_lidar_update(self):
        pose = (0.0, 0.0, 0.0)
        lidar_points = [(0.0, 1000.0)]
        self.planner.update_occupancy_from_lidar_and_yolo(pose, lidar_points)
        gx, gy = 120, 100
        val = np.max(self.planner.costmap[gx - 1 : gx + 2, gy - 1 : gy + 2])
        self.assertGreaterEqual(val, self.planner.OBSTACLE_COST)

    def test_yolo_update(self):
        pose = (0.0, 0.0, 0.0)
        lidar_points = []
        detections = [
            {"bbox": [0.4, 0.4, 0.6, 1.0], "label": "cone", "confidence": 0.9}
        ]
        self.planner.update_occupancy_from_lidar_and_yolo(
            pose, lidar_points, detections, self.camera_extrinsics
        )
        dist = 0.42
        gx = 100 + int(dist / 0.05)
        gy = 100
        found = False
        radius = 3
        submap = self.planner.costmap[
            gx - radius : gx + radius + 1, gy - radius : gy + radius + 1
        ]
        if np.any(submap >= self.planner.OBSTACLE_COST):
            found = True
        self.assertTrue(found, "YOLO detection not projected correctly on costmap")

    def test_fusion_inflate(self):
        pose = (0.0, 0.0, 0.0)
        lidar_points = [(0.0, 2000.0)]
        self.planner.update_occupancy_from_lidar_and_yolo(pose, lidar_points)
        gx = 100 + int(2.0 / 0.05)
        gy = 100
        self.assertGreaterEqual(
            self.planner.costmap[gx + 1, gy], self.planner.OBSTACLE_COST
        )

    def test_reactive_planning(self):
        # 1. No obstacles -> Go straight (0 steering)
        self.planner.clear_map()
        steering, throttle, safety = self.planner.plan_reactive()
        # Should be roughly 0 steering, high safety
        self.assertAlmostEqual(steering, 0.0, delta=0.1)
        self.assertGreater(safety, 0.8)

        # 2. Obstacle in front (1m)
        pose = (0.0, 0.0, 0.0)
        lidar_points = [(0.0, 1000.0)]  # 1m front
        self.planner.update_occupancy_from_lidar_and_yolo(pose, lidar_points)

        # Plan
        steering, throttle, safety = self.planner.plan_reactive()

        # Should steer (avoidance) or stop if blocked
        # With 1 obstacle at 0 deg, it should find gap left or right.
        # Since map is symmetric and search starts from -60 to 60, depending on impl it might pick either.
        # But steering should NOT be 0 (unless 0 is the only gap, which is blocked).
        # Actually with obstacle at 0, gap at +/- small angle is blocked.
        # It should pick a wider angle.

        # Verify steering is significant
        self.assertNotAlmostEqual(steering, 0.0, delta=0.01)

        # 3. Obstacle to the LEFT (Positive Y) -> Steer RIGHT (Negative)
        self.planner.clear_map()
        # 1m distance, 20 degrees left
        lidar_points = [(20.0, 1000.0)]
        self.planner.update_occupancy_from_lidar_and_yolo(pose, lidar_points)

        steering, throttle, safety = self.planner.plan_reactive()

        # Expect negative steering (Right)
        # Note: Depending on gap finding, it might find a gap further Left if wide enough?
        # But Right gap is closer to 0 deg (Forward) than Left gap (beyond 20 deg).
        # Gap right: -60 to +15 approx. Center ~ -22.
        # Gap left: +25 to +60. Center ~ +42.
        # It picks widest? Or closest to 0?
        # Impl: "Pick widest gap" -> max(clusters, key=len).
        # Right gap (-60 to +15) is 75 deg. Left gap (+25 to +60) is 35 deg.
        # So it should pick Right gap -> Negative steering.

        self.assertLess(steering, 0.0)


if __name__ == "__main__":
    unittest.main()
