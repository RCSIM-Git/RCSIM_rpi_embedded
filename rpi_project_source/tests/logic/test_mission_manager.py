import os
import sys
import unittest
from unittest.mock import MagicMock

# Add the project source directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from logic.mission_manager import MissionManager


class TestMissionManager(unittest.TestCase):
    def setUp(self):
        self.mock_nav_manager = MagicMock()
        # Setup nav_manager attributes that MissionManager accesses
        self.mock_nav_manager.current_path = None

        self.mission_manager = MissionManager(self.mock_nav_manager)

    def test_start_mission(self):
        """Test starting a mission with waypoints."""
        waypoints = [(10.0, 10.0), (20.0, 20.0)]
        self.mission_manager.start_mission(waypoints, loop=False)

        self.assertTrue(self.mission_manager.is_running)
        self.assertFalse(self.mission_manager.loop_mission)
        self.assertEqual(len(self.mission_manager.mission_queue), 2)

        # Check first item is loaded (index 0 -> becomes 1 after _next_item call in start)
        self.assertIsNotNone(self.mission_manager.current_mission_item)
        self.assertEqual(
            self.mission_manager.current_mission_item["target"], (10.0, 10.0)
        )
        self.assertEqual(self.mission_manager.current_waypoint_index, 1)

    def test_stop_mission(self):
        """Test stopping a mission."""
        self.mission_manager.is_running = True
        self.mission_manager.stop_mission()

        self.assertFalse(self.mission_manager.is_running)
        self.assertIsNone(self.mission_manager.current_mission_item)
        self.mock_nav_manager.clear_path.assert_called()

    def test_update_reached_waypoint(self):
        """Test update logic when a waypoint is reached."""
        waypoints = [(10.0, 10.0), (20.0, 20.0)]
        self.mission_manager.start_mission(waypoints)

        # Current target is (10, 10). Simulate robot being at (10, 10)
        current_pose = (10.0, 10.0, 0.0)
        grid_map = MagicMock()

        # First update - should detect we reached wp1 and switch to wp2
        self.mission_manager.update(current_pose, grid_map)

        self.assertEqual(
            self.mission_manager.current_mission_item["target"], (20.0, 20.0)
        )
        self.assertEqual(self.mission_manager.current_waypoint_index, 2)
        # Verify clear_path was called when switching waypoints
        self.mock_nav_manager.clear_path.assert_called()

    def test_update_replanning(self):
        """Test that update triggers replanning if path is missing and not at target."""
        waypoints = [(10.0, 10.0)]
        self.mission_manager.start_mission(waypoints)

        # Robot at (0, 0), target (10, 10). Distance > 0.3
        current_pose = (0.0, 0.0, 0.0)
        grid_map = MagicMock()

        # Nav manager has no path
        self.mock_nav_manager.current_path = None

        self.mission_manager.update(current_pose, grid_map)

        # Should call plan_global_path
        self.mock_nav_manager.plan_global_path.assert_called_with(
            grid_map, (0.0, 0.0), (10.0, 10.0)
        )

    def test_loop_mission(self):
        """Test mission looping behavior."""
        waypoints = [(10.0, 10.0)]
        self.mission_manager.start_mission(waypoints, loop=True)

        # Reach the only waypoint
        current_pose = (10.0, 10.0, 0.0)
        grid_map = MagicMock()

        self.mission_manager.update(current_pose, grid_map)

        # Should have looped back to start.
        # _restart_queue sets index to 0. _next_item increments to 1.
        self.assertEqual(self.mission_manager.current_waypoint_index, 1)
        self.assertEqual(
            self.mission_manager.current_mission_item["target"], (10.0, 10.0)
        )
        self.assertTrue(self.mission_manager.is_running)


if __name__ == "__main__":
    unittest.main()
