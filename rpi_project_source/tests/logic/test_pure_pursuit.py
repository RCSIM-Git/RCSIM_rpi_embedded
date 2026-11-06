"""PL: Testuje wybór punktu prowadzącego na ścieżce i osiągnięcie celu.
EN: Tests path lookahead selection and goal arrival behavior.
"""

import pytest
from logic.navigation_manager import NavigationManager


class TestPurePursuit:
    """PL: Testy punktu lookahead wyznaczanego przez menedżera nawigacji.
    EN: Tests lookahead waypoint behavior from the navigation manager.
    """

    @pytest.fixture
    def nav_manager(self):
        """PL: Tworzy menedżera nawigacji dla testów punktów trasy.
        EN: Creates a navigation manager for waypoint tests.
        """
        return NavigationManager()

    def test_lookahead_point_straight_line(self, nav_manager):
        """PL: Sprawdza wybór punktu na prostej przekraczającego dystans lookahead.
        EN: Checks selection of the point beyond the lookahead distance on a straight path.
        """
        # Path: (0,0) -> (0, 0.0001) -> (0, 0.0002) ... (approx 11m per 0.0001 deg lat)
        # 1 deg lat is approx 111km. 0.00001 is approx 1.11m.

        # Let's use simpler relative coordinates and mock haversine if possible,
        # but haversine is imported directly.
        # We will use small lat/lon increments.

        start_pose = (50.0, 19.0)
        # Create a path going North
        path = []
        for i in range(10):
            # 0.00001 deg is approx 1.11 meters
            path.append((50.0 + i * 0.00001, 19.0))

        nav_manager.current_path = path
        nav_manager.lookahead_distance = 2.0  # meters

        # Robot at start
        carrot = nav_manager.get_next_waypoint(start_pose)

        assert carrot is not None
        # Should be the point approx 2m away (index 2 or 3)
        # 0: 0m, 1: 1.11m, 2: 2.22m
        # Logic finds first point > lookahead (2.0)
        # So it should be index 2 (2.22m)
        assert carrot == path[2]

    def test_lookahead_point_end_of_path(self, nav_manager):
        """PL: Sprawdza zwrócenie końcowego punktu krótkiej ścieżki.
        EN: Checks that the endpoint is returned for a short path.
        """
        start_pose = (50.0, 19.0)
        path = [(50.0, 19.0), (50.00001, 19.0)]  # 1.1m path

        nav_manager.current_path = path
        nav_manager.lookahead_distance = 5.0  # Lookahead bigger than path

        carrot = nav_manager.get_next_waypoint(start_pose)

        # Should return the last point
        assert carrot == path[-1]

    def test_goal_tolerance_reached(self, nav_manager):
        """PL: Sprawdza czyszczenie ścieżki po osiągnięciu celu w tolerancji.
        EN: Checks that the path is cleared when the goal is within tolerance.
        """
        goal = (50.00001, 19.0)
        start_pose = (50.00001, 19.0)  # Robot at goal
        path = [(50.0, 19.0), goal]

        nav_manager.current_path = path
        nav_manager.goal_tolerance = 0.5

        carrot = nav_manager.get_next_waypoint(start_pose)

        assert carrot is None
        assert nav_manager.current_path is None

    def test_off_path_behavior(self, nav_manager):
        """PL: Sprawdza wybór punktu prowadzącego, gdy robot jest poza ścieżką.
        EN: Checks lookahead selection when the robot is off the path.
        """
        # Path goes North
        path = [(50.0, 19.0), (50.0001, 19.0)]  # 0 to ~11m
        nav_manager.current_path = path
        nav_manager.lookahead_distance = 2.0

        # Robot is sideways (East) from start
        robot_pos = (50.0, 19.00002)  # ~1.4m East

        carrot = nav_manager.get_next_waypoint(robot_pos)

        # Should still find a point on path > 2m away from robot
        # Dist to (50.0, 19.0) is ~1.4m ( < 2.0)
        # Dist to (50.00002, 19.0) is sqrt(2.2^2 + 1.4^2) > 2.0
        assert carrot is not None
        assert carrot[0] > 50.0  # Should be further North
