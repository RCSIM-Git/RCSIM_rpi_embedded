"""
Tests for AIManager refactoring (using map_utils).
"""

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

# Mock libraries not available in test environment
sys.modules["tflite_runtime"] = MagicMock()
sys.modules["tflite_runtime.interpreter"] = MagicMock()
sys.modules["hailort"] = MagicMock()

# Determine project root and add to path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
sys.path.insert(0, PROJECT_ROOT)

from modules.ai_manager import AIManager


@pytest.fixture
def mock_logger():
    return MagicMock()


@pytest.fixture
def ai_manager(mock_logger):
    config = {"model_path": "dummy.tflite"}
    # Patch load_model to avoid file checks and real loading
    with patch.object(AIManager, "load_model", return_value=True):
        manager = AIManager(mock_logger, config)
    return manager


def test_create_sensor_vector_uses_map_utils(ai_manager):
    """
    Test that _create_sensor_vector calls haversine_distance and calculate_bearing
    from map_utils (which we imported in ai_manager.py).
    """
    telemetry_data = {
        "position": {"lat": 10.0, "lon": 20.0, "speed": 10.0},
        "orientation": {"heading": 0.0, "pitch": 0.0, "roll": 0.0},
        "home_position": {"lat": 10.01, "lon": 20.01},  # Slightly away
        "lidar_scan": [],
    }

    # We need to mock the functions imported IN ai_manager.py
    # Since they are imported as: from .map_utils import haversine_distance, calculate_bearing
    # We patch modules.ai_manager.haversine_distance and .calculate_bearing

    with patch(
        "modules.ai_manager.haversine_distance", return_value=123.45
    ) as mock_dist, patch(
        "modules.ai_manager.calculate_bearing", return_value=45.0
    ) as mock_bearing:

        vector = ai_manager._create_sensor_vector(telemetry_data)

        # Verify calls
        mock_dist.assert_called_once_with(10.0, 20.0, 10.01, 20.01)
        mock_bearing.assert_called_once_with(10.0, 20.0, 10.01, 20.01)

        # Verify vector content (distance is normalized by /100.0, clipped to 1.0)
        # 123.45 / 100.0 = 1.2345 -> clipped to 1.0
        # Index of distance_norm is last element (-1)
        assert vector[-1] == 1.0

        # Check bearing error calculation logic
        # target_bearing = 45.0
        # current_heading = 0.0
        # heading_error = (45 - 0 + 180) % 360 - 180 = 45
        # heading_error_norm = 45 / 180 = 0.25
        # Index of heading_error_norm is second to last (-2)
        assert vector[-2] == pytest.approx(0.25)
