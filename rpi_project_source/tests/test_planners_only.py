import pytest
import sys
import os
import numpy as np
sys.path.append(os.path.abspath('.'))
from modules.planners.global_planner import GlobalPlanner
from modules.planners.astar_planner import AStarPlanner

def test_global_planner_compiles():
    planner = GlobalPlanner(resolution=0.1, inflation_radius=0.1)
    grid = np.zeros((100, 100), dtype=np.uint8)
    planner.plan_path(grid, (-2.0, -2.0), (2.0, 2.0))
    assert True

def test_astar_planner_compiles():
    assert hasattr(AStarPlanner, "plan_path")
