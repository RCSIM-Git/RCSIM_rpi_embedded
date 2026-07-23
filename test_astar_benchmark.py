import time
import numpy as np
import math
import heapq
from rpi_project_source.modules.planners.costmap_manager import CostmapManager
from rpi_project_source.modules.planners.astar_planner import AStarPlanner

cm = CostmapManager()
cm.costmap = np.zeros((cm.grid_size, cm.grid_size), dtype=np.float32)
# Add some obstacles
cm.costmap[50:150, 100] = cm.OBSTACLE_COST

planner = AStarPlanner(cm)

start_pose = (cm.map_offset_x - 4.0, cm.map_offset_y)
goal_global = (cm.map_offset_x + 4.0, cm.map_offset_y)

t0 = time.time()
for _ in range(10):
    planner.plan_path(start_pose, goal_global)
t1 = time.time()

print(f"Time: {t1-t0:.4f}s")
