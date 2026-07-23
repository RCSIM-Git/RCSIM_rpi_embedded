import time
import numpy as np
import math
import heapq
from rpi_project_source.modules.planners.costmap_manager import CostmapManager
from rpi_project_source.modules.planners.astar_planner import AStarPlanner

class AStarPlannerOctile(AStarPlanner):
    def plan_path(self, start_pose, goal_global, world_coordinates=True):
        sx, sy = self.cm.world_to_grid(start_pose[0], start_pose[1])
        gx, gy = self.cm.world_to_grid(goal_global[0], goal_global[1])

        height, width = self.cm.costmap.shape

        if not (0 <= sx < height and 0 <= sy < width):
            self.cm.check_and_scroll_map(start_pose[0], start_pose[1])
            sx, sy = self.cm.world_to_grid(start_pose[0], start_pose[1])
            if not (0 <= sx < height and 0 <= sy < width):
                return None

        if not (0 <= gx < height and 0 <= gy < width):
            return None

        if self.cm.costmap[gx, gy] >= self.cm.OBSTACLE_COST:
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))

        came_from = {}
        g_score = {(sx, sy): 0.0}

        def heuristic(a, b):
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return (dx + dy) - 0.586 * min(dx, dy)

        neighbors = [
            (0, 1, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (-1, 0, 1.0),
            (1, 1, 1.414),
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (-1, -1, 1.414),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((sx, sy))
                grid_path = path[::-1]

                if world_coordinates:
                    world_path = []
                    for gx, gy in grid_path:
                        wx, wy = self.cm.grid_to_world(gx, gy)
                        world_path.append((float(wx), float(wy)))
                    return world_path

                return grid_path

            cx, cy = current
            for dx, dy, move_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < height and 0 <= ny < width:
                    cell_cost = self.cm.costmap[nx, ny]
                    if cell_cost >= self.cm.OBSTACLE_COST:
                        continue

                    tentative_g_score = g_score[current] + move_cost * (
                        1.0 + cell_cost * self.A_STAR_PENALTY
                    )
                    neighbor = (nx, ny)

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, (gx, gy))
                        heapq.heappush(open_set, (f_score, neighbor))

        return None

cm = CostmapManager()
cm.costmap = np.zeros((cm.grid_size, cm.grid_size), dtype=np.float32)
# Add some obstacles
cm.costmap[50:150, 100] = cm.OBSTACLE_COST

planner = AStarPlannerOctile(cm)

start_pose = (cm.map_offset_x - 4.0, cm.map_offset_y)
goal_global = (cm.map_offset_x + 4.0, cm.map_offset_y)

t0 = time.time()
for _ in range(10):
    planner.plan_path(start_pose, goal_global)
t1 = time.time()

print(f"Time: {t1-t0:.4f}s")
