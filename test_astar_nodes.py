import time
import numpy as np
import math
import heapq
from rpi_project_source.modules.planners.costmap_manager import CostmapManager
from rpi_project_source.modules.planners.astar_planner import AStarPlanner

cm = CostmapManager()
cm.costmap = np.zeros((cm.grid_size, cm.grid_size), dtype=np.float32)
cm.costmap[50:150, 100] = cm.OBSTACLE_COST

class AStarPlannerHypot(AStarPlanner):
    def plan_path(self, start_pose, goal_global, world_coordinates=True):
        self.nodes_expanded = 0
        # ... just copy-paste A* ...
        sx, sy = self.cm.world_to_grid(start_pose[0], start_pose[1])
        gx, gy = self.cm.world_to_grid(goal_global[0], goal_global[1])
        height, width = self.cm.costmap.shape
        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        neighbors = [
            (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

        while open_set:
            self.nodes_expanded += 1
            _, current = heapq.heappop(open_set)
            if current == (gx, gy): return True
            cx, cy = current
            for dx, dy, move_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < height and 0 <= ny < width:
                    cell_cost = self.cm.costmap[nx, ny]
                    if cell_cost >= self.cm.OBSTACLE_COST: continue
                    tentative_g_score = g_score[current] + move_cost * (1.0 + cell_cost * self.A_STAR_PENALTY)
                    neighbor = (nx, ny)
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, (gx, gy))
                        heapq.heappush(open_set, (f_score, neighbor))
        return False

class AStarPlannerOctile(AStarPlanner):
    def plan_path(self, start_pose, goal_global, world_coordinates=True):
        self.nodes_expanded = 0
        sx, sy = self.cm.world_to_grid(start_pose[0], start_pose[1])
        gx, gy = self.cm.world_to_grid(goal_global[0], goal_global[1])
        height, width = self.cm.costmap.shape
        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}

        def heuristic(a, b):
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return (dx + dy) - 0.586 * min(dx, dy)

        neighbors = [
            (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

        while open_set:
            self.nodes_expanded += 1
            _, current = heapq.heappop(open_set)
            if current == (gx, gy): return True
            cx, cy = current
            for dx, dy, move_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < height and 0 <= ny < width:
                    cell_cost = self.cm.costmap[nx, ny]
                    if cell_cost >= self.cm.OBSTACLE_COST: continue
                    tentative_g_score = g_score[current] + move_cost * (1.0 + cell_cost * self.A_STAR_PENALTY)
                    neighbor = (nx, ny)
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, (gx, gy))
                        heapq.heappush(open_set, (f_score, neighbor))
        return False

planner1 = AStarPlannerHypot(cm)
planner2 = AStarPlannerOctile(cm)
start_pose = (cm.map_offset_x - 4.0, cm.map_offset_y)
goal_global = (cm.map_offset_x + 4.0, cm.map_offset_y)

planner1.plan_path(start_pose, goal_global)
print("Hypot nodes:", planner1.nodes_expanded)

planner2.plan_path(start_pose, goal_global)
print("Octile nodes:", planner2.nodes_expanded)
