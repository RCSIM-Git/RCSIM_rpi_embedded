import time
import math
import heapq
import numpy as np

def run_astar(heuristic):
    grid = np.zeros((200, 200))
    grid[50:150, 100] = 100
    sx, sy = 10, 100
    gx, gy = 190, 100
    open_set = []
    heapq.heappush(open_set, (0.0, (sx, sy)))
    came_from = {}
    g_score = {(sx, sy): 0.0}
    height, width = 200, 200

    neighbors = [
        (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
    ]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == (gx, gy): return True
        cx, cy = current
        for dx, dy, move_cost in neighbors:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < height and 0 <= ny < width:
                cell_cost = grid[nx, ny]
                if cell_cost >= 100: continue
                tentative_g_score = g_score[current] + move_cost * (1.0 + cell_cost * 10.0)
                neighbor = (nx, ny)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, (gx, gy))
                    heapq.heappush(open_set, (f_score, neighbor))
    return False

def heuristic_hypot(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def heuristic_octile(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) - 0.586 * min(dx, dy)

t0 = time.time()
for _ in range(10): run_astar(heuristic_hypot)
t1 = time.time()
for _ in range(10): run_astar(heuristic_octile)
t2 = time.time()

print(f"A* Hypot: {t1-t0:.4f}s")
print(f"A* Octile: {t2-t1:.4f}s")
