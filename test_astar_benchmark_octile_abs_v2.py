import time
import math

def heuristic_hypot(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)

def heuristic_octile(ax, ay, bx, by):
    dx = abs(ax - bx)
    dy = abs(ay - by)
    return (dx + dy) + (-0.585786) * min(dx, dy)

ax, ay = 10, 20
bx, by = 50, 60

t0 = time.time()
for _ in range(1_000_000):
    heuristic_hypot(ax, ay, bx, by)
t1 = time.time()

t2 = time.time()
for _ in range(1_000_000):
    heuristic_octile(ax, ay, bx, by)
t3 = time.time()

print(f"Hypot: {t1-t0:.4f}s")
print(f"Octile: {t3-t2:.4f}s")
