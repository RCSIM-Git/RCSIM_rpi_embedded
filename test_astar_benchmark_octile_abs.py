import time
import math
import numpy as np

def heuristic_hypot(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def heuristic_octile(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (-0.585786) * min(dx, dy)

a = (10, 20)
b = (50, 60)

t0 = time.time()
for _ in range(1_000_000):
    heuristic_hypot(a, b)
t1 = time.time()

t2 = time.time()
for _ in range(1_000_000):
    heuristic_octile(a, b)
t3 = time.time()

print(f"Hypot: {t1-t0:.4f}s")
print(f"Octile: {t3-t2:.4f}s")
