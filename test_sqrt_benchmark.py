import time
import numpy as np

dist_sq = np.random.rand(10000) * 100
resolution = 0.05

t0 = time.time()
for _ in range(10000):
    dist_m = np.sqrt(dist_sq) * resolution
    # assume kept is all True
    dist_m_kept = dist_m
    weights = np.zeros_like(dist_m_kept)
    non_zero = dist_m_kept > 0
    weights[non_zero] = 1.0 / (dist_m_kept[non_zero] ** 2)
t1 = time.time()

t2 = time.time()
for _ in range(10000):
    dist_m_sq = dist_sq * (resolution ** 2)
    # assume kept is all True
    dist_m_sq_kept = dist_m_sq
    weights = np.zeros_like(dist_m_sq_kept, dtype=float)
    non_zero = dist_m_sq_kept > 0
    weights[non_zero] = 1.0 / dist_m_sq_kept[non_zero]
t3 = time.time()

print(f"Original: {t1-t0:.4f}s")
print(f"Optimized: {t3-t2:.4f}s")
