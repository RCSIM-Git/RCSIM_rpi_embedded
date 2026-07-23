import math
import time

def estimate_motion_orig(throttle, steering, dt, max_speed_mps=2.0, max_steer_rad=0.52, wheelbase_m=0.25):
    v = throttle * max_speed_mps
    steer_angle = steering * max_steer_rad
    dx = v * dt
    dy = 0.0
    safe_wheelbase = max(0.001, wheelbase_m)
    safe_steer = max(-1.48, min(1.48, steer_angle))
    dyaw = (v / safe_wheelbase) * math.tan(safe_steer) * dt
    return dx, dy, dyaw

t0 = time.time()
for _ in range(10_000_000):
    estimate_motion_orig(0.5, 0.5, 0.1)
t1 = time.time()
print(f"Orig: {t1-t0:.4f}s")
