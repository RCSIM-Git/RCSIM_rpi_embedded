import math
import time

def estimate_motion(
    throttle: float,
    steering: float,
    dt: float,
    max_speed_mps: float = 2.0,
    max_steer_rad: float = 0.52,
    wheelbase_m: float = 0.25,
    imu_data: dict | None = None,
):
    v = throttle * max_speed_mps
    steer_angle = steering * max_steer_rad
    dx = v * dt
    dy = 0.0
    dyaw = 0.0
    if imu_data and "gz" in imu_data and abs(imu_data["gz"]) > 0.01:
        dyaw = math.radians(imu_data["gz"]) * dt
    else:
        safe_wheelbase = max(0.001, wheelbase_m)
        safe_steer = max(-1.48, min(1.48, steer_angle))
        dyaw = (v / safe_wheelbase) * math.tan(safe_steer) * dt
    return dx, dy, dyaw

t0 = time.time()
for _ in range(1_000_000):
    estimate_motion(0.5, 0.5, 0.1)
t1 = time.time()
print(f"Kinematics: {t1-t0:.4f}s")
