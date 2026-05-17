"""
Simulation Loop for RCSIM logic validation.
Mocks sensors (LiDAR, IMU, Camera) and runs the control loop to verify
LocalPlanner fusion, SafetySupervisor overrides, and ControlSelector decisions.
"""

import logging
import time

from core.safety_supervisor import SafetySupervisor
from logic.control_selector import ControlSelector
from modules.planners.local_planner import LocalPlanner

# Setup minimal logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def simulate_loop(iterations=50):
    # 1. Initialize Components
    config = {
        "safety": {"emergency_stop_dist_m": 0.3, "avoid_dist_m": 0.8},
        "local_planner": {"resolution": 0.05, "grid_size": 100, "lookahead_dist": 0.6},
    }

    supervisor = SafetySupervisor(config)
    planner = LocalPlanner(config=config)
    selector = ControlSelector(None, None)  # Mocked managers

    current_mode = "AUTONOMOUS"

    logger.info(
        f"Starting simulation loop for {iterations} iterations in {current_mode} mode."
    )

    for i in range(iterations):
        # A. Mock Sensors
        # Simulate an obstacle appearing at iteration 20
        dist = 2000.0 if i < 20 else (2000.0 - (i - 20) * 100.0)
        dist = max(100.0, dist)

        sensor_data = {
            "lidar": [(0, dist), (10, dist + 100)],  # Obstacle ahead
            "imu": {"ax": 0.0, "ay": 0.0, "az": 1.0},
            "battery": {"voltage": 12.0},
        }

        # B. Update Planner (Fusion)
        # In real loop, we'd also have pose and AI detections
        pose = (0.0, 0.0, 0.0)
        planner.update_occupancy_from_lidar_and_yolo(pose, sensor_data["lidar"])

        # C. Reactive Planning
        p_steer, p_thrott, p_safety = planner.plan_reactive()

        # D. Frame Data for Selector
        frame_data = {**sensor_data, "planner_cmd": (p_steer, p_thrott, p_safety)}

        # E. Control Selection
        steering, throttle = selector.process_frame(current_mode, frame_data)

        # F. Safety Override
        supervisor.update(frame_data)
        final_steering, final_throttle = supervisor.process_controls(steering, throttle)

        # G. Log Progress
        logger.info(
            f"Iter {i:02d} | Dist: {dist/1000.:.2f}m | "
            f"State: {supervisor.state.name:8s} | "
            f"Plan: (S:{p_steer:+.2f}, T:{p_thrott:.2f}) | "
            f"Final: (S:{final_steering:+.2f}, T:{final_throttle:.2f})"
        )

        if final_throttle == 0.0 and i > 25:
            logger.info("Simulation Goal: Stop detected due to obstacle.")
            # break # Optional: stop earlier

        time.sleep(0.05)


if __name__ == "__main__":
    simulate_loop()
