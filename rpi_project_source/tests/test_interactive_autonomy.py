import os
import sys
import traceback


def log(msg):
    print(msg)
    sys.stdout.flush()


try:
    log(">>> DEBUG: Script Started (Flush Mode)")

    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_source_dir = os.path.dirname(current_dir)
    if project_source_dir not in sys.path:
        sys.path.insert(0, project_source_dir)

    log(f"Added to path: {project_source_dir}")

    from unittest.mock import MagicMock, patch

    sys.modules["breezyslam"] = MagicMock()
    sys.modules["breezyslam.algorithms"] = MagicMock()
    sys.modules["breezyslam.sensors"] = MagicMock()
    sys.modules["rpi_hardware"] = MagicMock()

    # Mock WebRTC/AV dependencies that crash on Windows/Partial Envs
    sys.modules["aiortc"] = MagicMock()
    sys.modules["aiortc.contrib.media"] = MagicMock()
    sys.modules["aioice"] = MagicMock()
    sys.modules["av"] = MagicMock()

    log("Imports starting...")
    from core.main_service import TelemetryWorker

    log("Imported TelemetryWorker")
    from modules.planners.global_planner import GlobalPlanner

    log("Imported GlobalPlanner")
    from logic.navigation_manager import NavigationManager

    log("Imported NavigationManager")

    def test_interaction_loop():
        log(">>> Starting Interactive Autonomy Logic Test")

        # 1. Setup Config
        rpi_config = {
            "slam": {
                "enabled": True,
                "map_size_pixels": 100,
                "map_size_meters": 10.0,
            },
            "main_loop_freq_hz": 10,
            "comm_mode": "MOCK",
            "hardware": {"steering_channel": 0, "throttle_channel": 1},
        }

        # 2. Initialize Worker (Mocked environment)
        import numpy as np

        with patch("modules.ai_manager.AIManager"), patch(
            "modules.managers.hardware_manager.HardwareManager"
        ), patch("modules.camera_manager.CameraManager"), patch(
            "modules.managers.legacy.slam_manager.SlamManager"
        ) as MockSlamManager:

            worker = TelemetryWorker(rpi_config)
            worker.slam_manager = MockSlamManager.return_value

            mock_grid = np.zeros((100, 100), dtype=np.uint8)
            mock_grid[40:60, 50] = 255
            worker.slam_manager.get_grid_array.return_value = mock_grid

            worker.nav_manager = NavigationManager()
            worker.nav_manager.global_planner = GlobalPlanner(resolution=0.1)

            log(f"[RPi] Service Initialized. Mode: {worker.current_mode}")

            cmd = {"type": "command", "command": "GO_TO", "target": [2.0, 2.0]}
            worker.current_pose = (0.0, 0.0, 0.0)

            log(f"[PC] Sending GO_TO command: {cmd}")
            worker._handle_command(cmd)

            if worker.current_mode == "AUTONOMOUS":
                log("[SUCCESS] RPi switched to AUTONOMOUS mode.")
            else:
                log(
                    f"[FAILURE] RPi did not switch mode. Current: {worker.current_mode}"
                )

            if (
                worker.nav_manager.current_path
                and len(worker.nav_manager.current_path) > 0
            ):
                log(
                    f"[SUCCESS] Path planned with {len(worker.nav_manager.current_path)} points."
                )
            else:
                log("[FAILURE] No path planned.")

            sensor_data = {"imu": {}, "gps": {}, "lidar": []}
            packet = worker._prepare_telemetry(sensor_data)

            if "navigation" in packet:
                nav_data = packet["navigation"]
                if "path" in nav_data and len(nav_data["path"]) > 0:
                    log("[SUCCESS] Telemetry contains navigation path.")
                else:
                    log("[FAILURE] Navigation data missing path.")
            else:
                log("[FAILURE] Telemetry packet missing 'navigation' key.")

    if __name__ == "__main__":
        test_interaction_loop()

except Exception as e:
    log("CRASHED!")
    traceback.print_exc()
