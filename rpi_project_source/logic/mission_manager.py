"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import logging
import time
from typing import Any


class MissionManager:
    """
    Zarządza misjami autonomicznymi (np. patrol, sekwencja punktów).
    Manages autonomous missions (e.g. patrol, waypoint sequence).
    """

    def __init__(self, nav_manager: Any) -> None:
        """
        Inicjalizuje menedżera misji.
        Initializes the mission manager.
        """
        self.logger = logging.getLogger("MissionManager")
        self.nav_manager = nav_manager

        self.mission_queue: list[dict[str, Any]] = []  # Queue of mission items
        self.current_mission_item: dict[str, Any] | None = None
        self.is_running = False
        self.loop_mission = False  # If true, repeat queue

        self.current_waypoint_index = 0

        self.retry_count = 0
        self.max_retries = 3
        self.last_retry_time = 0.0

    def start_mission(
        self, waypoints: list[tuple[float, float]], loop: bool = False
    ) -> None:
        """
        Rozpoczyna misję z listą punktów.
        Starts a mission with a list of waypoints.
        """
        self.mission_queue = []
        for wp in waypoints:
            self.mission_queue.append({"type": "goto", "target": wp})

        self.loop_mission = loop
        self.current_waypoint_index = 0
        self.is_running = True
        self.logger.info(
            f"Mission started with {len(waypoints)} waypoints. Loop={loop}"
        )
        self._next_item()

    def stop_mission(self) -> None:
        """
        Zatrzymuje misję.
        Stops the mission.
        """
        self.is_running = False
        self.current_mission_item = None
        self.nav_manager.clear_path()
        self.logger.info("Mission stopped.")

    def update(self, current_pose: tuple[float, float, float], grid_map: Any) -> None:
        """
        Aktualizuje stan misji. Wywoływane w pętli głównej.
        Updates mission state. Called in main loop.
        """
        if not self.is_running:
            return

        if self.current_mission_item is None:
            if not self._next_item():
                self.logger.info("Mission finished.")
                self.stop_mission()
                return

        # Check if current goal is reached
        if self.current_mission_item["type"] == "goto":
            target = self.current_mission_item["target"]
            dist = (
                (target[0] - current_pose[0]) ** 2 + (target[1] - current_pose[1]) ** 2
            ) ** 0.5

            if dist < 0.3:  # Reached goal tolerance
                self.logger.info(f"Waypoint {self.current_waypoint_index} reached.")
                if not self._next_item():
                    if self.loop_mission:
                        self.logger.info("Looping mission...")
                        self._restart_queue()
                        self._next_item()
                    else:
                        self.logger.info("Mission sequence complete.")
                        self.stop_mission()
            else:
                # Ensure path is planned
                if not self.nav_manager.current_path:
                    current_time = time.time()
                    if current_time - self.last_retry_time > 2.0:
                        self.logger.info(
                            f"Replanning to waypoint {self.current_waypoint_index}: {target}"
                        )
                        success = self.nav_manager.plan_global_path(
                            grid_map, (current_pose[0], current_pose[1]), target
                        )
                        if not success:
                            self.retry_count += 1
                            self.last_retry_time = current_time
                            if self.retry_count > self.max_retries:
                                self.logger.error(
                                    "Max retries reached. Skipping waypoint."
                                )
                                self._next_item()
                            else:
                                self.logger.warning(
                                    f"Failed to plan to waypoint. Retrying... ({self.retry_count}/{self.max_retries})"
                                )
                        else:
                            self.retry_count = 0
                            self.last_retry_time = current_time

    def _next_item(self) -> bool:
        """
        Przechodzi do następnego elementu w kolejce misji.
        Advances to the next item in the mission queue.
        """
        self.retry_count = 0
        self.last_retry_time = 0.0
        if self.current_waypoint_index < len(self.mission_queue):
            self.current_mission_item = self.mission_queue[self.current_waypoint_index]
            self.current_waypoint_index += 1
            # Clear old path so update() triggers replan
            self.nav_manager.clear_path()
            return True
        return False

    def _restart_queue(self) -> None:
        """
        Przewija licznik waypointów na początek w trybie pętli (loop).
        Rewinds waypoint counter to start in loop mode.
        """
        self.current_waypoint_index = 0
