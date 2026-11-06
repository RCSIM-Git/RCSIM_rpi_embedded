"""Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Zarządza sekwencjami punktów misji autonomicznej.
EN: Manages waypoint sequences for autonomous missions.
"""
import logging
import time
from typing import Any


class MissionManager:
    """PL: Uruchamia, aktualizuje i zatrzymuje misje nawigacyjne.
    EN: Starts, updates, and stops navigation missions.
    """

    def __init__(self, nav_manager: Any) -> None:
        """PL: Łączy menedżera misji z komponentem nawigacji.
        EN: Connects the mission manager to the navigation component.
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
        """PL: Tworzy kolejkę punktów i rozpoczyna misję.
        EN: Builds a waypoint queue and starts the mission.
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
        """PL: Zatrzymuje misję, czyści cel i bieżącą ścieżkę.
        EN: Stops the mission and clears its target and current path.
        """
        self.is_running = False
        self.current_mission_item = None
        self.nav_manager.clear_path()
        self.logger.info("Mission stopped.")

    def update(self, current_pose: tuple[float, float, float], grid_map: Any) -> None:
        """PL: Sprawdza postęp celu i planuje ponowne podejście w razie potrzeby.
        EN: Checks target progress and replans when needed.
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
        """PL: Przesuwa kolejkę do następnego celu i czyści starą ścieżkę.
        EN: Advances to the next mission target and clears the previous path.
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
        """PL: Ustawia licznik kolejki na początek misji zapętlonej.
        EN: Resets the queue index to the start of a looping mission.
        """
        self.current_waypoint_index = 0
