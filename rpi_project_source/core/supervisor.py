"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Supervisor Module - A Lightweight Process Manager for RCSIM.
Moduł Supervisora - Lekki Menedżer Procesów dla RCSIM.

This service runs as the main process inside the Docker container. Its primary
task is to manage the telemetry application (`main_service.py`) as a subprocess,
monitor its health, and handle hardware watchdog integration.

It communicates over a simple JSON-based UDP protocol for remote management.
This architecture ensures system resilience: if the main application fails,
the Supervisor remains operational and can restart it, while the hardware
watchdog ensures the Supervisor itself never hangs.
"""

import json
import logging
import os
import socket
import subprocess
import time
from typing import IO, Any

from core.utils.system_info import get_board_info

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


class SupervisorService:
    """
    Główna klasa usługi Supervisora, działająca jako menedżer procesów dla RCSIM.
    Main Supervisor service class, acting as a process manager for RCSIM.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 12348) -> None:
        """
        Inicjalizuje usługę Supervisora.
        Initializes the Supervisor service.

        Args:
            host (str): Adres hosta dla gniazda UDP. / Host address for UDP socket.
            port (int): Port dla gniazda UDP. / Port for UDP socket.
        """
        self.host: str = host
        self.port: int = port
        self.sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.config_path: str = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "config.json")
        )
        self.telemetry_process: subprocess.Popen | None = None
        self.watchdog_file: IO[str] | None = None
        self.service_enabled: bool = True  # Track if the service should be running

        # Planner status storage
        self.last_planner_status: dict[str, Any] = {
            "state": "UNKNOWN",
            "safety_score": 1.0,
            "target": None,
            "timestamp": 0,
        }

        try:
            self.watchdog_file = open("/dev/watchdog", "w")
            logging.info("Hardware watchdog successfully opened.")
        except IOError as e:
            if e.errno == 16:  # Device busy
                logging.warning(
                    "Hardware watchdog busy (likely system-managed). "
                    "Internal watchdog disabled."
                )
            else:
                logging.error(
                    f"Failed to open hardware watchdog: {e}. Watchdog is disabled."
                )

    def run(self) -> None:
        """
        Uruchamia główną pętlę usługi. Zarządza podprocesem telemetrii i watchdogiem.
        Starts the main service loop. Manages the telemetry subprocess and watchdog.
        """
        try:
            self.sock.bind((self.host, self.port))
            logging.info(f"Supervisor listening on {self.host}:{self.port}")
        except OSError as e:
            logging.critical(f"Failed to bind socket: {e}.")
            return

        self._start_telemetry_process()

        while True:
            try:
                # Pet the watchdog to prevent a system reset
                if self.watchdog_file:
                    self.watchdog_file.write("\n")
                    self.watchdog_file.flush()

                # Check the status of the telemetry process
                if (
                    self.service_enabled
                    and self.telemetry_process
                    and self.telemetry_process.poll() is not None
                ):
                    logging.warning(
                        "Telemetry process terminated unexpectedly. Restarting..."
                    )
                    self._start_telemetry_process()

                self.sock.settimeout(1.0)
                try:
                    data, addr = self.sock.recvfrom(4096)  # Increased buffer for status
                    message: dict[str, Any] = json.loads(data.decode("utf-8"))
                    self.handle_command(message, addr)
                except socket.timeout:
                    continue  # This is expected, just loop again
                except json.JSONDecodeError:
                    logging.warning("Received malformed JSON data.")

            except Exception as e:
                logging.error(f"Error in Supervisor loop: {e}")

            time.sleep(0.1)  # Main loop delay (reduced for responsiveness)

    def _start_telemetry_process(self) -> None:
        """
        Uruchamia `main_service.py` jako podproces.
        Starts `main_service.py` as a subprocess.
        """
        try:
            main_service_path = os.path.join(
                os.path.dirname(__file__), "main_service.py"
            )
            # We are in the same Python environment, so we can call it directly.
            self.telemetry_process = subprocess.Popen(["python3", main_service_path])
            logging.info(
                f"Telemetry service started with PID: {self.telemetry_process.pid}"
            )
        except Exception as e:
            logging.error(f"Failed to start telemetry process: {e}")
            self.telemetry_process = None

    def handle_command(self, message: dict[str, Any], addr: tuple[str, int]) -> None:
        """
        Przetwarza otrzymane polecenie i wysyła odpowiedź.
        Processes a received command and sends a response.

        Args:
            message (dict[str, Any]): Otrzymana wiadomość (JSON). /
                                     Received message (JSON).
            addr (tuple[str, int]): Adres nadawcy. / Sender address.
        """
        cmd = message.get("cmd")
        response: dict[str, Any] = {}

        if cmd == "PING":
            info = get_board_info()
            response = {
                "status": "PONG",
                "model": info.get("model_name", "Unknown"),
                "service_active": self.is_service_active(),
                "cpu_load": info.get("cpu_usage", 0.0),
                "ram_usage": info.get("ram_usage", 0.0),
                "cpu_temp": info.get("cpu_temp", 0.0),
                "planner": self.last_planner_status,
            }
        elif cmd == "PLANNER_UPDATE":
            # Receive status from Planner
            data = message.get("data", {})
            self.last_planner_status.update(data)
            self.last_planner_status["timestamp"] = time.time()
            # No response needed usually for updates, but keep protocol happy
            response = {"status": "OK"}

        elif cmd == "GET_CONFIG":
            response = self._get_config()
        elif cmd == "SET_CONFIG":
            config_data = message.get("config")
            if isinstance(config_data, dict):
                response = self._set_config(config_data)
            else:
                response = {
                    "status": "ERROR",
                    "message": "Invalid or missing 'config' data.",
                }
        elif cmd == "START_SERVICE":
            self.service_enabled = True
            if not self.is_service_active():
                self._start_telemetry_process()
            response = {"status": "OK", "message": "Service started."}
        elif cmd == "STOP_SERVICE":
            self.service_enabled = False
            if self.telemetry_process:
                self.telemetry_process.terminate()
            response = {"status": "OK", "message": "Service stopped."}
        elif cmd == "RESTART_SERVICE":
            self.service_enabled = True
            response = self._restart_service()
        elif cmd == "REBOOT":
            response = self._reboot()
        else:
            response = {"status": "ERROR", "message": "Unknown command"}

        self.sock.sendto(json.dumps(response).encode("utf-8"), addr)

    def is_service_active(self) -> bool:
        """
        Sprawdza, czy podproces telemetrii jest uruchomiony.
        Checks if the telemetry subprocess is running.

        Returns:
            bool: True jeśli proces działa, False w przeciwnym razie. /
                  True if process is running, else False.
        """
        return (
            self.telemetry_process is not None and self.telemetry_process.poll() is None
        )

    def _get_config(self) -> dict[str, Any]:
        """
        Odczytuje i zwraca plik konfiguracyjny.
        Reads and returns the configuration file.

        Returns:
            dict[str, Any]: Słownik statusu i konfiguracji. / Status and configuration dictionary.
        """
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                return {"status": "OK", "config": json.load(f)}
        except (FileNotFoundError, json.JSONDecodeError) as e:
            logging.error(f"Error reading config file: {e}")
            return {"status": "ERROR", "message": f"Config file error: {e}"}

    def _set_config(self, new_config: dict[str, Any]) -> dict[str, str]:
        """
        Zapisuje nową konfigurację do pliku w sposób atomowy.
        Atomically writes the new configuration to a file.

        Args:
            new_config (dict[str, Any]): Nowa konfiguracja. / New configuration.

        Returns:
            dict[str, str]: Status operacji. / Operation status.
        """
        tmp_path = self.config_path + ".tmp"
        try:
            # Check directory writability for cross-platform permission errors
            dir_path = os.path.dirname(self.config_path) or "."
            if not os.access(dir_path, os.W_OK):
                raise PermissionError(f"Permission denied: {dir_path}")

            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(new_config, f, indent=4)
            # Use os.replace for atomic overwrite semantics across platforms
            os.replace(tmp_path, self.config_path)
            logging.info("Configuration file updated successfully.")
            return {"status": "OK"}
        except (IOError, OSError, PermissionError) as e:
            logging.error(f"Error writing config file: {e}")
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
            return {"status": "ERROR", "message": str(e)}

    def _restart_service(self) -> dict[str, str]:
        """
        Restartuje podproces telemetrii.
        Restarts the telemetry subprocess.

        Returns:
            dict[str, str]: Status operacji. / Operation status.
        """
        logging.info("Restart command received for telemetry service.")
        if self.telemetry_process:
            try:
                self.telemetry_process.terminate()
                self.telemetry_process.wait(timeout=5)
                logging.info("Telemetry process terminated.")
            except subprocess.TimeoutExpired:
                logging.warning(
                    "Telemetry process did not terminate gracefully, killing."
                )
                self.telemetry_process.kill()
                self.telemetry_process.wait()
            except Exception as e:
                logging.error(f"Error terminating telemetry process: {e}")

        self._start_telemetry_process()
        return {"status": "OK" if self.is_service_active() else "ERROR"}

    def _reboot(self) -> dict[str, str]:
        """
        Restartuje system operacyjny (wymaga uprzywilejowanego kontenera).
        Reboots the operating system (requires a privileged container).

        Returns:
            dict[str, str]: Status operacji. / Operation status.
        """
        try:
            logging.warning("Reboot command received. Rebooting system.")
            if os.path.exists("/proc/sysrq-trigger"):
                # Synchronize disks
                with open("/proc/sysrq-trigger", "w") as f:
                    f.write("s")
                time.sleep(1)
                # Hard reboot
                with open("/proc/sysrq-trigger", "w") as f:
                    f.write("b")
            else:
                subprocess.run(["sudo", "reboot"], check=True)
            return {"status": "OK"}
        except Exception as e:
            logging.error(f"Failed to execute reboot command: {e}")
            return {"status": "ERROR", "message": f"Reboot command failed: {e}"}

    def __del__(self) -> None:
        """
        Zapewnia zamknięcie watchdoga przy wyjściu.
        Ensures the watchdog is closed on exit.
        """
        if self.watchdog_file:
            self.watchdog_file.close()
            logging.info("Hardware watchdog closed.")


if __name__ == "__main__":
    service = SupervisorService()
    service.run()
