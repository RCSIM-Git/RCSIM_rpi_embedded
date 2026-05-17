"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import logging
import threading
import time
import os
from typing import Any, Callable, List

from pymavlink import mavutil

# Force MAVLink 2.0 for modern telemetry features
os.environ["MAVLINK20"] = "1"


logger = logging.getLogger(__name__)


class MAVLinkService:
    """
    Zaawansowany serwis MAVLink dla RPi.
    Obsługuje telemetrię, sterowanie RC_OVERRIDE oraz MISSION_PROTOCOL.
    """

    def __init__(
        self,
        connection_str: str = "udpin:0.0.0.0:14550",
        system_id: int = 10,
        on_rc_channels: Callable[[List[int]], None] = None,
        on_mission_updated: Callable[[List[Any]], None] = None,
        on_arm_disarm: Callable[[bool], None] = None,
    ):
        self.connection_str = connection_str
        self.system_id = system_id
        self.component_id = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        self.master: mavutil.mavlink_connection | None = None
        self.running = False
        self.thread: threading.Thread | None = None
        self.boot_time = time.time()

        # Callbacks
        self.on_rc_channels = on_rc_channels
        self.on_mission_updated = on_mission_updated
        self.on_arm_disarm = on_arm_disarm

        # Mission state
        self.waypoints = []
        self.mission_receive_in_progress = False

        # Link tracking
        self.last_remote_heartbeat = 0.0
        self.HEARTBEAT_TIMEOUT = 5.0  # Increased from 2.0 for stable RF link
        self.armed = False

        # [DIAG-RPi-001] Diagnostics
        self._tx_telem_count: int = 0
        self._rx_msg_count: int = 0
        self._last_diag_log: float = 0.0
        self._diag_interval: float = 5.0
        self._gcs_heartbeat_seen: bool = False

    def start(self):
        """Uruchamia serwis MAVLink."""
        try:
            logger.info(f"Starting MAVLink Service on {self.connection_str}...")

            # Obsługa formatu port:baud dla połączeń szeregowych
            connection_params = {}
            conn_str = self.connection_str

            if "/" in conn_str and ":" in conn_str:
                device, baud = conn_str.split(":")
                conn_str = device
                connection_params["baud"] = int(baud)
                logger.info(f"MAVLink Serial detected: {device} at {baud} bps")

            self.master = mavutil.mavlink_connection(
                conn_str,
                source_system=self.system_id,
                protocol_version=2,
                **connection_params,
            )
            self.running = True
            self.thread = threading.Thread(target=self._update_loop, daemon=True)
            self.thread.start()
            logger.info("✅ MAVLink Service started.")
        except Exception as e:
            logger.error(f"❌ Failed to start MAVLink Service: {e}")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.master:
            self.master.close()

    def _update_loop(self):
        """Pętla główna: Heartbeat i nasłuch wiadomości."""
        last_heartbeat = 0
        while self.running:
            now = time.time()

            # Send Heartbeat at 1Hz
            if now - last_heartbeat >= 1.0:
                self.send_heartbeat()
                last_heartbeat = now

            # Receive messages
            try:
                msg = self.master.recv_match(blocking=False)
                if msg:
                    self._rx_msg_count += 1
                    msg_type = msg.get_type()
                    # Track message types
                    if not hasattr(self, "_rx_types"):
                        self._rx_types = {}
                    self._rx_types[msg_type] = self._rx_types.get(msg_type, 0) + 1
                    self._handle_message(msg)
            except Exception as e:
                logger.error(f"Error receiving MAVLink message: {e}")

            # [DIAG-RPi-001] Periodic diagnostic summary
            if now - self._last_diag_log >= self._diag_interval:
                types_str = ""
                if hasattr(self, "_rx_types") and self._rx_types:
                    top = sorted(self._rx_types.items(), key=lambda x: -x[1])[:6]
                    types_str = " Types=[" + ", ".join(f"{t}:{c}" for t, c in top) + "]"
                    self._rx_types = {}  # Reset per interval
                logger.info(
                    f"MAVLink DIAG(RPi): TX_telem={self._tx_telem_count} "
                    f"RX_msg={self._rx_msg_count} "
                    f"GCS_HB={'YES' if self._gcs_heartbeat_seen else 'NO'} "
                    f"link={'ACTIVE' if self.link_active else 'DEAD'} "
                    f"armed={self.armed}"
                    f"{types_str}"
                )
                self._last_diag_log = now

            time.sleep(0.01)

    def _handle_message(self, msg):
        """Obsługa przychodzących wiadomości MAVLink."""
        msg_type = msg.get_type()

        if msg_type == "BAD_DATA":
            # Log hex bytes of bad data to diagnose baudrate issues
            # We only log this occasionally to avoid spam
            if not hasattr(self, "_last_bad_data_log"):
                self._last_bad_data_log = 0
            if time.time() - self._last_bad_data_log > 5.0:
                raw_data = getattr(msg, "data", b"")
                # Log first 48 bytes in HEX to diagnose framing/version issues
                hex_str = " ".join([f"{b:02x}" for b in raw_data[:48]])
                logger.warning(
                    f"⚠️ MAVLink BAD_DATA (len={len(raw_data)}): {hex_str}..."
                )
                self._last_bad_data_log = time.time()

        elif msg_type == "HEARTBEAT":
            src = msg.get_srcSystem()
            self.last_remote_heartbeat = time.time()
            if not self._gcs_heartbeat_seen:
                self._gcs_heartbeat_seen = True
                logger.info(
                    "✅ First GCS Heartbeat received! "
                    f"SysID={src}, CompID={msg.get_srcComponent()}"
                )

        elif msg_type == "RC_CHANNELS_OVERRIDE":
            # Obsługa kanałów RC (1-18)
            channels = [
                msg.chan1_raw,
                msg.chan2_raw,
                msg.chan3_raw,
                msg.chan4_raw,
                msg.chan5_raw,
                msg.chan6_raw,
                msg.chan7_raw,
                msg.chan8_raw,
                msg.chan9_raw,
                msg.chan10_raw,
                msg.chan11_raw,
                msg.chan12_raw,
                msg.chan13_raw,
                msg.chan14_raw,
                msg.chan15_raw,
                msg.chan16_raw,
                msg.chan17_raw,
                msg.chan18_raw,
            ]
            # Log periodic RC updates (every 50 packets to avoid spam)
            if not hasattr(self, "_rc_count"):
                self._rc_count = 0
            self._rc_count += 1
            if self._rc_count % 50 == 0:
                ch_str = ", ".join([f"CH{i+1}={channels[i]}" for i in range(min(len(channels), 8))])
                logger.info(f"MAVLink RC Override received: {ch_str}")

            # MAVLink używa 65535 jako "no change"
            if self.on_rc_channels:
                self.on_rc_channels(channels)

        elif msg_type == "MISSION_REQUEST_LIST":
            # GCS prosi o listę waypointów
            self.master.mav.mission_count_send(
                msg.get_srcSystem(), msg.get_srcComponent(), len(self.waypoints)
            )

        elif msg_type == "MISSION_COUNT":
            # GCS informuje, ile waypointów będzie wysyłać
            self.expected_mission_count = msg.count
            self.temp_waypoints = []
            self.mission_receive_in_progress = True
            # Prosimy o pierwszy element
            self.master.mav.mission_request_int_send(
                msg.get_srcSystem(), msg.get_srcComponent(), 0
            )

        elif msg_type == "MISSION_ITEM_INT" or msg_type == "MISSION_ITEM":
            if self.mission_receive_in_progress:
                msg.seq
                self.temp_waypoints.append(msg)
                if len(self.temp_waypoints) < self.expected_mission_count:
                    # Request next
                    self.master.mav.mission_request_int_send(
                        msg.get_srcSystem(),
                        msg.get_srcComponent(),
                        len(self.temp_waypoints),
                    )
                else:
                    # Done
                    self.waypoints = self.temp_waypoints
                    self.mission_receive_in_progress = False
                    self.master.mav.mission_ack_send(
                        msg.get_srcSystem(),
                        msg.get_srcComponent(),
                        mavutil.mavlink.MAV_MISSION_ACCEPTED,
                    )
                    if self.on_mission_updated:
                        self.on_mission_updated(self.waypoints)

        elif msg_type == "COMMAND_LONG":
            # Obsługa komend AI
            if msg.command == mavutil.mavlink.MAV_CMD_USER_1:
                # Przykład: START AI
                logger.info("MAVLink: Received custom COMMAND_LONG (USER_1)")
            elif msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                self.armed = msg.param1 == 1.0
                logger.info(
                    f"MAVLink: Received ARM/DISARM command. Param1={msg.param1} -> Armed: {self.armed}"
                )
                if self.on_arm_disarm:
                    self.on_arm_disarm(self.armed)

            # Ack
            self.master.mav.command_ack_send(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED
            )

    def send_heartbeat(self):
        if not self.master:
            return
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            (
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                if getattr(self, "armed", False)
                else 0
            ),
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def send_status(self, voltage: float, current: float, cpu_load: int):
        """Wysyła SYS_STATUS."""
        if not self.master:
            return
        # Clamp battery current to int16 (10mA units)
        bat_curr_scaled = max(-32768, min(32767, int(current * 100)))

        self.master.mav.sys_status_send(
            0,
            0,
            0,
            cpu_load * 10,
            int(voltage * 1000),
            bat_curr_scaled,
            -1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def send_attitude(
        self, roll: float, pitch: float, yaw: float, imu_data: dict = None
    ):
        """Wysyła ATTITUDE (rad) oraz opcjonalnie RAW_IMU."""
        if not self.master:
            return
        self._tx_telem_count += 1
        time_boot_ms = int((time.time() - self.boot_time) * 1000)

        self.master.mav.attitude_send(
            time_boot_ms,
            roll,
            pitch,
            yaw,
            imu_data.get("gx", 0) if imu_data else 0,
            imu_data.get("gy", 0) if imu_data else 0,
            imu_data.get("gz", 0) if imu_data else 0,
        )

        if imu_data:
            # Clamp values to int16 range to prevent overflow crashes
            def clamp16(v):
                return max(-32768, min(32767, int(v)))

            self.master.mav.raw_imu_send(
                int(time.time() * 1e6),
                clamp16(imu_data.get("ax", 0) * 1000),
                clamp16(imu_data.get("ay", 0) * 1000),
                clamp16(imu_data.get("az", 0) * 1000),
                clamp16(imu_data.get("gx", 0) * 1000),
                clamp16(imu_data.get("gy", 0) * 1000),
                clamp16(imu_data.get("gz", 0) * 1000),
                clamp16(imu_data.get("mx", 0) * 10),  # 1 uT = 10 mG
                clamp16(imu_data.get("my", 0) * 10),
                clamp16(imu_data.get("mz", 0) * 10),
            )

    def send_position(
        self, lat: float, lon: float, alt: float, speed_kmh: float, hdg: float
    ):
        """Wysyła GLOBAL_POSITION_INT."""
        if not self.master:
            return
        time_boot_ms = int((time.time() - self.boot_time) * 1000)
        self.master.mav.global_position_int_send(
            time_boot_ms,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1000),
            0,  # relative_alt
            0,
            0,
            0,  # vx, vy, vz
            int(hdg * 100),
        )

    def send_obstacle_distance(self, distances_cm: List[int], increment: float = 1.0):
        """Wysyła OBSTACLE_DISTANCE (zoptymalizowany LiDAR)."""
        if not self.master:
            return
        # MAVLink OBSTACLE_DISTANCE przyjmuje 72 sektory (co 5 stopni)
        # Jeśli mamy 360 punktów, musimy zrobić downsampling.
        if len(distances_cm) > 72:
            step = len(distances_cm) // 72
            new_distances = [distances_cm[i * step] for i in range(72)]
        else:
            new_distances = distances_cm + [65535] * (72 - len(distances_cm))

        self.master.mav.obstacle_distance_send(
            int(time.time() * 1e6),
            0,  # sensor_type
            new_distances,
            5,  # increment (5 degrees)
            min(new_distances),
            max([d for d in new_distances if d < 65535] or [65535]),
            0,
            0,  # dummy
        )

    def send_statustext(
        self, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_INFO
    ):
        if not self.master:
            return
        self.master.mav.statustext_send(severity, text.encode("utf-8"))

    @property
    def link_active(self) -> bool:
        """Zwraca True, jeśli otrzymano Heartbeat w ciągu ostatnich 2 sekund."""
        return (time.time() - self.last_remote_heartbeat) < self.HEARTBEAT_TIMEOUT
