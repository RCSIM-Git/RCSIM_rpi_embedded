"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Delegat realizujący wzorzec Command.
Dekoduje komendy z PC (zarówno pakiety binarne i ramki JSON)
przekierowując zadania na menedżery systemu telemetrii rPi.
"""

import json
import logging
import os
import struct
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from core.main_service import TelemetryWorker

from core.binary_path_assembler import BinaryPathAssembler
from pymavlink import mavutil

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class CommandDispatcher:
    """
    Dyspozytor komend z komputera PC do usług Raspberry Pi.
    Dispatcher of commands from PC to Raspberry Pi services.
    """

    def __init__(self, worker: "TelemetryWorker"):
        self.worker = worker
        self.logger = logging.getLogger("CommandDispatcher")
        self.path_assembler = BinaryPathAssembler()
        self.last_processed_tx_time = 0.0

    def on_data_received(self, data: Any) -> None:
        """
        Callback obsługujący dane przychodzące z PC (WebRTC/UDP).
        Callback handling incoming data from PC (WebRTC/UDP).

        Rozpoznaje czy jest to pakiet binarny (CT) czy słownik JSON.
        """
        w = self.worker
        w.last_packet_time = time.time()

        if isinstance(data, bytes) and data.startswith(b"HS"):
            self._handle_handshake(data)
            return

        if isinstance(data, bytes) and data.startswith(b"CT"):
            # Binary Control Logic
            self._handle_binary_control(data)
            return

        if isinstance(data, bytes) and data.startswith(b"PT"):
            # [PLAN-001] Binary Path Logic
            self._handle_binary_path(data)
            return

        # MAVLink detection (v1: 0xFE, v2: 0xFD)
        if isinstance(data, bytes) and len(data) > 0 and data[0] in [0xFD, 0xFE]:
            self._handle_mavlink_packet(data)
            return

        try:
            decoded_text = data.decode("utf-8") if isinstance(data, bytes) else data
            msg = json.loads(decoded_text)
            msg_type = msg.get("type")

            if msg_type == "control":
                self._handle_json_control(msg)
            elif msg_type in ["command", "ai_command"]:
                self.handle_command(msg)

        except Exception as e:
            self.logger.warning(f"Data error (JSON): {e}")

    def _handle_binary_control(self, data: bytes) -> None:
        """
        Dekoduje sterowanie po magistrali binarnej UDP/WebRTC.
        """
        w = self.worker
        try:
            packet_len = len(data)
            if not w.pca_armed:
                return

            num_channels = 0
            channels = []

            tx_time = None

            if packet_len in [19, 27] and len(data) >= packet_len:
                payload = data[2:18]
                if len(payload) != 16:
                    return
                num_channels = 8
                channels = struct.unpack("<8H", payload)
                if packet_len == 27:
                    tx_bytes = data[18:26]
                    if len(tx_bytes) == 8:
                        tx_time = struct.unpack("<d", tx_bytes)[0]

            elif packet_len in [35, 43] and len(data) >= packet_len:
                payload = data[2:34]
                if len(payload) != 32:
                    return
                num_channels = 16
                channels = struct.unpack("<16H", payload)
                if packet_len == 43:
                    tx_bytes = data[34:42]
                    if len(tx_bytes) == 8:
                        tx_time = struct.unpack("<d", tx_bytes)[0]
            else:
                return

            received_checksum = data[packet_len - 1]
            calculated_checksum = 0
            for b in data[: packet_len - 1]:
                calculated_checksum ^= b

            if calculated_checksum != received_checksum:
                self.logger.warning("Binary Control CRC Error")
                return

            if tx_time is not None:
                if tx_time <= self.last_processed_tx_time:
                    self.logger.debug(f"Odrzucono przestarzały pakiet binarny CT: tx_time={tx_time:.3f} <= last={self.last_processed_tx_time:.3f}")
                    return
                self.last_processed_tx_time = tx_time
                w.last_pc_timestamp = tx_time
            else:
                w.last_pc_timestamp = time.time()

            # Priority Logic for HYBRID mode
            if w.comm_mode == "HYBRID" and w.elrs_link_established:
                # RF Link is dominant, ignore PC controls
                return

            hw = w.hw_manager
            if hw:
                s_min, s_max = hw.actuators.steering_range
                s_center = (s_min + s_max) / 2.0
                s_div = (s_max - s_min) / 2.0

                t_min, t_max = hw.actuators.throttle_range
                t_center = (t_min + t_max) / 2.0
                t_div = (t_max - t_min) / 2.0

                w.last_control_input = {
                    "manual_controls": {
                        "steering": (
                            (channels[0] - s_center) / s_div if s_div != 0 else 0.0
                        ),
                        "throttle": (
                            (channels[1] - t_center) / t_div if t_div != 0 else 0.0
                        ),
                    }
                }

                w.extra_channels_data = {}
                for i in range(2, num_channels):
                    w.extra_channels_data[i] = channels[i]

            if w.current_mode == "FAILSAFE":
                w.current_mode = "MANUAL"
                self.logger.info("Link recovered via Binary - Exiting FAILSAFE.")
            return

        except Exception as e:
            self.logger.warning(f"Binary parse error: {e}")
            return

    def _handle_json_control(self, msg: dict[str, Any]) -> None:
        """
        Dekoduje sterowanie w oparciu o obiekt JSON.
        """
        w = self.worker
        if not w.pca_armed:
            return
        channels = msg.get("channels", [])
        # Priority Logic for HYBRID mode
        if w.comm_mode == "HYBRID" and w.elrs_link_established:
            return

        t_pc = msg.get("t")
        if t_pc is not None:
            tx_time = float(t_pc)
            if tx_time <= self.last_processed_tx_time:
                self.logger.debug(f"Odrzucono przestarzały pakiet JSON control: t={tx_time:.3f} <= last={self.last_processed_tx_time:.3f}")
                return
            self.last_processed_tx_time = tx_time
            w.last_pc_timestamp = tx_time
        else:
            w.last_pc_timestamp = time.time()

        if len(channels) >= 2 and w.hw_manager:
            s_min, s_max = w.hw_manager.actuators.steering_range
            s_center = (s_min + s_max) / 2.0
            s_div = (s_max - s_min) / 2.0

            t_min, t_max = w.hw_manager.actuators.throttle_range
            t_center = (t_min + t_max) / 2.0
            t_div = (t_max - t_min) / 2.0

            w.last_control_input = {
                "manual_controls": {
                    "steering": (
                        (channels[0] - s_center) / s_div if s_div != 0 else 0.0
                    ),
                    "throttle": (
                        (channels[1] - t_center) / t_div if t_div != 0 else 0.0
                    ),
                }
            }

            if w.current_mode == "FAILSAFE":
                w.current_mode = "MANUAL"
                self.logger.info("Link recovered via JSON - Exiting FAILSAFE.")

    def handle_command(self, msg: dict[str, Any]) -> None:
        """
        Rozpoznaje i wykonuje komendę otrzymaną od klienta.
        Recognizes and executes a command received from the client.

        Args:
            msg (dict[str, Any]): Wiadomość zawierająca instrukcję.
        """
        w = self.worker
        cmd = msg.get("command")

        if cmd == "set_mode":
            new_mode = msg.get("mode")
            if new_mode:
                w.current_mode = new_mode
                self.logger.info(f"Mode changed to: {new_mode}")

        elif cmd == "ARM_PCA":
            self.logger.info("COMMAND: ARM_PCA received. System ARMED.")
            w.pca_armed = True
            if w.hw_manager:
                w.hw_manager.arm_pca()

        elif cmd == "DISARM_PCA":
            self.logger.info("COMMAND: DISARM_PCA received. System DISARMED.")
            w.pca_armed = False
            if w.hw_manager:
                w.hw_manager.disable_all_channels()

        elif cmd == "SET_AI_MODE":
            payload = msg.get("payload", {})
            mode = payload.get("mode")
            enabled = payload.get("enabled", True)

            if w.nav_manager:
                w.nav_manager.auto_explore_active = False

            if enabled:
                self.logger.info(f"COMMAND: SET_AI_MODE active -> {mode}")
                if mode == "Return to Home":
                    w.current_mode = "RTH"
                elif mode == "Auto-Explore":
                    w.current_mode = "AUTONOMOUS"
                    if w.nav_manager:
                        w.nav_manager.auto_explore_active = True
                        w.nav_manager.clear_path()
                else:
                    w.current_mode = "AUTONOMOUS"
                    if w.ai_manager and hasattr(w.ai_manager, "set_behavior"):
                        w.ai_manager.set_behavior(mode)
            else:
                self.logger.info("COMMAND: SET_AI_MODE disabled -> Reverting to MANUAL")
                w.current_mode = "MANUAL"

        elif cmd == "CALIBRATE_VAL":
            val = msg.get("value", 1500)
            ch = msg.get("channel", -1)
            self.logger.info(f"CALIBRATION: Force PWM Ch {ch} -> {val}us")
            if w.hw_manager:
                w.hw_manager.set_pwm(val, force=True, channel=ch)

        elif cmd == "CALIBRATE_ESC":
            ch = msg.get("channel", -1)
            self.logger.info(f"COMMAND: CALIBRATE_ESC received for Ch {ch}.")
            if w.hw_manager:
                w.hw_manager.calibrate_esc(channel=ch)
                self.logger.info(f"CALIBRATE_ESC for Ch {ch} finished.")

        elif cmd == "set_home":
            w.home_position = msg.get("position")
            self.logger.info(f"Home position updated: {w.home_position}")

        elif cmd == "UPDATE_CONFIG":
            payload = msg.get("payload", {})
            hw_config = payload.get("hardware")
            if hw_config:
                if w.hw_manager:
                    w.hw_manager.config.update(hw_config)

                # Propagacja limitu prędkosci również do planera przestrzennego
                if "max_speed_mps" in hw_config and getattr(w, "local_planner", None):
                    auto_nav = w.config.setdefault("autonomous_navigation", {})
                    pp_cfg = auto_nav.setdefault("pure_pursuit", {})
                    pp_cfg["max_speed_mps"] = hw_config["max_speed_mps"]
                    w.local_planner.update_config(w.config)

                self.logger.info(f"COMMAND: UPDATE_CONFIG applied: {hw_config}")

        elif cmd == "CALIBRATE_IMU":
            self.logger.info("COMMAND: CALIBRATE_IMU received.")
            if w.hw_manager:
                success = w.hw_manager.calibrate_imu()
                if success:
                    self.logger.info("IMU Calibration successful.")
                else:
                    self.logger.error("IMU Calibration failed.")

        elif cmd == "FAULT_INJECT":
            fault_type = msg.get("fault_type")
            enabled = msg.get("enabled", True)
            if fault_type and hasattr(w, "safety_supervisor") and w.safety_supervisor:
                w.safety_supervisor.inject_fault(fault_type, enabled)
                self.logger.info(f"COMMAND: FAULT_INJECT {fault_type}={enabled}")

        elif cmd == "set_comm_mode":
            new_comm_mode = msg.get("mode")
            if new_comm_mode in ["WEBRTC", "UDP", "HYBRID"]:
                w._switch_comm_mode(new_comm_mode)

        elif cmd == "RESET_SAFETY":
            self.logger.info("COMMAND: RESET_SAFETY received.")
            w.safety_supervisor.reset_impact()
            if w.current_mode == "FAILSAFE":
                w.current_mode = "MANUAL"

        elif cmd == "VERSION_WARNING":
            payload = msg.get("payload", {})
            gcs_ver = payload.get("gcs_version", "unknown")
            from core.version import APP_VERSION
            self.logger.warning(
                f"!!! Wykryto starszą wersję na pojeździe! GCS Wersja: {gcs_ver}, RPi Wersja: {APP_VERSION} !!!"
            )

        elif cmd == "GO_TO":
            target = msg.get("target")
            if target and hasattr(w, "slam_manager") and hasattr(w, "nav_manager"):
                self.logger.info(f"COMMAND: GO_TO received: {target}")
                try:
                    # Get fresh map
                    grid = w.slam_manager.get_grid_array()
                    start_pose = (w.current_pose[0], w.current_pose[1])
                    target_pose = (float(target[0]), float(target[1]))

                    # Plan path
                    success = w.nav_manager.plan_global_path(
                        grid, start_pose, target_pose
                    )

                    if success:
                        self.logger.info(
                            "Global path planned successfully. Switching to AUTONOMOUS."
                        )
                        w.current_mode = "AUTONOMOUS"
                    else:
                        self.logger.warning("Failed to plan global path.")
                except Exception as e:
                    self.logger.error(f"Error handling GO_TO: {e}")

        elif cmd == "START_PATROL":
            waypoints = msg.get("waypoints")
            loop = msg.get("loop", False)
            if waypoints and w.mission_manager:
                self.logger.info(
                    f"COMMAND: START_PATROL received with {len(waypoints)} points."
                )
                w.mission_manager.start_mission(waypoints, loop)
                w.current_mode = "AUTONOMOUS"

                w.mission_manager.stop_mission()
                w.current_mode = "MANUAL"
                self.logger.info("COMMAND: STOP_MISSION received.")

    def _handle_handshake(self, data: bytes) -> None:
        """Obsługuje binarny sygnał Handshake (HS) i odpowiada HR."""
        w = self.worker
        try:
            # GCS przesyła HS + timestamp (4B)
            gcs_time = struct.unpack("<I", data[2:6])[0]
            self.logger.info(f"Handshake received from GCS (Time: {gcs_time})")

            # Response: HR [TYPE:1B][VER:2B][CAPS:4B]
            # Type 3 = RPi, Ver 38 = V38, Caps: 0x01 (SLAM), 0x02 (AI)
            caps = 0x00
            if w.slam_manager: caps |= 0x01
            if w.ai_manager: caps |= 0x02

            # Respond via UDP immediately to establish return path
            response = b"HR" + struct.pack("<BH", 3, 38)
            if w.udp_service:
                w.udp_service.send_data(response)
                self.logger.info("Handshake Response (HR) sent to GCS via UDP.")
            
            # Reset failsafe since we have valid contact
            w.last_packet_time = time.time()
            w.link_established = True
        except Exception as e:
            self.logger.error(f"Handshake error: {e}")

    def _handle_binary_path(self, data: bytes) -> None:
        """
        Obsługuje fragmenty binarnej ścieżki [PLAN-001].
        """
        w = self.worker
        waypoints = self.path_assembler.add_chunk(data)

        if waypoints:
            self.logger.info(
                f"Binary path reassembled: {len(waypoints)} points. Starting mission."
            )
            if w.mission_manager:
                w.mission_manager.start_mission(
                    waypoints, loop=False
                )  # Domyślnie bez pętli
                w.current_mode = "AUTONOMOUS"

    def _handle_mavlink_packet(self, data: bytes) -> None:
        """
        Obsługuje surowe pakiety MAVLink otrzymane przez dowolny transport (UDP/WebRTC).
        """
        if not hasattr(self, "_mav_parser"):
            self._mav_parser = mavutil.mavlink_connection("mem:", source_system=255)

        self._mav_parser.write(data)
        while True:
            msg = self._mav_parser.recv_match(blocking=False)
            if not msg:
                break

            msg_type = msg.get_type()
            if msg_type == "RC_CHANNELS_OVERRIDE":
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
                self._handle_mavlink_rc(channels)
            elif msg_type == "COMMAND_LONG":
                self._handle_mavlink_command(msg)

    def _handle_mavlink_rc(self, channels: list[int]) -> None:
        """Konwertuje kanały MAVLink na sterowanie RCSIM."""
        w = self.worker
        if not w.hw_manager:
            return

        hw = w.hw_manager
        s_min, s_max = hw.actuators.steering_range
        s_center = (s_min + s_max) / 2.0
        s_div = (s_max - s_min) / 2.0

        t_min, t_max = hw.actuators.throttle_range
        t_center = (t_min + t_max) / 2.0
        t_div = (t_max - t_min) / 2.0

        # MAVLink uses 65535 to indicate "no change"
        # We need to handle this to avoid huge values from (65535-1500)/500
        
        last_s = w.last_control_input.get("manual_controls", {}).get("steering", 0.0) if w.last_control_input else 0.0
        last_t = w.last_control_input.get("manual_controls", {}).get("throttle", 0.0) if w.last_control_input else 0.0

        steering = (channels[0] - s_center) / s_div if s_div != 0 and channels[0] < 65535 else last_s
        throttle = (channels[1] - t_center) / t_div if t_div != 0 and channels[1] < 65535 else last_t

        w.last_control_input = {
            "manual_controls": {
                "steering": steering,
                "throttle": throttle,
            }
        }
        w.last_packet_time = time.time()
        
        if w.pca_armed:
            self.logger.debug(f"MAVLink Normalized Controls: Steer={steering:.3f}, Throt={throttle:.3f} (Mode: {w.current_mode})")

        for i in range(2, min(len(channels), 16)):
            if channels[i] < 65535:
                w.extra_channels_data[i] = channels[i]

        if w.current_mode == "FAILSAFE":
            w.current_mode = "MANUAL"
            self.logger.info("Link recovered via MAVLink - Exiting FAILSAFE.")

        if not w.pca_armed:
            if getattr(self, "_last_disarm_msg", 0) < time.time() - 5.0:
                self.logger.warning(
                    "MAVLink RC data received but system is DISARMED. "
                    "Send ARM command to enable actuation."
                )
                self._last_disarm_msg = time.time()
            return

    def _handle_mavlink_command(self, msg) -> None:
        """Tłumaczy MAV_CMD na komendy RCSIM."""
        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.param1 == 1:
                self.handle_command({"command": "ARM_PCA"})
            else:
                self.handle_command({"command": "DISARM_PCA"})
        # Ack
        if hasattr(self.worker, "mavlink_service") and self.worker.mavlink_service:
            self.worker.mavlink_service.master.mav.command_ack_send(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED
            )
