#!/usr/bin/env python3
"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Mostek RPi -> ROS2 (Wariant B).
Działa jako węzeł ROS2 na Raspberry Pi. Komunikuje się z `main_service.py` używając UDP:
- Subskrybuje telemetrię RPi (UDP port 12347) i publikuje topics ROS2 (/scan, /imu/data_raw, /odom)
- Subskrybuje topic /cmd_vel i wysyła pakiety sterujące UDP do RPi (port 12346)
Aby Wariant B zadziałał, w config.json należy ustawić "GCS_IP": "127.0.0.1".
"""

import base64
import json
import math
import socket
import struct
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


class RpiRos2Bridge(Node):
    """
    Węzeł ROS2 pełniący rolę mostka dla Raspberry Pi (Wariant B).
    ROS2 node acting as a bridge for Raspberry Pi (Variant B).
    """

    def __init__(self) -> None:
        """
        Inicjalizuje węzeł mostka ROS2.
        Initializes the ROS2 bridge node.
        """
        super().__init__("rpi_ros2_bridge")

        self.declare_parameter("udp_listen_port", 12347)
        self.declare_parameter("udp_send_port", 12346)
        self.declare_parameter("rpi_ip", "127.0.0.1")

        # Max prędkość fizyczna modelu dla normalizacji sterowania manual_controls
        # Throttle/Steering w RCSIM spodziewają się zakresów -1.0 do 1.0 (manual_controls)
        self.declare_parameter("max_speed_mps", 5.0)
        self.declare_parameter("max_steer_angle_rad", 0.523)  # ~30 stopni

        self.udp_listen_port = self.get_parameter("udp_listen_port").value
        self.udp_send_port = self.get_parameter("udp_send_port").value
        self.rpi_ip = self.get_parameter("rpi_ip").value
        self.max_speed = self.get_parameter("max_speed_mps").value
        self.max_steer = self.get_parameter("max_steer_angle_rad").value

        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Gniazdo uDP do wysyłania komend do main_service.py
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.running = True
        self.recv_thread = threading.Thread(target=self.udp_recv_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info(
            f"RPi->ROS2 Bridge uruchomiony (nasłuch: {self.udp_listen_port}, wysyłanie: {self.udp_send_port})"
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Callback dla topicu poleceń prędkości (cmd_vel).
        Callback for velocity commands topic (cmd_vel).

        Args:
            msg (Twist): Wiadomość Twist z żądaną prędkością. / Twist message with requested velocity.
        """
        # Normalizacja prędkosci do -1.0 .. 1.0 dla protokołu RCSIM
        throttle = msg.linear.x / self.max_speed
        throttle = max(-1.0, min(1.0, throttle))

        steering = msg.angular.z / self.max_steer
        steering = max(-1.0, min(1.0, steering))

        packet = {
            "type": "control",
            "t": time.time(),
            "manual_controls": {"steering": steering, "throttle": throttle},
        }

        try:
            data = json.dumps(packet).encode("utf-8")
            self.send_sock.sendto(data, (self.rpi_ip, self.udp_send_port))
        except Exception as e:
            self.get_logger().warn(f"Błąd wysyłania komendy do RPi: {e}")

    def get_ros_time(self) -> Any:
        """
        Pobiera aktualny czas ROS.
        Gets current ROS time.

        Returns:
            Any: Czas ROS w formacie wiadomości. / ROS time as a message.
        """
        t = self.get_clock().now()
        msg = t.to_msg()
        return msg

    def udp_recv_loop(self) -> None:
        """
        Pętla nasłuchująca i odbierająca pakiety telemetryczne UDP z RCSIM.
        Loop listening and receiving UDP telemetry packets from RCSIM.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", self.udp_listen_port))
        except OSError as e:
            self.get_logger().error(f"Nie powiodło się otwarcie portu UDP: {e}")
            return

        sock.settimeout(0.5)
        while self.running and rclpy.ok():
            try:
                data, _ = sock.recvfrom(65535)
                self.process_telemetry(data)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Błąd odbierania UDP: {e}")

        sock.close()

    def process_telemetry(self, data: bytes) -> None:
        """
        Przetwarza surowe dane telemetrii i publikuje wiadomości ROS2.
        Processes raw telemetry data and publishes ROS2 messages.

        Args:
            data (bytes): Surowe bajty JSON. / Raw JSON bytes.
        """
        try:
            packet = json.loads(data.decode("utf-8"))
            if packet.get("type") != "telemetry":
                return

            stamp = self.get_ros_time()

            # --- ODOMETRY ---
            pose = packet.get("pose")
            if pose and len(pose) == 3:
                x, y, yaw = pose
                odom_msg = Odometry()
                odom_msg.header.stamp = stamp
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                odom_msg.pose.pose.position.x = float(x)
                odom_msg.pose.pose.position.y = float(y)
                odom_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                odom_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
                self.odom_pub.publish(odom_msg)

            # --- IMU ---
            imu_data = packet.get("imu")
            if imu_data:
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "imu_link"
                imu_msg.linear_acceleration.x = float(imu_data.get("ax", 0.0))
                imu_msg.linear_acceleration.y = float(imu_data.get("ay", 0.0))
                imu_msg.linear_acceleration.z = float(imu_data.get("az", 0.0))
                imu_msg.angular_velocity.x = float(imu_data.get("gx", 0.0))
                imu_msg.angular_velocity.y = float(imu_data.get("gy", 0.0))
                imu_msg.angular_velocity.z = float(imu_data.get("gz", 0.0))
                self.imu_pub.publish(imu_msg)

            # --- LIDAR ---
            lidar_b64 = packet.get("lidar")
            if lidar_b64:
                # Odkodowanie b64 i array z 360 odleglosci w milimetrach
                raw = base64.b64decode(lidar_b64)
                distances_mm = struct.unpack("<360H", raw)

                scan_msg = LaserScan()
                scan_msg.header.stamp = stamp
                scan_msg.header.frame_id = "lidar_link"
                scan_msg.angle_min = 0.0
                scan_msg.angle_max = 2.0 * math.pi
                scan_msg.angle_increment = (2.0 * math.pi) / 360.0
                scan_msg.range_min = 0.1
                scan_msg.range_max = 12.0

                scan_msg.ranges = [0.0] * 360
                for i in range(360):
                    d = distances_mm[i] / 1000.0
                    if d >= 0.1 and d <= 12.0:
                        scan_msg.ranges[i] = float(d)
                    else:
                        scan_msg.ranges[i] = float("inf")

                self.scan_pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().debug(f"Telemetry parse error: {e}")

    def destroy_node(self) -> None:
        """
        Czyści i usuwa bezpiecznie zasoby węzła.
        Cleans up and safely removes node resources.
        """
        self.running = False
        if self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        self.send_sock.close()
        super().destroy_node()


def main(args: Any = None) -> None:
    """
    Główna funkcja uruchamiająca węzeł mostka.
    Main function to run the bridge node.

    Args:
        args (Any, optional): Argumenty wiersza poleceń. / Command line arguments.
    """
    rclpy.init(args=args)
    node = RpiRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Zatrzymywanie mostka RPi->ROS2...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
