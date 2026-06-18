# 🍓 RCSIM Deployment Module (Raspberry Pi 5 + Hailo-8)

This repository contains the onboard software and deployment infrastructure for the **RCSIM (Race Ready Autonomous System)** autonomous vehicle, designed to run directly on a **Raspberry Pi 5** with a **Hailo-8 / Hailo-8L** hardware accelerator.

The system is responsible for direct control of the physical RC vehicle, real-time sensor processing, AI inference on the NPU (Neural Processing Unit), path planning (SLAM/Cartographer/A*/Pure Pursuit), and low-latency two-way streaming/telemetry (WebRTC/UDP) with the Ground Control Station (GCS) on PC.

---

## 🚀 Key Responsibilities and Features

1. **Hardware I/O & Vehicle Control**
   - Integration with the **PCA9685** PWM controller for vehicle steering and throttle control.
   - Support for IMU sensors and GPS receivers (NMEA protocols, **RTK/NTRIP** correction client).
   - Reading RC transmitter input via the **CRSF (Crossfire)** parser.

2. **AI Inference & Detection (Hailo-8 / Hailo-8L)**
   - Hardware-accelerated End-to-End Regression (RCSIM) on the Hailo-8 NPU using `.hef` packages.

3. **Low-Latency Communication (Streaming & Telemetry)**
   - H.264 video streaming from the IMX219 camera module via a native **MediaMTX** pipeline (WebRTC/WHEP and RTSP).
   - Low-level packet fragmentation protocol (**Chunking**) preventing IP fragmentation issues by capping telemetry and map packets under the MTU limit (max 1100 bytes).
   - Support for the **MAVLink** protocol for integration with external flight controllers/autopilots.

4. **Autonomous Navigation & SLAM**
   - **CostmapManager**: Real-time occupancy grid management based on LiDAR scans.
   - **Global Planner**: Optimal path calculation using the **A*** algorithm.
   - **Local & Reactive Planner**: Collision avoidance and path tracking utilizing the **Pure Pursuit** algorithm.
   - **State Machine** and **Safety Supervisor**: Independent guard rails monitoring heartbeat, IMU crash G-forces, and obstacle proximity (Failsafe with automatic vehicle stop).

---

## 📂 Project Structure (`rpi_project_source`)

```bash
rpi_project_source/
├── core/                        # Core RPi OS logic
│   ├── main_service.py          # Main service orchestrating application lifecycle
│   ├── supervisor.py            # Onboard process and thread supervisor
│   ├── safety_supervisor.py     # Hard Safety Rules, Failsafe, and crash handling
│   ├── webrtc_manager.py        # WebRTC (WHEP) bridge for video and command routing
│   ├── chunking.py              # Map and SLAM packet fragmentation (under 1100 MTU)
│   ├── crsf_parser.py           # RC transmitter channel decoder
│   └── mavlink_service.py       # MAVLink telemetry and command service
│
├── modules/                     # Device drivers and AI inference
│   ├── ai_manager.py            # Hailo-8 NPU inference and .hef model loader
│   ├── camera_manager.py        # RTSP client receiving feed from local MediaMTX server
│   ├── pca9685.py               # I2C PWM controller driver for servos and ESC
│   ├── gps.py                   # LC29H GPS module integration with NTRIP RTK client
│   │
│   └── planners/                # Navigation & Autonomy Subsystem
│       ├── costmap_manager.py   # Occupancy grid generation and distance transform
│       ├── astar_planner.py     # Global path planner
│       ├── pure_pursuit_planner.py # Path tracker with dynamic lookahead
│       ├── reactive_planner.py  # Obstacle avoidance system
│       └── local_planner.py     # Facade coordinating sensors and planners
│
├── deployment/                  # Docker configs, startup scripts, systemd services
└── tests/                       # Unit and integration tests (pytest)
```

---

## 🛠️ Getting Started & Deployment (Docker)

The onboard software is fully containerized, ensuring a reproducible environment on the Raspberry Pi running in *headless* mode (without an X11 window server).

### Prerequisites:
- Raspberry Pi 5 running a compatible Linux OS.
- Docker and Docker Compose installed.
- Hailo RT drivers installed on the host OS (if NPU acceleration is used).
- **MediaMTX** installed and running on the host OS.

### Quick Start:

1. **Configuration**
   All settings (PC Ground Station IP, PWM limits, Pure Pursuit parameters, NTRIP credentials) are located in `rpi_project_source/config.json`. Ensure the file has a valid JSON format before launching.

2. **Build and Run Containers**
   ```bash
   cd rpi_project_source
   # Build the Docker image
   docker-compose build
   # Start the service container in the background
   docker-compose up -d
   ```

3. **Check Application Logs**
   ```bash
   docker-compose logs -f
   ```

---

## 🔌 Hardware Connections & Wiring Diagram

To set up the physical RC car, connect your sensors, controllers, and peripherals to the Raspberry Pi 5 GPIO header according to the following diagram:

### 1. I2C Bus Connections (PCA9685 & IMU GY-87/BMX160)
The PWM controller and the IMU share the I2C bus (Pins 3 and 5).
| Peripheral | Pin on Peripheral | Raspberry Pi 5 Pin / Name | Description |
|---|---|---|---|
| **PCA9685 (PWM)** | VCC | Pin 1 (3.3V) | Logic power supply |
| **PCA9685 (PWM)** | GND | Pin 9 (GND) | Logic ground |
| **PCA9685 (PWM)** | SDA | Pin 3 (GPIO 2 / SDA) | Data line |
| **PCA9685 (PWM)** | SCL | Pin 5 (GPIO 3 / SCL) | Clock line |
| **GY-87 IMU** | VCC | Pin 17 (3.3V) | Sensor power supply |
| **GY-87 IMU** | GND | Pin 25 (GND) | Sensor ground |
| **GY-87 IMU** | SDA | Pin 3 (GPIO 2 / SDA) | Shared Data line |
| **GY-87 IMU** | SCL | Pin 5 (GPIO 3 / SCL) | Shared Clock line |

*Note: Connect the Steering Servo to Channel 0 and the ESC (Electronic Speed Controller / Motor) to Channel 1 on the PCA9685.*

### 2. Serial & USB Connections (GPS, LiDAR, RC/MAVLink Receiver)
| Device | Pin / Port on Device | Raspberry Pi 5 Pin / Port | System Port | Description |
|---|---|---|---|---|
| **LC29H GPS** | TX | Pin 10 (GPIO 15 / RXD0) | `/dev/ttyAMA0` (UART0) | GPS telemetry RX |
| **LC29H GPS** | RX | Pin 8 (GPIO 14 / TXD0) | `/dev/ttyAMA0` (UART0) | GPS configuration TX |
| **CRSF / MAVLink** | TX | Pin 21 (GPIO 9 / RXD3) | `/dev/ttyAMA3` (UART3) | Telemetry / Control RX |
| **CRSF / MAVLink** | RX | Pin 24 (GPIO 8 / TXD3) | `/dev/ttyAMA3` (UART3) | Telemetry / Control TX |
| **LD08 LiDAR** | USB Connector | USB 2.0 / 3.0 Port | `/dev/rcsim/lidar` | Connected via USB-to-UART adapter |

*Always verify your `config.json` serial port paths match the physical hardware configuration.*

---

## 🔌 Connecting & Verifying Runtime Status

### 1. Connecting to the Raspberry Pi (via Terminal/CMD)
To log into the Raspberry Pi from your PC using Command Prompt (Windows) or Terminal (Linux/macOS):
```bash
# Connect using the SSH client (replace 'pi' and IP with your credentials)
ssh pi@<RASPBERRY_PI_IP>
```
If you are using **Tailscale VPN**, replace `<RASPBERRY_PI_IP>` with the RPi's Tailscale IP (e.g., `100.x.x.x`).

### 2. Verifying if RCSIM is Running Correctly
Once logged in, run the following commands to check system health:

- **Check if the Docker container is active:**
  ```bash
  docker ps
  # Look for a running container named "rcsim_industrial"
  ```
- **Inspect live application logs:**
  ```bash
  docker logs -f rcsim_industrial
  # Look for "All checks passed. Starting supervisor..." and periodic sensor/telemetry updates.
  ```
- **Verify MediaMTX Video Stream:**
  Make sure the RTSP/WebRTC streaming server is healthy:
  ```bash
  sudo systemctl status mediamtx
  # Inspect active paths (should show camera_ai ready):
  curl http://localhost:9997/v3/paths/list | jq
  ```
- **Check Hardware Access (I2C/Serial/NPU):**
  Verify that the container has successfully opened the I2C bus and Hailo-8 NPU:
  ```bash
  # Check if Hailo NPU is detected by the OS:
  hailortcli fw-control identify
  # Check I2C devices (PCA9685 should be on address 0x40):
  i2cdetect -y 1
  ```

---

## ⚠️ Development Guidelines & Hard Safety Rules

All developers contributing to this module must strictly adhere to the following safety and software standards:

1. **Hard-Safety Enforcement:**
   Never modify or bypass the safety checks in `safety_supervisor.py`. The hardware watchdog, heartbeat loss detection, and Emergency Stop procedures protect the physical vehicle from damage.
2. **MTU Limit Compliance:**
   Do not send telemetry or map packets larger than 1100 bytes over UDP to avoid IP fragmentation and packet loss. Always route large payloads through the `chunking.py` helper.
3. **Resource Cleanup:**
   Ensure all threads, UDP sockets, I2C buses, and Hailo NPU contexts are clean-closed inside class `cleanup()` / `stop()` methods to prevent resource leaks.
4. **No Plain Secrets:**
   Never commit passwords, RTK caster keys, or private IPs to `config.json`. Use environment variables or local ignored files for production credentials.
