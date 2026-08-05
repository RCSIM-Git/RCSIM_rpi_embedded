# RCSIM Embedded (RPi 5) - Architektura & Mapa Drogowa (Roadmap)

## Aktualna Architektura Produkcyjna

RCSIM RPi wykorzystuje wielowątkową architekturę o niskich opóźnieniach w czystym Pythonie, zoptymalizowaną pod kątem bezpieczeństwa sprzętowego i kontroli real-time:

### 1. Komunikacja & Szyna Danych (`main_service.py`)
- **Transport**: MAVLink v2 (UDP / Port Szeregowy RF / ELRS) + zoptymalizowany WebRTC (WHEP / H.264) do transmisji FPV.
- **Wątki Robocze**: `TelemetryWorker` (20-50Hz pętla główna), `ActuatorWorker` (50Hz pętla PWM/I2C z wysokim priorytetem), `CameraManager` (Wizja & NPU Hailo-8).
- **Zarządzanie Połączeniem**: Watchdog sprzętowy (Pin GPIO 26) oraz programowy `SafetySupervisor` (Failsafe z progiem timeoutu).

### 2. Percepcja, SLAM & Fuzja Czujników
- **SLAM**: BreezySLAM / LiDAR 2D + Odometria z muteksem C i chunkowaniem pakietów UDP do GCS.
- **Kamera & AI**: Akcelerator Hailo-8 NPU z bezpośrednim przetwarzaniem klatek wizyjnych i detekcją przeszkód.
- **Fuzja**: Zintegrowana fabryka sensorów I2C (`sensor_factory.py`) z odczytem MPU6050/MPU9250/QMC5883L/BMP280 oraz synchro z GPS RTK.

### 3. Planowanie Lokalne i Reaktywne unikanie przeszkód
- **Planer Lokalny**: Algorytm DWA (Dynamic Window Approach) + Siatka Kosztów (`CostmapManager`).
- **Planer Reaktywny**: `ReactivePlanner` z modułem `reactive_override` dynamicznie redukującym gaz i korygującym tor jazdy na bazie `safety_score`.
- **Adaptacyjny Tempomat (ACC)**: Reaktywne wyliczanie dystansu ze skanów LiDAR i płynna redukcja przepustnicy.

### 4. Arbitraż Sterowania (`control_selector.py`) & RTH
- **Tryby**: `MANUAL`, `AI_STEER_ONLY`, `FULL_AUTOPILOT`, `AUTONOMOUS`, `RTH`, `FAILSAFE`.
- **RTH (Return-To-Home)**:
  - Nawigacja geograficzna (PID Bearing / GPS RTK lub SLAM pose fallback).
  - Płynne zwalnianie przepustnicy przy zbliżaniu się do punktu domowego (< 10m).
  - **Pełne unikanie przeszkód**: Przepuszczanie komend RTH przez `reactive_override` oraz aktywne sprawdzanie przeszkód przez `planner_cmd` i ACC.

---

## Status Zadań (Roadmap Tasks)

- [x] Zastąpienie zarchiwizowanego protokołu CRSF/FBW natywnym serwisem MAVLink v2.
- [x] Zapewnienie pełnej telemetrii prędkości w `GLOBAL_POSITION_INT` (składowe $v_x, v_y$).
- [x] Zintegrowanie `ReactivePlanner` z trybem RTH (reaktywne unikanie przeszkód podczas powrotu).
- [x] Płynna deakceleracja RTH przy zbliżaniu się do celu.
- [x] Ujednolicenie i czyszczenie podwójnych inicjalizacji oraz nieużywanych wywołań w `control_selector` i `main_service`.
- [x] Konsolidacja zestawu testów jednostkowych dla `NavigationManager`.
- [x] Dalsza optymalizacja siatki kosztów (Costmap) pod kątem stromej rzeźby terenu (3D PointCloud).
- [x] Stworzenie i pełna implementacja produkcyjnej architektury ROS2 (C++) w `RCSIM_RPi_tier_5_ros2` zastępującej `RCSIM_rpi_embedded` na Raspberry Pi 5 (z zachowaniem 100% kompatybilności MAVLink v2 z GCS, wsparciem Cartographer SLAM, Nav2 Ackermann RPP, fizycznych czujników IMU I2C, Quectel LC29H RTK GPS z Zero-Jump Guard, 2D LiDAR, PCA9685 I2C PWM z watchdogiem 500ms, INA219 UPS Power Monitor oraz maszyny stanów RTH).
- [x] Zintegrowanie wielokanałowego mostka komunikacyjnego (UDP 12347 JSON Telemetry + WebRTC / WHEP FPV Video z serwerem STUN Google).
- [x] Stworzenie pakietów symulacji trójwymiarowej Gazebo 3D (`rcsim_description` z modelem URDF/Xacro pojazdu Ackermann + `rcsim_simulation` z launcherem 3D).
- [x] Pełne zbudowanie i zweryfikowanie 13 pakietów ROS2 w C++ (`colcon build`) oraz uruchomienie 100% przechodzącego zestawu testów jednostkowych C++ GTest (`colcon test-result`).