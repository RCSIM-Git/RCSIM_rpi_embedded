# 🍓 RCSIMDEPLOY Module (Raspberry Pi Deployment)

This directory contains the core software and installation infrastructure target-deployed on the onboard robot device (Raspberry Pi 5).

## Responsibilities (Core Tasks)
1. Providing communication code (I2C) for hardware controllers (e.g., PCA9685).
2. Handling **Hailo-8** hardware acceleration utilizing `.hef` model packages.
3. WebRTC pipeline (h264 video streaming from the IMX219 camera module) along with the UDP fragmentation protocol.
4. The environment is managed using Docker scripts (`docker-compose.yml`, `Dockerfile`) - the project requires compatibility with headless environments (no windowing system).

## Structure:
* `rpi_project_source/core/`: Core RPi logic (including `chunking` and WebRTC connector).
* `rpi_project_source/modules/`: Peripheral controllers configuration and image detection (Hailo).
* `rpi_project_source/deployment/`: Docker helpers and startup logic.
* `rpi_project_source/tests/`: Unit tests for the Raspberry Pi onboard software.
