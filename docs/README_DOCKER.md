# Docker Deployment Guide on Raspberry Pi (RCSIM)

This document describes how to build and run the RCSIM application on a Raspberry Pi using Docker.

## Prerequisites

The following software must be installed on your Raspberry Pi:
- **Docker**: Install using: `curl -fsSL https://get.docker.com -o get-docker.sh && sh get-docker.sh`
- **Docker Compose**: Usually comes pre-installed as a Docker plugin (`docker compose`).

## Directory Structure

Ensure the following directory structure is set up on the Raspberry Pi:

```text
rpi_project_source/
├── docker-compose.yml
├── Dockerfile
├── requirements.txt
├── core/
│   ├── supervisor.py
│   └── ...
└── modules/
    └── ...
```

## Running the Application

1. **Navigate to the deployment directory**:
   ```bash
   cd rpi_project_source
   ```

2. **Build and start the containers**:
   Use the `docker compose up` command with the `--build` flag to force a rebuild of the image after code modifications.
   ```bash
   docker compose up --build -d
   ```
   - `-d`: Runs the containers in the background (detached mode).

3. **View application logs**:
   ```bash
   docker compose logs -f
   ```

4. **Stop the containers**:
   ```bash
   docker compose down
   ```

## Technical Notes

- **Hardware Permissions**: The container runs in `privileged: true` and `network_mode: "host"` modes, which are required for direct access to the GPIO pins, I2C bus, camera interface, and serial ports.
- **Audio Support**: The container image has `portaudio19-dev` and `pulse/alsa` libraries installed to support microphone and speaker feedback. Ensure that audio on the host OS is not blocked by another process.
- **WebRTC**: UDP ports are dynamic, but thanks to `network_mode: "host"`, manual port forwarding is not necessary.

## Troubleshooting

- **"ModuleNotFoundError"**: Ensure you have copied all files from the `core` folder into `rpi_project_source/core`.
- **Audio Interface Issues**: Verify that the SSH user on the Raspberry Pi belongs to the `audio` group.