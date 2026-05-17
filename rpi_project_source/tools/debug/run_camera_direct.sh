#!/bin/bash
# RCSIM Camera Wrapper - DEPRECATED (2026-02-03)
# This script is NO LONGER USED - MediaMTX now uses native 'rpiCamera' source
# Kept for reference only. See README-CAMERA.md for current configuration.
# Handles binary detection and cleanup for MediaMTX Direct Mode

# 1. Kill invalid hold on port 8554 or camera
pkill -9 rpicam-vid || true
pkill -9 libcamera-vid || true
# Stop legacy service if it somehow restarted
sudo systemctl stop rtsp-camera.service || true

# 2. Detect Camera Binary
CAM_BIN=""
if [ -f "/usr/bin/rpicam-vid" ]; then
    CAM_BIN="/usr/bin/rpicam-vid"
elif [ -f "/usr/bin/libcamera-vid" ]; then
    CAM_BIN="/usr/bin/libcamera-vid"
else
    echo "❌ Error: No camera binary found (rpicam-vid or libcamera-vid)"
    exit 1
fi

# Enable debug mode
set -x

echo "[$(date)] 📸 Starting Camera with: $CAM_BIN" >&2
echo "[$(date)] ⚙️  RTSP Target: rtsp://localhost:$RTSP_PORT/$MTX_PATH" >&2

# 3. Stream to MediaMTX
# Added -loglevel debug to ffmpeg for detailed error info
# Added LIBCAMERA_LOG_LEVELS=*:0 for full libcamera debug
export LIBCAMERA_LOG_LEVELS=*:0
exec $CAM_BIN -t 0 --camera 0 \
    --width 1280 --height 720 \
    --framerate 30 --intra 30 \
    --bitrate 1000000 \
    --profile baseline --level 4.0 \
    --inline --codec h264 --libav-format h264 \
    --nopreview -o - 2>> /tmp/rcsim_camera.log | \
    ffmpeg -loglevel debug -use_wallclock_as_timestamps 1 -fflags +genpts -i pipe:0 -c copy -f rtsp rtsp://localhost:8554/camera_ai >> /tmp/rcsim_camera.log 2>&1
