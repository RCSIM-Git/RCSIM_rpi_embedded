# RCSIM Camera Streaming - MediaMTX Native Mode

## Current Configuration (2026-02-03)

**Status**: ✅ Working perfectly with native `rpiCamera` source

The system now uses **MediaMTX's built-in RPi camera support** instead of external scripts.

## Quick Start

### 1. Deploy Configuration to Raspberry Pi

From the deployment app or manually:

```bash
# Copy updated mediamtx.yml to RPi
scp mediamtx.yml pi@<RPI_IP>:/tmp/
ssh pi@<RPI_IP> "sudo cp /tmp/mediamtx.yml /etc/mediamtx.yml"

# Restart MediaMTX
ssh pi@<RPI_IP> "sudo systemctl restart mediamtx"
```

### 2. Verify Stream

```bash
# Check MediaMTX status
ssh pi@<RPI_IP> "sudo systemctl status mediamtx"

# Check API for stream status
curl http://<RPI_IP>:9997/v3/paths/list | jq

# Test RTSP stream (from PC)
ffplay rtsp://<RPI_IP>:8554/camera_ai
```

## Configuration Details

### MediaMTX Native Camera Mode

**File**: `mediamtx.yml`

```yaml
paths:
  camera_ai:
    source: rpiCamera           # Native MediaMTX camera integration
    rpiCameraWidth: 1280
    rpiCameraHeight: 720
    rpiCameraFPS: 30
    rpiCameraBitrate: 5000000   # 5 Mbps for high quality
```

**Benefits**:
- ✅ Single, optimized encoding path (libcamera → MediaMTX)
- ✅ No external scripts, no ffmpeg overhead
- ✅ Proven stable on RPi 5
- ✅ Zero H.264 decoder errors
- ✅ Perfect 30 FPS performance

### Network Configuration

MediaMTX listens on **0.0.0.0** (all interfaces) for Tailscale support:

```yaml
apiAddress: 0.0.0.0:9997        # API endpoint
rtspAddress: 0.0.0.0:8554       # RTSP server
webrtcAddress: 0.0.0.0:8889     # WebRTC endpoint
```

**Accessible via**:
- Local network: `192.168.x.x`
- Tailscale VPN: `100.x.x.x` (Tailscale IP)

## Stream Endpoints

### For Docker Container (RPi)
- **Protocol**: RTSP
- **URL**: `rtsp://127.0.0.1:8554/camera_ai`
- **Transport**: UDP (configured in `camera_manager.py`)
- **Quality**: Fixed 5 Mbps

### For PC Application
- **Protocol**: WebRTC (recommended) or RTSP
- **WebRTC URL**: `http://<RPI_IP>:8889/camera_ai/whep`
- **RTSP URL**: `rtsp://<RPI_IP>:8554/camera_ai`
- **Quality**: Adaptive bitrate (WebRTC) or fixed 5 Mbps (RTSP)

**Application Mode**: Select **"WebRTC (RPi5 - Video + Data)"** for best performance

## Legacy Files (Not Used)

The following files are **no longer used** but kept for reference:

- `run_camera_direct.sh` - External camera script (replaced by native mode)
- `rtsp-camera.service` - Old systemd service (replaced by MediaMTX)
- `install-rtsp-service.sh` - Old installation script

**Note**: MediaMTX service (`mediamtx.service`) handles everything now.

## Performance Results

### Test Results (2026-02-03)

**PC Client (RTSP)**:
```
Testing RTSP stream: rtsp://192.168.31.224:8554/camera_ai
Stream opened successfully.
Received 100 frames. Resolution: 1280x720
✅ Success rate: 100/100 frames
✅ Zero H.264 decoder errors
✅ Perfect 30 FPS
```

**Docker Container**:
```
✅ Stream Opened. Resolution: 1280.0x720.0 @ 30.0 FPS
✅ No frame read failures
✅ No FAILSAFE triggers
```

**MediaMTX**:
```json
{
  "name": "camera_ai",
  "source": {"type": "rpiCameraSource"},
  "ready": true,
  "tracks": ["H264"],
  "bytesReceived": "76+ MB",
  "bytesSent": "35+ MB"
}
```

## Troubleshooting

### Stream not available

```bash
# Check MediaMTX service
sudo systemctl status mediamtx
sudo journalctl -u mediamtx -f

# Verify camera detection
ls -la /dev/video*

# Check libcamera
libcamera-hello --list-cameras
```

### Low quality / compression artifacts

Edit `mediamtx.yml` and increase bitrate:
```yaml
rpiCameraBitrate: 8000000  # 8 Mbps (very high quality)
```

Then restart:
```bash
sudo systemctl restart mediamtx
```

### Tailscale connection issues

Verify MediaMTX listens on all interfaces:
```bash
sudo netstat -tlnp | grep mediamtx
# Should show 0.0.0.0:8554 and 0.0.0.0:8889
```

If shows `127.0.0.1` or `:::`, update `mediamtx.yml` addresses to `0.0.0.0:PORT`

## Deployment Notes

When deploying via the deployment app, ensure:
1. `mediamtx.yml` is copied to `/etc/mediamtx.yml` on RPi
2. MediaMTX service is restarted after config changes
3. Old camera scripts (`run_camera_direct.sh`) are **not** used

The deployment app should handle this automatically.

## Files

- `mediamtx.yml` - **Main configuration** (native rpiCamera mode)
- `README-CAMERA.md` - This file
- `config.json` - Docker app config (references RTSP URL)
- `modules/camera_manager.py` - Docker camera manager (RTSP client)

## Summary

**Architecture**: RPi Camera → libcamera → MediaMTX (native) → RTSP/WebRTC

**Status**: ✅ Production ready, tested, and verified

**Quality**: 1280x720 @ 30 FPS, 5 Mbps bitrate

**Compatibility**: Local network + Tailscale VPN, RTSP + WebRTC
