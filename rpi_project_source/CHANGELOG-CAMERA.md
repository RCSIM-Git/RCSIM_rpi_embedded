# Camera Configuration Changes - 2026-02-03 & 2026-02-20

## Summary

### 2026-02-20 Updates
Resolved deployment failures by removing the redundant `mediamtx` Docker container from `docker-compose.yml`. The system now relies exclusively on the host-level `mediamtx.service` to prevent port (9997, 8554) and camera hardware (`/dev/video*`) conflicts on the Raspberry Pi 5.

### 2026-02-03 Updates
Fixed H.264 video stream corruption by switching from external script pipeline to MediaMTX native `rpiCamera` source.

## Changes Made

### 1. MediaMTX Configuration (`mediamtx.yml`)

**Changed from** (External Script Mode):
```yaml
paths:
  camera_ai:
    source: publisher
    sourceOnDemand: no
    runOnInit: /usr/local/bin/rcsim-cam
    runOnInitRestart: yes
```

**Changed to** (Native Camera Mode):
```yaml
paths:
  camera_ai:
    source: rpiCamera
    rpiCameraWidth: 1280
    rpiCameraHeight: 720
    rpiCameraFPS: 30
    rpiCameraBitrate: 5000000  # 5 Mbps for high quality
```

**Network Binding** (for Tailscale support):
```yaml
apiAddress: 0.0.0.0:9997      # Was: :9997
rtspAddress: 0.0.0.0:8554     # Was: :8554
webrtcAddress: 0.0.0.0:8889   # Was: :8889
```

### 2. External Camera Script (`run_camera_direct.sh`)

- **Status**: Deprecated (no longer used by MediaMTX)
- **Kept**: For reference purposes only
- **Added**: Deprecation notice in file header

### 3. Documentation

**New Files**:
- `README-CAMERA.md` - Comprehensive guide for native camera mode
- `CHANGELOG-CAMERA.md` - This file

**Updated Files**:
- `run_camera_direct.sh` - Added deprecation notice

## Problem Solved

### Before
- H.264 stream corruption with macroblock errors
- Frame read failures every ~100ms in Docker container
- FPS reporting: 90000.0 (incorrect H.264 timebase)
- RTP packet loss: 3-39 packets per interval
- Complex pipeline: rpicam-vid → ffmpeg → RTSP → MediaMTX

### After
- ✅ Zero H.264 decoder errors
- ✅ 100/100 frames received cleanly
- ✅ FPS reporting: 30.0 (correct)
- ✅ No RTP packet loss
- ✅ Simple pipeline: libcamera → MediaMTX (direct, native)

## Performance

**Test Results**:
- Resolution: 1280x720
- Frame rate: 30 FPS (stable)
- Bitrate: 5 Mbps (high quality)
- Success rate: 100% (100/100 frames)
- H.264 errors: 0

**Network Support**:
- ✅ Local network (192.168.x.x)
- ✅ Tailscale VPN (100.x.x.x)

## Deployment Notes

When deploying to RPi:
1. Copy `mediamtx.yml` to `/etc/mediamtx.yml`
2. Restart MediaMTX: `sudo systemctl restart mediamtx`
3. **Do NOT** copy or use `run_camera_direct.sh` (deprecated)

The deployment app should handle this automatically.

## Technical Details

### Why Native Mode Works

MediaMTX's `rpiCamera` source uses libcamera API directly:
- Single encoding pass (camera → H.264 encoder → RTSP)
- No intermediate processes or buffers
- Optimized for Raspberry Pi 5 hardware encoder
- Proper timestamp handling (no ffmpeg manipulation)
- Proven stable implementation

### Why External Script Failed

The rpicam-vid + ffmpeg pipeline had multiple failure points:
- rpicam-vid produced invalid NAL units
- Pipe buffer overruns during handoff
- ffmpeg timestamp/metadata manipulation
- Multiple encoding/decoding stages
- Environment variable fragility

## Endpoints

### RTSP (Docker Container)
- `rtsp://127.0.0.1:8554/camera_ai` (localhost)
- `rtsp://<RPI_IP>:8554/camera_ai` (network)

### WebRTC (PC Application)
- `http://<RPI_IP>:8889/camera_ai/whep` (WHEP protocol)
- Adaptive bitrate based on network conditions

## See Also

- `README-CAMERA.md` - Full documentation
- `mediamtx.yml` - Current configuration
- `modules/camera_manager.py` - Docker RTSP client
- PC app: `core/comm/mediamtx_strategy.py` - WebRTC client

## Status

✅ **Production Ready** - Tested and verified 2026-02-03
