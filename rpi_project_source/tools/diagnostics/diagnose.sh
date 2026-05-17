#!/bin/bash
# RCSIM Diagnostic Tool
# Run this on the Raspberry Pi to check hardware and service status

echo "🔍 RCSIM Diagnostic Tool"
echo "========================"
echo "Date: $(date)"
echo "User: $USER"
echo ""

# 1. Check Root Privileges
if [ "$EUID" -ne 0 ]; then
  echo "⚠️  Warning: Not running as root. Some checks (systemctl, i2cdetect) might fail."
  echo "   Suggestion: Run with 'sudo ./diagnose.sh'"
  echo ""
fi

# 2. Check I2C Bus
echo "🔎 Checking I2C Bus 1..."
if command -v i2cdetect &> /dev/null; then
    i2cdetect -y 1
    echo "   (If the grid is empty or slow, check hardware connections!)"
else
    echo "❌ 'i2cdetect' not found. Install i2c-tools: sudo apt install i2c-tools"
fi
echo ""

# 3. Check Camera Binaries
echo "🔎 Checking Camera Binaries..."
if [ -f "/usr/bin/rpicam-vid" ]; then
    echo "✅ rpicam-vid found."
elif [ -f "/usr/bin/libcamera-vid" ]; then
    echo "✅ libcamera-vid found."
else
    echo "❌ No supported camera binary found (rpicam-vid or libcamera-vid)!"
fi
echo ""

# 4. Check MediaMTX Service
echo "🔎 Checking MediaMTX Service..."
if systemctl is-active --quiet mediamtx.service; then
    echo "✅ MediaMTX service is RUNNING."
else
    echo "❌ MediaMTX service is NOT running."
    echo "   Status output:"
    systemctl status mediamtx.service --no-pager | head -n 10
fi
echo ""

# 5. Check Docker Container
echo "🔎 Checking RCSIM Docker Container..."
if docker ps | grep -q "rcsim_industrial"; then
    echo "✅ Container 'rcsim_industrial' is RUNNING."
else
    echo "❌ Container 'rcsim_industrial' is NOT running."
    echo "   Last 5 lines of logs:"
    docker logs --tail 5 rcsim_industrial 2>&1
fi
echo ""

# 6. Check Camera Log
echo "🔎 Checking Camera Log (/tmp/rcsim_camera.log)..."
if [ -f "/tmp/rcsim_camera.log" ]; then
    echo "   Last 10 lines:"
    tail -n 10 /tmp/rcsim_camera.log
else
    echo "⚠️  Log file /tmp/rcsim_camera.log not found."
fi

echo ""
echo "========================"
echo "✅ Diagnostic run complete."
