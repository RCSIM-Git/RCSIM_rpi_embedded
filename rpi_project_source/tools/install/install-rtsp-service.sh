#!/bin/bash
# Script to SETUP Camera for MediaMTX Direct Mode
# This replaces the old legacy service installation

echo "🛑 Disabling Legacy RTSP Camera Service..."
sudo systemctl stop rtsp-camera.service || true
sudo systemctl disable rtsp-camera.service || true

echo "📸 Installing Camera Wrapper..."
# Assumes run_camera_direct.sh is in the same directory as this script
if [ -f "run_camera_direct.sh" ]; then
    sudo cp run_camera_direct.sh /usr/local/bin/rcsim-cam
    sudo chmod +x /usr/local/bin/rcsim-cam
    echo "✅ Wrapper installed to /usr/local/bin/rcsim-cam"
else
    echo "⚠️  run_camera_direct.sh not found in current directory!"
    # Fallback creation if file missing during copy
    echo '#!/bin/bash' | sudo tee /usr/local/bin/rcsim-cam > /dev/null
    echo 'exit 1' | sudo tee -a /usr/local/bin/rcsim-cam > /dev/null
    sudo chmod +x /usr/local/bin/rcsim-cam
fi

echo "🔄 Restarting MediaMTX to apply changes..."
sudo systemctl restart mediamtx || true

echo "✅ Camera setup complete. MediaMTX is now the commander."
exit 0
