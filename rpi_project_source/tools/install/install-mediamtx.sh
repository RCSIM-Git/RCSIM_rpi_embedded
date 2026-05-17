#!/bin/bash
# MediaMTX Installation Script for RCSIM
# Installs MediaMTX WebRTC server for dual-stream camera setup

set -e

echo "🎥 Installing MediaMTX WebRTC Server..."

# Detect user
INSTALL_USER="${SUDO_USER:-$USER}"
echo "👤 Configuring for user: $INSTALL_USER"

# Install directory
MEDIAMTX_DIR="/opt/mediamtx"
CONFIG_FILE="/etc/mediamtx.yml"

# Detect architecture (ARMv7 vs ARM64)
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    echo " Detected: 64-bit Architecture (ARM64)"
    FILE_ARCH="arm64v8"
else
    echo " Detected: 32-bit Architecture (ARMv7)"
    FILE_ARCH="armv7"
fi

# Download MediaMTX
echo "📦 Downloading MediaMTX ($FILE_ARCH)..."
MEDIAMTX_VERSION="v1.10.0"
DOWNLOAD_URL="https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/mediamtx_${MEDIAMTX_VERSION}_linux_${FILE_ARCH}.tar.gz"

SOURCE_DIR=$(pwd)
mkdir -p "$MEDIAMTX_DIR"
cd "$MEDIAMTX_DIR"

curl -L "$DOWNLOAD_URL" -o mediamtx.tar.gz
tar -xzf mediamtx.tar.gz
rm mediamtx.tar.gz
chmod +x mediamtx

echo "✅ MediaMTX binary installed to $MEDIAMTX_DIR"

# Copy configuration from source
if [ -f "$SOURCE_DIR/mediamtx.yml" ]; then
    echo "📄 Installing provided MediaMTX configuration..."
    cp "$SOURCE_DIR/mediamtx.yml" "$CONFIG_FILE"
else
    echo "⚠️ Warning: mediamtx.yml not found in $SOURCE_DIR, skipping (Deployment script will handle this)."
fi

# Install Camera Script
if [ -f "$SOURCE_DIR/run_camera_direct.sh" ]; then
    echo "📜 Installing camera script..."
    cp "$SOURCE_DIR/run_camera_direct.sh" /usr/local/bin/rcsim-cam
    chmod +x /usr/local/bin/rcsim-cam
else
    echo "⚠️ Warning: run_camera_direct.sh not found, skipping script install."
fi

echo "✅ Configuration created at $CONFIG_FILE"

# Create systemd service
echo "📄 Creating systemd service..."
cat > /etc/systemd/system/mediamtx.service << EOF
[Unit]
Description=MediaMTX WebRTC Server (RCSIM)
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$INSTALL_USER
WorkingDirectory=$MEDIAMTX_DIR
ExecStart=$MEDIAMTX_DIR/mediamtx $CONFIG_FILE
Restart=always
RestartSec=5
StartLimitInterval=60
StartLimitBurst=10

# Resource limits
LimitNOFILE=65536

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable service
echo "🔄 Enabling MediaMTX service..."
systemctl daemon-reload
systemctl enable mediamtx.service
systemctl start mediamtx.service

# Wait a moment for service to start
sleep 2

# Check status
echo ""
echo "📺 MediaMTX Installation Complete!"
echo ""
echo "Service status:"
systemctl status mediamtx.service --no-pager || true
echo ""
echo "🌐 WebRTC Endpoints:"
echo "  Main FPV:  http://localhost:8889/camera/whep"
echo "  AI Stream: http://localhost:8889/camera_ai/whep"
echo "  API:       http://localhost:9997/"
echo ""
echo "📋 View logs: sudo journalctl -u mediamtx.service -f"
echo "🧪 Test stream: curl http://localhost:8889/camera"
