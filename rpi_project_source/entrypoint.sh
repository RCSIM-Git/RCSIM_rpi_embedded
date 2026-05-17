#!/bin/bash
# entrypoint.sh - Container startup script with MediaMTX integration

set -e

echo "🎬 Starting RCSIM in Sidecar Mode..."

# === STEP 1: Wait for MediaMTX to be ready (Host Service) ===
echo "⏳ Waiting for Host MediaMTX to start on port 8554..."
MAX_RETRIES=30
for i in $(seq 1 $MAX_RETRIES); do
    if timeout 1 bash -c "echo >/dev/tcp/127.0.0.1/8554" 2>/dev/null; then
        echo "   ✅ MediaMTX is UP! (attempt $i/$MAX_RETRIES)"
        break
    fi
    if [ $i -eq $MAX_RETRIES ]; then
        echo "   ❌ TIMEOUT: MediaMTX failed to start on port 8554!"
        echo "   Please ensure 'mediamtx.service' is running on the host RPi."
        echo "   Check logs on host: 'sudo journalctl -u mediamtx -n 50'"
        exit 1
    fi
    sleep 1
done

# === STEP 3: Set Python environment ===
echo "🐍 Configuring Python environment..."
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.11/dist-packages:/usr/lib/python3/dist-packages:/usr/lib/python3.11/dist-packages

# === STEP 4: Start RCSIM Supervisor ===
echo "🐍 Starting RCSIM Supervisor..."
exec python3 core/supervisor.py

