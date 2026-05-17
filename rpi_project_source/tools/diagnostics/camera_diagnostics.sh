#!/bin/bash
# Skrypt diagnostyczny dla kamery IMX219 na Raspberry Pi 5

echo "=== RCSIM Camera Diagnostics ==="
echo ""

echo "1. Sprawdzanie sprzętu kamery..."
echo "--------------------------------"
ls -la /dev/video* 2>/dev/null || echo "❌ Brak /dev/video*"
ls -la /dev/media* 2>/dev/null || echo "❌ Brak /dev/media*"
ls -la /dev/dma_heap* 2>/dev/null || echo "❌ Brak /dev/dma_heap"
echo ""

echo "2. Sprawdzanie konfiguracji systemu..."
echo "---------------------------------------"
echo "📄 /boot/firmware/config.txt (camera settings):"
grep -E "camera|dtoverlay.*219" /boot/firmware/config.txt
echo ""

echo "3. Sprawdzanie libcamera..."
echo "---------------------------"
libcamera-hello --list-cameras 2>&1 || echo "❌ libcamera error"
echo ""

echo "4. v4l2-ctl devices..."
echo "----------------------"
v4l2-ctl --list-devices 2>&1 || echo "❌ v4l2-ctl not available"
echo ""

echo "5. Sprawdzanie dmesg (ostatnie 50 linii o kamerze)..."
echo "-----------------------------------------------------"
sudo dmesg | grep -i "camera\|imx219\|csi" | tail -50
echo ""

echo "6. Sprawdzanie uprawnień użytkownika..."
echo "----------------------------------------"
echo "Grupy użytkownika:"
groups
echo ""
echo "ID użytkownika:"
id
echo ""

echo "7. Sprawdzanie config.json w kontenerze..."
echo "-------------------------------------------"
sudo docker exec rcsim_industrial cat /app/config.json 2>/dev/null | grep -A 5 "camera" || echo "❌ Container not running or config missing"
echo ""

echo "8. Status serwisu MediaMTX..."
echo "--------------------------"
sudo systemctl status mediamtx.service --no-pager || echo "❌ MediaMTX service NOT found"
echo ""

echo "=== KONIEC DIAGNOSTYKI ==="
