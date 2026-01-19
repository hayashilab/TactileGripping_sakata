#!/usr/bin/env bash
set -euo pipefail

echo "=== Uninstalling systemd services ==="

# サービスを停止
echo "Stopping services..."
sudo systemctl stop run_imaging.service 2>/dev/null || true
sudo systemctl stop ws2812.service 2>/dev/null || true

# サービスを無効化
echo "Disabling services..."
sudo systemctl disable run_imaging.service 2>/dev/null || true
sudo systemctl disable ws2812.service 2>/dev/null || true

# serviceファイルを削除
echo "Removing service files..."
sudo rm -f /etc/systemd/system/run_imaging.service
sudo rm -f /etc/systemd/system/ws2812.service

# systemdをリロード
echo "Reloading systemd..."
sudo systemctl daemon-reload

echo ""
echo "=== Uninstallation complete ==="
