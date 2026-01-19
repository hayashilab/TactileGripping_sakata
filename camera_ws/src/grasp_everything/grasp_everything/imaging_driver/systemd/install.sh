#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Installing systemd services ==="

# serviceファイルをコピー
echo "Copying service files..."
sudo cp "$SCRIPT_DIR/ws2812.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/run_imaging.service" /etc/systemd/system/

# systemdをリロード
echo "Reloading systemd..."
sudo systemctl daemon-reload

# サービスを有効化
echo "Enabling services..."
sudo systemctl enable ws2812.service
sudo systemctl enable run_imaging.service

echo ""
echo "=== Installation complete ==="
echo ""
echo "Commands:"
echo "  Start:   sudo systemctl start run_imaging.service"
echo "  Stop:    sudo systemctl stop run_imaging.service"
echo "  Status:  sudo systemctl status run_imaging.service ws2812.service"
echo "  Logs:    sudo journalctl -u run_imaging.service -u ws2812.service -f"
echo ""
echo "Note: Starting run_imaging.service will automatically start ws2812.service"
echo "      Stopping run_imaging.service will automatically stop ws2812.service"
