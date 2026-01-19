#!/usr/bin/env bash
set -euo pipefail

# ---- settings ----
HOST="imaging.local"
USER="hayashi-rpi"
PORT="65535"
KEY="$HOME/.ssh/id_ed25519"  # このスクリプトから見た秘密鍵のパス
REMOTE_CMD="bash imaging_driver/bash_scripts/setup_cam_and_led.sh"
# ------------------

# 鍵ファイルの存在チェック
if [[ ! -f "$KEY" ]]; then
  echo "ERROR: SSH key not found: $KEY" >&2
  exit 1
fi

# 既知ホスト確認で止まるのが嫌なら StrictHostKeyChecking を調整してください
# -t: TTYを割り当てることでCtrl+C (SIGINT) がリモートに正しく伝わる
ssh -t -i "$KEY" -p "$PORT" "${USER}@${HOST}" -- "$REMOTE_CMD"
