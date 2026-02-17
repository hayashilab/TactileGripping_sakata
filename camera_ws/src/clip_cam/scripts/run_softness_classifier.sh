#!/bin/bash
# OpenCLIP softness classifier ROS2 node launcher
# uv仮想環境を有効化してからノードを実行

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

# uv仮想環境を有効化
source "$PACKAGE_DIR/.venv/bin/activate"

# ROS2ワークスペースをsource
source /home/hayashi/worksp/camera_ws/install/setup.bash

# ノードを実行
exec python -m clip_cam.softness_classifier_node "$@"
