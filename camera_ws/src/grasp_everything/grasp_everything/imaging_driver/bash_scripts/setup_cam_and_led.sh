#!/usr/bin/env bash
set -Eeuo pipefail

SYS_PY="/usr/bin/python3"
RUN_PY="/home/hayashi-rpi/imaging_driver/run.py"

VENV_PY="/home/hayashi-rpi/venvs/ws2812/bin/python"
LED_PY="/home/hayashi-rpi/imaging_driver/ws2812.py"

LED_LOG="/tmp/ws2812.log"
LED_PID_FILE="/tmp/ws2812.pid"

log() {
  printf "%(%F %T)T [%s] %s\n" -1 "${1:-INFO}" "${2:-}"
}

start_led() {
  # rootで起動してPIDをファイルに保存（nohupを削除）
  sudo bash -c "'$VENV_PY' '$LED_PY' >>'$LED_LOG' 2>&1 & echo \$! > '$LED_PID_FILE'"
  sleep 0.5
  if [[ -f "$LED_PID_FILE" ]]; then
    cat "$LED_PID_FILE"
  else
    echo ""
  fi
}

stop_led() {
  local pid="${LED_PID:-}"
  
  # PIDファイルからも取得を試みる
  if [[ -z "$pid" ]] && [[ -f "$LED_PID_FILE" ]]; then
    pid="$(cat "$LED_PID_FILE" 2>/dev/null || true)"
  fi
  
  # プロセス名でも検索（フォールバック）
  if [[ -z "$pid" ]] || ! sudo kill -0 "$pid" 2>/dev/null; then
    pid="$(pgrep -f 'python.*ws2812.py' 2>/dev/null | head -1 || true)"
  fi
  
  if [[ -n "$pid" ]] && sudo kill -0 "$pid" 2>/dev/null; then
    log INFO "Stopping LED (pid=$pid) ..."
    sudo kill -INT "$pid" 2>/dev/null || true
    sleep 0.3
    if sudo kill -0 "$pid" 2>/dev/null; then
      log WARN "LED still running; sending SIGTERM ..."
      sudo kill -TERM "$pid" 2>/dev/null || true
    fi
    sleep 0.3
    if sudo kill -0 "$pid" 2>/dev/null; then
      log WARN "LED still running; sending SIGKILL ..."
      sudo kill -KILL "$pid" 2>/dev/null || true
    fi
    log INFO "LED stopped."
  else
    log INFO "LED already stopped or not found."
  fi
  
  # PIDファイル削除
  sudo rm -f "$LED_PID_FILE" 2>/dev/null || true
}

on_int() {
  log INFO "Ctrl+C detected. Shutting down camera server and LED..."
}

cleanup() {
  log INFO "Cleanup started."
  stop_led
  log INFO "Cleanup finished."
  log INFO "LED log: $LED_LOG"
}

trap on_int INT
trap cleanup EXIT TERM

log INFO "Starting LED..."
LED_PID="$(start_led)"
log INFO "LED started (pid=$LED_PID). Logging to $LED_LOG"

log INFO "Starting camera server..."
"$SYS_PY" "$RUN_PY"

log INFO "Camera server exited."
