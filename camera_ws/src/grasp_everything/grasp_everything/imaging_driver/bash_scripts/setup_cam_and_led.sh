#!/usr/bin/env bash
set -Eeuo pipefail

SYS_PY="/usr/bin/python3"
RUN_PY="/home/hayashi-rpi/imaging_driver/run.py"

VENV_PY="/home/hayashi-rpi/venvs/ws2812/bin/python"
LED_PY="/home/hayashi-rpi/imaging_driver/ws2812.py"

LED_LOG="/tmp/ws2812.log"

log() {
  # 例: 2025-12-29 14:23:10 [INFO] message
  printf "%(%F %T)T [%s] %s\n" -1 "${1:-INFO}" "${2:-}"
}

start_led() {
  # root側で起動して “本体PID” をechoする（sudoのPIDではない）
  sudo bash -c "nohup '$VENV_PY' '$LED_PY' >>'$LED_LOG' 2>&1 & echo \$!"
}

stop_led() {
  if [[ -n "${LED_PID:-}" ]] && sudo kill -0 "$LED_PID" 2>/dev/null; then
    log INFO "Stopping LED (pid=$LED_PID) ..."
    sudo kill -INT "$LED_PID" 2>/dev/null || true
    sleep 0.2
    if sudo kill -0 "$LED_PID" 2>/dev/null; then
      log WARN "LED still running; sending SIGTERM ..."
      sudo kill -TERM "$LED_PID" 2>/dev/null || true
    fi
    sleep 0.2
    if sudo kill -0 "$LED_PID" 2>/dev/null; then
      log WARN "LED still running; sending SIGKILL ..."
      sudo kill -KILL "$LED_PID" 2>/dev/null || true
    fi
    log INFO "LED stopped."
  else
    log INFO "LED already stopped."
  fi
}

on_int() {
  log INFO "Ctrl+C detected. Shutting down camera server and LED..."
  # run.py は前面で動いているので、ここでは何もしなくても SIGINT は run.py に入る
  # ただし、trapが先に動いてしまう環境もあるので、明示的に続行してEXITへ
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
