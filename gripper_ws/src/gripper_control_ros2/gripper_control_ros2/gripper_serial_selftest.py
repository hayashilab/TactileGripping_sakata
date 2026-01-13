#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Serial self-test for the gripper stepper controller.

What this script does (in a safe-by-default manner):
  1) Opens the serial port.
  2) Reads several status registers (encoder, pulses, enable pin, shaft status).
  3) Optionally toggles enable pin (recommended).
  4) Optionally performs a very small relative move (disabled by default).

Usage examples:
  - Basic (no motion):
      ./gripper_serial_selftest.py --port /dev/FT232R_gripper --baud 9600

  - Toggle enable pin and re-read status:
      ./gripper_serial_selftest.py --port /dev/FT232R_gripper --baud 9600 --toggle-enable

  - Small relative move (CAUTION: will move):
      ./gripper_serial_selftest.py --port /dev/FT232R_gripper --baud 9600 --move-step 200 --rpm 200

Exit codes:
  0: success (communication OK)
  2: serial port open failed
  3: communication failed (no valid replies)
  4: move command failed (if motion requested)
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback


def _import_step_control():
    """Import STEP_CONTROL from the installed package or from a local workspace."""
    try:
        from gripper_control_ros2.motor_control import STEP_CONTROL  # type: ignore
        return STEP_CONTROL
    except Exception:
        # Fallback for running from a workspace without installation
        # (e.g., python3 path/to/script.py while in repo root).
        import os
        from pathlib import Path

        here = Path(__file__).resolve()
        # Common layouts:
        #   <ws>/src/gripper_control_ros2/gripper_control_ros2/motor_control.py
        #   <pkg>/gripper_control_ros2/motor_control.py
        candidates = [
            here.parent / "gripper_control_ros2",
            here.parent.parent / "gripper_control_ros2",
            Path(os.getcwd()) / "gripper_control_ros2",
            Path(os.getcwd()) / "src" / "gripper_control_ros2" / "gripper_control_ros2",
        ]
        for c in candidates:
            if (c / "motor_control.py").exists():
                sys.path.insert(0, str(c.parent))
                try:
                    from gripper_control_ros2.motor_control import STEP_CONTROL  # type: ignore
                    return STEP_CONTROL
                except Exception:
                    pass
        raise


def _fmt_ret(ret: int) -> str:
    """Human-friendly interpretation of chkReturn codes."""
    if ret == 1:
        return "OK"
    if ret == -1:
        return "ERR(-1): RX length mismatch (timeout / no reply)"
    if ret == -2:
        return "ERR(-2): ID mismatch (talking to a different device?)"
    if ret == -3:
        return "ERR(-3): checksum mismatch (baudrate/noise?)"
    return f"ERR({ret})"


def _read_with_retries(name: str, fn, tries: int, delay_s: float):
    last = None
    for i in range(tries):
        last = fn()
        # Most read APIs return (value, ret)
        if isinstance(last, tuple) and len(last) == 2:
            value, ret = last
            if ret == 1:
                return value, ret, i + 1
        time.sleep(delay_s)
    # Fallback shape
    if isinstance(last, tuple) and len(last) == 2:
        value, ret = last
        return value, ret, tries
    return last, -999, tries


def main() -> int:
    ap = argparse.ArgumentParser(description="Serial self-test for gripper stepper controller")
    ap.add_argument("--port", default="/dev/ttyUSB_Gripper", help="Serial device path (e.g., /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=9600, help="Baud rate")
    ap.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout (seconds)")
    ap.add_argument("--device-id", type=lambda x: int(x, 0), default=0xE0, help="Device ID in hex (default 0xE0)")
    ap.add_argument("--tries", type=int, default=5, help="Read retries")
    ap.add_argument("--delay", type=float, default=0.15, help="Delay between retries (seconds)")
    ap.add_argument("--toggle-enable", action="store_true", help="Toggle enable pin ON then OFF then ON (CAUTION: affects motor enable)")
    ap.add_argument("--enable-only", action="store_true", help="Only attempt enable pin ON once (no OFF).")
    ap.add_argument("--move-step", type=int, default=0, help="Relative move in pulses/steps (0 disables motion). CAUTION: moves motor.")
    ap.add_argument("--rpm", type=int, default=200, help="RPM used for move command (if --move-step is nonzero)")
    ap.add_argument("--stop-after-move", action="store_true", help="Send stopMotor after move command (extra safety).")
    args = ap.parse_args()

    STEP_CONTROL = _import_step_control()

    print("=== Gripper serial self-test ===")
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Timeout: {args.timeout}s")
    print(f"Device ID: 0x{args.device_id:02X}")
    print("")

    sc = STEP_CONTROL(id=args.device_id, com_port=args.port, baudrate=args.baud)

    # Override timeout if present
    try:
        sc.ser.timeout = args.timeout
    except Exception:
        pass

    # Open
    try:
        opened = sc.open()
    except Exception as e:
        print(f"[FAIL] Could not open serial port: {e}")
        print("")
        print("Checklist:")
        print("  - Device path exists (ls -l /dev/ttyUSB* /dev/ttyACM*).")
        print("  - You have permissions (groups: dialout).")
        print("  - No other process is using the port (lsof /dev/XXX).")
        return 2

    if not opened:
        print("[FAIL] Serial port did not open (is_open=False).")
        return 2

    print("[OK] Serial port opened.")

    # Optional: clear stale bytes (pyserial)
    try:
        sc.ser.reset_input_buffer()
        sc.ser.reset_output_buffer()
    except Exception:
        pass

    # Read status block
    ok_reads = 0

    enc, enc_ret, enc_n = _read_with_retries("encoder", sc.readEncoderValue, args.tries, args.delay)
    print(f"readEncoderValue: value={enc} ret={enc_ret} ({_fmt_ret(enc_ret)}) tries={enc_n}")
    ok_reads += int(enc_ret == 1)

    pulses, pul_ret, pul_n = _read_with_retries("pulses", sc.readReceivedPulses, args.tries, args.delay)
    print(f"readReceivedPulses: value={pulses} ret={pul_ret} ({_fmt_ret(pul_ret)}) tries={pul_n}")
    ok_reads += int(pul_ret == 1)

    ang, ang_ret, ang_n = _read_with_retries("shaft_angle", sc.readMotorShaftAngle, args.tries, args.delay)
    print(f"readMotorShaftAngle: value={ang} ret={ang_ret} ({_fmt_ret(ang_ret)}) tries={ang_n}")
    ok_reads += int(ang_ret == 1)

    eang, eang_ret, eang_n = _read_with_retries("error_angle", sc.readMotorShaftErrorAngle, args.tries, args.delay)
    print(f"readMotorShaftErrorAngle: value={eang} ret={eang_ret} ({_fmt_ret(eang_ret)}) tries={eang_n}")
    ok_reads += int(eang_ret == 1)

    en, en_ret, en_n = _read_with_retries("en_status", sc.readEnPinStatus, args.tries, args.delay)
    print(f"readEnPinStatus: value={en} ret={en_ret} ({_fmt_ret(en_ret)}) tries={en_n}")
    ok_reads += int(en_ret == 1)

    shaft, sh_ret, sh_n = _read_with_retries("shaft_status", sc.readShaftStatus, args.tries, args.delay)
    print(f"readShaftStatus: value={shaft} ret={sh_ret} ({_fmt_ret(sh_ret)}) tries={sh_n}")
    ok_reads += int(sh_ret == 1)

    print("")
    if ok_reads == 0:
        print("[FAIL] No valid replies from the controller.")
        print("Most common causes:")
        print("  - Wrong baudrate (try 115200 etc.)")
        print("  - Controller board not powered (USB-serial alone may not be enough)")
        print("  - Wrong device ID (default 0xE0)")
        print("  - Wiring/GND issue or the port is not the controller")
        try:
            sc.close()
        except Exception:
            pass
        return 3

    # Enable pin operations (optional)
    if args.toggle_enable or args.enable_only:
        print("=== Enable pin test ===")
        # ON
        ret_on = sc.setEnablePin(True)
        print(f"setEnablePin(ON): ret={ret_on} ({_fmt_ret(ret_on)})")
        time.sleep(0.2)
        en2, en2_ret, _ = _read_with_retries("en_status_after_on", sc.readEnPinStatus, args.tries, args.delay)
        print(f"readEnPinStatus(after ON): value={en2} ret={en2_ret} ({_fmt_ret(en2_ret)})")

        if args.toggle_enable:
            # OFF
            ret_off = sc.setEnablePin(False)
            print(f"setEnablePin(OFF): ret={ret_off} ({_fmt_ret(ret_off)})")
            time.sleep(0.2)
            en3, en3_ret, _ = _read_with_retries("en_status_after_off", sc.readEnPinStatus, args.tries, args.delay)
            print(f"readEnPinStatus(after OFF): value={en3} ret={en3_ret} ({_fmt_ret(en3_ret)})")

            # ON again
            ret_on2 = sc.setEnablePin(True)
            print(f"setEnablePin(ON again): ret={ret_on2} ({_fmt_ret(ret_on2)})")
            time.sleep(0.2)
            en4, en4_ret, _ = _read_with_retries("en_status_after_on2", sc.readEnPinStatus, args.tries, args.delay)
            print(f"readEnPinStatus(after ON2): value={en4} ret={en4_ret} ({_fmt_ret(en4_ret)})")

        print("")

    # Optional motion test
    if args.move_step != 0:
        print("=== Motion test (CAUTION) ===")
        print(f"Requesting relative move: step={args.move_step}, rpm={args.rpm}")
        ret_mv = sc.setMotorRelativePose(args.rpm, args.move_step)
        print(f"setMotorRelativePose: ret={ret_mv} ({_fmt_ret(ret_mv)})")
        if ret_mv != 1:
            try:
                sc.close()
            except Exception:
                pass
            return 4

        # Wait an estimated amount of time then read pulses
        try:
            dt = sc.calDelayTime(args.rpm, args.move_step)
        except Exception:
            dt = 0.5
        dt = max(0.2, min(float(dt), 5.0))
        time.sleep(dt)

        pulses2, pul2_ret, _ = _read_with_retries("pulses_after_move", sc.readReceivedPulses, args.tries, args.delay)
        print(f"readReceivedPulses(after move): value={pulses2} ret={pul2_ret} ({_fmt_ret(pul2_ret)})")

        if args.stop_after_move:
            ret_stop = sc.stopMotor()
            print(f"stopMotor: ret={ret_stop} ({_fmt_ret(ret_stop)})")
        print("")

    # Close
    try:
        sc.close()
    except Exception:
        pass

    print("[DONE] Self-test completed.")
    print("Interpretation tips:")
    print("  - If reads ret=-1: no reply / timeout (power, baudrate, wrong port).")
    print("  - If reads ret=-3: checksum mismatch (baudrate mismatch/noise).")
    print("  - If reads ret=-2: ID mismatch (device id differs from 0xE0).")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
        raise
    except Exception:
        print("\n[CRASH] Unexpected error:")
        traceback.print_exc()
        raise
