#!/usr/bin/env python3
"""
ROS 2 port of the original ROS 1 gripper_control node.

Topics (kept the same as the ROS 1 implementation):
- Subscribed:
    - gripper/gripper_pose_in_step (std_msgs/Int32)
    - gripper/gripper_pose_in_mm   (std_msgs/Int32)
- Provided service:
    - gripper/set_zero (std_srvs/Empty)

Parameters (private in ROS 1; regular ROS 2 parameters here):
- port_stepper      (string) default: "/dev/ttyUSB_GRIPPER"
- baudrate_stepper  (int)    default: 9600
- speed_rpm         (int)    default: 3000
- cmd_timeout       (int)    default: 10   # max retries per command
- max_mm            (int)    default: 130
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32
from std_srvs.srv import Empty

from scipy.interpolate import InterpolatedUnivariateSpline

from .motor_control import STEP_CONTROL


class GripperControlNode(Node):
    def __init__(self) -> None:
        super().__init__('gripper_control')

        # Parameters (mirroring ROS 1 "~port_stepper", "~baudrate_stepper")
        self.declare_parameter('port_stepper', '/dev/ttyUSB_Gripper')
        self.declare_parameter('baudrate_stepper', 9600)
        self.declare_parameter('speed_rpm', 3000)
        self.declare_parameter('cmd_timeout', 10)
        self.declare_parameter('max_mm', 130)

        self.com_port: str = self.get_parameter('port_stepper').get_parameter_value().string_value
        self.baudrate: int = self.get_parameter('baudrate_stepper').get_parameter_value().integer_value
        self.speed_rpm: int = self.get_parameter('speed_rpm').get_parameter_value().integer_value
        self.cmd_timeout: int = self.get_parameter('cmd_timeout').get_parameter_value().integer_value
        self.max_mm: int = self.get_parameter('max_mm').get_parameter_value().integer_value

        # Original ROS 1 state
        self._previous_cmd_pose: int = -1000000

        # Command arbitration (avoid blocking subscriber callbacks)
        self._lock = threading.Lock()
        self._target_step: Optional[int] = None
        self._pending_retries: int = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub_step = self.create_subscription(
            Int32, 'gripper/gripper_pose_in_step', self._on_step_cmd, qos
        )
        self._sub_mm = self.create_subscription(
            Int32, 'gripper/gripper_pose_in_mm', self._on_mm_cmd, qos
        )

        self._srv_zero = self.create_service(Empty, 'gripper/set_zero', self._on_set_zero)

        # Hardware
        self.step_control = STEP_CONTROL(com_port=self.com_port, baudrate=self.baudrate)

        self._open_or_die()

        # Diagnostics (best-effort, non-fatal)
        self._log_motor_state()

        # mm -> step mapping (copied from ROS 1 code)
        x = [0, 10, 22, 33, 47, 58, 81, 102, 121, 150]  # actual mm measured
        y = [-41000, -39612, -37500, -35000, -32500, -30000, -25000, -20000, -15000, -10000]  # stepper pose
        self.mm2step_gripper = InterpolatedUnivariateSpline(x, y, k=5, check_finite=False)

        # Timer-based command sender
        self._timer = self.create_timer(0.02, self._process_command_queue)  # 50 Hz

        self.get_logger().info(
            f"Initialized with port_stepper={self.com_port}, baudrate_stepper={self.baudrate}, speed_rpm={self.speed_rpm}"
        )

    def _open_or_die(self) -> None:
        try:
            ok = self.step_control.open()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.com_port}: {e}")
            raise

        if not ok:
            raise RuntimeError(f"Could not open serial port: {self.com_port}")

        ret = self.step_control.setEnablePin(True)
        if ret != 1:
            self.get_logger().warn(f"Failed to enable motor (setEnablePin): ret={ret}")
        else:
            self.get_logger().info("Motor enabled.")

    def _log_motor_state(self) -> None:
        try:
            enc, _ = self.step_control.readEncoderValue()
            pulses, _ = self.step_control.readReceivedPulses()
            ang, _ = self.step_control.readMotorShaftAngle()
            err, _ = self.step_control.readMotorShaftErrorAngle()
            en, _ = self.step_control.readEnPinStatus()
            shaft, _ = self.step_control.readShaftStatus()

            self.get_logger().info(f"encoder value: {enc}")
            self.get_logger().info(f"pulse received: {pulses}")
            self.get_logger().info(f"motor angle: {ang}")
            self.get_logger().info(f"error angle: {err}")
            self.get_logger().info(f"en pin status: {en}")
            self.get_logger().info(f"shaft status: {shaft}")
        except Exception as e:
            self.get_logger().warn(f"Motor diagnostics failed (non-fatal): {e}")

    def _on_mm_cmd(self, msg: Int32) -> None:
        mm = int(msg.data)
        mm = min(mm, self.max_mm)

        pose_in_step = int(self.mm2step_gripper(mm))
        pose_in_step = max(pose_in_step, -41000)

        self._enqueue_step_target(pose_in_step)

    def _on_step_cmd(self, msg: Int32) -> None:
        self._enqueue_step_target(int(msg.data))

    def _enqueue_step_target(self, step_pose: int) -> None:
        with self._lock:
            # Deduplicate at the "queue" edge
            if step_pose == self._previous_cmd_pose:
                return
            self._target_step = step_pose
            self._pending_retries = 0

    def _process_command_queue(self) -> None:
        with self._lock:
            target = self._target_step
            retries = self._pending_retries

        if target is None:
            return

        if target == self._previous_cmd_pose:
            # already executed
            with self._lock:
                self._target_step = None
            return

        # Try to send once per timer tick. Retries are spread over time to keep node responsive.
        try:
            ret = self.step_control.setMotorAbsolutePose(self.speed_rpm, target)
        except Exception as e:
            ret = None
            self.get_logger().error(f"Exception while sending command to motor: {e}")

        if ret == 1:
            self.get_logger().debug(f"Set stepper to {target} (OK)")
            self._previous_cmd_pose = target
            with self._lock:
                self._target_step = None
            return

        retries += 1
        if retries >= self.cmd_timeout:
            self.get_logger().warn(f"Failed to command motor to {target} after {retries} retries (last ret={ret})")
            with self._lock:
                self._target_step = None
        else:
            with self._lock:
                self._pending_retries = retries

    def _on_set_zero(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        # This can take some time; keep it synchronous but do minimal work elsewhere via timers
        self.get_logger().info("Set Zero requested.")
        try:
            ret = self.step_control.setZero(step=50, rpm=50)
            enc, _ = self.step_control.readEncoderValue()
            self.get_logger().info(f"Set Zero result: {ret}, encoder: {enc}")
            self._previous_cmd_pose = 0
        except Exception as e:
            self.get_logger().error(f"Set Zero failed: {e}")
        return response

    def destroy_node(self) -> bool:
        try:
            self.step_control.close()
        except Exception as e:
            self.get_logger().warn(f"Failed to close serial port cleanly: {e}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
