#!/usr/bin/env python3
"""
ROS 2 friendly port of GRIPPER_UTILS.

Notes:
- This is a *helper* class intended to be imported by your scripts/nodes.
- In ROS 1, it published to '/gripper/position_in_mm_sub' (Float64). That topic name
  does not match the ROS 1 gripper_control subscribers by default. Keep or remap
  as needed in your system.
"""

from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64
from std_srvs.srv import Empty

from scipy.interpolate import InterpolatedUnivariateSpline


@dataclass
class GripperUtilsConfig:
    topic_cmd_mm: str = '/gripper/position_in_mm_sub'
    srv_init_home: str = '/gripper/init_home'


class GripperUtils:
    def __init__(self, node: Node, config: GripperUtilsConfig = GripperUtilsConfig()) -> None:
        self._node = node
        self._cfg = config

        self._pub = self._node.create_publisher(Float64, self._cfg.topic_cmd_mm, QoSProfile(depth=1))
        self._pose = Float64()

        x = [-11, 5, 10, 30, 78, 110, 134, 141]  # actual value measure
        y = [100, 93, 90, 80, 60, 40, 20, 0]  # value to send to stepper motor
        self.mm2step_gripper = InterpolatedUnivariateSpline(x, y, k=5, check_finite=False)

    def set_gripper(self, pose_mm: float) -> None:
        # In the original code this prints "step in mm actual"; leaving as debug log.
        self._node.get_logger().debug(f"mm2step_gripper({pose_mm}) = {self.mm2step_gripper(pose_mm)}")
        self._pose.data = float(int(self.mm2step_gripper(pose_mm)))
        self._pub.publish(self._pose)

    def set_gripper_raw(self, raw: float) -> None:
        self._pose.data = float(int(raw))
        self._pub.publish(self._pose)

    def init_home(self, timeout_sec: float = 5.0) -> None:
        self._node.get_logger().info("init gripper home")
        client = self._node.create_client(Empty, self._cfg.srv_init_home)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service not available: {self._cfg.srv_init_home}")

        req = Empty.Request()
        future = client.call_async(req)
        rclpy = __import__('rclpy')
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            raise RuntimeError(f"init_home service call failed: {future.exception()}")
        self._node.get_logger().info("gripper homing finished")
