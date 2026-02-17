#!/usr/bin/env python3
"""
Close the gripper slowly until tactile contact is detected.

Example:
  ros2 run gripper_control gripper_close_until_contact --ros-args \
    -p start_mm:=68.0 -p min_mm:=10.0 -p step_mm:=0.2 -p publish_hz:=5.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class GripperCloseUntilContact(Node):
    def __init__(self):
        super().__init__("gripper_close_until_contact")

        self.declare_parameter("start_mm", 68.0)
        self.declare_parameter("min_mm", 10.0)
        self.declare_parameter("step_mm", 0.2)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("pose_topic", "gripper/gripper_pose_in_mm")
        self.declare_parameter("contact_state_topic", "/tactile/contact_state")
        self.declare_parameter("auto_shutdown_on_contact", True)

        self.start_mm = float(self.get_parameter("start_mm").value)
        self.min_mm = float(self.get_parameter("min_mm").value)
        self.step_mm = float(self.get_parameter("step_mm").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.contact_state_topic = str(self.get_parameter("contact_state_topic").value)
        self.auto_shutdown_on_contact = bool(
            self.get_parameter("auto_shutdown_on_contact").value
        )

        if self.step_mm <= 0.0:
            self.get_logger().warn("step_mm <= 0.0, using 0.1")
            self.step_mm = 0.1

        if self.start_mm < self.min_mm:
            self.get_logger().warn("start_mm < min_mm, clamping start_mm to min_mm")
            self.start_mm = self.min_mm

        self._current_mm = self.start_mm
        self._contacted = False
        self._done = False

        self.pose_pub = self.create_publisher(Float32, self.pose_topic, 10)
        self.create_subscription(
            String, self.contact_state_topic, self._on_contact_state, 10
        )

        period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "Closing gripper until CONTACT: start_mm=%.3f min_mm=%.3f step_mm=%.3f hz=%.2f"
            % (self.start_mm, self.min_mm, self.step_mm, self.publish_hz)
        )
        self.get_logger().info("Publishing to %s", self.pose_topic)
        self.get_logger().info("Listening on %s", self.contact_state_topic)

    def _on_contact_state(self, msg: String):
        state = msg.data.strip()
        if state == "CONTACT" and not self._contacted:
            self._contacted = True
            self.get_logger().info("CONTACT detected, stopping publish loop")
            if self.auto_shutdown_on_contact:
                self._done = True

    def _on_timer(self):
        if self._done:
            self.timer.cancel()
            self.get_logger().info("Shutdown requested")
            rclpy.shutdown()
            return

        if self._contacted:
            return

        msg = Float32()
        msg.data = float(self._current_mm)
        self.pose_pub.publish(msg)

        if self._current_mm <= self.min_mm:
            self.get_logger().info("Reached min_mm without CONTACT, stopping")
            self._done = True
            return

        self._current_mm = max(self.min_mm, self._current_mm - self.step_mm)


def main(args=None):
    rclpy.init(args=args)
    node = GripperCloseUntilContact()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
